// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>
#include <networktables/NetworkTableInstance.h>

#include <algorithm>

#include "subsystems/IntakeSubsystem.h"
#include "Constants.h"

using namespace IntakeConstants;

   /* Intake subsystem contructor. */
IntakeSubsystem::IntakeSubsystem()
   : m_intakeSparkMax{kIntakeMotorCanId, rev::CANSparkMax::MotorType::kBrushless},
     m_intakeFeedFoward{kIntakekS, kIntakekV, kIntakekA},
     m_noteDetect{kNoteDetectInputChannel}
{
   // Factory reset, so we get the SPARKS MAX to a known state before configuring
   // them. This is useful in case a SPARK MAX is swapped out.
   m_intakeSparkMax.RestoreFactoryDefaults();

   // Set the intake motor inverted state based on the constant expression.
   m_intakeSparkMax.SetInverted(true);

   // Apply position and velocity conversion factors for the driving encoder. The
   // native units for position and velocity are rotations and RPM, respectively,
   // but we want meters and meters per second to use with WPILib's swerve APIs.
   m_intakeEncoder.SetPositionConversionFactor(kIntakeEncoderPositionFactor);
   m_intakeEncoder.SetVelocityConversionFactor(kIntakeEncoderVelocityFactor);

   // Set the Drive PID Controller to use the internal encoder.
   m_intakePIDController.SetFeedbackDevice(m_intakeEncoder);

   // Set the PID gains for the driving motor. Note these are example gains, and
   // you may need to tune them for your own robot!
   m_intakePIDController.SetP(kIntakeP);
   m_intakePIDController.SetI(kIntakeI);
   m_intakePIDController.SetD(kIntakeD);
   m_intakePIDController.SetFF(kIntakeFF);
   m_intakePIDController.SetOutputRange(kIntakeMinOutput, kIntakeMaxOutput);

   // Set the intake motors idle mode and current limit.
   m_intakeSparkMax.SetIdleMode(kIntakeMotorIdleMode);
   m_intakeSparkMax.SetSmartCurrentLimit(kInakeMotorCurrentLimit.value());

   /* Enable voltage compensation on the intake spark max motor        */
   /* controllers.                                                     */
   m_intakeSparkMax.EnableVoltageCompensation(12.0);

   /* *** NOTE *** Check out the below resources for more information on*/
   /*            what the below does.  If you increase the Odometry     */
   /*            Periodic Rate you might need to change the values      */
   /*            below.                                                 */
// see https://github.com/bovlb/frc-tips/blob/main/can-bus/README.md
// see https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames for details.

   /* Set the spark max periodic frame periods for each of the intake   */
   /* spark max motor controllers.                                      */
   m_intakeSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus0, 55);      // Applied *** Output, Faults, Stick Faults, Is Follower
   m_intakeSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus1, 15);      // Motor Velocity, Motor Tempature, Motor Voltage, Motor Current
   m_intakeSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 65535);   // Motor Position
   m_intakeSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus3, 65535);   // Analog Sensor Voltage, Velocity, Position
   m_intakeSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus4, 65535);   // Alternate Encoder Velocity, Position
   m_intakeSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus5, 65535);   // Duty Cycle Absolute Encoder Position, Absolute Angle
   m_intakeSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus6, 65565);   // Duty Cycle Absolute Encoder Velocity, Frequency

   // Save the SPARK MAX configurations. If a SPARK MAX browns out during
   // operation, it will maintain the above configurations.
   m_intakeSparkMax.BurnFlash();

   /* Initialize the current setpoint speed to zero.                    */
   m_currentSetpointSpeed = 0.0_rpm;

   // Start publishing intake state information with the "/Intake" key
   m_intakeMeasuredVelocityPublisher  = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("/Intake/MeasuredVelocity").Publish();
   m_intakeMeasuredCurrentPublisher   = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("/Intake/MeasuredCurrent").Publish();
   m_intakeSetpointPublisher          = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("/Intake/Setpoint").Publish();
   m_intakeNoteDetectPublisher        = nt::NetworkTableInstance::GetDefault().GetBooleanTopic("/Intake/IsNoteDetected").Publish();
}

// This method will be called once per scheduler run
void IntakeSubsystem::Periodic()
{
   /* Log the current measured intake velocity.                         */
   m_intakeMeasuredVelocityPublisher.Set(m_intakeEncoder.GetVelocity());

   /* Log the current measured intake output current.                   */
   m_intakeMeasuredCurrentPublisher.Set(m_intakeSparkMax.GetOutputCurrent());

   /* Log the current state of the note detection input.                */
   m_intakeNoteDetectPublisher.Set(m_noteDetect.Get());
}

   /* Run the intake motor at the specified speed in RPMs.              */
void IntakeSubsystem::RunIntake(units::revolutions_per_minute_t speed)
{
   /* Clamp the speed to the minimum and maximum.                       */
   speed = std::clamp(speed, kIntakeMinimumSpeed, kIntakeMaximumSpeed);

   /* Log the set point state information.                              */
   m_intakeSetpointPublisher.Set(speed.value());

   /* Set the current setpoint speed variable to the specified speed.   */
   m_currentSetpointSpeed = speed;

   // Command the intake motor to run as the specified velocity
   m_intakePIDController.SetReference(speed.value(), rev::CANSparkLowLevel::ControlType::kVelocity, /*PID Slot*/0, m_intakeFeedFoward.Calculate(units::angular_velocity::turns_per_second_t{speed}).value(), rev::SparkMaxPIDController::ArbFFUnits::kVoltage);
}

   /* Stop the intake motor.                                            */
void IntakeSubsystem::StopIntake(void)
{
   /* Log the set point state information.                              */
   m_intakeSetpointPublisher.Set(0.0);

   /* Set the current setpoint speed variable to zero.                  */
   m_currentSetpointSpeed = 0.0_rpm;

   // Command the intake motor to stop the motor.
   m_intakePIDController.SetReference(0.0, rev::CANSparkLowLevel::ControlType::kVelocity, /*PID Slot*/0, m_intakeFeedFoward.Calculate(units::angular_velocity::turns_per_second_t{0.0}).value(), rev::SparkMaxPIDController::ArbFFUnits::kVoltage);
}

  /**
   * Generates a command to grab a note until a note is detected.
   */
frc2::CommandPtr IntakeSubsystem::GrabNoteCommand(units::revolutions_per_minute_t speed)
{
   /* Generated a command that runs the intake on initialization, does  */
   /* nothing while executing, stops the intake when finished or        */
   /* interrupted.  Uses note detection switch to stop the command.     */
   frc2::CommandPtr ret_val = frc2::FunctionalCommand(
                                                      [this, speed] { this->RunIntake(speed); },
                                                      [] {},
                                                      [this](bool interrupted){ this->StopIntake(); },
                                                      [this]{ return(this->IsNoteDetected()); },
                                                      {this} ).ToPtr();

   /* Give the command a readable name.                                 */
   ret_val.get()->SetName("GrabNoteCommand()");

   /* Return the command to the caller.                                 */
   return(ret_val);
}

   /* Get the current note detect state.                                */
bool IntakeSubsystem::IsNoteDetected(void)
{
   /* Return the current note detection input state to the caller.      */
   return(m_noteDetect.Get());
}

   /* Get the current speed of the intake.                              */
units::revolutions_per_minute_t IntakeSubsystem::GetSpeed(void)
{
   /* Simply return the current velocity as reported by the intake      */
   /* encoder.                                                          */
   return(units::revolutions_per_minute_t(m_intakeEncoder.GetVelocity()));
}

   /* Get the current setpoint speed for the intake.                    */
units::revolutions_per_minute_t IntakeSubsystem::GetSetpointSpeed(void)
{
   /* Simply return the current setpoint speed.                         */
   return(m_currentSetpointSpeed);
}


