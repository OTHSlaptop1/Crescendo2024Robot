// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ArmSubsystem.h"

#include "Constants.h"

#include <networktables/NetworkTableInstance.h>

#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

using namespace ArmConstants;

ArmSubsystem::ArmSubsystem()
    // The TrapezoidProfileSubsystem used by the subsystem
    : TrapezoidProfileSubsystem(
                // The constraints for the motion profiles
                {kArmMaxVelocity, kArmMaxAcceleration},
                // The initial position of the mechanism
                0_rad),
      m_leaderSparkMax(kArmLeaderCanId, rev::CANSparkMax::MotorType::kBrushless),
      m_followerSparkMax(kArmFollwerCanId, rev::CANSparkMax::MotorType::kBrushless),
      m_feedForward{kS, kG, kV, kA}

{
   // Factory reset, so we get the SPARKS MAX to a known state before configuring
   // them. This is useful in case a SPARK MAX is swapped out.
   m_leaderSparkMax.RestoreFactoryDefaults();
   m_followerSparkMax.RestoreFactoryDefaults();

   /* Set the follower spark max to follow the leader and have its      */
   /* output inverted.                                                  */
   m_followerSparkMax.Follow(m_leaderSparkMax, true);

   // Apply position and velocity conversion factors for the leader encoder. We
   // want these in radians and radians per second to use with the ARM.
   m_armAbsoluteEncoder.SetPositionConversionFactor(kArmEncoderPositionFactor);
   m_armAbsoluteEncoder.SetVelocityConversionFactor(kArmEncoderVelocityFactor);

   // Set the leader PID Controller to use the duty cycle encoder on the swerve
   // module instead of the built in NEO encoder.
   m_leaderPIDController.SetFeedbackDevice(m_armAbsoluteEncoder);

   // Set the PID gains for the arm motor.
   m_leaderPIDController.SetP(kArmP);
   m_leaderPIDController.SetI(kArmI);
   m_leaderPIDController.SetD(kArmD);
   m_leaderPIDController.SetFF(kArmFF);
   m_leaderPIDController.SetOutputRange(kArmMinOutput, kArmMaxOutput);

   m_leaderSparkMax.SetIdleMode(kArmMotorIdleMode);
   m_followerSparkMax.SetIdleMode(kArmMotorIdleMode);

   m_leaderSparkMax.SetSmartCurrentLimit(kLeaderMotorCurrentLimit.value());
   m_followerSparkMax.SetSmartCurrentLimit(kFollowerMotorCurrentLimit.value());

   /* Enable voltage compensation on the driving and turning spark max  */
   /* motor controllers.                                                */
   m_leaderSparkMax.EnableVoltageCompensation(12.0);
   m_followerSparkMax.EnableVoltageCompensation(12.0);

   /* *** NOTE *** Check out the below resources for more information on*/
   /*            what the below does.  If you increase the Odometry     */
   /*            Periodic Rate you might need to change the values      */
   /*            below.                                                 */
// see https://github.com/bovlb/frc-tips/blob/main/can-bus/README.md
// see https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames for details.

   /* Set the spark max periodic frame periods for each of the arm spark*/
   /* max motor controllers.                                            */
   /* ** NOTE ** Periodic Frame Status 0 is used by the follower to set */
   /*            its output.                                            */
   m_leaderSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus0, 10);      // Applied *** Output, Faults, Stick Faults, Is Follower
   m_leaderSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus1, 20);      // Motor Velocity, Motor Tempature, Motor Voltage, Motor Current
   m_leaderSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 20);      // Motor Position
   m_leaderSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus3, 65535);   // Analog Sensor Voltage, Velocity, Position
   m_leaderSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus4, 65535);   // Alternate Encoder Velocity, Position
   m_leaderSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus5, 20);      // Duty Cycle Absolute Encoder Position, Absolute Angle
   m_leaderSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus6, 65565);   // Duty Cycle Absolute Encoder Velocity, Frequency

   // Save the SPARK MAX configurations. If a SPARK MAX browns out during
   // operation, it will maintain the above configurations.
   m_leaderSparkMax.BurnFlash();
   m_followerSparkMax.BurnFlash();

   // Start publishing arm state information with the "/Arm" key
   m_armMeasuredPositionPublisher  = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("/Arm/MeasuredPosition").Publish();
   m_armSetpointPositionPublisher  = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("/Arm/SetpointPosition").Publish();
   m_armSetpointVelocityPublisher  = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("/Arm/SetpointVelocity").Publish();
   m_armOutputFeedForwardPublisher = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("/Arm/OutputFeedForward").Publish();
}

   /* The following function consumes the output of the trapezoid       */
   /* profile controller and the current setpoint state (which is used  */
   /* for computing a feedforward value).  The trapezoid profile (that  */
   /* is part of this object) automatically calls this method from its  */
   /* perodic() block and passes this function the computed output from */
   /* the control loop.                                                 */
void ArmSubsystem::UseState(frc::TrapezoidProfile<units::radians>::State setpoint)
{
   /* Calculate the feed forward from the specified setpoint.           */
   units::volt_t feedforward = m_feedForward.Calculate(setpoint.position, setpoint.velocity);

   /* Log the current arm output values.                                */
   m_armMeasuredPositionPublisher.Set(units::degree_t(units::radian_t{m_armAbsoluteEncoder.GetPosition()}).value());
   m_armSetpointPositionPublisher.Set(units::degree_t(setpoint.position).value());
   m_armSetpointVelocityPublisher.Set(setpoint.velocity.value());
   m_armOutputFeedForwardPublisher.Set(feedforward.value());

   /* Set the position for the motor with the calculated feed forward   */
   /* value.                                                            */
   m_leaderPIDController.SetReference(setpoint.position.value(), rev::CANSparkMax::ControlType::kPosition, 0/*PID Slot*/, feedforward.value(), rev::SparkMaxPIDController::ArbFFUnits::kVoltage);
}

   /* Generates a command to set the arm position.                      */
frc2::CommandPtr ArmSubsystem::SetArmPositionCommand(units::degree_t setpoint)
{
   /* Create a command to run the arm to the specified setpoint.        */
   return(frc2::cmd::RunOnce([this, setpoint] { this->SetGoal(units::radian_t{setpoint}); }, {this}));
}

