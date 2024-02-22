// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/ShooterSubsystem.h"

#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>

#include <units/angular_velocity.h>
#include <networktables/NetworkTableInstance.h>
#include <frc2/command/Commands.h>

using namespace ShooterConstants;
using namespace ctre::phoenix6;

   /* Shooter subsystem constructor.                                    */
ShooterSubsystem::ShooterSubsystem()
   : m_leftFlywheelFeedForward{kShooterLeftkS, kShooterLeftkV, kShooterLeftkA},
     m_rightFlywheelFeedForward{kShooterRightkS, kShooterRightkV, kShooterRightkA}
{
   configs::TalonFXConfiguration TalonFXConfigs{};
   configs::Slot0Configs         slot0Configs{};

   /* Configure the left motor with the parameters stored in constants. */
   TalonFXConfigs.CurrentLimits.SupplyCurrentLimit       = kShooterMotorCurrentLimit;
   TalonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

   TalonFXConfigs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
   TalonFXConfigs.MotorOutput.Inverted    = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;

   /* Apply the configuration changes to the left flywheel motor        */
   /* controllers.                                                      */
   m_leftFlywheel.GetConfigurator().Apply(TalonFXConfigs);

   /* Configure the right motor with the parameters stored in constants.*/
   TalonFXConfigs.CurrentLimits.SupplyCurrentLimit       = kShooterMotorCurrentLimit;
   TalonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

   TalonFXConfigs.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Brake;
   TalonFXConfigs.MotorOutput.Inverted    = ctre::phoenix6::signals::InvertedValue::Clockwise_Positive;

   m_rightFlywheel.GetConfigurator().Apply(TalonFXConfigs);

   /* Set the update frequency for the left and right flywheel velocity */
   /* signals to be 50 Hz.                                              */
   BaseStatusSignal::SetUpdateFrequencyForAll(50_Hz, m_leftFlywheel.GetVelocity(), m_rightFlywheel.GetVelocity(), m_leftFlywheel.GetSupplyCurrent(), m_rightFlywheel.GetSupplyCurrent());

   /* Now optimize bus utilization for the left and right motors (this  */
   /* disables all unused signals for these devices.                    */
   m_leftFlywheel.OptimizeBusUtilization();
   m_rightFlywheel.OptimizeBusUtilization();

   /* Now initialize the velocity PID configuration parameters and set  */
   /* them to the left flywheel motor.                                  */
   slot0Configs.kP = kShooterLeftP;
   slot0Configs.kI = kShooterLeftI;
   slot0Configs.kD = kShooterLeftD;
   slot0Configs.kS = 0.0;
   slot0Configs.kV = 0.0;
   slot0Configs.kA = 0.0;
   m_leftFlywheel.GetConfigurator().Apply(slot0Configs);

   /* Now initialize the velocity PID configuration parameters and set  */
   /* them to the right flywheel motor.                                 */
   slot0Configs.kP = kShooterRightP;
   slot0Configs.kI = kShooterRightI;
   slot0Configs.kD = kShooterRightD;
   slot0Configs.kS = 0.0;
   slot0Configs.kV = 0.0;
   slot0Configs.kA = 0.0;
   m_rightFlywheel.GetConfigurator().Apply(slot0Configs);

   /* Initialize the current setpoint speed to zero for each of the     */
   /* flywheels.                                                        */
   m_leftSetpointSpeed  = 0.0_rpm;
   m_rightSetpointSpeed = 0.0_rpm;

   // Start publishing intake state information with the "/Shooter" key
   m_shooterLeftMeasuredPublisher  = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("/Shooter/LeftMeasuredSpeed").Publish();
   m_shooterRightMeasuredPublisher = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("/Shooter/RightMeasuredSpeed").Publish();

   m_shooterLeftSetpointPublisher  = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("/Shooter/LeftSetpointSpeed").Publish();
   m_shooterRightSetpointPublisher = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("/Shooter/RightSetpointSpeed").Publish();

   m_shooterLeftSupplyCurrentPublisher  = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("/Shooter/LeftMeasuredSupplyCurrent").Publish();
   m_shooterRightSupplyCurrentPublisher = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("/Shooter/RightMeasuredSupplyCurrent").Publish();;
}

   /* Periodic method called once per scheduled loop.                   */
void ShooterSubsystem::Periodic()
{
   /* Log the current measured velocity (in RPMs) of the left and right */
   /* motors.                                                           */
   m_shooterLeftMeasuredPublisher.Set((m_leftFlywheel.GetVelocity().GetValueAsDouble())*60.0);
   m_shooterRightMeasuredPublisher.Set((m_rightFlywheel.GetVelocity().GetValueAsDouble())*60.0);

   /* Log the current "supply" current measurement of the left and right*/
   /* motors.                                                           */
   m_shooterLeftSupplyCurrentPublisher.Set((m_leftFlywheel.GetSupplyCurrent().GetValueAsDouble()));
   m_shooterRightSupplyCurrentPublisher.Set((m_rightFlywheel.GetSupplyCurrent().GetValueAsDouble()));
}

   /* Run the shooter motor at the specified speed in RPM.              */
void ShooterSubsystem::RunShooter(units::revolutions_per_minute_t leftSpeed, units::revolutions_per_minute_t rightSpeed)
{
   /* Create the velocity voltage object to set the velocity PID.       */
   // parameters:
   //         velocity in RPS
   //         acceleration in RPS^2
   //         EnableFOC -> requires Pro License
   //         FeedForward -> Feedforward to apply in volts.
   //         ConfigurationSlot - 0
   //         OverrideBrakeDurNeutral
   //         LimitForwardMotion - use limit switch for forward motion
   //         LimitReverseMotion - use limit switch for reverse motion
   controls::VelocityVoltage m_leftVoltageVelocity{units::angular_velocity::turns_per_second_t{leftSpeed},
                                                   0_tr_per_s_sq,
                                                   false,
                                                   m_leftFlywheelFeedForward.Calculate(units::angular_velocity::turns_per_second_t{leftSpeed}),
                                                   0,
                                                   false,
                                                   false,
                                                   false
                                                  };

   controls::VelocityVoltage m_rightVoltageVelocity{units::angular_velocity::turns_per_second_t{rightSpeed},
                                                    0_tr_per_s_sq,
                                                    false,
                                                    m_rightFlywheelFeedForward.Calculate(units::angular_velocity::turns_per_second_t{rightSpeed}),
                                                    0,
                                                    false,
                                                    false,
                                                    false
                                                   };


   /* Save the specified speeds to the current set point speed values.  */
   m_leftSetpointSpeed  = leftSpeed;
   m_rightSetpointSpeed = rightSpeed;

   /* Log the set point state information.                              */
   m_shooterLeftSetpointPublisher.Set(leftSpeed.value());       // in RPM
   m_shooterRightSetpointPublisher.Set(rightSpeed.value());

   /* Apply the new set point values to the flywheels.                  */
   m_leftFlywheel.SetControl(m_leftVoltageVelocity);
   m_rightFlywheel.SetControl(m_rightVoltageVelocity);
}

   /* Stop the shooter motor.                                           */
void ShooterSubsystem::StopShooter(void)
{
   /* Create the velocity voltage object to set the velocity PID.       */
   // parameters:
   //         velocity in RPS
   //         acceleration in RPS^2
   //         EnableFOC -> requires Pro License
   //         FeedForward -> Feedforward to apply in volts.
   //         ConfigurationSlot - 0
   //         OverrideBrakeDurNeutral
   //         LimitForwardMotion - use limit switch for forward motion
   //         LimitReverseMotion - use limit switch for reverse motion
   controls::VelocityVoltage m_leftVoltageVelocity{units::angular_velocity::turns_per_second_t{0.0},
                                                   0_tr_per_s_sq,
                                                   false,
                                                   m_leftFlywheelFeedForward.Calculate(units::angular_velocity::turns_per_second_t{0.0}),
                                                   0,
                                                   false,
                                                   false,
                                                   false
                                                  };

   controls::VelocityVoltage m_rightVoltageVelocity{units::angular_velocity::turns_per_second_t{0.0},
                                                    0_tr_per_s_sq,
                                                    false,
                                                    m_rightFlywheelFeedForward.Calculate(units::angular_velocity::turns_per_second_t{0.0}),
                                                    0,
                                                    false,
                                                    false,
                                                    false
                                                   };

   /* Save the specified speeds to the current set point spseed values.  */
   m_leftSetpointSpeed  = 0.0_rpm;
   m_rightSetpointSpeed = 0.0_rpm;

   /* Log the set point state information.                              */
   m_shooterLeftSetpointPublisher.Set(0.0); // in RPM
   m_shooterRightSetpointPublisher.Set(0.0);

   /* Apply the new set point values to the flywheels.                  */
   m_leftFlywheel.SetControl(m_leftVoltageVelocity);
   m_rightFlywheel.SetControl(m_rightVoltageVelocity);
}

   /* Set the speed of the left shooter flywheel to running at.         */
void ShooterSubsystem::SetLeftFlywheelSpeed(units::revolutions_per_minute_t leftSpeed)
{
   /* Create the velocity voltage object to set the velocity PID.       */
   // parameters:
   //         velocity in RPS
   //         acceleration in RPS^2
   //         EnableFOC -> requires Pro License
   //         FeedForward -> Feedforward to apply in volts.
   //         ConfigurationSlot - 0
   //         OverrideBrakeDurNeutral
   //         LimitForwardMotion - use limit switch for forward motion
   //         LimitReverseMotion - use limit switch for reverse motion
   controls::VelocityVoltage m_leftVoltageVelocity{units::angular_velocity::turns_per_second_t{leftSpeed},
                                                   0_tr_per_s_sq,
                                                   false,
                                                   m_leftFlywheelFeedForward.Calculate(units::angular_velocity::turns_per_second_t{leftSpeed}),
                                                   0,
                                                   false,
                                                   false,
                                                   false
                                                  };

   /* Save the specified speeds to the current set point speed values.  */
   m_leftSetpointSpeed = leftSpeed;

   /* Log the set point state information.                              */
   m_shooterLeftSetpointPublisher.Set(leftSpeed.value()); // in RPM

   /* Apply the new set point values to the flywheel.                   */
   m_leftFlywheel.SetControl(m_leftVoltageVelocity);
}

   /* Set the speed of the right shooter flywheel to running at.        */
void ShooterSubsystem::SetRightFlywheelSpeed(units::revolutions_per_minute_t rightSpeed)
{
   /* Create the velocity voltage object to set the velocity PID.       */
   // parameters:
   //         velocity in RPS
   //         acceleration in RPS^2
   //         EnableFOC -> requires Pro License
   //         FeedForward -> Feedforward to apply in volts.
   //         ConfigurationSlot - 0
   //         OverrideBrakeDurNeutral
   //         LimitForwardMotion - use limit switch for forward motion
   //         LimitReverseMotion - use limit switch for reverse motion
   controls::VelocityVoltage m_rightVoltageVelocity{units::angular_velocity::turns_per_second_t{rightSpeed},
                                                    0_tr_per_s_sq,
                                                    false,
                                                    m_rightFlywheelFeedForward.Calculate(units::angular_velocity::turns_per_second_t{rightSpeed}),
                                                    0,
                                                    false,
                                                    false,
                                                    false
                                                   };

   /* Save the specified speeds to the current set point speed values.  */
   m_rightSetpointSpeed = rightSpeed;

   /* Log the set point state information.                              */
   m_shooterLeftSetpointPublisher.Set(rightSpeed.value()); // in RPM

   /* Apply the new set point values to the flywheel.                   */
   m_leftFlywheel.SetControl(m_rightVoltageVelocity);
}

   /* Get the current speed of the shooter left fly wheels.             */
units::revolutions_per_minute_t ShooterSubsystem::GetCurrentLeftSpeed(void)
{
   /* Simply get the current RPS, convert to RPMs and return to the     */
   /* caller.                                                           */
   return(units::revolutions_per_minute_t(m_leftFlywheel.GetVelocity().GetValueAsDouble()*60));
}

   /* Get the current speed of the shooter right fly wheels.            */
units::revolutions_per_minute_t ShooterSubsystem::GetCurrentRightSpeed(void)
{
   /* Simply get the current RPS, convert to RPMs and return to the     */
   /* caller.                                                           */
   return(units::revolutions_per_minute_t(m_rightFlywheel.GetVelocity().GetValueAsDouble()*60));
}


  /* Get the set point speed of the left shooter flywheel.              */
units::revolutions_per_minute_t ShooterSubsystem::GetLeftSetpointSpeed(void)
{
   /* Simply return the current setpoint speed for the left flywheel.   */
   return(m_leftSetpointSpeed);
}

   /* Get the set point speed of the right shooter flywheel.             */
units::revolutions_per_minute_t ShooterSubsystem::GetRightSetpointSpeed(void)
{
   /* Simply return the current setpoint speed for the right flywheel.  */
   return(m_rightSetpointSpeed);
}

  /**
   * Generates a command to shoot a note with a trigger.
   *
   *  @param speed  Speed to run the shooter motor (forward/backwards), in RPMs
   *
   *  @return A command pointer that runs the shooter until the command is interrupted.
   *          (Start/End Command Type)
   */
  frc2::CommandPtr ShooterSubsystem::ShootNoteWithTriggerCommand(units::revolutions_per_minute_t leftSpeed, units::revolutions_per_minute_t rightSpeed)
{
   /* Generated a command that runs the shooter when scheduled and stops*/
   /* the shooter when interrupted.                                     */
   frc2::CommandPtr ret_val = frc2::cmd::StartEnd(
                                                  [this, leftSpeed, rightSpeed] { this->RunShooter(leftSpeed, rightSpeed); },
                                                  [this] { this->StopShooter(); },
                                                  {this}
                                                  );

   /* Give the command a readable name.                                 */
   ret_val.get()->SetName("ShootNoteWithTriggerCommand()");

   /* Return the command to the caller.                                 */
   return(ret_val);
}
