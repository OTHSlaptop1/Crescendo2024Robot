// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <frc/controller/SimpleMotorFeedforward.h>
#include <networktables/DoubleTopic.h>
#include <units/angular_velocity.h>

#include "Constants.h"
#include "ctre/phoenix6/TalonFX.hpp"

using namespace ShooterConstants;

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Run the shooter flywheel motors at the specified speed.
   *
   * @param leftSpeed  Speed to run the left shooter flywheel motor ((+)forward/(-)backwards), in RPMs
   *
   * @param rightSpeed  Speed to run the left shooter flywheel motor ((+)forward/(-)backwards), in RPMs
   */
  void RunShooter(units::revolutions_per_minute_t leftSpeed, units::revolutions_per_minute_t rightSpeed);

  /**
   * Stop the shooter flywheel motors from running.
   */
  void StopShooter(void);

  /*
  * Set the speed of the left shooter flywheel to running at.
  *
   * @param leftSpeed  Speed to run the left shooter flywheel motor ((+)forward/(-)backwards), in RPMs
  */
  void SetLeftFlywheelSpeed(units::revolutions_per_minute_t leftSpeed);

  /*
  * Set the speed of the right shooter flywheel to running at.
  *
   * @param rightSpeed  Speed to run the right shooter flywheel motor ((+)forward/(-)backwards), in RPMs
  */
  void SetRightFlywheelSpeed(units::revolutions_per_minute_t rightSpeed);

  /*
  * Get the current speed of the left shooter flywheel is running at.
  *
  *  @return Get the current speed of the left fly wheel.
  */
  units::revolutions_per_minute_t GetCurrentLeftSpeed(void);

  /*
  * Get the current speed of the right shooter flywheel is running at.
  *
  *  @return Get the current speed of the right fly wheel.
  */
  units::revolutions_per_minute_t GetCurrentRightSpeed(void);

  /*
  * Get the set point speed of the left shooter flywheel.
  *
  *  @return Get the current set point speed of the left fly wheel.
  */
  units::revolutions_per_minute_t GetLeftSetpointSpeed(void);

  /*
  * Get the set point speed of the right shooter flywheel.
  *
  *  @return Get the current set point speed of the right fly wheel.
  */
  units::revolutions_per_minute_t GetRightSetpointSpeed(void);

  /**
   * Generates a command to shoot a note with a trigger.
   *
   *  @param leftSpeed  Speed to run the left shooter motor ((+)forward/(-)backwards), in RPMs
   *
   *  @param rightSpeed  Speed to run the right shooter motor ((+)forward/(-)backwards), in RPMs
   *
   *  @return A command pointer that runs the shooter until the command is interrupted.
   *          (Start/End Command Type)
   *
   */
  frc2::CommandPtr ShootNoteWithTriggerCommand(units::revolutions_per_minute_t leftSpeed, units::revolutions_per_minute_t rightSpeed);

 private:

    /* Initialize the Falcons for use as flywheels on the RoboRio's CAN */
    /* bus.  Names of fly wheels are robot releative.  The left Flywheel*/
    /* is the on the left of the robot facing forward.                  */
    ctre::phoenix6::hardware::TalonFX m_leftFlywheel{kShooterLeftMotorCanId, kShooterBusName};
    ctre::phoenix6::hardware::TalonFX m_rightFlywheel{kShooterRightMotorCanId, kShooterBusName};

    /* Helper that computes the feed foreward voltages for a simple     */
    /* permanent-magnet DC motor.                                       */
    frc::SimpleMotorFeedforward<units::turns> m_leftFlywheelFeedForward;
    frc::SimpleMotorFeedforward<units::turns> m_rightFlywheelFeedForward;

    /* The following variable holds the current setpoint speed for each */
    /* of the flywheels.                                                */
    units::revolutions_per_minute_t m_leftSetpointSpeed;
    units::revolutions_per_minute_t m_rightSetpointSpeed;

    // Publisher variables for the shooter.
    nt::DoublePublisher m_shooterLeftMeasuredPublisher;
    nt::DoublePublisher m_shooterRightMeasuredPublisher;
    nt::DoublePublisher m_shooterLeftSetpointPublisher;
    nt::DoublePublisher m_shooterRightSetpointPublisher;
    nt::DoublePublisher m_shooterLeftSupplyCurrentPublisher;
    nt::DoublePublisher m_shooterRightSupplyCurrentPublisher;
};
