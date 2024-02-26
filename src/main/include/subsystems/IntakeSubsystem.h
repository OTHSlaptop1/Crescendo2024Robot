// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <networktables/DoubleTopic.h>
#include <networktables/BooleanTopic.h>

#include <units/angular_velocity.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include <frc2/command/Commands.h>
#include <frc/controller/SimpleMotorFeedforward.h>

#include <rev/CANSparkMax.h>
#include <rev/SparkMaxPIDController.h>
#include <rev/SparkMaxRelativeEncoder.h>

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Run the intake at the specified speed.
   *
   * @param speed  Speed to run the intake motor (forward/backwards), in RPMs
   */
  void RunIntake(units::revolutions_per_minute_t speed);

  /**
   * Stop the intake motor from running.
   */
  void StopIntake(void);

  /**
   * Generates a command to grab a note until a note is detected.
   *
   *  @param speed  Speed to run the intake motor (forward/backwards), in RPMs
   *
   *  @return A command pointer that runs the intake until a note is detected then stops the intake.  If canceled, stops the intake early.
   *          (FunctionalCommand command type)
   *
   */
  frc2::CommandPtr GrabNoteCommand(units::revolutions_per_minute_t speed);

  /**
   * Returns the current note detection input state.
   *
   * @return The current note detection input state.
   */
  bool IsNoteDetected(void);

  /**
   * Get the current speed of the intake.
   *
   * @return The current speed of the intake.
   */
  units::revolutions_per_minute_t GetSpeed(void);

  /**
   * Get the current setpoint speed for the intake.
   *
   * @return The last setpoint speed of the intake.
   */
  units::revolutions_per_minute_t GetSetpointSpeed(void);

// void FeedShooter(speed)   // feeds a note into the shooter  (command)

 private:

   /* Spark Max using CAN to control the intake motors. */
   rev::CANSparkMax m_intakeSparkMax;

   /* Spark Max encoder for the intake motor.           */
   rev::SparkRelativeEncoder m_intakeEncoder = m_intakeSparkMax.GetEncoder();

   /* Spark Max PID controller for the index motor.     */
   rev::SparkPIDController m_intakePIDController = m_intakeSparkMax.GetPIDController();

   /* Helper that computes the feed foreward voltages for a simple      */
   /* permanent-magnet DC motor.                                        */
   frc::SimpleMotorFeedforward<units::turns> m_intakeFeedFoward;

   /* Digitial Input used to detect absents/presents of a note in the intake. */
   frc::DigitalInput m_noteDetect;

   /* The following variable holds the current setpoint speed.          */
   units::revolutions_per_minute_t m_currentSetpointSpeed;

   // Publisher variables for the intake.
   nt::DoublePublisher  m_intakeMeasuredVelocityPublisher;
   nt::DoublePublisher  m_intakeMeasuredCurrentPublisher;
   nt::DoublePublisher  m_intakeSetpointPublisher;
   nt::BooleanPublisher m_intakeNoteDetectPublisher;
};
