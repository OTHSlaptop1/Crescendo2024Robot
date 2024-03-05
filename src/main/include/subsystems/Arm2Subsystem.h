// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include "Constants.h"

#include <networktables/DoubleTopic.h>

#include <rev/CANSparkMax.h>
#include <rev/SparkMaxAbsoluteEncoder.h>

#include <frc2/command/Commands.h>
#include <frc/controller/ArmFeedforward.h>
#include <units/angle.h>

using namespace ArmConstants;

typedef enum
{
   asInitializing,
   asMovingUp,
   asMovingDown,
   asStopped,
   asDisabled
} ArmState_t;

class Arm2Subsystem : public frc2::SubsystemBase {
 public:
  Arm2Subsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Get the current angle of the arm.
   *
   * @return The current angle of the Arm in degrees. The range is mapped
   *         to -180 to 180 degrees.
   *
   */
  units::degree_t GetArmAngle(void);

  /**
   * Set the arm position.
   *
   *  @param setpoint  Set point position to move the arm to.
   *
   */
  void SetArmPosition(units::degree_t setpoint);

  /**
   * Disable the Arms motors.
   */
  void DisableArm(void);

  /**
   * Generates a command to set the arm position.
   *
   *  @param setpoint  Set point position to move the arm to.
   *
   *  @return A command pointer that runs the arm to the specified position.
   */
  frc2::CommandPtr SetArmPositionCommand(units::degree_t setpoint);

  /**
   * Generates a command to move the arm up.
   *
   *  @return A command pointer that runs the arm up.
   */
  frc2::CommandPtr ArmUpCommand(void);

  /**
   * Generates a command to move the arm down.
   *
   *  @return A command pointer that runs the arm down.
   */
  frc2::CommandPtr ArmDownCommand(void);

 private:
    /* The following variable hold the controller objects for each of the*/
    /* spark max modules.                                                */
    rev::CANSparkMax m_leaderSparkMax;
    rev::CANSparkMax m_followerSparkMax;

    /* The following variable holds the arm absolute encoder.            */
    rev::SparkAbsoluteEncoder m_armAbsoluteEncoder = m_leaderSparkMax.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle);

    /* The following variable holds the arm leader PID controller.       */
    rev::SparkPIDController m_leaderPIDController = m_leaderSparkMax.GetPIDController();

    // The Arm Feed Forward Controller.
    frc::ArmFeedforward m_feedForward;

    // The following variable holds the current Arm State;
    ArmState_t m_ArmState;

    // The following variable holds the timer used to force a delay
    // before reading the angle sensor and initialzing our initial state.
    units::second_t m_initializationTimer;

    /* Trapezoidal Profiles for Up and Down Arm Movements.              */
    frc::TrapezoidProfile<units::radians> m_upProfile{{kArmUpMaxVelocity, kArmUpMaxAcceleration}};
    frc::TrapezoidProfile<units::radians> m_downProfile{{kArmDownMaxVelocity, kArmDownMaxAcceleration}};

    /* The following variable hold the goal and current state of the    */
    /* motion.                                                          */
    frc::TrapezoidProfile<units::radians>::State m_goal;
    frc::TrapezoidProfile<units::radians>::State m_current;

    // Publisher variables for the arm.
    nt::DoublePublisher m_armMeasuredPositionPublisher;
    nt::DoublePublisher m_armSetpointPositionPublisher;
    nt::DoublePublisher m_armSetpointVelocityPublisher;
    nt::DoublePublisher m_armOutputFeedForwardPublisher;
    nt::DoublePublisher m_armCurrentPublisher;
    nt::DoublePublisher m_armAppliedOutputPublisher;
};
