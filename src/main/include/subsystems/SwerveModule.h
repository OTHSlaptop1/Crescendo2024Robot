// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <rev/CANSparkMax.h>
#include <rev/SparkMaxAbsoluteEncoder.h>
#include <rev/SparkMaxPIDController.h>
#include <rev/SparkMaxRelativeEncoder.h>

class SwerveModule {
 public:
  /**
   * Constructs a SwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the
   * MK4i Swerve Module built with NEOs, SPARKS MAX, and a CTRE SRX Mag Duty Cycle
   * Encoder connected to the SPARK MAX modules.
   */
  SwerveModule(int driveCANId, int turningCANId);

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  frc::SwerveModuleState GetState() const;

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  frc::SwerveModulePosition GetPosition() const;

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   *
   * @return the optimized state that was used by the module.
   */
  frc::SwerveModuleState SetDesiredState(const frc::SwerveModuleState& desiredState);

  /**
   * Zeroes all the SwerveModule encoders.
   */
  void ResetEncoders();

 private:
  rev::CANSparkMax m_drivingSparkMax;
  rev::CANSparkMax m_turningSparkMax;

  rev::SparkRelativeEncoder m_drivingEncoder         = m_drivingSparkMax.GetEncoder();
  rev::SparkAbsoluteEncoder m_turningAbsoluteEncoder = m_turningSparkMax.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle);

  rev::SparkPIDController m_drivingPIDController = m_drivingSparkMax.GetPIDController();
  rev::SparkPIDController m_turningPIDController = m_turningSparkMax.GetPIDController();

  frc::SwerveModuleState m_desiredState{units::meters_per_second_t{0.0}, frc::Rotation2d()};
};
