// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>
#include <networktables/DoubleArrayTopic.h>

#include <ctre/phoenix6/Pigeon2.hpp>

#include <frc/controller/PIDController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc2/command/SubsystemBase.h>

#include "Constants.h"
#include "SwerveModule.h"

class DriveSubsystem : public frc2::SubsystemBase {
 public:

    /* Drive Subsystem Constructor.                                     */
    DriveSubsystem(bool fieldRelativeState, bool limitSlewRate);

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
   * and the linear speeds have no effect on the angular speed.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward/backwards).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   *
   */
  void Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed, units::radians_per_second_t rot);

  /**
   * Drives the robot using the specified chassis speed in a robot relative way.
   *
   * @param RobotRelativeSpeeds    ChassisSpeeds for the robot.
   */
  void DriveRobotRelative(frc::ChassisSpeeds RobotRelativeSpeeds);

  /**
   * Drives the robot at given x and y speed while using PID to control the rotation.
   * Rotation will be locked to the speakers location.  Speeds range from [-1, 1] and
   * the linear speeds have no effect on the angular speed.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward/backwards).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   *
   * ** NOTE ** This function uses field relative controls while operating.
   */
  void DriveWithSpeakerAim(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed);

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  void SetX();

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  void ResetEncoders();

  /**
   * Sets the swerve module states.
   */
  void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

  /**
   *  Get the swerve module states.
   */
   wpi::array<frc::SwerveModuleState, 4> GetModuleStates();

   /*
    *  Get a single swerve modules state.
    */
   frc::SwerveModuleState GetFrontLeftModuleState();
   frc::SwerveModuleState GetFrontRightModuleState();
   frc::SwerveModuleState GetRearLeftModuleState();
   frc::SwerveModuleState GetRearRightModuleState();

   /**
    *  Get the swerve module positions.
    */
   wpi::array<frc::SwerveModulePosition, 4> GetModulePositions();

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  units::degree_t GetHeading() const;

  /**
   * Returns the heading of the robot as a rotation 2d object..
   *
   * @return the robot's heading as a Rotation2d
   */
  frc::Rotation2d GetRotation2dHeading() const;

  /**
   * Zeroes the heading of the robot.
   */
  void ZeroHeading();

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  double GetTurnRate();

   /**
    * Gets the current robot relative chassis speeds
    * @return current chassis speeds
    */
   frc::ChassisSpeeds GetRobotRelativeChassisSpeeds();

   /**
    * Set the current maximum speed
    * @param new maximum speed in meters per second
    */
   void SetMaxSpeed(units::meters_per_second_t maxSpeed);

   /**
    * Get the current maximum speed
    * @return current maximum speed
    */
   units::meters_per_second_t GetMaxSpeed(void);

   /**
    * Set the current maximum angular speed
    * @param new maximum angular speed in radians per second
    */
   void SetMaxAngularSpeed(units::radians_per_second_t maxAngularSpeed);

   /**
    * Get the current maximum angular speed
    * @return current maximum angular speed
    */
   units::radians_per_second_t GetMaxAngularSpeed(void);

   /**
    * Set the field relative drive state.
    * @param  new field relative drive state.
    */
   void SetFieldRelativeState(bool fieldRelativeState);

   /**
    * Get the field relative drive state.
    * @return current field relative drive state.
    */
   bool GetFieldRelativeState(void);

/**
    * Set the limit slew rate state.
    * @param  new limit slew rate state.
    */
   void SetLimitSlewRateState(bool limitSlewRate);

   /**
    * Get the limit slew rate state.
    * @return current limit slew rate state.
    */
   bool GetLimitSlewRateState(void);

  frc::SwerveDriveKinematics<4> kDriveKinematics{
      frc::Translation2d{ DriveConstants::kWheelBase/2,  DriveConstants::kTrackWidth/2},   // front left
      frc::Translation2d{ DriveConstants::kWheelBase/2, -DriveConstants::kTrackWidth/2},   // front right
      frc::Translation2d{-DriveConstants::kWheelBase/2,  DriveConstants::kTrackWidth/2},   // rear left
      frc::Translation2d{-DriveConstants::kWheelBase/2, -DriveConstants::kTrackWidth/2}};  // rear right

 private:
  // Components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods.

  SwerveModule m_frontLeft;
  SwerveModule m_frontRight;
  SwerveModule m_rearLeft;
  SwerveModule m_rearRight;

  // The gyro sensor - Pigeon 2.0
  ctre::phoenix6::hardware::Pigeon2 pidgey_gyro{DriveConstants::kPigeonGyroCanId, DriveConstants::kPigeonBusName};

  // The following variable holds the PID controller used to automatically aim
  // at the speaker.
  frc::PIDController m_aimController;

  // Slew rate filter variables for controlling lateral acceleration
  double m_currentRotation       = 0.0;
  double m_currentTranslationDir = 0.0;
  double m_currentTranslationMag = 0.0;

  frc::SlewRateLimiter<units::scalar> m_magLimiter{DriveConstants::kMagnitudeSlewRate/1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{DriveConstants::kRotationalSlewRate/1_s};
  double m_prevTime = wpi::Now() * 1e-6; // in microseconds

  // Maximum Speed and Angular Speed for the drive subsystem.
  units::meters_per_second_t   m_maxSpeed;
  units::radians_per_second_t  m_maxAngularSpeed;

  // Current Field Relative Driving Setting.
  bool m_fieldRelative;

  // Current Slew Rate Limit Setting.
  bool m_limitSlewRate;

  // Publisher variables for a swerve module states.
  nt::StructArrayPublisher<frc::SwerveModuleState> m_swerveMeasuredPublisher;
  nt::StructArrayPublisher<frc::SwerveModuleState> m_swerveSetpointsPublisher;
  nt::StructArrayPublisher<frc::SwerveModuleState> m_swerveSetPointsOptimizedPublisher;
  nt::DoubleArrayPublisher                         m_swerveDriveCurrentPublisher;
  nt::DoubleArrayPublisher                         m_swerveTurnCurrentPublisher;
};
