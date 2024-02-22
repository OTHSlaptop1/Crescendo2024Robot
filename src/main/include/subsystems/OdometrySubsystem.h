// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <networktables/StructTopic.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/geometry/Transform2d.h>
#include <frc/smartdashboard/Field2d.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/VisionSubsystem.h"

class OdometrySubsystem : public frc2::SubsystemBase {
 public:

  /* Odometry Subsystem Constructor.                                    */
  OdometrySubsystem(DriveSubsystem *driveSubsystemPtr, VisionSubsystem *visionSubsystemPtr, frc::Field2d &gameField);

  /* Odometry Subsystem Destructor.                                     */
  ~OdometrySubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  frc::Pose2d GetPose();

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void ResetOdometry(frc::Pose2d pose);

  /**
   * Returns the field length.
   *
   * @return  The length of the field.
   */
  units::meter_t GetFieldLength(void);

  /**
   * Returns the field width.
   *
   * @return  The width of the field.
   */
  units::meter_t GetFieldWidth(void);

  /**
   * Returns the april tag field layout.
   *
   * @return  The april tag field layout.
   */
  frc::AprilTagFieldLayout GetAprilTagFieldLayout(void);

  /**
   * Get a pose with an offset from an April Tag predefined location.
   *
   * @param ID The ID of the april tag to get the pose from.
   *
   * @param poseOffset The offset from the april tag
   *
   */
  frc::Pose2d GetPoseFromAprilTagId(int ID, frc::Transform2d poseOffset);

  /**
   * Creates a command using path planner to drive a path to the specified position
   *
   *  @param endPose                 The end position for the path.
   *  @param goalEndVelocity         The goal end velocity of the robot when reaching the target pose
   *  @param rotationDelayDistance   The distance the robot should move from the start position before attempting to rotate to the final rotation
   *
   *  @return A command pointer that of a command that will run the path to the position.
   */
  frc2::CommandPtr PathToPoseCommand(frc::Pose2d endPose, units::meters_per_second_t goalEndVelocity, units::meter_t rotationDelayDistance);

 private:

  // Odometry class for tracking robot pose 4 defines the number of swerve modules
  frc::SwerveDriveOdometry<4> *m_odometryPtr;

  // Pose Estimator class for tracking robot pose 4 defines the number of swerve modules
  frc::SwerveDrivePoseEstimator<4> *m_poseEstimatorPtr;

  /* The following variable holds a pointer to the drive subsystem.     */
  DriveSubsystem  *m_driveSubsystemPtr;

  /* The following variable holds a pointer to the vision subsystem.    */
  /* ** NOTE ** If more cameras are added, add more vision subsystem    */
  /*            members here.                                           */
  VisionSubsystem *m_visionSubsystemPtr;

  /* The following variabld holds a reference to the field object in    */
  /* which to add pose and tragectory information to for display on a   */
  /* dashboard.                                                         */
  frc::Field2d &m_field;

  /* Load the april tag field layout for this years field.             */
  frc::AprilTagFieldLayout aprilTagFieldLayout = frc::LoadAprilTagLayoutField(frc::AprilTagField::k2024Crescendo);

  /* Publisher for the current robot position.                          */
  nt::StructPublisher<frc::Pose2d>      odometryRobotPublisher;
  nt::StructPublisher<frc::Pose3d>      odometryEstimatedRobotPublisher;
  nt::StructArrayPublisher<frc::Pose2d> odometryTrajectoryPublisher;
  nt::StructPublisher<frc::Pose2d>      odometryTrajectorySetpointPublisher;
};
