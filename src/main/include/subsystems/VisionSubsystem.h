// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <frc/geometry/Transform3d.h>

#include <photon/PhotonPoseEstimator.h>
#include <photon/PhotonCamera.h>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

#include <networktables/StructTopic.h>

#include <utility>
#include <optional>

class VisionSubsystem : public frc2::SubsystemBase {
 public:

  /* Constructs a Vision Subsystem module and configures the camera and
   * estimator for use.
   *
   * @param CameraName - C string name of the camera for this subsystem.
   *
   * @param robotToCamera - A transform 3d for the position of the camera on the robot
  */
  VisionSubsystem(const char CameraName[], frc::Transform3d robotToCamera, frc::AprilTagFieldLayout aprilTagFieldLayout);

  /* Vision Subsystem Destructor.                                       */
  ~VisionSubsystem();


  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /* The following function is responsible for consuming the last
   * estimated robot pose then resetting its value internally.
   *
   *  @return EstimatedRobotPose -  the latest robot pose result or
   *                                if there isn't currently a valid
   *                                result.
  */
  std::optional<photon::EstimatedRobotPose> ConsumeLastEstimatedRobotPose(void);

 private:

  /* The photon camera associated with this vision subsystem object.    */
  photon::PhotonCamera m_PhotonCamera;

  /* The photon pose estimator associated with this vision subsystem    */
  /* object.                                                            */
  photon::PhotonPoseEstimator *m_photonPoseEstimator;

  /* The following variables hold the field length and width as returned*/
  /* from the april tag field layout object.                            */
  units::meter_t m_fieldLength;
  units::meter_t m_fieldWidth;

  /* The following variable holds the last estimated robot pose.        */
  std::optional<photon::EstimatedRobotPose> m_lastEstimatedRobotPose;

  /* Network Table Publisher variables for the vision subsystem.        */
  nt::StructPublisher<frc::Pose2d>  m_visionEstimatedRobotPublisher;
};
