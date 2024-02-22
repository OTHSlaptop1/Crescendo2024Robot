// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/VisionSubsystem.h"

#include <photon/PhotonPoseEstimator.h>
#include <photon/targeting/PhotonPipelineResult.h>
#include <photon/PhotonCamera.h>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>

#include <networktables/NetworkTable.h>

#include <units/angle.h>
#include <units/length.h>

#include <utility>

  /* Constructs a Vision Subsystem module and configures the camera and */
  /* estimator for use.                                                 */
VisionSubsystem::VisionSubsystem(const char CameraName[], frc::Transform3d robotToCamera,  frc::AprilTagFieldLayout aprilTagFieldLayout)
   : m_PhotonCamera{CameraName}
{
   /* Initialzie the field length and width values for later use.       */
   m_fieldLength = aprilTagFieldLayout.GetFieldLength();
   m_fieldWidth  = aprilTagFieldLayout.GetFieldWidth();

   /* Next define the position of the camera.  Example of how this is   */
   /* defined below                                                     */
//   frc::Transform3d robotToCamera{0.5_m, 0_m, 0.5_m, Rotation3d{0_rad, 0_rad, 0_rad}};

   /* Create a new Photon Pose Estimator.                               */
   m_photonPoseEstimator = new photon::PhotonPoseEstimator(aprilTagFieldLayout, photon::PoseStrategy::MULTI_TAG_PNP_ON_COPROCESSOR, robotToCamera);

   /* Set the position estimation strategy used in multi-tag mode when  */
   /* only one tag can be seen.                                         */
   m_photonPoseEstimator->SetMultiTagFallbackStrategy(photon::PoseStrategy::LOWEST_AMBIGUITY);

   /* Initialize the last estimated pose to a known invalid value.      */
   m_lastEstimatedRobotPose = std::nullopt;

   // Start publishing vision state information with the "/Vision" key
   m_visionEstimatedRobotPublisher  = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("/Vision/" + std::string(CameraName) + "/EstimatedRobot").Publish();
}

    /* Vision Subsystem Destructor.                                     */
VisionSubsystem::~VisionSubsystem()
{
   /* Check to see if the photon pose estimator object appears to be    */
   /* valid.                                                            */
   if(m_photonPoseEstimator != NULL)
   {
      /* The photon pose estimator object appears to be valid.  Free the*/
      /* memory previous allocated for its creation and reset the       */
      /* pointer to an invalid state.                                   */
      delete m_photonPoseEstimator;
      m_photonPoseEstimator = NULL;
   }
}

   // This method will be called once per scheduler run
void VisionSubsystem::Periodic()
{
   photon::PhotonPipelineResult latestResult;

   /* Get the latest results from the photon camera.                    */
   latestResult = m_PhotonCamera.GetLatestResult();

   /* Now we need to validate the results that we got from the camera.  */
   /* Check to see if the results has targets, it is has multiple       */
   /* targets (size > 1) or if only one target that target has an       */
   /* ambiguity less that our threshold.                                */
   if((latestResult.HasTargets()) && ((latestResult.targets.size() > 1) || (latestResult.targets[0].GetPoseAmbiguity() < 0.185)))
   {
      /* The latest results appear to be at least semi-valid.  Lets     */
      /* update he pose estimator with these results.                   */
      std::optional<photon::EstimatedRobotPose> estimatedPoseResult = m_photonPoseEstimator->Update(latestResult);

      /* Now check to see if the estimator returned a result.           */
      if(estimatedPoseResult.has_value())
      {
         /* Output the current estimated position for logging.       */
         m_visionEstimatedRobotPublisher.Set(estimatedPoseResult.value().estimatedPose.ToPose2d());

         /* The estimator appears to have returned a result.  Now lets  */
         /* validate that the results appear to be at least semi-valid. */
         /* We are checking to make sure the results appear with in the */
         /* bounds of the field.                                        */
         if((estimatedPoseResult.value().estimatedPose.X() > 0.0_m) && (estimatedPoseResult.value().estimatedPose.X()<= m_fieldLength))
         {
            if((estimatedPoseResult.value().estimatedPose.Y() > 0.0_m) && (estimatedPoseResult.value().estimatedPose.Y()<= m_fieldWidth))
            {
               /* The estimated pose result appears to be within the    */
               /* bounds of the field.                                  */

// consider doing a confidence calculation based on distance to the tag..
// some also just straight up reject if distance to the tag is greater then a set distance (like 3 meters).

               /* Save this estimated robot pose to the last results    */
               /* value.                                                */
               m_lastEstimatedRobotPose = estimatedPoseResult;
            }
         }
      }
   }
}

  /* The following function is responsible for consuming the last       */
  /* estimated robot pose then resetting its value internally.          */
std::optional<photon::EstimatedRobotPose> VisionSubsystem::ConsumeLastEstimatedRobotPose(void)
{
   std::optional<photon::EstimatedRobotPose> ret_val;

   /* Get the last result and store it temporarily to the value to be   */
   /* returned.                                                         */
   ret_val = m_lastEstimatedRobotPose;

   /* Reset the last estimated robot pose to an invalid state.          */
   m_lastEstimatedRobotPose = std::nullopt;

   /* Return the last estimated pose to the caller.                     */
   return(ret_val);
}
