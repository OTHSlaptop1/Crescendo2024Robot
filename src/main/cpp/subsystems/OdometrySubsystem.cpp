// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/OdometrySubsystem.h"

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/util/HolonomicPathFollowerConfig.h>
#include <pathplanner/lib/util/PIDConstants.h>
#include <pathplanner/lib/util/ReplanningConfig.h>
#include <pathplanner/lib/util/PathPlannerLogging.h>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/DriverStation.h>
#include <units/angle.h>
#include <units/length.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/VisionSubsystem.h"

using namespace OdometryConstants;

    /* Odometry Subsystem Constructor.                                  */
OdometrySubsystem::OdometrySubsystem(DriveSubsystem *driveSubsystemPtr, VisionSubsystem *visionSubsystemPtr, frc::Field2d &gameField)
   : m_field{gameField}
{
   /* First check to make sure that the specified parameters appear to  */
   /* be at least semi-valid.                                           */
   if(driveSubsystemPtr != NULL)
   {
      /* The specified parameters appear to be at least semi-valid.     */

      /* Save the specified sub systems to member variables for later   */
      /* use.                                                           */
      m_driveSubsystemPtr  = driveSubsystemPtr;
      m_visionSubsystemPtr = visionSubsystemPtr;

      /* Initialize these pointers to a known invalid state.            */
      m_odometryPtr      = NULL;
      m_poseEstimatorPtr = NULL;

      /* Now check to see if a vision subsystem was specified.          */
      if(m_visionSubsystemPtr == NULL)
      {
         /* Currently no vision subsystem.  Fall back to simple swerve  */
         /* drive odometry.                                             */

         /* Next attempt to create the odometry class object.           */
         m_odometryPtr = new frc::SwerveDriveOdometry<4>{m_driveSubsystemPtr->DriveSubsystem::kDriveKinematics,   // The swerve drive kinematics for your drivetrain.
                                                         m_driveSubsystemPtr->GetRotation2dHeading(),             // The angle reported by the gyroscope.
                                                         m_driveSubsystemPtr->GetModulePositions(),               // The initial wheel positions reported by swerve modules.
                                                         frc::Pose2d{}};                                          // The starting position of the robot on the field.
      }
      else
      {
         /* A vision subsystem was specified.  So we can use it for     */
         /* position estimation in our odometry.                        */
         m_poseEstimatorPtr = new frc::SwerveDrivePoseEstimator<4>(m_driveSubsystemPtr->DriveSubsystem::kDriveKinematics,   // The swerve drive kinematics for your drivetrain.
                                                                   m_driveSubsystemPtr->GetRotation2dHeading(),             // The angle reported by the gyroscope.
                                                                   m_driveSubsystemPtr->GetModulePositions(),               // The initial wheel positions reported by swerve modules.
                                                                   frc::Pose2d{},                                            // The starting position of the robot on the field.
                                                                   { kXStateStdDev.value(), kYStateStdDev.value(), units::radian_t{kHeadingStateStdDev}.value() },
                                                                   { kXVisionStdDev.value(), kYVisionStdDev.value(), units::radian_t{kHeadingVisionStdDev}.value() } );
      }

      // Start publishing an array of module states with the "/Odometry" key
      odometryRobotPublisher              = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("/Odometry/Robot").Publish();
      odometryEstimatedRobotPublisher     = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose3d>("/Odometry/EstimatedRobot").Publish();
      odometryTrajectoryPublisher         = nt::NetworkTableInstance::GetDefault().GetStructArrayTopic<frc::Pose2d>("/Odometry/Trajectory").Publish();
      odometryTrajectorySetpointPublisher = nt::NetworkTableInstance::GetDefault().GetStructTopic<frc::Pose2d>("Odometry/TrajectorySetpoint").Publish();

      /* Configure the Path Planner Auto builder with the parameters    */
      /* related to our drive subsystem.                                */
      pathplanner::AutoBuilder::configureHolonomic(
                        [this](){ return this->GetPose(); },                                                                             // a function that returns the robot's current pose
                        [this](frc::Pose2d pose){ this->ResetOdometry(pose); },                                                          // a function used for resetting the robot's pose (will be called if your auto has a starting pose)
                        [this](){ return m_driveSubsystemPtr->GetRobotRelativeChassisSpeeds(); },                                        // a function that returns the robot's current robot relative chassis speeds
                        [this](frc::ChassisSpeeds robotRelativeSpeeds){ m_driveSubsystemPtr->SetChassisSpeed(robotRelativeSpeeds); },    // a function for setting the robot's robot-relative chassis speeds
                        pathplanner::HolonomicPathFollowerConfig(                                                                        // HolonomicPathFollowerConfig for configuring the path following commands
                           pathplanner::PIDConstants(kHolonomicPathFollowerConfigTranslationP,
                                                     kHolonomicPathFollowerConfigTranslationI,
                                                     kHolonomicPathFollowerConfigTranslationD),                                          // Translation PID constants
                           pathplanner::PIDConstants(kHolonomicPathFollowerConfigRotationP,
                                                     kHolonomicPathFollowerConfigRotationI,
                                                     kHolonomicPathFollowerConfigRotationD),                                             // Rotation PID constants
                           kHolonomicPathFollowerConfigMaxSpeed,                                                                         // Max module speed, in m/s
                           kHolonomicPathFollowerConfigDriveBaseRadius,                                                                  // Drive base radius in meters. Distance from robot center to furthest module.
                           pathplanner::ReplanningConfig()                                                                               // Default path replanning config. See the API for the options here
                        ),
                        []() {
                                 // Boolean supplier that controls when the path will be mirrored for the red alliance
                                 // This will flip the path being followed to the red side of the field.
                                 // This will maintain a global blue alliance origin.
                                 auto alliance = frc::DriverStation::GetAlliance();
                                 if (alliance) {
                                     return alliance.value() == frc::DriverStation::Alliance::kRed;
                                 }
                                 return false;
                             },
                        m_driveSubsystemPtr                                                                                              // a pointer to the subsystem for the robot's drive
                        );

       /* Logging callback for the active path, this is sent as a vector*/
       /* of poses and logged to the network table entry                */
       /* "/Odometry/Trajectory".                                       */
       pathplanner::PathPlannerLogging::setLogActivePathCallback([this](std::vector<frc::Pose2d> poses){

          /* Simply log the vector of poses passed to this callback.    */
          odometryTrajectoryPublisher.Set(poses);

          /* Simply log the vector of poses passed to this callback as  */
          /* the trajectory.                                            */
          m_field.GetObject("Trajectory")->SetPoses(poses);
       });

       /* Logging callback for target robot pose, this is logged tothe  */
       /* network table entry "/Odometry/TrajectorySetpoint".           */
       pathplanner::PathPlannerLogging::setLogTargetPoseCallback([this](frc::Pose2d pose){

          /* Simply log the pose associated with this trajectory        */
          /* setpoint.                                                  */
          odometryTrajectorySetpointPublisher.Set(pose);

          /* Add the target position to the field object.               */
          m_field.GetObject("Target Pose")->SetPose(pose);
       });
   }
   else
   {
      throw std::invalid_argument("Invalid argument: none NULL pointers required.");
   }
}

    /* Odometry Subsystem Destructor.                                   */
OdometrySubsystem::~OdometrySubsystem()
{
   /* Check to see if the odometry object appears to be valid.          */
   if(m_odometryPtr != NULL)
   {
      /* The odometry object appears to be valid.  Free the memory      */
      /* previous allocated for its creation and reset the pointer to an*/
      /* invalid state.                                                 */
      delete m_odometryPtr;
      m_odometryPtr = NULL;
   }

   /* Check to see if the pose estimator object appears to be valid.    */
   if(m_poseEstimatorPtr != NULL)
   {
      /* The pose estimator object appears to be valid.  Free the memory*/
      /* previous allocated for its creation and reset the pointer to an*/
      /* invalid state.                                                 */
      delete m_poseEstimatorPtr;
      m_poseEstimatorPtr = NULL;
   }
}

// This method will be called once per scheduler run
void OdometrySubsystem::Periodic()
{
   frc::Pose2d robotPose2d;

   /* First make sure that the required member variables appear to be at*/
   /* least semi-valid.                                                 */
   if((m_driveSubsystemPtr != NULL) && (((m_visionSubsystemPtr == NULL) && (m_odometryPtr != NULL)) || ((m_visionSubsystemPtr != NULL) && (m_poseEstimatorPtr != NULL))))
   {
      /* Check to see if a vision subsystem is currently available.     */
      if(m_visionSubsystemPtr == NULL)
      {
         /* The vision subsystem is not currently available.  Fallback  */
         /* to using simply odometry.                                   */

         /* Update the odometry with the currently known field position.*/
         robotPose2d = m_odometryPtr->Update(m_driveSubsystemPtr->GetRotation2dHeading(), m_driveSubsystemPtr->GetModulePositions());
      }
      else
      {
         /* A vision subsystem is available.                            */

         /* Update the pose estimator with the current known field      */
         /* position.                                                   */
         robotPose2d = m_poseEstimatorPtr->Update(m_driveSubsystemPtr->GetRotation2dHeading(), m_driveSubsystemPtr->GetModulePositions());

         /* Get the estimated robot position from the vision subsystem. */
         std::optional<photon::EstimatedRobotPose> estimatedPoseResult = m_visionSubsystemPtr->ConsumeLastEstimatedRobotPose();

         /* Check to see if a new estimated pose appears to be valid.   */
         if(estimatedPoseResult.has_value())
         {
            /* The estimated position appears to be valid.              */

            /* Output the current estimated position for logging.       */
            odometryEstimatedRobotPublisher.Set(estimatedPoseResult.value().estimatedPose);

            /* Add vision measurements to the pose estimators.          */
            m_poseEstimatorPtr->AddVisionMeasurement(estimatedPoseResult.value().estimatedPose.ToPose2d(), estimatedPoseResult.value().timestamp);

#if 0
//xxx can include dynamic vision measurement standard deviations?
//xxx confidence calculator in 2023-Robot-Joe-Test and photon vision sample...
            m_poseEstimatorPtr->AddVisionMeasurement(const Pose2d visionRobotPose, units::second_t timestamp, const wpi::array< double, 3 > & visionMeasurementStdDevs);
#endif
         }
      }

      /* Output the current position for logging.                       */
      odometryRobotPublisher.Set(robotPose2d);

      /* Add the robot position to the field object.                    */
      m_field.SetRobotPose(robotPose2d);
   }
}

   /* This function gets the position of the robot on the field.        */
frc::Pose2d OdometrySubsystem::GetPose()
{
   frc::Pose2d ret_val;

   /* Now check to see if a vision subsystem was specified.             */
   if(m_visionSubsystemPtr == NULL)
   {
      /* Currently no vision subsystem.  Fall back to simple swerve     */
      /* drive odometry.                                                */

      /* Simply get the current position on the field.                  */
      ret_val = m_odometryPtr->GetPose();
   }
   else
   {
      /* A vision subsystem was specified.  So we can use it for        */
      /* position estimation in our odometry.                           */
      ret_val = m_poseEstimatorPtr->GetEstimatedPosition();
   }

   return(ret_val);
}

   /* This function resets the position of the robot on the field.      */
void OdometrySubsystem::ResetOdometry(frc::Pose2d pose)
{
   /* First check to make sure that the required member variables appear*/
   /* to be at least semi-valid.                                        */
   if((m_odometryPtr != NULL) && (m_driveSubsystemPtr != NULL))
   {
      m_odometryPtr->ResetPosition(m_driveSubsystemPtr->GetRotation2dHeading(), m_driveSubsystemPtr->GetModulePositions(), pose);
   }
}

  /* This function gets the length of the field.                        */
units::meter_t OdometrySubsystem::GetFieldLength(void)
{
   return(aprilTagFieldLayout.GetFieldLength());
}

  /* This function gets the width of the field.                         */
units::meter_t OdometrySubsystem::GetFieldWidth(void)
{
   return(aprilTagFieldLayout.GetFieldWidth());
}

   /* The function gets the april tag field layout.                     */
frc::AprilTagFieldLayout OdometrySubsystem::GetAprilTagFieldLayout(void)
{
   return(aprilTagFieldLayout);
}
  /* Get a pose with an offset from an April Tag predefined location.   */
frc::Pose2d OdometrySubsystem::GetPoseFromAprilTagId(int ID, frc::Transform2d poseOffset)
{
   frc::Pose2d ret_val;

   /* Get the april tag position based on the tag ID.                   */
   std::optional<frc::Pose3d> tagPose = aprilTagFieldLayout.GetTagPose(ID);

   /* Now check to make sure that the tag position was successfully     */
   /* returned.                                                         */
   if(tagPose.has_value())
   {
      /* The tag position was successfully returned.  Convert the       */
      /* position to a 2d position.                                     */
      ret_val = tagPose.value().ToPose2d();
   }
   else
   {
      /* The tag position was not successuflly returned.  In this case  */
      /* create an empty position.                                      */
      ret_val = frc::Pose2d{};
   }

   /* Return the position transformed by the specifed pose offset.      */
   return(ret_val.TransformBy(poseOffset));
}

   /* Creates a command using path planner to drive a path to the       */
   /* specified position                                                */
frc2::CommandPtr OdometrySubsystem::PathToPoseCommand(frc::Pose2d endPose, units::meters_per_second_t goalEndVelocity, units::meter_t rotationDelayDistance)
{
   /* Create the constrains to use while finding the path to the        */
   /* specified position.                                               */
   pathplanner::PathConstraints constraints = pathplanner::PathConstraints(
                                                                           kHolonomicPathFollowerConfigMaxSpeed, // Max linear velocity (M/S)
                                                                           3.5_mps_sq,                           // Max linear acceleration (M/S^2)
                                                                           360_deg_per_s,                        // Max angular velocity (Deg/S)
                                                                           540_deg_per_s_sq                      // Max angular acceleration (Deg/S^2)
                                                                           );

   /* Use AutoBuilder to build the pathfinding command to the specified */
   /* pose.                                                             */
   return(pathplanner::AutoBuilder::pathfindToPose(
                                                   endPose,
                                                   constraints,
                                                   goalEndVelocity,       // Goal end velocity in meters/sec
                                                   rotationDelayDistance  // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
                                                   ));
}

   /* Creates a command using path planner to drive a path to the       */
   /* specified position using the path flipping supplier.              */
frc2::CommandPtr OdometrySubsystem::PathToPoseFlippedCommand(frc::Pose2d endPose, units::meters_per_second_t goalEndVelocity, units::meter_t rotationDelayDistance)
{
   /* Create the constrains to use while finding the path to the        */
   /* specified position.                                               */
   pathplanner::PathConstraints constraints = pathplanner::PathConstraints(
                                                                           kHolonomicPathFollowerConfigMaxSpeed, // Max linear velocity (M/S)
                                                                           3.5_mps_sq,                           // Max linear acceleration (M/S^2)
                                                                           360_deg_per_s,                        // Max angular velocity (Deg/S)
                                                                           540_deg_per_s_sq                      // Max angular acceleration (Deg/S^2)
                                                                           );

   /* Use AutoBuilder to build the pathfinding command to the specified */
   /* pose.                                                             */
   return(pathplanner::AutoBuilder::pathfindToPose(
                                                   endPose,
                                                   constraints,
                                                   goalEndVelocity,       // Goal end velocity in meters/sec
                                                   rotationDelayDistance  // Rotation delay distance in meters. This is how far the robot should travel before attempting to rotate.
                                                   ));
}

#if 0
//xxx from photonvision swerve sample code..  Figure out these for dynamic STd dev. calculation.
Eigen::Matrix<double, 3, 1> GetEstimationStdDevs(frc::Pose2d estimatedPose)
{
    Eigen::Matrix<double, 3, 1> estStdDevs = constants::Vision::kSingleTagStdDevs;

    int numTags            = 0;
    units::meter_t avgDist = 0_m;
    auto targets           = GetLatestResult().GetTargets();  // probably this getBestCameraToTarget instread.. since we won't have it as a result.  or we calculated confidence in vision subsystem.

    for(const auto& tgt : targets)
    {
      auto tagPose = photonEstimator.GetFieldLayout().GetTagPose(tgt.GetFiducialId());

      if(tagPose.has_value())
      {
        numTags++;
        avgDist += tagPose.value().ToPose2d().Translation().Distance(estimatedPose.Translation());
      }
    }

    if(numTags == 0)
    {
      return estStdDevs;
    }

    avgDist /= numTags;

    if(numTags > 1)
    {
      estStdDevs = constants::Vision::kMultiTagStdDevs;
    }

    if((numTags == 1) && (avgDist > 4_m))
    {
      estStdDevs = (Eigen::MatrixXd(3, 1) << std::numeric_limits<double>::max(), std::numeric_limits<double>::max(), std::numeric_limits<double>::max()).finished();
    }
    else
    {
      estStdDevs = estStdDevs * (1 + (avgDist.value() * avgDist.value() / 30));
    }

    return estStdDevs;
  }

//xxx from Robot-Joe-Test Java code...
  private Matrix<N3, N1> confidenceCalculator(EstimatedRobotPose estimation)
  {
    double smallestDistance = Double.POSITIVE_INFINITY;
    for(var target : estimation.targetsUsed)
    {
      var t3d = target.getBestCameraToTarget();
      var distance = Math.sqrt(Math.pow(t3d.getX(), 2) + Math.pow(t3d.getY(), 2) + Math.pow(t3d.getZ(), 2));
      if (distance < smallestDistance)
        smallestDistance = distance;
    }

    double poseAmbiguityFactor = estimation.targetsUsed.size() != 1
        ? 1
        : Math.max(
            1,
            (estimation.targetsUsed.get(0).getPoseAmbiguity()
                + Constants.VisionConstants.POSE_AMBIGUITY_SHIFTER)
                * Constants.VisionConstants.POSE_AMBIGUITY_MULTIPLIER);

    double confidenceMultiplier = Math.max(
        1,
        (Math.max(
            1,
            Math.max(0, smallestDistance - Constants.VisionConstants.NOISY_DISTANCE_METERS)
                * Constants.VisionConstants.DISTANCE_WEIGHT)
            * poseAmbiguityFactor)
            / (1
                + ((estimation.targetsUsed.size() - 1) * Constants.VisionConstants.TAG_PRESENCE_WEIGHT)));

    return Constants.VisionConstants.VISION_MEASUREMENT_STANDARD_DEVIATIONS.times(confidenceMultiplier);
  }
#endif
