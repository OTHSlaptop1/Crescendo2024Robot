// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/kinematics/SwerveModuleState.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include <wpi/sendable/SendableBuilder.h>

#include "Constants.h"
#include "utils/SwerveUtils.h"

using namespace DriveConstants;
using namespace ctre::phoenix6;

    /* Drive Subsystem Constructor.                                     */
DriveSubsystem::DriveSubsystem(bool fieldRelativeState, bool limitSlewRate)
    : m_frontLeft{kFrontLeftDrivingCanId, kFrontLeftTurningCanId},
      m_frontRight{kFrontRightDrivingCanId, kFrontRightTurningCanId},
      m_rearLeft{kRearLeftDrivingCanId, kRearLeftTurningCanId},
      m_rearRight{kRearRightDrivingCanId, kRearRightTurningCanId}
{

   /* Set the name for this subsystem.                                  */
   this->SetName("Drive Subsystem");

   /* ** NOTE ** We are setting the pigeon configuration after the      */
   /*            odometry object was created that used the pigeon       */
   /*            current angle.  If something in the configuration is   */
   /*            going to change this current angle, might have to      */
   /*            initialize the odometry object differently.  I think   */
   /*            doing this is ok for now?                              */

   /* Configure Pigeon2, a blank configuration uses the                 */
   /* factory-defaults.                                                 */
   configs::Pigeon2Configuration configurationToApply{};

   /* User can change the configs if they want, or leave it empty for   */
   /* factory-default.                                                  */
   pidgey_gyro.GetConfigurator().Apply(configurationToApply);

   /* ** NOTE ** Somethings to possible look at for the gyro.           */
   /*              yaw.setUpdateFrequency(Module.ODOMETRY_FREQUENCY);   */
   /*              pigeon.optimizeBusUtilization();                     */

   /* Initialize the max speed and max angular speed values.            */
   m_maxSpeed        = kDefaultMaxSpeed;
   m_maxAngularSpeed = kDefaultMaxAngularSpeed;

   /* Set the field relative state and limit slew rate state based on   */
   /* the specifie parameters.                                          */
   m_fieldRelative = fieldRelativeState;
   m_limitSlewRate = limitSlewRate;

   // Start publishing an array of module states with the "/SwerveStates" key
   swerveMeasuredPublisher           = nt::NetworkTableInstance::GetDefault().GetStructArrayTopic<frc::SwerveModuleState>("/SwerveStates/Measured").Publish();
   swerveSetpointsPublisher          = nt::NetworkTableInstance::GetDefault().GetStructArrayTopic<frc::SwerveModuleState>("/SwerveStates/Setpoints").Publish();
   swerveSetPointsOptimizedPublisher = nt::NetworkTableInstance::GetDefault().GetStructArrayTopic<frc::SwerveModuleState>("/SwerveStates/SetpointsOptimized").Publish();
}

   /* Function called periodically by the scheduler.                    */
void DriveSubsystem::Periodic()
{
   /* Get the current measured state for each of the swerve modules and */
   /* output it for logging.                                            */
   swerveMeasuredPublisher.Set(std::vector{m_frontLeft.GetState(), m_frontRight.GetState(), m_rearLeft.GetState(), m_rearRight.GetState()});
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed, units::meters_per_second_t ySpeed, units::radians_per_second_t rot)
{
   double                 xSpeedCommanded;
   double                 ySpeedCommanded;
   frc::SwerveModuleState frontLeftOptimizedState{};
   frc::SwerveModuleState frontRightOptimizedState{};
   frc::SwerveModuleState rearLeftOptimizedState{};
   frc::SwerveModuleState rearRightOptimizedState{};
   frc::ChassisSpeeds     ChassisSpeeds;

   /* First check to see if we need to use rate limiting while driving  */
   /* to the new specified location.                                    */
   if(m_limitSlewRate)
   {
      // Convert XY to polar for rate limiting
      double inputTranslationDir = atan2(ySpeed.value(), xSpeed.value());
      double inputTranslationMag = sqrt(pow(xSpeed.value(), 2) + pow(ySpeed.value(), 2));

      // Calculate the direction slew rate based on an estimate of the lateral
      // acceleration
      double directionSlewRate;
      if(m_currentTranslationMag != 0.0)
      {
         directionSlewRate = abs(DriveConstants::kDirectionSlewRate / m_currentTranslationMag);
      }
      else
      {
         // some high number that means the slew rate is effectively instantaneous
         directionSlewRate = 500.0;
      }

      double currentTime = wpi::Now() * 1e-6;
      double elapsedTime = currentTime - m_prevTime;
      double angleDif    = SwerveUtils::AngleDifference(inputTranslationDir, m_currentTranslationDir);  // calculate the

      if(angleDif < 0.45 * std::numbers::pi)   // angle difference less than 0.45*Pi rad or 81 degrees
      {
        m_currentTranslationDir = SwerveUtils::StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, (directionSlewRate * elapsedTime));
        m_currentTranslationMag = m_magLimiter.Calculate(inputTranslationMag);
      }
      else if(angleDif > 0.85 * std::numbers::pi)  // angle difference greater than 0.85*Pi rad or 153 degrees
      {
        /* Compare versus some small number to avoid floating-point     */
        /* errors with equality checking.  This is effectively a        */
        /* comparison vs zero value.                                    */
        /* ** NOTE ** I think this keeps the turn motors from changing  */
        /*            positions when the commanded veleocity is zero.   */
        /* ** NOTE ** See https://www.youtube.com/watch?v=0Xi9yb1IMyA   */
        /*            around 9:15 second mark for example of this.  If  */
        /*            this is no what this is doing, it might be worth  */
        /*            adding it to the SetDesiredState() function in the*/
        /*            actual swerve module code.                        */
        if(m_currentTranslationMag > 1e-4)
        {
           // keep currentTranslationDir unchanged
           m_currentTranslationMag = m_magLimiter.Calculate(0.0);
        }
        else
        {
           m_currentTranslationDir = SwerveUtils::WrapAngle(m_currentTranslationDir + std::numbers::pi);
           m_currentTranslationMag = m_magLimiter.Calculate(inputTranslationMag);
        }
      }
      else
      {
         m_currentTranslationDir = SwerveUtils::StepTowardsCircular(m_currentTranslationDir, inputTranslationDir, (directionSlewRate * elapsedTime));
         m_currentTranslationMag = m_magLimiter.Calculate(0.0);
      }

      m_prevTime = currentTime;

      xSpeedCommanded   = m_currentTranslationMag * cos(m_currentTranslationDir);
      ySpeedCommanded   = m_currentTranslationMag * sin(m_currentTranslationDir);
      m_currentRotation = m_rotLimiter.Calculate(rot.value());
   }
   else
   {
      /* Not using rate limiting.  Just set the commanded speeds to the */
      /* values that were specified.                                    */
      xSpeedCommanded   = xSpeed.value();
      ySpeedCommanded   = ySpeed.value();
      m_currentRotation = rot.value();
   }

   // Convert the commanded speeds into the correct units for the drivetrain
   units::meters_per_second_t xSpeedDelivered = xSpeedCommanded * m_maxSpeed;
   units::meters_per_second_t ySpeedDelivered = ySpeedCommanded * m_maxSpeed;
   units::radians_per_second_t rotDelivered   = m_currentRotation * m_maxAngularSpeed;

   /* Next determine if we are using field relative movements.          */
   if(m_fieldRelative)
   {
      /* We are currently using field relative movements.               */

      /* Now build the chassis speeds assuming field relative movement. */
      ChassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, pidgey_gyro.GetRotation2d());

//xxx example of how to use Discretize if deem useful.   Not totallly sure what to use for period here.   there is a (GetPeriod()) function that could be used or as var passed into drive.
//xxx play around a little bit with this, didn't notice much differnce on our floor, maybe make more difference on carpet.
//  ChassisSpeeds = frc::ChassisSpeeds::Discretize(frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, pidgey_gyro.GetRotation2d()), 0.02_s);
   }
   else
   {
      /* We are not currently using field relative movements.           */

      /* Now build the chassis speeed assuming none field relative      */
      /* movement.                                                      */
      ChassisSpeeds = frc::ChassisSpeeds{xSpeedDelivered, ySpeedDelivered, rotDelivered};

//xxx example of how to use Discretize if deem useful.   Not totallly sure what to use for period here.   there is a (GetPeriod()) function that could be used or as var passed into drive.
//xxx play around a little bit with this, didn't notice much differnce on our floor, maybe make more difference on carpet.
//  ChassisSpeeds = frc::ChassisSpeeds::Discretize(frc::ChassisSpeeds{xSpeedDelivered, ySpeedDelivered, rotDelivered}, 0.02_s);
   }

   /* Next calculate the drive kinematics to get the required swerve    */
   /* module states based on the specified Chassis Speeds.              */
   auto setpointStates = kDriveKinematics.ToSwerveModuleStates(ChassisSpeeds);

   /* Renormalizes the wheel speeds if any individual speed is above the*/
   /* specified maximum.                                                */
   /* ** NOTE ** Sometimes, after inverse kinematics, the requested     */
   /*            speed from one or more modules may be above the max    */
   /*            attainable speed for the driving motor on that module. */
   /*            To fix this issue, one can reduce all the wheel speeds */
   /*            to make sure that all requested module speeds are      */
   /*            at-or-below the absolute threshold, while maintaining  */
   /*            the ratio of speeds between modules.                   */
   kDriveKinematics.DesaturateWheelSpeeds(&setpointStates, m_maxSpeed);

   /* Break up the states into the individual swerve components.        */
   auto [fl, fr, rl, rr] = setpointStates;

   /* Log the set points states information.                            */
   swerveSetpointsPublisher.Set(setpointStates);

   /* Set the desired state for each of the swerve modules and save the */
   /* return value to the optimized desired state.                      */
   frontLeftOptimizedState  = m_frontLeft.SetDesiredState(fl);
   frontRightOptimizedState = m_frontRight.SetDesiredState(fr);
   rearLeftOptimizedState   = m_rearLeft.SetDesiredState(rl);
   rearRightOptimizedState  = m_rearRight.SetDesiredState(rr);

   /* Log the set points optimized information.                         */
   swerveSetPointsOptimizedPublisher.Set(std::vector{frontLeftOptimizedState, frontRightOptimizedState, rearLeftOptimizedState, rearRightOptimizedState});
}

   /* This function drive the robot using the specified robot relative  */
   /* speeds.                                                           */
void DriveSubsystem::DriveRobotRelative(frc::ChassisSpeeds RobotRelativeSpeeds)
{
   frc::SwerveModuleState frontLeftOptimizedState{};
   frc::SwerveModuleState frontRightOptimizedState{};
   frc::SwerveModuleState rearLeftOptimizedState{};
   frc::SwerveModuleState rearRightOptimizedState{};

   /* Next calculate the drive kinematics to get the required swerve    */
   /* module states based on the specified Chassis Speeds.              */
   auto setpointStates = kDriveKinematics.ToSwerveModuleStates(RobotRelativeSpeeds);

   /* Renormalizes the wheel speeds if any individual speed is above the*/
   /* specified maximum.                                                */
   /* ** NOTE ** Sometimes, after inverse kinematics, the requested     */
   /*            speed from one or more modules may be above the max    */
   /*            attainable speed for the driving motor on that module. */
   /*            To fix this issue, one can reduce all the wheel speeds */
   /*            to make sure that all requested module speeds are      */
   /*            at-or-below the absolute threshold, while maintaining  */
   /*            the ratio of speeds between modules.                   */
   /* ** NOTE ** Since this function isn't typically used for teleops,  */
   /*            we are setting the max speed to use in desaturation to */
   /*            the default maximum speed.                             */
   kDriveKinematics.DesaturateWheelSpeeds(&setpointStates, kDefaultMaxSpeed);

   /* Break up the states into the individual swerve components.        */
   auto [fl, fr, rl, rr] = setpointStates;

   /* Log the set points states information.                            */
   swerveSetpointsPublisher.Set(setpointStates);

   /* Set the desired state for each of the swerve modules and save the */
   /* return value to the optimized desired state.                      */
   frontLeftOptimizedState  = m_frontLeft.SetDesiredState(fl);
   frontRightOptimizedState = m_frontRight.SetDesiredState(fr);
   rearLeftOptimizedState   = m_rearLeft.SetDesiredState(rl);
   rearRightOptimizedState  = m_rearRight.SetDesiredState(rr);

   /* Log the set points optimized information.                         */
   swerveSetPointsOptimizedPublisher.Set(std::vector{frontLeftOptimizedState, frontRightOptimizedState, rearLeftOptimizedState, rearRightOptimizedState});
}

   /* This function set the wheels into an X configuration with the     */
   /* wheel velocities set to zero.                                     */
void DriveSubsystem::SetX()
{
   m_frontLeft.SetDesiredState(frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
   m_frontRight.SetDesiredState(frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
   m_rearLeft.SetDesiredState(frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
   m_rearRight.SetDesiredState(frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
}

   /* This function set the swerve module states to the specified       */
   /* desired state.                                                    */
void DriveSubsystem::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates)
{
   /* Renormalizes the wheel speeds if any individual speed is above the*/
   /* specified maximum.                                                */
   /* ** NOTE ** Sometimes, after inverse kinematics, the requested     */
   /*            speed from one or more modules may be above the max    */
   /*            attainable speed for the driving motor on that module. */
   /*            To fix this issue, one can reduce all the wheel speeds */
   /*            to make sure that all requested module speeds are      */
   /*            at-or-below the absolute threshold, while maintaining  */
   /*            the ratio of speeds between modules.                   */
   /* ** NOTE ** Since this function isn't typically used for teleops,  */
   /*            we are setting the max speed to use in desaturation to */
   /*            the default maximum speed.                             */
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates, kDefaultMaxSpeed);

  /* Set the desired state for each of the swerve modules.              */
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

   /* This function gets the swerve module states.                      */
wpi::array<frc::SwerveModuleState, 4> DriveSubsystem::GetModuleStates()
{
   return(wpi::array<frc::SwerveModuleState, 4>{m_frontLeft.GetState(), m_frontRight.GetState(), m_rearLeft.GetState(), m_rearRight.GetState()});
}

   /* Get the front left swerve modules state.                          */
frc::SwerveModuleState DriveSubsystem::GetFrontLeftModuleState()
{
   return(m_frontLeft.GetState());
}

   /* Get the front right swerve modules state.                         */
frc::SwerveModuleState DriveSubsystem::GetFrontRightModuleState()
{
   return(m_frontRight.GetState());
}

   /* Get the rear left swerve modules state.                           */
frc::SwerveModuleState DriveSubsystem::GetRearLeftModuleState()
{
   return(m_rearLeft.GetState());
}

   /* Get the rear right swerve modules state.                          */
frc::SwerveModuleState DriveSubsystem::GetRearRightModuleState()
{
   return(m_rearRight.GetState());
}

   /* Get the swerve module positions.                                  */
wpi::array<frc::SwerveModulePosition, 4> DriveSubsystem::GetModulePositions()
{
   return(wpi::array<frc::SwerveModulePosition, 4>{m_frontLeft.GetPosition(), m_frontRight.GetPosition(), m_rearLeft.GetPosition(), m_rearRight.GetPosition()});
}

   /* This function resets the encoders associated with each of the     */
   /* swerve modules.                                                   */
void DriveSubsystem::ResetEncoders()
{
   /* ** NOTE ** This is resetting the driver encoder positions to zero.*/
   m_frontLeft.ResetEncoders();
   m_frontRight.ResetEncoders();
   m_rearLeft.ResetEncoders();
   m_rearRight.ResetEncoders();
}

   /* This function gets the current gyro angle from the module (in     */
   /* degrees).                                                         */
   /* ** NOTE ** The Pidgeon documentation says that this value is      */
   /*            Increasing as the Pigeon 2 turns clockwise.  All       */
   /*            Kinematics and Odometry functions expect increasing in */
   /*            the counter clockwise direction.  So should probably   */
   /*            not be used with them without some testing.            */
units::degree_t DriveSubsystem::GetHeading() const
{
   /* Simply return the current gyro angle in degress to the caller.    */
   return(units::degree_t{pidgey_gyro.GetAngle()});
}

   /* This function gets the current gyro angle as a Rotation2d.        */
frc::Rotation2d DriveSubsystem::GetRotation2dHeading() const
{
   /* Simply rturn the current gyro heading as a Rotation2d to the      */
   /* caller.                                                           */
   return(pidgey_gyro.GetRotation2d());
}

   /* This function resets the gyro angle being accumlated to zero.     */
void DriveSubsystem::ZeroHeading()
{
   pidgey_gyro.SetYaw(units::angle::degree_t(0.0));

   /* ** NOTE ** Could change to a call to pidgey_gyro.Reset() if you   */
   /*            want the IMU to recalibrate as well.                   */
}

   /* This function gets the current turn rate in degrees per second.   */
double DriveSubsystem::GetTurnRate()
{
   /* The rate is positive as the Pigeon 2 turns clockwise when looked  */
   /* at from the top.  The Kinematics and Odometry functions expect    */
   /* increasing in the counter clockwise direction so we are negating  */
   /* the value that is returned.                                       */
   return(-pidgey_gyro.GetRate());
}

   /* This function gets the current robot relative chassis speeds.     */
frc::ChassisSpeeds DriveSubsystem::GetRobotRelativeChassisSpeeds()
{
   return(kDriveKinematics.ToChassisSpeeds(GetModuleStates()));
}

   /* Set the current maximum speed                                     */
void DriveSubsystem::SetMaxSpeed(units::meters_per_second_t maxSpeed)
{
   m_maxSpeed = maxSpeed;
}

   /* Get the current maximum speed                                     */
units::meters_per_second_t DriveSubsystem::GetMaxSpeed(void)
{
   return(m_maxSpeed);
}

   /* Set the current maximum angular speed                             */
void DriveSubsystem::SetMaxAngularSpeed(units::radians_per_second_t maxAngularSpeed)
{
   m_maxAngularSpeed = maxAngularSpeed;
}

   /* Get the current maximum angular speed                             */
units::radians_per_second_t DriveSubsystem::GetMaxAngularSpeed(void)
{
   return(m_maxAngularSpeed);
}

   /* Set the field relative drive state.                               */
void DriveSubsystem::SetFieldRelativeState(bool fieldRelativeState)
{
   m_fieldRelative = fieldRelativeState;
}

   /* Get the field relative drive state.                               */
bool DriveSubsystem::GetFieldRelativeState(void)
{
   return(m_fieldRelative);
}

   /* Set the limit slew rate state.                                    */
void DriveSubsystem::SetLimitSlewRateState(bool limitSlewRate)
{
   m_limitSlewRate = limitSlewRate;
}

   /* Get the limit slew rate state.                                    */
bool DriveSubsystem::GetLimitSlewRateState(void)
{
   return(m_limitSlewRate);
}

