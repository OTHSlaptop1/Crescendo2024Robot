// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/DriveFieldRelativeWithAngularVelocityCommand.h"

#include <frc/DriverStation.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/filter/SlewRateLimiter.h>

#include "Constants.h"

#include "utils/SwerveUtils.h"

DriveFieldRelativeWithAngularVelocityCommand::DriveFieldRelativeWithAngularVelocityCommand(DriveSubsystem *driveSubsystem, OdometrySubsystem *odometrySubsystem, std::function<units::meters_per_second_t()> xSpeedSupplier, std::function<units::meters_per_second_t()> ySpeedSupplier, std::function<units::radians_per_second_t()> rotationSpeedSupplier)
{
   /* The requirements for this command.  This command uses the drive   */
   /* subsystem and the odometry subsystem so add them to the           */
   /* requirements list.                                                */
   AddRequirements(driveSubsystem);
   AddRequirements(odometrySubsystem);

   m_driveSubsystemPtr    = driveSubsystem;
   m_odometrySubsystemPtr = odometrySubsystem;

   /* Give this command a user readable name.                           */
   this->SetName("DriveFieldRelativeWithAngularVelocityCommand()");

   /* Save the various speed suppliers for later use.                   */
   m_xSpeedSupplier        = xSpeedSupplier;
   m_ySpeedSupplier        = ySpeedSupplier;
   m_rotationSpeedSupplier = rotationSpeedSupplier;
}

// Called when the command is initially scheduled.
void DriveFieldRelativeWithAngularVelocityCommand::Initialize()
{
   /* Reset the slew rate limiters to a known state.                    */
   m_magLimiter.Reset(0.0);
   m_rotLimiter.Reset(0.0);
}

// Called repeatedly when this Command is scheduled to run
void DriveFieldRelativeWithAngularVelocityCommand::Execute()
{
   double             xSpeedCommanded;
   double             ySpeedCommanded;
   frc::Rotation2d    rotation2D;
   frc::ChassisSpeeds ChassisSpeeds;

   /* Get the current commanded values from the speed suppliers.        */
   units::meters_per_second_t  xSpeed        = m_xSpeedSupplier();
   units::meters_per_second_t  ySpeed        = m_ySpeedSupplier();
   units::radians_per_second_t rotationSpeed = m_rotationSpeedSupplier();

   /* ** NOTE ** The below is a mess, at some point it would be nice to */
   /*            go thru this and simplify it.  There are plenty of     */
   /*            examples out there that are much simpler then this for */
   /*            limiting slew rate etc.                                */

   /* First check to see if we need to use rate limiting while driving  */
   /* to the new specified location.                                    */
   if(m_driveSubsystemPtr->GetLimitSlewRateState())
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
        /*            positions when the commanded velocity is zero??   */
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
      m_currentRotation = m_rotLimiter.Calculate(rotationSpeed.value());
   }
   else
   {
      /* Not using rate limiting.  Just set the commanded speeds to the */
      /* values that were specified.                                    */
      xSpeedCommanded   = xSpeed.value();
      ySpeedCommanded   = ySpeed.value();
      m_currentRotation = rotationSpeed.value();
   }

   // Convert the commanded speeds into the correct units for the drivetrain
   units::meters_per_second_t  xSpeedDelivered   = xSpeedCommanded * m_driveSubsystemPtr->GetMaxSpeed();
   units::meters_per_second_t  ySpeedDelivered   = ySpeedCommanded * m_driveSubsystemPtr->GetMaxSpeed();
   units::radians_per_second_t rotationDelivered = m_currentRotation * m_driveSubsystemPtr->GetMaxAngularSpeed();

   /* Get the current estimated pose rotation based on the pose         */
   /* estimated odometry.                                               */
   rotation2D = m_odometrySubsystemPtr->GetPose().Rotation();

   if(frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed)
   {
      rotation2D = rotation2D + frc::Rotation2d{180_deg};
   }

   /* Now build the chassis speeds assuming field relative movement. */
   ChassisSpeeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotationDelivered, rotation2D);

   /* Now take the calculated chassis speed and set the drive subsystem */
   /* to use it.                                                        */
   m_driveSubsystemPtr->SetChassisSpeed(ChassisSpeeds);
}

// Called once the command ends or is interrupted.
void DriveFieldRelativeWithAngularVelocityCommand::End(bool interrupted)
{
   /* When the command ends simply stop the robot.                      */
   m_driveSubsystemPtr->SetChassisSpeed(frc::ChassisSpeeds{});
}

// Returns true when the command should end.
bool DriveFieldRelativeWithAngularVelocityCommand::IsFinished()
{
   /* Never finish.                                                     */
   return(false);
}
