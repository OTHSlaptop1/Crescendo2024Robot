// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "utils/SwerveSendable.h"

SwerveSendable::SwerveSendable(DriveSubsystem *driveSubsystemPtr)
{
    /* Save the drive subsystem pointer for use when running the        */
    /* sendable builder.                                                */
    m_driveSubsystemPtr = driveSubsystemPtr;
}

    /* Initailize the sendable associated with this object.             */
void SwerveSendable::InitSendable(wpi::SendableBuilder& builder)
{
   /* Set the build to indicate this data is associated with the swerve */
   /* drive.                                                            */
   builder.SetSmartDashboardType("SwerveDrive");

   /* First make sure that the drive subsystem pointer appears to be at*/
   /* least semi-valid.                                                */
   if(m_driveSubsystemPtr != NULL)
   {
      /* Using the captured state information, publish the information  */
      /* using the sendable object.                                     */
      builder.AddDoubleProperty("Front Left Angle", [this] { return(m_driveSubsystemPtr->GetFrontLeftModuleState().angle.Radians().value()); }, nullptr );
      builder.AddDoubleProperty("Front Left Velocity", [this] { return(m_driveSubsystemPtr->GetFrontLeftModuleState().speed.value()); }, nullptr );

      builder.AddDoubleProperty("Front Right Angle", [this] { return(m_driveSubsystemPtr->GetFrontRightModuleState().angle.Radians().value()); }, nullptr );
      builder.AddDoubleProperty("Front Right Velocity", [this] { return(m_driveSubsystemPtr->GetFrontRightModuleState().speed.value()); }, nullptr );

      builder.AddDoubleProperty("Back Left Angle", [this] { return(m_driveSubsystemPtr->GetRearLeftModuleState().angle.Radians().value()); }, nullptr );
      builder.AddDoubleProperty("Back Left Velocity", [this] { return(m_driveSubsystemPtr->GetRearLeftModuleState().speed.value()); }, nullptr );

      builder.AddDoubleProperty("Back Right Angle", [this] { return(m_driveSubsystemPtr->GetRearRightModuleState().angle.Radians().value()); }, nullptr );
      builder.AddDoubleProperty("Back Right Velocity", [this] { return(m_driveSubsystemPtr->GetRearRightModuleState().speed.value()); }, nullptr );

      builder.AddDoubleProperty("Robot Angle", [this] { return(m_driveSubsystemPtr->GetRotation2dHeading().Radians().value()); }, nullptr );
   }
   else
   {
      /* The drive subsystem pointer appears to be invalid.  Populate   */
      /* the information to the default state.                          */
      builder.AddDoubleProperty("Front Left Angle", [this] { return(0.0); }, nullptr );
      builder.AddDoubleProperty("Front Left Velocity", [this] { return(0.0); }, nullptr );

      builder.AddDoubleProperty("Front Right Angle", [this] { return(0.0); }, nullptr );
      builder.AddDoubleProperty("Front Right Velocity", [this] { return(0.0); }, nullptr );

      builder.AddDoubleProperty("Back Left Angle", [this] { return(0.0); }, nullptr );
      builder.AddDoubleProperty("Back Left Velocity", [this] { return(0.0); }, nullptr );

      builder.AddDoubleProperty("Back Right Angle", [this] { return(0.0); }, nullptr );
      builder.AddDoubleProperty("Back Right Velocity", [this] { return(0.0); }, nullptr );

      builder.AddDoubleProperty("Robot Angle", [this] { return(0.0); }, nullptr );
   }
}

