// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

#include <cameraserver/CameraServer.h>
#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <wpinet/PortForwarder.h>

void Robot::RobotInit()
{
   // Starts recording to data log
   frc::DataLogManager::Start();

   // Record both DS control and joystick data
   frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());

   /* Start automatic streaming of a drivers camera.                */
//   cs::UsbCamera camera = frc::CameraServer::StartAutomaticCapture();

   // Set the resolution
//   camera.SetResolution(640, 480);

   // Forward the photon vision port so it can be seen from computers
   // when tethered to the USB port on the roboRIO.
   // NOTE: I havne't been able to get this to work when testing, I have only
   //       been able to get access to photon vision through the FRC Radio Connection.
//   wpi::PortForwarder::GetInstance().Add(5800, "photonvision.local", 5800);
}

/**
 * This function is called every 20 ms, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
void Robot::RobotPeriodic()
{
   /* Log the power distribution.                                       */
   m_container.LogPowerDistribution();

   /* Get the latest values from shuffle board and set the subsystems to*/
   /* the updated state.                                                */
   m_container.PumpShuffleBoard();

   /* Pump the drive speed governor.                                    */
   m_container.PumpDriveGovernor();

   /* Run the command scheduler.                                        */
   frc2::CommandScheduler::GetInstance().Run();
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit()
{
   /* Disable any rumble that might currently be active.             */
   m_container.DisableRumble();
}

void Robot::DisabledPeriodic() {}

/**
 * This autonomous (along with the chooser code above) shows how to select
 * between different autonomous modes using the dashboard. The sendable chooser
 * code works with the Java SmartDashboard. If you prefer the LabVIEW Dashboard,
 * remove all of the chooser code and uncomment the GetString line to get the
 * auto name from the text box below the Gyro.
 *
 * You can add additional auto modes by adding additional comparisons to the
 * if-else structure below with additional strings. If using the SendableChooser
 * make sure to add them to the chooser code above as well.
 */
void Robot::AutonomousInit()
{
   /* Disable any rumble that might currently be active.                */
   m_container.DisableRumble();

   /* Get the currently selected autonomous command from the robot      */
   /* container.                                                        */
   m_autonomousCommand = m_container.GetAutonomousCommand();

   /* Check to make sure that this command appears to be valid.         */
   if(m_autonomousCommand)
   {
      /* The selected command appears to be valid.  Attempt to scheduler*/
      /* it for execution.                                              */
      m_autonomousCommand->Schedule();
   }
}

void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit()
{
  // This makes sure that the autonomous stops running when
  // teleop starts running. If you want the autonomous to
  // continue until interrupted by another command, remove
  // this line or comment it out.
  if(m_autonomousCommand)
  {
     m_autonomousCommand->Cancel();
     m_autonomousCommand.reset();
  }

   /* Disable any rumble that might currently be active.                */
   m_container.DisableRumble();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic()
{
   /* Pump the periodic polling in the Robot Container that is used to  */
   /* control the controller rumble function.                           */
   m_container.PumpRumble();
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
