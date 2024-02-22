// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/smartdashboard/Field2d.h>

#include <functional>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/OdometrySubsystem.h"
#include "subsystems/VisionSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/ArmSubsystem.h"

#include "utils/SwerveSendable.h"

   /* The following constants define the April Tag ID associated with   */
   /* specific objects on the field.                                    */
#define RED_AMP_TAG_ID                                               (5)
#define BLUE_AMP_TAG_ID                                              (6)

#define USE_INTAKE
#define USE_SHOOTER
#define USE_ARM
//#define USE_VISION

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

  void PumpShuffleBoard(void);

 private:

 // The robot's subsystems
  DriveSubsystem    m_drive{true, true};         // drive subsystem -> field relative, limit slew rate params
#ifdef USE_VISION
  OdometrySubsystem m_odometry{&m_drive, &m_vision, m_field};  // odometry subsystem
#else
  OdometrySubsystem m_odometry{&m_drive, NULL, m_field};  // odometry subsystem
#endif
#ifdef USE_INTAKE
  IntakeSubsystem   m_intake;                  // note intake subsystem
#endif
#ifdef USE_SHOOTER
  ShooterSubsystem  m_shooter;                 // shooter subsystem
#endif
#ifdef USE_VISION
  VisionSubsystem   m_vision{"photonvision_0", frc::Transform3d{0_m, 0_m, 0_m, frc::Rotation3d{units::radian_t{0_deg}, units::radian_t{0_deg}, units::radian_t{0_deg}}}, m_odometry.GetAprilTagFieldLayout()};    // vision subsystem
#endif
#ifdef USE_ARM
  ArmSubsystem m_arm;                          // arm subsystem
#endif

  // Static private function which acts as a command factory for getting
  // autonomous routines from path planner auto builder.
  static frc2::CommandPtr PathPlannerCommandFactory(std::string autoName) noexcept;

  // The driver's controller
  frc::XboxController m_driverController{OIConstants::kDriverControllerPort};

  // Shuffleboard Entries
  nt::GenericEntry *m_SetPoseXEntryPtr;
  nt::GenericEntry *m_SetPoseYEntryPtr;
  nt::GenericEntry *m_allowSetPoseEntryPtr;

  bool              m_resetOdometryButton;

  nt::GenericEntry *m_maxSpeedEntryPtr;
  nt::GenericEntry *m_maxAngularSpeedEntryPtr;
  nt::GenericEntry *m_triggerBasedSpeedControlEntryPtr;
  nt::GenericEntry *m_fieldRelativeStateEntryPtr;
  nt::GenericEntry *m_limitSlewRateEntryPtr;

  bool              m_resetGyroButton;

  nt::GenericEntry *m_allowIntakeControlEntryPtr;
  nt::GenericEntry *m_intakeSpeedEntryPtr;

  bool              m_lastAllowIntakeControlState;

  nt::GenericEntry *m_allowShooterControlEntryPtr;
  nt::GenericEntry *m_shooterLeftSpeedEntryPtr;
  nt::GenericEntry *m_shooterRightSpeedEntryPtr;

  bool              m_lastAllowShooterControlState;

  /* Create the command that are used in various tabs on the            */
  /* Shuffleboard.                                                      */

  /* Subsytem Commands.                                                 */
  frc2::CommandPtr m_grabNote = m_intake.GrabNoteCommand(4500_rpm);

  /* Teleops Commands.                                                  */
  frc2::CommandPtr m_pathToBlueAmp = m_odometry.PathToPoseCommand(m_odometry.GetPoseFromAprilTagId(BLUE_AMP_TAG_ID, {0_m, -((DriveConstants::kWheelBase/2)+0.25_m), frc::Rotation2d{90_deg}}), 0_mps, 0_m).WithName("Path To Blue Amp");
  frc2::CommandPtr m_pathToRedAmp  = m_odometry.PathToPoseCommand(m_odometry.GetPoseFromAprilTagId(RED_AMP_TAG_ID, {0_m, -((DriveConstants::kWheelBase/2)+0.25_m), frc::Rotation2d{90_deg}}), 0_mps, 0_m).WithName("Path To Red Amp");

  /* Current field for use with shuffle board.                          */
  frc::Field2d m_field;

  /* Sendable object for swerve drive to be displayed in elastic Smart Dashboard.*/
  SwerveSendable m_swerveSendable{&m_drive};

  // The chooser for the autonomous routines
  frc::SendableChooser<std::function<frc2::CommandPtr()>> m_chooser;

  void ConfigureButtonBindings();
};
