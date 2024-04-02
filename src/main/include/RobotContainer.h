// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/button/CommandXboxController.h>
#include <frc/GenericHID.h>
#include <frc/PowerDistribution.h>

#include <networktables/DoubleTopic.h>
#include <networktables/StructTopic.h>

#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/apriltag/AprilTagFields.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc/DigitalOutput.h>
#include <units/time.h>

#include <functional>

#include "Constants.h"

#include "commands/ShootNoteFromIntakeCommand.h"

#include "subsystems/DriveSubsystem.h"
#include "subsystems/OdometrySubsystem.h"
#include "subsystems/VisionSubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/Arm2Subsystem.h"
#include "subsystems/LiftSubsystem.h"
#include "subsystems/DisplaySubsystem.h"

#include "utils/SwerveSendable.h"

   /* The following constants define the April Tag ID associated with   */
   /* specific objects on the field.                                    */
#define RED_AMP_TAG_ID                                               (5)
#define BLUE_AMP_TAG_ID                                              (6)

#define USE_OPERATOR_CONTROLLER
#define USE_INTAKE
#define USE_SHOOTER
#define USE_ARM
#define USE_VISION
#define USE_LIFT
#define USE_DISPLAY

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

  /* The following function is responsible for logging the power        */
  /* distribution.                                                      */
  void LogPowerDistribution(void);

  /* The following function is responsible for pumping the shuffle board*/
  /* items to keep them update date.  This should be called during a    */
  /* robots periodic loop.                                              */
  void PumpShuffleBoard(void);

  /* The following function is responsible for disabling rumble.        */
  void DisableRumble(void);

  /* The following function is responsible for pumping the logic used to*/
  /* detect events that will enable and disable the rumble function of  */
  /* the controller.  This should be called during the robots period    */
  /* loop.                                                              */
  void PumpRumble(void);

  /* The following function is responsible for pumping the drive speed  */
  /* governor.  The goal of this function is to watch the current angle */
  /* of the arm and if it is not currently in the down position to      */
  /* automatically slow the maximum speed allowable for driving.        */
  void PumpDriveGovernor(void);

  /* Log shooting from pose information for data on if this could work. */
  void LogShootFromPose(void) noexcept;

  /* Pumps the testing if the vision system currently has targets.      */
  void PumpHasTarget(void);
 
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
  VisionSubsystem   m_vision{"photonvision_0", frc::Transform3d{-13_in, 0_m, 11.02_in, frc::Rotation3d{units::radian_t{0_deg}, units::radian_t{-35_deg}, units::radian_t{180.0_deg}}}, m_odometry.GetAprilTagFieldLayout()};    // vision subsystem
#endif
#ifdef USE_ARM
  Arm2Subsystem m_arm;                          // arm subsystem
#endif
#ifdef USE_LIFT
  LiftSubsystem m_lift;
#endif
#ifdef USE_DISPLAY
  DisplaySubsystem m_display;
#endif

  // Static private function which acts as a command factory for getting
  // autonomous routines from path planner auto builder.
  static frc2::CommandPtr PathPlannerCommandFactory(std::string autoName) noexcept;

  /* Command Factory for creating a command that can shot from a        */
  /* predefined location.                                               */
  frc2::CommandPtr ShootFromFixedPose(void) noexcept;

  /* Command Factory for creating a command that can prepare for        */
  /* lifting the robot on the chain.                                    */
  frc2::CommandPtr PrepareForLiftCommand(void);

  /* Command Factory for creating a command that locks the lift in      */
  /* preparation for lifting the robot on the chain.                    */
  frc2::CommandPtr LockLiftCommand(void);
  
  // The driver's controller
  frc2::CommandXboxController m_driverController{OIConstants::kDriverControllerPort};

#ifdef USE_OPERATOR_CONTROLLER
  // The operator's controller
//  frc::GenericHID m_operatorController{OIConstants::kOperatorControllerPort};
  frc2::CommandXboxController m_operatorController{OIConstants::kOperatorControllerPort};
#endif

  /* The following variable holds the power distribution panel object. */
  frc::PowerDistribution m_PDP{25, frc::PowerDistribution::ModuleType::kRev};

   // Publisher variables for the power distribution.
   nt::DoublePublisher m_pdpVoltagePublisher;
   nt::DoublePublisher m_pdpCurrentPublisher;
   nt::DoublePublisher m_pdpPowerPublisher;
   nt::DoublePublisher m_pdpTotalPowerPublisher;

   double m_totalPower;

   // Publisher variables for logging shoot from pose function.
   nt::DoublePublisher m_distanceFromSpeaker;
   nt::DoublePublisher m_armAngle;
   nt::DoublePublisher m_angleToSpeaker;
   nt::StructPublisher<frc::Pose2d> m_shootingPosePublisher;

  /* The following variable holds the variable use to control the rumble*/
  /* duty cycle.                                                        */
  units::second_t m_rumbleDutyCycleCount;
  bool            m_rumbleState;

  // Shuffleboard Entries
  nt::GenericEntry *m_SetPoseXEntryPtr;
  nt::GenericEntry *m_SetPoseYEntryPtr;
  nt::GenericEntry *m_allowSetPoseEntryPtr;

  bool              m_resetOdometryButton;

  nt::GenericEntry *m_maxSpeedEntryPtr;
  nt::GenericEntry *m_maxAngularSpeedEntryPtr;
  nt::GenericEntry *m_triggerBasedSpeedControlEntryPtr;
  nt::GenericEntry *m_driveSpeedGovernorEntryPtr;
  nt::GenericEntry *m_fieldRelativeStateEntryPtr;
  nt::GenericEntry *m_limitSlewRateEntryPtr;

  bool              m_resetGyroButton;
  bool              m_driveGovernorActive;

  nt::GenericEntry *m_allowIntakeControlEntryPtr;
  nt::GenericEntry *m_intakeSpeedEntryPtr;

  bool              m_lastAllowIntakeControlState;

  nt::GenericEntry *m_allowShooterControlEntryPtr;
  nt::GenericEntry *m_shooterLeftSpeedEntryPtr;
  nt::GenericEntry *m_shooterRightSpeedEntryPtr;

  bool              m_lastAllowShooterControlState;

  nt::GenericEntry *m_allowArmControlEntryPtr;
  nt::GenericEntry *m_armPositionEntryPtr;

  bool              m_disableArmButton;
  bool              m_lastAllowArmControlState;
  double            m_armLastDashboardSetPoint;

  frc::DigitalOutput m_hasTagDetect{9};

  /* Create the command that are used in various tabs on the            */
  /* Shuffleboard.                                                      */

  /* Subsytem Commands.                                                 */
#ifdef USE_INTAKE
  frc2::CommandPtr m_grabNote       = m_intake.GrabNoteCommand(9000_rpm);
#ifdef USE_SHOOTER
  frc2::CommandPtr m_slowRemoveNote = ShootNoteFromIntakeCommand(
                                                                  &m_intake,     // intake subsystem pointer
                                                                  9000_rpm,      // intake output to shooter speed (rpm)
                                                                  &m_shooter,    // shooter subsystem pointer
                                                                  500_rpm,       // shooter left flywheel speed (rpm)
                                                                  500_rpm,       // shooter right flywheel speed (rpm
                                                                  7.5_s,         // shooter flywheel spinup timeout
                                                                  1.0_s          // after intake enabled time till complete
                                                                ).ToPtr();
#endif
#endif

#ifdef USE_ARM
  frc2::CommandPtr m_armUp          = m_arm.ArmUpCommand();
  frc2::CommandPtr m_armDown        = m_arm.ArmDownCommand();
#endif
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
