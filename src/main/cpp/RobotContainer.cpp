// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"

#include <pathplanner/lib/auto/AutoBuilder.h>
#include <pathplanner/lib/commands/PathPlannerAuto.h>
#include <pathplanner/lib/auto/NamedCommands.h>

#include <frc/DriverStation.h>
#include <frc/geometry/Translation2d.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc2/command/Commands.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <frc2/command/button/NetworkButton.h>

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardContainer.h>

#include <utility>
#include <functional>
#include <limits>

#include "Constants.h"

#include "commands/ShootNoteFromIntakeCommand.h"
#include "commands/ShootNoteFromIntakeByArmPositionCommand.h"
#include "commands/DriveFieldRelativeWithAngularVelocityCommand.h"

#include "subsystems/DriveSubsystem.h"
#include "subsystems/OdometrySubsystem.h"
#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"

using namespace DriveConstants;

RobotContainer::RobotContainer() {

#ifdef USE_INTAKE
  /* Register the named commands that will be used by path planner.     */
  pathplanner::NamedCommands::registerCommand("IntakeGrabNote", std::move(m_intake.GrabNoteCommand(4500_rpm)));

#ifdef USE_SHOOTER
  pathplanner::NamedCommands::registerCommand("ShootNoteFromIntake", std::move(ShootNoteFromIntakeCommand(
                                                                                                           &m_intake,      // intake subsystem pointer
                                                                                                           9500_rpm,       // intake output to shooter speed (rpm)
                                                                                                           &m_shooter,     // shooter subsystem pointer
                                                                                                           4875_rpm,       // shooter left flywheel speed (rpm)
                                                                                                           4375_rpm,       // shooter right flywheel speed (rpm
                                                                                                           3.0_s,            // shooter flywheel spinup timeout
                                                                                                           0.750_s         // after intake enabled time till complete
                                                                                                          ).ToPtr()));
#endif
#endif

#ifdef USE_ARM
  /* Register the Arm Up Command.                                            */
  pathplanner::NamedCommands::registerCommand("ArmUp", std::move(m_arm.ArmUpCommand()));

  /* Register the Arm Down Command.                                          */
  pathplanner::NamedCommands::registerCommand("ArmDown", std::move(m_arm.ArmDownCommand()));
#endif

  /* Initialze the power distribution panel logging.                          */
  m_pdpVoltagePublisher     = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("/PDP/Voltage").Publish();
  m_pdpCurrentPublisher     = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("/PDP/Current").Publish();
  m_pdpPowerPublisher       = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("/PDP/Power").Publish();
  m_pdpTotalPowerPublisher  = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("/PDP/TotalPower").Publish();

  /* Initialize the total power consumtion.                             */
  m_totalPower              = 0;

  /* Initialize the internal variables to a known initial state.        */
  m_rumbleDutyCycleCount         = 0_s;
  m_rumbleState                  = false;

  m_driveGovernorActive          = false;
  m_resetGyroButton              = false;
  m_resetOdometryButton          = false;

  m_lastAllowIntakeControlState  = false;
  m_lastAllowShooterControlState = false;
  m_lastAllowArmControlState     = false;

  m_disableArmButton             = false;
  m_armLastDashboardSetPoint     = std::numeric_limits<double>::quiet_NaN();

  // Initialize all of your commands and subsystems here

/// test display....
   m_display.DisplayToggle(1500_ms, frc::Color8Bit{128, 128, 0}, frc::Color8Bit{0, 0, 128});

  // Configure the button bindings
  ConfigureButtonBindings();

  /* ****************************************************************** */
  /* *********************** Autonomous ******************************* */
  /* ****************************************************************** */

  frc::Shuffleboard::GetTab("Autonomous").GetLayout("Set Pose", frc::BuiltInLayouts::kList).WithSize(3, 4).WithPosition(0, 2);
    // Add sliders to track or set the position.
  m_SetPoseXEntryPtr     = frc::Shuffleboard::GetTab("Autonomous").GetLayout("Set Pose").Add("X meters", m_odometry.GetPose().X().value()).WithWidget(frc::BuiltInWidgets::kNumberSlider).WithProperties({{"min_value", nt::Value::MakeDouble(0.0)}, {"max_value", nt::Value::MakeDouble((m_odometry.GetFieldLength()).value())}}).WithPosition(0, 1).GetEntry();
  m_SetPoseYEntryPtr     = frc::Shuffleboard::GetTab("Autonomous").GetLayout("Set Pose").Add("Y meters", m_odometry.GetPose().Y().value()).WithWidget(frc::BuiltInWidgets::kNumberSlider).WithProperties({{"min_value", nt::Value::MakeDouble(0.0)}, {"max_value", nt::Value::MakeDouble((m_odometry.GetFieldWidth()).value())}}).WithPosition(0, 2).GetEntry();

  m_allowSetPoseEntryPtr = frc::Shuffleboard::GetTab("Autonomous").GetLayout("Set Pose").Add("Allow Set Pose", false).WithWidget(frc::BuiltInWidgets::kToggleSwitch).WithPosition(0, 3).GetEntry();

  /* Add a button to set the position using a triggered network button  */
  /* command.  Note this clears the boolean after running the function  */
  /* to reset the odometry.                                             */
  frc::Shuffleboard::GetTab("Autonomous").GetLayout("Set Pose").AddBoolean("Reset Odometry", [this]{ return(m_resetOdometryButton); }).WithWidget(frc::BuiltInWidgets::kToggleButton).WithPosition(0, 4);
  frc2::NetworkButton(nt::NetworkTableInstance::GetDefault().GetBooleanTopic("/Shuffleboard/Autonomous/Set Pose/Reset Odometry")).OnTrue(frc2::cmd::RunOnce([this] { m_odometry.ResetOdometry({units::meter_t(m_SetPoseXEntryPtr->GetDouble(0.0)), units::meter_t(m_SetPoseYEntryPtr->GetDouble(0.0)), m_drive.GetRotation2dHeading()}); m_resetOdometryButton = false; }, {&m_drive, &m_odometry}));

  // Add Path Planner autos into the autonomous command chooser
  //blue paths
   m_chooser.SetDefaultOption("BlueShootAndStayAmpSide", std::bind(RobotContainer::PathPlannerCommandFactory, "Blue Source- Autonomus 8")); //start at middle of field
   m_chooser.AddOption("BlueAmpSide", std::bind(RobotContainer::PathPlannerCommandFactory, "Blue Source- Autonomus 1")); //start at the side closer to the speaker
   m_chooser.AddOption("BlueSourceSide", std::bind(RobotContainer::PathPlannerCommandFactory, "Blue Source- Autonomus 2")); //start at the side closer to the opposing teams source
   m_chooser.AddOption("BlueShootAndRunAmpSide", std::bind(RobotContainer::PathPlannerCommandFactory, "Blue Source- Autonomus 3"));
  m_chooser.AddOption("BlueShootAndRunSourceSide", std::bind(RobotContainer::PathPlannerCommandFactory, "Blue Source- Autonomus 4"));
   m_chooser.AddOption("BlueRapidFireAmpSide", std::bind(RobotContainer::PathPlannerCommandFactory, "Blue Source- Autonomus 5"));
   m_chooser.AddOption("BlueDoubleShootSourceSide", std::bind(RobotContainer::PathPlannerCommandFactory, "Blue Source- Autonomus 6"));
   m_chooser.AddOption("BlueDoubleShootAndRunAmpSide", std::bind(RobotContainer::PathPlannerCommandFactory, "Blue Source- Autonomus 7"));
   m_chooser.AddOption("ShootAndStay", std::bind(RobotContainer::PathPlannerCommandFactory, "Blue Source- Autonomus 8"));
   
  //red paths
   m_chooser.AddOption("RedSourceSide", std::bind(RobotContainer::PathPlannerCommandFactory, "Red Source- Autonomous 2"));
   m_chooser.AddOption("RedAmpSide", std::bind(RobotContainer::PathPlannerCommandFactory, "Red Source- Autonomous 1"));
   m_chooser.AddOption("RedShootAndRunAmpSide", std::bind(RobotContainer::PathPlannerCommandFactory, "Red Source- Autonomus 3"));
   m_chooser.AddOption("RedShootAndRunSourceSide", std::bind(RobotContainer::PathPlannerCommandFactory, "Red Source- Autonomus 4"));
   m_chooser.AddOption("RedRapidFireSourceSide", std::bind(RobotContainer::PathPlannerCommandFactory, "Red Source- Autonomus 5"));
   m_chooser.AddOption("RedRapidFireAmpSide", std::bind(RobotContainer::PathPlannerCommandFactory, "Red Source- Autonomus 6"));
   m_chooser.AddOption("RedShootAndStaySourceSide", std::bind(RobotContainer::PathPlannerCommandFactory, "Red Source- Autonomus 7"));
   m_chooser.AddOption("RedShootAndStayAmpSide", std::bind(RobotContainer::PathPlannerCommandFactory, "Red Source- Autonomus 8"));
   



  // Put the chooser on the dashboard
  frc::Shuffleboard::GetTab("Autonomous").Add("Select Autonomous Path", m_chooser).WithSize(3, 2).WithPosition(0, 0);

  // Add a Match Timer to the autonomous tab.
  frc::Shuffleboard::GetTab("Autonomous").AddDouble("Match Time", [this]{ return(frc::DriverStation::GetMatchTime().value()); }).WithWidget("Match Time").WithSize(0, 2).WithPosition(3, 5);

  // Swerve Drive State information as a sendable.
  frc::Shuffleboard::GetTab("Autonomous").Add("Swerve Drive", m_swerveSendable).WithSize(3,3).WithPosition(5,5);

  /* ****************************************************************** */
  /* ************************* Teleops ******************************** */
  /* ****************************************************************** */

//  frc::Shuffleboard::GetTab("Teleoperated").GetLayout("Blue Commands", frc::BuiltInLayouts::kList).WithSize(2, 5).WithPosition(0, 0);
//  frc::Shuffleboard::GetTab("Teleoperated").GetLayout("Blue Commands").Add("Path To Amp", *(m_pathToBlueAmp.get())).WithPosition(0, 0);
//  frc::Shuffleboard::GetTab("Teleops").GetLayout("Blue Commands").Add("Speaker Top", ).WithPosition(0, 1);
//  frc::Shuffleboard::GetTab("Teleops").GetLayout("Blue Commands").Add("Speaker Mid", ).WithPosition(0, 2);
//  frc::Shuffleboard::GetTab("Teleops").GetLayout("Blue Commands").Add("Speaker Bottom", ).WithPosition(0, 3);

// This is another way to add a command to shuffleboard, but above is probably perferfered for commands that don't have variable parameters.
//  frc::Shuffleboard::GetTab("Teleops").GetLayout("Blue Commands").AddBoolean("Path To Amp", [this]{ return(m_pathToAmpButton); }).WithWidget(frc::BuiltInWidgets::kToggleButton).WithProperties({{"Label position", nt::Value::MakeString("HIDDEN")}}).WithPosition(0, 0);
//  frc2::NetworkButton(nt::NetworkTableInstance::GetDefault().GetBooleanTopic("/Shuffleboard/Teleops/Blue Commands/Path To Amp")).OnTrue( m_odometry.PathToPoseCommand(m_odometry.GetPoseFromAprilTagId(BLUE_AMP_TAG_ID, {0_m, -((kWheelBase/2)+0.25_m), frc::Rotation2d{90_deg}}), 0_mps, 0_m) );

  frc::Shuffleboard::GetTab("Teleoperated").Add("Field", m_field).WithProperties({{"robot_width", nt::Value::MakeDouble(kTrackWidth.value())}, {"robot_length", nt::Value::MakeDouble(kWheelBase.value())}}).WithSize(10, 5).WithPosition(2, 0);

//  frc::Shuffleboard::GetTab("Teleoperated").GetLayout("Red Commands", frc::BuiltInLayouts::kList).WithSize(2, 5).WithPosition(12, 0);
//  frc::Shuffleboard::GetTab("Teleoperated").GetLayout("Red Commands").Add("Path To Amp", *(m_pathToRedAmp.get())).WithPosition(0, 0);
//  frc::Shuffleboard::GetTab("Teleops").GetLayout("Red Commands").Add("Speaker Top", ).WithPosition(0, 1);
//  frc::Shuffleboard::GetTab("Teleops").GetLayout("Red Commands").Add("Speaker Mid", ).WithPosition(0, 2);
//  frc::Shuffleboard::GetTab("Teleops").GetLayout("Red Commands").Add("Speaker Bottom", ).WithPosition(0, 3);

  /* ****************************************************************** */
  /* ****************** Drive Subsystem ******************************* */
  /* ****************************************************************** */

  /* ** NOTE ** .WithSize() doesn't seem to do anything when in a       */
  /*            layout.  .WithPosition() does seem to set relative      */
  /*            position with in the layout (sometimes).                */
  frc::Shuffleboard::GetTab("Subsystems").GetLayout("Drive", frc::BuiltInLayouts::kList).WithSize(2, 8).WithPosition(0, 0);
  frc::Shuffleboard::GetTab("Subsystems").GetLayout("Drive").Add("Drive Subsystem", m_drive).WithPosition(0, 0);
  m_maxSpeedEntryPtr                 = frc::Shuffleboard::GetTab("Subsystems").GetLayout("Drive").Add("Maximum Speed", m_drive.GetMaxSpeed().value()).WithWidget(frc::BuiltInWidgets::kNumberSlider).WithProperties({{"min_value", nt::Value::MakeDouble(0.0)}, {"max_value", nt::Value::MakeDouble(kDefaultMaxSpeed.value())}}).WithPosition(0, 1).GetEntry();
  m_maxAngularSpeedEntryPtr          = frc::Shuffleboard::GetTab("Subsystems").GetLayout("Drive").Add("Maximum Angular Speed", units::degrees_per_second_t(m_drive.GetMaxAngularSpeed()).value()).WithWidget(frc::BuiltInWidgets::kNumberSlider).WithProperties({{"min_value", nt::Value::MakeDouble(0.0)}, {"max_value", nt::Value::MakeDouble(units::degrees_per_second_t(kDefaultMaxAngularSpeed).value())}}).WithPosition(0, 2).GetEntry();
  m_driveSpeedGovernorEntryPtr       = frc::Shuffleboard::GetTab("Subsystems").GetLayout("Drive").Add("Use Drive Speed Governor", true).WithWidget(frc::BuiltInWidgets::kToggleSwitch).WithPosition(0, 3).GetEntry();
  m_triggerBasedSpeedControlEntryPtr = frc::Shuffleboard::GetTab("Subsystems").GetLayout("Drive").Add("Use Trigger Speed Control", true).WithWidget(frc::BuiltInWidgets::kToggleSwitch).WithPosition(0, 3).GetEntry();
//  m_fieldRelativeStateEntryPtr       = frc::Shuffleboard::GetTab("Subsystems").GetLayout("Drive").Add("Field Relative", m_drive.GetFieldRelativeState()).WithWidget(frc::BuiltInWidgets::kToggleSwitch).WithPosition(0, 4).GetEntry();
  m_limitSlewRateEntryPtr            = frc::Shuffleboard::GetTab("Subsystems").GetLayout("Drive").Add("Limit Slew Rate", m_drive.GetLimitSlewRateState()).WithWidget(frc::BuiltInWidgets::kToggleSwitch).WithPosition(0, 5).GetEntry();
//  frc::Shuffleboard::GetTab("Subsystems").GetLayout("Drive").AddDouble("Gyro Heading", [this]{ return(std::fabs((std::fmod(m_drive.GetHeading().value(), 360.0)))); }).WithWidget(frc::BuiltInWidgets::kGyro).WithPosition(0, 6);
  frc::Shuffleboard::GetTab("Subsystems").GetLayout("Drive").Add("Gyro Heading", m_drive.GetGyro()).WithWidget(frc::BuiltInWidgets::kGyro).WithPosition(0, 6);

  /* Add a button to reset the gyro using a triggered network button    */
  /* command.  Note this clears the boolean after running the function  */
  /* to zero heading.                                                   */
  frc::Shuffleboard::GetTab("Subsystems").GetLayout("Drive").AddBoolean("Reset Gyro", [this]{ return(m_resetGyroButton); }).WithWidget(frc::BuiltInWidgets::kToggleButton).WithProperties({{"Label position", nt::Value::MakeString("HIDDEN")}}).WithPosition(3, 1);
  frc2::NetworkButton(nt::NetworkTableInstance::GetDefault().GetBooleanTopic("/Shuffleboard/Subsystems/Drive/Reset Gyro")).OnTrue(frc2::cmd::RunOnce([this] { m_drive.ZeroHeading(); m_resetGyroButton = false; }, {&m_drive}));

#ifdef USE_INTAKE
  /* ****************************************************************** */
  /* ****************** Intake Subsystem ****************************** */
  /* ****************************************************************** */

  frc::Shuffleboard::GetTab("Subsystems").GetLayout("Intake", frc::BuiltInLayouts::kList).WithSize(2, 6).WithPosition(2, 0);
  frc::Shuffleboard::GetTab("Subsystems").GetLayout("Intake").Add("Intake Subsystem", m_intake).WithPosition(0, 1);
  frc::Shuffleboard::GetTab("Subsystems").GetLayout("Intake").AddDouble("Setpoint Speed", [this]{ return(m_intake.GetSetpointSpeed().value()); }).WithWidget(frc::BuiltInWidgets::kNumberBar).WithProperties({{"min_value", nt::Value::MakeDouble(IntakeConstants::kIntakeMinimumSpeed.value())}, {"max_value", nt::Value::MakeDouble(IntakeConstants::kIntakeMaximumSpeed.value())}}).WithPosition(0, 2);
  frc::Shuffleboard::GetTab("Subsystems").GetLayout("Intake").AddDouble("Current Speed", [this]{ return(m_intake.GetSpeed().value()); }).WithWidget(frc::BuiltInWidgets::kNumberBar).WithProperties({{"min_value", nt::Value::MakeDouble(IntakeConstants::kIntakeMinimumSpeed.value())}, {"max_value", nt::Value::MakeDouble(IntakeConstants::kIntakeMaximumSpeed.value())}}).WithPosition(0, 3);
  frc::Shuffleboard::GetTab("Subsystems").GetLayout("Intake").AddBoolean("Is Note Detected", [this]{ return(m_intake.IsNoteDetected()); }).WithPosition(0, 4);
  m_allowIntakeControlEntryPtr = frc::Shuffleboard::GetTab("Subsystems").GetLayout("Intake").Add("Allow Dashboard Control", false).WithWidget(frc::BuiltInWidgets::kToggleSwitch).WithPosition(0, 5).GetEntry();
  m_intakeSpeedEntryPtr        = frc::Shuffleboard::GetTab("Subsystems").GetLayout("Intake").Add("Intake Speed", 0.0).WithWidget(frc::BuiltInWidgets::kNumberSlider).WithProperties({{"min_value", nt::Value::MakeDouble(IntakeConstants::kIntakeMinimumSpeed.value())}, {"max_value", nt::Value::MakeDouble(IntakeConstants::kIntakeMaximumSpeed.value())}}).WithPosition(0, 6).WithSize(2, 1).GetEntry();
  frc::Shuffleboard::GetTab("Subsystems").GetLayout("Intake").Add("Grab Note", *(m_grabNote.get())).WithProperties({{"show_type", nt::Value::MakeBoolean(false)}}).WithPosition(0, 7);
#endif

#ifdef USE_SHOOTER
  /* ****************************************************************** */
  /* ****************** Shooter Subsystem ***************************** */
  /* ****************************************************************** */

  frc::Shuffleboard::GetTab("Subsystems").GetLayout("Shooter", frc::BuiltInLayouts::kList).WithSize(2, 6).WithPosition(4, 0);
  frc::Shuffleboard::GetTab("Subsystems").GetLayout("Shooter").Add("Shooter Subsystem", m_shooter).WithPosition(0, 0);
  frc::Shuffleboard::GetTab("Subsystems").GetLayout("Shooter").AddDouble("Left Measured Speed", [this]{ return(m_shooter.GetCurrentLeftSpeed().value()); }).WithWidget(frc::BuiltInWidgets::kNumberBar).WithProperties({{"min_value", nt::Value::MakeDouble(ShooterConstants::kShooterMinimumSpeed.value())}, {"max_value", nt::Value::MakeDouble(ShooterConstants::kShooterMaximumSpeed.value())}, {"divisions", nt::Value::MakeInteger(5)}}).WithPosition(0, 1);
  frc::Shuffleboard::GetTab("Subsystems").GetLayout("Shooter").AddDouble("Right Measured Speed", [this]{ return(m_shooter.GetCurrentRightSpeed().value()); }).WithWidget(frc::BuiltInWidgets::kNumberBar).WithProperties({{"min_value", nt::Value::MakeDouble(ShooterConstants::kShooterMinimumSpeed.value())}, {"max_value", nt::Value::MakeDouble(ShooterConstants::kShooterMaximumSpeed.value())}, {"divisions", nt::Value::MakeInteger(5)}}).WithPosition(0, 2);
  m_allowShooterControlEntryPtr = frc::Shuffleboard::GetTab("Subsystems").GetLayout("Shooter").Add("Allow Dashboard Control", false).WithWidget(frc::BuiltInWidgets::kToggleSwitch).WithPosition(0, 3).GetEntry();

  m_shooterLeftSpeedEntryPtr  = frc::Shuffleboard::GetTab("Subsystems").GetLayout("Shooter").Add("Left Set Speed", 0.0).WithWidget(frc::BuiltInWidgets::kNumberSlider).WithProperties({{"min_value", nt::Value::MakeDouble(ShooterConstants::kShooterMinimumSpeed.value())}, {"max_value", nt::Value::MakeDouble(ShooterConstants::kShooterMaximumSpeed.value())}}).WithPosition(0, 4).GetEntry();
  m_shooterRightSpeedEntryPtr = frc::Shuffleboard::GetTab("Subsystems").GetLayout("Shooter").Add("Right Set Speed", 0.0).WithWidget(frc::BuiltInWidgets::kNumberSlider).WithProperties({{"min_value", nt::Value::MakeDouble(ShooterConstants::kShooterMinimumSpeed.value())}, {"max_value", nt::Value::MakeDouble(ShooterConstants::kShooterMaximumSpeed.value())}}).WithPosition(0, 5).GetEntry();

  frc::Shuffleboard::GetTab("Subsystems").GetLayout("Shooter").Add("Slow Remove Note", *(m_slowRemoveNote.get())).WithProperties({{"show_type", nt::Value::MakeBoolean(false)}}).WithPosition(0, 7);
#endif

#ifdef USE_ARM
  frc::Shuffleboard::GetTab("Subsystems").GetLayout("Arm", frc::BuiltInLayouts::kList).WithSize(2, 6).WithPosition(6, 0);
  frc::Shuffleboard::GetTab("Subsystems").GetLayout("Arm").Add("Arm Subsystem", m_arm).WithPosition(0, 0);

  frc::Shuffleboard::GetTab("Subsystems").GetLayout("Arm").Add("Arm Up", *(m_armUp.get())).WithProperties({{"show_type", nt::Value::MakeBoolean(false)}}).WithPosition(0, 1);
  frc::Shuffleboard::GetTab("Subsystems").GetLayout("Arm").Add("Arm Down", *(m_armDown.get())).WithProperties({{"show_type", nt::Value::MakeBoolean(false)}}).WithPosition(0, 2);

  frc::Shuffleboard::GetTab("Subsystems").GetLayout("Arm").AddDouble("Measured Angle", [this]{ return(m_arm.GetArmAngle().value()); }).WithWidget(frc::BuiltInWidgets::kNumberBar).WithProperties({{"min_value", nt::Value::MakeDouble(ArmConstants::kArmMinimumAngle.value())}, {"max_value", nt::Value::MakeDouble(ArmConstants::kArmMaximumAngle.value())}, {"divisions", nt::Value::MakeInteger(5)}}).WithPosition(0, 3);
  m_allowArmControlEntryPtr = frc::Shuffleboard::GetTab("Subsystems").GetLayout("Arm").Add("Allow Dashboard Control", false).WithWidget(frc::BuiltInWidgets::kToggleSwitch).WithPosition(0, 4).GetEntry();
  m_armPositionEntryPtr     = frc::Shuffleboard::GetTab("Subsystems").GetLayout("Arm").Add("Set Angle", 0.0).WithWidget(frc::BuiltInWidgets::kNumberSlider).WithProperties({{"min_value", nt::Value::MakeDouble(ArmConstants::kArmMinimumAngle.value())}, {"max_value", nt::Value::MakeDouble(ArmConstants::kArmMaximumAngle.value())}}).WithPosition(0, 5).WithSize(2, 1).GetEntry();

  /* Add a button to reset the arm encoder position using a triggered   */
  /* network button command.  Note this clears the boolean after        */
  /* running the function to reset the arm encoder position.            */
  frc::Shuffleboard::GetTab("Subsystems").GetLayout("Arm").AddBoolean("Disable Arm", [this]{ return(m_disableArmButton); }).WithWidget(frc::BuiltInWidgets::kToggleButton).WithProperties({{"Label position", nt::Value::MakeString("HIDDEN")}}).WithPosition(3, 1);
  frc2::NetworkButton(nt::NetworkTableInstance::GetDefault().GetBooleanTopic("/Shuffleboard/Subsystems/Arm/Disable Arm")).OnTrue(frc2::cmd::RunOnce([this] { m_arm.DisableArm(); m_disableArmButton = false; }, {&m_arm}));
#endif

  /* Change the Tab to automatically start on the subsystems tab.       */
  frc::Shuffleboard::SelectTab("Subsystems");

#if 1
  /* Create a command to be used as the default command for the drive   */
  /* subsystem.  The left stick controls translation of the robot.      */
  /* Turning is controlled by the X axis of the right stick.            */
//xxx should we get rid of the rate limitting in the DriveSubsystems and use control rate limiting instead?
  frc2::CommandPtr JoystickDriveCommand = frc2::RunCommand(
                                                            [this] {
                                                                     m_drive.Drive(
                                                                                    -units::meters_per_second_t{frc::ApplyDeadband(m_driverController.GetLeftY(), OIConstants::kDriveDeadband)},
                                                                                    -units::meters_per_second_t{frc::ApplyDeadband(m_driverController.GetLeftX(), OIConstants::kDriveDeadband)},
                                                                                    -units::radians_per_second_t{frc::ApplyDeadband(m_driverController.GetRightX(), OIConstants::kDriveDeadband)}
                                                                                  );
                                                                   },
                                                                   {&m_drive}).ToPtr();

  /* Give the command a user understandable name.                       */
  JoystickDriveCommand.get()->SetName("Joystick Drive Default Command");

  // Set up default drive command
  m_drive.SetDefaultCommand(std::move(JoystickDriveCommand));
#else
  // Set up the default drive command.
  m_drive.SetDefaultCommand(std::move(DriveFieldRelativeWithAngularVelocityCommand(
                                                                                   &m_drive,
                                                                                   &m_odometry,
                                                                                   [this]{ return(-units::meters_per_second_t{frc::ApplyDeadband(m_driverController.GetLeftY(), OIConstants::kDriveDeadband)}); },
                                                                                   [this]{ return(-units::meters_per_second_t{frc::ApplyDeadband(m_driverController.GetLeftX(), OIConstants::kDriveDeadband)}); },
                                                                                   [this]{ return(-units::meters_per_second_t{frc::ApplyDeadband(m_driverController.GetRightX(), OIConstants::kDriveDeadband)}); }
                                                                                  ).ToPtr()));
#endif
}

void RobotContainer::ConfigureButtonBindings()
{
   /* Set the B button to set the wheels in X position to stop          */
   /* quickly.                                                          */
   m_driverController.B().WhileTrue(frc2::cmd::Run([this] { m_drive.SetX(); }, {&m_drive}));

   /* Set the X button to zero the gyro heading.                        */
   m_driverController.X().OnTrue(frc2::cmd::RunOnce([this] { m_drive.ZeroHeading(); }, {&m_drive}));

#ifdef USE_INTAKE
   /* Set the Right Trigger to run the intake until a note is grabbed or*/
   /* the right trigger is released.                                    */
   m_driverController.RightTrigger(0.75).WhileTrue(m_intake.GrabNoteCommand(9000_rpm));
#endif

#ifdef USE_SHOOTER
#if 1
   /* Set the Right Bumper to run the shooter while the right bumper is */
   /* pressed then stop the shooter once the bumper is released.        */
   m_driverController.RightBumper().WhileTrue(std::move(ShootNoteFromIntakeCommand(
                                                                                  &m_intake,      // intake subsystem pointer
                                                                                  9500_rpm,       // intake output to shooter speed (rpm)
                                                                                  &m_shooter,     // shooter subsystem pointer
                                                                                  4875_rpm,       // shooter left flywheel speed (rpm)
                                                                                  4375_rpm,       // shooter right flywheel speed (rpm
                                                                                  3.0_s,          // shooter flywheel spinup timeout
                                                                                  0.750_s         // after intake enabled time till complete
                                                                                  ).ToPtr()));

   /* Set the Left Bumper to run the shooter while the left bumper is   */
   /* pressed then stop the shooter once the bumper is released.        */
   m_driverController.LeftBumper().WhileTrue(std::move(ShootNoteFromIntakeCommand(
                                                                                  &m_intake,      // intake subsystem pointer
                                                                                  9500_rpm,       // intake output to shooter speed (rpm)
                                                                                  &m_shooter,     // shooter subsystem pointer
                                                                                  2000_rpm,       // shooter left flywheel speed (rpm)
                                                                                  2000_rpm,       // shooter right flywheel speed (rpm
                                                                                  2_s,            // shooter flywheel spinup timeout
                                                                                  0.750_s         // after intake enabled time till complete
                                                                                  ).ToPtr()));
#else
   /* Set the Right Bumper to run the shooter while the right bumper is */
   /* pressed then stop the shooter once the bumper is released.        */
   m_driverController.RightBumper().WhileTrue(std::move(ShootNoteFromIntakeByArmPositionCommand(
                                                                                                &m_intake,      // intake subsystem pointer
                                                                                                9500_rpm,       // intake output to shooter speed (rpm)
                                                                                                &m_shooter,     // shooter subsystem pointer
                                                                                                4875_rpm,       // shooter left flywheel speed (rpm) when arm is in down position
                                                                                                4375_rpm,       // shooter right flywheel speed (rpm) when arm is in down position
                                                                                                2000_rpm,       // shooter left flywheel speed (rpm) when arm is in up position
                                                                                                2000_rpm,       // shooter right flywheel speed (rpm) when arm is in up position
                                                                                                3.0_s,          // shooter flywheel spinup timeout
                                                                                                0.750_s,        // after intake enabled time till complete
                                                                                                &m_arm          // arm subsystem pointer
                                                                                                ).ToPtr()));

   /* Set the Left Bummper to toggle between Field Relative and None    */
   /* Field Relative states.                                            */
   m_driverController.LeftBumper().OnTrue(frc2::cmd::RunOnce([this] { m_drive.SetFieldRelativeState(false); }, {&m_drive}))
                                  .OnFalse(frc2::cmd::RunOnce([this] { m_drive.SetFieldRelativeState(true); }, {&m_drive}));
#endif
#endif

#ifdef USE_OPERATOR_CONTROLLER
#ifdef USE_ARM
   /* Set the "POV" D-pad up button to command the arm to move up.       */
   frc2::POVButton(&m_operatorController, 0).WhileTrue(m_arm.ArmUpCommand());
   frc2::POVButton(&m_operatorController, 45).WhileTrue(m_arm.ArmUpCommand());
   frc2::POVButton(&m_operatorController, 315).WhileTrue(m_arm.ArmUpCommand());

   /* Set the "POV" D-pad down button to command the arm to move down.   */
   frc2::POVButton(&m_operatorController, 180).WhileTrue(m_arm.ArmDownCommand());
   frc2::POVButton(&m_operatorController, 135).WhileTrue(m_arm.ArmDownCommand());
   frc2::POVButton(&m_operatorController, 225).WhileTrue(m_arm.ArmDownCommand());
#endif
#ifdef USE_INTAKE
//xxx add some command for in and out the intake to possible fix stuck notes..
#endif
#endif
}

   // The following function is responsible for getting the currently
   // selected autonomous command.
frc2::CommandPtr RobotContainer::GetAutonomousCommand()
{
   /* Get the command factory function that is currently selected with  */
   /* the sendable chooser.                                             */
   std::function<frc2::CommandPtr()> commandFactory = m_chooser.GetSelected();

   /* Get the selected command.                                         */
   return(commandFactory());
}

  /* The following function is responsible for logging the power        */
  /* distribution.                                                      */
void RobotContainer::LogPowerDistribution(void)
{
   double pdpVoltage;
   double pdpCurrent;

   /* Get the instantanous voltage and current.                         */
   pdpVoltage = m_PDP.GetVoltage();
   pdpCurrent = m_PDP.GetTotalCurrent();

   /* Add the current instantaneous power to the total accumulated power*/
   /* used.                                                             */
   m_totalPower = m_totalPower + (pdpVoltage*pdpCurrent);

   /* Log the current state of the power distribution panel             */
   m_pdpVoltagePublisher.Set(pdpVoltage);
   m_pdpCurrentPublisher.Set(pdpCurrent);
   m_pdpPowerPublisher.Set((pdpVoltage*pdpCurrent));
   m_pdpTotalPowerPublisher.Set(m_totalPower);
}

   /* The following function is responsible for pumping changes made in */
   /* shuffle board to pass to the various subsystems.                  */
void RobotContainer::PumpShuffleBoard(void)
{
   double tempValue;
   double leftTriggerValue;

   /* Check to see if the drive speed governor is currently active.     */
   if(!m_driveGovernorActive)
   {
      /* The drive speed governor is not currently active.              */

      /* Get the current state of the trigger based speed control       */
      /* switch.                                                        */
      if(m_triggerBasedSpeedControlEntryPtr->GetBoolean(false))
      {
         /* The trigger based speed control switch indicates we need to */
         /* use the triggers for speed control.                         */

         /* Get the current left trigger axis value.                    */
         leftTriggerValue = m_driverController.GetLeftTriggerAxis();

         /* Take the default maximum speed and subtract off a minimum   */
         /* speed and multiply this by the left trigger value (acts as a*/
         /* percentage).                                                */
         tempValue = 1.0 /*mps*/ + ((1.0 - leftTriggerValue) * (kDefaultMaxSpeed - 1_mps).value());

         /* Set the max speed to drive and update the widget on the     */
         /* shuffle board with this value.                              */
         m_drive.SetMaxSpeed(units::meters_per_second_t{tempValue});
         m_maxSpeedEntryPtr->SetDouble(tempValue);

         /* Take the default maximum angular speed and subtract off a   */
         /* minimum angular speed and multiply this by the left trigger */
         /* value (acts as a percentage).                               */
         tempValue = units::radians_per_second_t{120_deg_per_s}.value() + (1.0 - leftTriggerValue) * (kDefaultMaxAngularSpeed - units::radians_per_second_t{45_deg_per_s}).value();

         /* Set the max speed to drive and update the widget on the     */
         /* shuffle board with this value.                              */
         m_drive.SetMaxAngularSpeed(units::radians_per_second_t{tempValue});
         m_maxAngularSpeedEntryPtr->SetDouble(units::degrees_per_second_t(units::radians_per_second_t{tempValue}).value());
      }
      else
      {
         /* The trigger based speed control switch indicates we need to */
         /* use the number sliders for speed control.  Set the speeds   */
         /* based on current number slider values.                      */
         m_drive.SetMaxSpeed(units::meters_per_second_t{m_maxSpeedEntryPtr->GetDouble(kDefaultMaxSpeed.value())});
         m_drive.SetMaxAngularSpeed(units::radians_per_second_t{units::degrees_per_second_t(m_maxAngularSpeedEntryPtr->GetDouble(kDefaultMaxAngularSpeed.value()))});
      }
   }

   /* Update the field relative state and the limit slew rate state     */
   /* based on the current position of the shuffle board switches.      */
//   m_drive.SetFieldRelativeState(m_fieldRelativeStateEntryPtr->GetBoolean(true));
   m_drive.SetLimitSlewRateState(m_limitSlewRateEntryPtr->GetBoolean(true));

#ifdef USE_INTAKE
   /* Get the current state of the allow intake speed control switch.   */
   if(m_allowIntakeControlEntryPtr->GetBoolean(false))
   {
      /* The allow intake speed control switch is enabled.              */

      /* In this case get the current intake speed slider value and set */
      /* the intake to run at this speed.                               */
      m_intake.RunIntake(units::revolutions_per_minute_t(m_intakeSpeedEntryPtr->GetDouble(0.0)));

      /* Set the last allow intake control state to true.               */
      m_lastAllowIntakeControlState = true;
   }
   else
   {
      /* The allow intake speed control switch is disabled.             */

      /* In this case ignore the value on the slider and force it to    */
      /* zero.                                                          */
      m_intakeSpeedEntryPtr->SetDouble(0.0);

      /* Check to see if the last allow intake control state is true.   */
      if(m_lastAllowIntakeControlState)
      {
         /* The last state of the intake control was true and is now    */
         /* false.  In this case make sure the intake is stopped.       */
         m_intake.StopIntake();
      }

      /* Set the last allow intake control state to false.              */
      m_lastAllowIntakeControlState = false;
   }
#endif

#ifdef USE_SHOOTER
   /* Get the current state of the allow shooter speed control switch.  */
   if(m_allowShooterControlEntryPtr->GetBoolean(false))
   {
      /* The allow shooter speed control switch is enabled.             */

      /* In this case get the current shooter speed slider values and   */
      /* set the shooter to run at this speed.                          */
      m_shooter.RunShooter(units::revolutions_per_minute_t(m_shooterLeftSpeedEntryPtr->GetDouble(0.0)),
                           units::revolutions_per_minute_t(m_shooterRightSpeedEntryPtr->GetDouble(0.0))
                           );

      /* Set the last allow shooter control state to true.              */
      m_lastAllowShooterControlState = true;
   }
   else
   {
      /* The allow shooter speed control switch is disabled.            */

      /* In this case ignore the value on the slider instead have this  */
      /* slider display the set point for the motors.                   */
      m_shooterLeftSpeedEntryPtr->SetDouble(m_shooter.GetLeftSetpointSpeed().value());
      m_shooterRightSpeedEntryPtr->SetDouble(m_shooter.GetRightSetpointSpeed().value());

      /* Check to see if the last allow shooter control state is true.  */
      if(m_lastAllowShooterControlState)
      {
         /* The last state of the shooter control was true and is now   */
         /* false.  In this case make sure the shooter is stopped.      */
         m_shooter.StopShooter();
      }

      /* Set the last allow shooter control state to false.             */
      m_lastAllowShooterControlState = false;
   }
#endif

#ifdef USE_ARM
   /* Get the current state of the allow arm position control switch.   */
   if(m_allowArmControlEntryPtr->GetBoolean(false))
   {
      /* The allow arm position control switch is enabled.              */

      /* Get the current arm setpoint from the dashboard.               */
      tempValue = m_armPositionEntryPtr->GetDouble(0.0);

      /* Check to see if the value read from the dashboard is differnt  */
      /* than the last set point specified via the dashboard.           */
      if(m_armLastDashboardSetPoint != tempValue)
      {
         /* The new setpoint is different the previously stored last    */
         /* setpoint value.  Move to this angle as the new goal.        */
         m_arm.SetArmPosition(units::degree_t{tempValue});

         /* Save the new set point value to the last dashboard setpoint */
         /* value.                                                      */
         m_armLastDashboardSetPoint = tempValue;
      }

      /* Set the last allow arm control state to true.                  */
      m_lastAllowArmControlState = true;
   }
   else
   {
      /* The allow arm position control switch is disabled.             */

      /* Check to see if the last allow arm control state is true.      */
      if(m_lastAllowArmControlState)
      {
         /* The last state of the arm control was true and is now false.*/
         /* In this case make sure the arm stops and stays at the       */
         /* current location.                                           */
         m_arm.SetArmPosition(m_arm.GetArmAngle());

         /* Invalidate the last dashboard setpoint that was stored.     */
         m_armLastDashboardSetPoint = std::numeric_limits<double>::quiet_NaN();
      }

      /* Set the last allow arm control state to false.                 */
      m_lastAllowArmControlState = false;
   }
#endif

   /* Now check to see if setting the pose is currently allowed or if we*/
   /* are just going to display the current pose.                       */
   if(m_allowSetPoseEntryPtr->GetBoolean(false))
   {
      /* Setting the pose is currently allowed.                         */

      /* In this case the pose will be settable via the numbers sliders */
      /* the value will be set when the "Set Pose" button is set.       */
   }
   else
   {
      /* Set the pose is not currently allowed.  In this case just get  */
      /* the current pose and set the sliders to the current pose value.*/
      m_SetPoseXEntryPtr->SetDouble(m_odometry.GetPose().X().value());
      m_SetPoseYEntryPtr->SetDouble(m_odometry.GetPose().Y().value());
   }

//xxx old way of updating the robot position on the game field..  delete me when we see the new way is working.
//xxx new way passes the field object by refernce to the odometry so it can update the position and add things like target pose and trajectory..
#if 0
   /* Get the current position of the robot and update the position with*/
   /* in the field object on the shuffleboard.                          */
   m_field.SetRobotPose(m_odometry.GetPose());
#endif
}

   /* The following function is responsible for disabling rumble.       */
void RobotContainer::DisableRumble(void)
{
   /* Stop rumbling.                                                     */
   m_driverController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);

   /* The display is set to be synchronized with the rumble function for*/
   /* note indication.  Set the display to off.                         */
   m_display.DisplayOff();

}

  /* The following function is responsible for pumping the logic used to*/
  /* detect events that will enable and disable the rumble function of  */
  /* the controller.  This should be called during the robots period    */
  /* loop.                                                              */
void RobotContainer::PumpRumble(void)
{
   /* Check to see if there is currently a NOTE in the intake.          */
   if(m_intake.IsNoteDetected())
   {
      /* There is currently a NOTE in the intake.                       */

      /* Adjust the rumble duty cycle count.                            */
      m_rumbleDutyCycleCount = m_rumbleDutyCycleCount - 0.02_s;

      /* Check to see if the rumble duty cycle counter has expired.     */
      if(m_rumbleDutyCycleCount <= 0_s)
      {
         /* The rumble duty cycle counter has expired.  Check to see if */
         /* rumble is currently active.                                 */
         if(m_rumbleState)
         {
            /* Rumble is currently active.  Stop rumbling.              */
            m_driverController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);

            /* Reset the rumble duty cycle counter using the rumble off */
            /* time.                                                    */
            m_rumbleDutyCycleCount = OIConstants::kDriverRumbleOffTime;

            /* Set the rumble state to indicate that the rumble is no   */
            /* longer active.                                           */
            m_rumbleState = false;

            /* There is a note in the intake to indicate this we will   */
            /* toggle between 2 colors.  Set the display to "yellow".   */
            m_display.DisplayOn(frc::Color8Bit{128, 111, 0});

         }
         else
         {
            /* Rumble is not currently active.  Start rumbling.         */
            m_driverController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, OIConstants::kDriverRumble);

            /* Reset the rumble duty cycle counter using the rumble on  */
            /* time.                                                    */
            m_rumbleDutyCycleCount = OIConstants::kDriverRumbleOnTime;

            /* Set the rumble state to indicate that the rumble is      */
            /* currently active.                                        */
            m_rumbleState = true;

            /* There is a note in the intake to indicate this we will   */
            /* toggle between 2 colors.  Set the display to "blue".     */
            m_display.DisplayOn(frc::Color8Bit{0, 0, 128});
         }
      }
   }
   else
   {
      /* There is not currently a NOTE in the intake.  Disable rumble.  */
      m_driverController.SetRumble(frc::GenericHID::RumbleType::kBothRumble, 0.0);

      /* There isn't a note in the intake.  Turn the display off.       */
      m_display.DisplayOff();

      /* Reset the rumble duty cycle counter.                           */
      m_rumbleDutyCycleCount = 0_s;

      /* Set the rumble state to indicate that the rumble is no longer  */
      /* active.                                                        */
      m_rumbleState = false;
   }
}

  /* The following function is responsible for pumping the drive speed  */
  /* governor.  The goal of this function is to watch the current angle */
  /* of the arm and if it is not currently in the down position to      */
  /* automatically slow the maximum speed allowable for driving.        */
void RobotContainer::PumpDriveGovernor(void)
{
#ifdef USE_ARM
   /* Check to see if the drive speed governor is currently enabled.    */
   if(m_driveSpeedGovernorEntryPtr->GetBoolean(false))
   {
      /* The drive speed governor is currently enabled.                 */

      /* Get the current angle of the arm and see if it is over the     */
      /* minimum angle to activate the drive governor.                  */
      if(m_arm.GetArmAngle() > kDriveGovernorArmActiveAngle)
      {
         /* The arm angle is greater that the amount to activate the    */
         /* drive speed governor.                                       */

         /* Set the maximum speed for driving to being the safe speed   */
         /* for when the arm is up.                                     */
         m_drive.SetMaxSpeed(kDriveGovernorMaxSpeed);
         m_maxSpeedEntryPtr->SetDouble(kDriveGovernorMaxSpeed.value());
         m_drive.SetMaxAngularSpeed(kDriveGovernorMaxAngularSpeed);
         m_maxAngularSpeedEntryPtr->SetDouble(kDriveGovernorMaxAngularSpeed.value());

         /* Set the flag indicating that the drive speed governor is    */
         /* currently active.                                           */
         m_driveGovernorActive = true;
      }
      else
      {
         /* The arm is under the minimum angle for the drive governor to*/
         /* be active.                                                  */

         /* Set the drive governor active boolean to indicate it is not */
         /* active.                                                     */
         m_driveGovernorActive = false;
      }
   }
#endif
}

  /* Static command factory used to retrieve autonoumous paths from the  */
  /* sendable chooser built with Path Planner.                           */
frc2::CommandPtr RobotContainer::PathPlannerCommandFactory(std::string autoName) noexcept
{
  return(pathplanner::PathPlannerAuto(autoName).ToPtr());
}
