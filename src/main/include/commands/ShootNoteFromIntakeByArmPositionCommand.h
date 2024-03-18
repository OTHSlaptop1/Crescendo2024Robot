// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <units/angular_velocity.h>
#include <units/time.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/Arm2Subsystem.h"

/**
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ShootNoteFromIntakeByArmPositionCommand : public frc2::CommandHelper<frc2::Command, ShootNoteFromIntakeByArmPositionCommand>
{
 public:

  ShootNoteFromIntakeByArmPositionCommand(IntakeSubsystem *intakeSubsystem, units::revolutions_per_minute_t intakeOutputSpeed, ShooterSubsystem *shooterSubsystem, units::revolutions_per_minute_t armDownLeftTargetSpeed, units::revolutions_per_minute_t armDownRightTargetSpeed, units::revolutions_per_minute_t armUpLeftTargetSpeed, units::revolutions_per_minute_t armUpRightTargetSpeed, units::second_t shooterSpinUpTimeout, units::second_t launchTimeout, Arm2Subsystem *armSubsystem);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:

   /* The following enumerated type defines all of the possible commands*/
   /* states.                                                           */
   typedef enum
   {
      csWaitingForShooterSpinUp,
      csWaitingForLaunchTimeout,
      csCommandOver
   } CommandState_t;

    /* The following variables are pointer to the various subsystem to  */
    /* be used by this command.                                         */
    IntakeSubsystem  *m_intakeSubsystem;
    ShooterSubsystem *m_shooterSubsystem;
    Arm2Subsystem    *m_armSubsystem;

    /* The following variables holds the desired target speeds for the  */
    /* left and right flywheels in the shooter if the arm is in the down*/
    /* position.                                                        */
    units::revolutions_per_minute_t m_armDownLeftTargetSpeed;
    units::revolutions_per_minute_t m_armDownRightTargetSpeed;

    /* The following variables holds the desired target speeds for the  */
    /* left and right flywheels in the shooter if the arm is in the up  */
    /* position.                                                        */
    units::revolutions_per_minute_t m_armUpLeftTargetSpeed;
    units::revolutions_per_minute_t m_armUpRightTargetSpeed;

    /* The following variables holds the target speeds to used for the  */
    /* left and right flywheels in the shooter.  These are the actual   */
    /* speed the shooter will spin up to based on the arm position.     */
    units::revolutions_per_minute_t m_leftTargetSpeed;
    units::revolutions_per_minute_t m_rightTargetSpeed;

    /* The following variable holds the speed in which to run the intake*/
    /* when passing the note into the shooter flywheels.                */
    units::revolutions_per_minute_t m_intakeOutputSpeed;

    /* The following variable holds the amount of time to give the      */
    /* shooter to spin up before giving up and forcing the launch.      */
    units::second_t m_shooterSpinUpTimeout;

    /* The following variable holds the amount of time after enabling   */
    /* the intake to push the note into the shooter till the command is */
    /* considered finish.                                               */
    units::second_t m_launchTimeout;

    /* The following variable holds the current command state.          */
    CommandState_t m_commandState;

    /* The following variable holds the current state time.             */
    units::second_t m_stateTimer;
};
