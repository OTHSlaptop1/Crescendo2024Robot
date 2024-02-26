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

   /* The following enumerated type defines all of the possible commands*/
   /* states.                                                           */
typedef enum
{
   csWaitingForShooterSpinUp,
   csWaitingForLaunchTimeout,
   csCommandOver
} CommandState_t;

/**
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ShootNoteFromIntakeCommand : public frc2::CommandHelper<frc2::Command, ShootNoteFromIntakeCommand>
{
 public:
  ShootNoteFromIntakeCommand(IntakeSubsystem *intakeSubsystem, units::revolutions_per_minute_t intakeOutputSpeed, ShooterSubsystem *shooterSubsystem, units::revolutions_per_minute_t leftTargetSpeed, units::revolutions_per_minute_t rightTargetSpeed, units::second_t shooterSpinUpTimeout, units::second_t launchTimeout);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
    /* The following variables are pointer to ths intake and shooter    */
    /* subsystems to be used by this command.                           */
    IntakeSubsystem  *m_intakeSubsystem;
    ShooterSubsystem *m_shooterSubsystem;

    /* The following variable holds the desired target speeds for the   */
    /* left and right flywheels in the shooter.                         */
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
