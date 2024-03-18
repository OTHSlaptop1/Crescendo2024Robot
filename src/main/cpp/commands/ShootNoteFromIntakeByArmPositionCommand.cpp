// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ShootNoteFromIntakeByArmPositionCommand.h"

#include <units/angular_velocity.h>
#include <units/time.h>

#include "subsystems/IntakeSubsystem.h"
#include "subsystems/ShooterSubsystem.h"
#include "subsystems/Arm2Subsystem.h"

   /* The following constant defines the period in which the execute    */
   /* command is called.                                                */
#define EXECUTE_PERIOD_IN_SECONDS                                    (0.020_s) // 20 ms, 50 Hz default time

   /* The following constant defines the threhold percentage of the     */
   /* targeted speed that the flywheels must attain to be considered at */
   /* speed.                                                            */
#define FLYWHEEL_THRESHOLD_PERCENT                                   (0.95)

   /* The following constant defines the threshold angle for the ARM in */
   /* which to use the ARM Up fly wheel speeds.                         */
#define ARM_UP_THRESHOLD_ANGLE                                       (60.0_deg)

ShootNoteFromIntakeByArmPositionCommand::ShootNoteFromIntakeByArmPositionCommand(IntakeSubsystem *intakeSubsystem, units::revolutions_per_minute_t intakeOutputSpeed, ShooterSubsystem *shooterSubsystem, units::revolutions_per_minute_t armDownLeftTargetSpeed, units::revolutions_per_minute_t armDownRightTargetSpeed, units::revolutions_per_minute_t armUpLeftTargetSpeed, units::revolutions_per_minute_t armUpRightTargetSpeed, units::second_t shooterSpinUpTimeout, units::second_t launchTimeout, Arm2Subsystem *armSubsystem)
{
   /* The requirements for this command.  This command uses the intake  */
   /* subsystem and the shooter subsystem so add them to the            */
   /* requirements list.                                                */
   AddRequirements(intakeSubsystem);
   AddRequirements(shooterSubsystem);

   /* ** NOTE ** Since we are only reading the Arm Subsystems Angle, I  */
   /*            am not considering it a requirement since it doesn't   */
   /*            use the motors.                                        */


   /* Give this command a user readable name.                           */
   this->SetName("ShootNoteFromIntakeByArmPositionCommand()");

   /* Save the pointer to the required subsystems for later use.        */
   m_intakeSubsystem  = intakeSubsystem;
   m_shooterSubsystem = shooterSubsystem;
   m_armSubsystem     = armSubsystem;

   /* Save the target speeds for the left and right flywheels in the    */
   /* shooter for when the arm is in the down position.                 */
   m_armDownLeftTargetSpeed  = armDownLeftTargetSpeed;
   m_armDownRightTargetSpeed = armDownRightTargetSpeed;

   /* Save the target speeds for the left and right flywheels in the    */
   /* shooter for when the arm is in the up position.                   */
   m_armUpLeftTargetSpeed  = armUpLeftTargetSpeed;
   m_armUpRightTargetSpeed = armUpRightTargetSpeed;

   /* Initialize the target speeds to a known value (in this case       */
   /* default to the down speeds).                                      */
   m_leftTargetSpeed  = armDownLeftTargetSpeed;
   m_rightTargetSpeed = armDownRightTargetSpeed;

   /* Save the intake output speed for use when running the intake.     */
   m_intakeOutputSpeed = intakeOutputSpeed;

   /* Save the timeout values to use while executing the command.       */
   m_shooterSpinUpTimeout = shooterSpinUpTimeout;
   m_launchTimeout        = launchTimeout;
}

// Called when the command is initially scheduled.
void ShootNoteFromIntakeByArmPositionCommand::Initialize()
{
   /* Initialize the command state to indicate we are waiting for the   */
   /* shooter flywheels to spin up to speed.                            */
   m_commandState = csWaitingForShooterSpinUp;

   /* Reset the state timer.                                            */
   m_stateTimer = 0_s;

   /* Next determine the current arm position in order to decide on     */
   /* which speed set to use for the shooter.                           */
   if(m_armSubsystem->GetArmAngle() >= ARM_UP_THRESHOLD_ANGLE)
   {
      /* The arm appears to be in the up position.  Set the left and    */
      /* right target speeds to the Arm Up speeds.                      */
      m_leftTargetSpeed  = m_armUpLeftTargetSpeed;
      m_rightTargetSpeed = m_armUpRightTargetSpeed;
   }
   else
   {
      /* The arm appears to be in the down position.  Set the left and  */
      /* right target speeds to the Arm down speeds.                    */
      m_leftTargetSpeed  = m_armDownLeftTargetSpeed;
      m_rightTargetSpeed = m_armDownRightTargetSpeed;
   }

   /* Start the shooters flywheels using the speeds specified in the    */
   /* command.                                                          */
   m_shooterSubsystem->RunShooter(m_leftTargetSpeed, m_rightTargetSpeed);
}

// Called repeatedly when this Command is scheduled to run
void ShootNoteFromIntakeByArmPositionCommand::Execute()
{
   /* Determine the current command state.                              */
   switch(m_commandState)
   {
      case csWaitingForShooterSpinUp:
         /* Currently waiting for the shooter to spin up.               */

         /* Adjust the state timer by the execute period.               */
         m_stateTimer = m_stateTimer + EXECUTE_PERIOD_IN_SECONDS;

         /* Now check to see if the shooter spin up timerout has        */
         /* expired.                                                    */
         if(m_stateTimer <= m_shooterSpinUpTimeout)
         {
            /* The shooter spin up timeout has not expired.             */

            /* Get the current speed for both of the flywheels.         */
            units::revolutions_per_minute_t leftFlywheelSpeed  = m_shooterSubsystem->GetCurrentLeftSpeed();
            units::revolutions_per_minute_t rightFlywheelSpeed = m_shooterSubsystem->GetCurrentRightSpeed();

            /* Determine if the flywheels have reached their targets    */
            /* speed with in a set threshold.                           */
            if((leftFlywheelSpeed > (m_leftTargetSpeed * FLYWHEEL_THRESHOLD_PERCENT)) && (rightFlywheelSpeed > (m_rightTargetSpeed * FLYWHEEL_THRESHOLD_PERCENT)))
            {
               /* The flywheels have reached there target speed.        */

               /* Reset the state timer.                                */
               m_stateTimer = 0_s;

               /* Set the state to the next state in the state machine. */
               m_commandState = csWaitingForLaunchTimeout;

               /* Start the intake to push the note into the shooters   */
               /* flywheels.                                            */
               m_intakeSubsystem->RunIntake(m_intakeOutputSpeed);
            }
         }
         else
         {
            /* The shooter spin up timeout has expired.                 */

            /* Reset the state timer.                                   */
            m_stateTimer = 0_s;

            /* Set the state to the next state in the state machine.    */
            m_commandState = csWaitingForLaunchTimeout;

            /* Start the intake to push the note into the shooters      */
            /* flywheels.                                               */
            m_intakeSubsystem->RunIntake(m_intakeOutputSpeed);
         }
         break;
      case csWaitingForLaunchTimeout:
         /* Currently waiting for the launch timeout to expire.         */

         /* Adjust the state timer by the execute period.               */
         m_stateTimer = m_stateTimer + EXECUTE_PERIOD_IN_SECONDS;

         /* Now check to see if the launch timeout has expired.         */
         if(m_stateTimer > m_launchTimeout)
         {
            /* The launch timeout has expired.                          */

            /* Stop the intake and shooter since launch has completed.  */
            m_intakeSubsystem->StopIntake();
            m_shooterSubsystem->StopShooter();

            /* Reset the state timer.                                   */
            m_stateTimer = 0_s;

            /* Set the state to the next state in the state machine.    */
            m_commandState = csCommandOver;
         }
         break;
      case csCommandOver:
         /* The command has completed.  Do nothing.                     */
         break;
   }
}

// Called once the command ends or is interrupted.
void ShootNoteFromIntakeByArmPositionCommand::End(bool interrupted)
{
   /* Check to see if the command was interrupted early.                */
   if(interrupted)
   {
      /* The command was interrupts.  In this shutdown the intake and   */
      /* shooer.                                                        */
      m_intakeSubsystem->StopIntake();
      m_shooterSubsystem->StopShooter();

      /* Reset the state timer.                                         */
      m_stateTimer = 0_s;

      /* Set the state to the next state in the state machine.          */
      m_commandState = csCommandOver;
   }
}

// Returns true when the command should end.
bool ShootNoteFromIntakeByArmPositionCommand::IsFinished()
{
  /* Return if the command state indicates tht the command has          */
  /* completed.                                                         */
  return((m_commandState == csCommandOver));
}
