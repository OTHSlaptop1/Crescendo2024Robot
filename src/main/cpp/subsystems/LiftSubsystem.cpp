// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LiftSubsystem.h"

#include "Constants.h"

#include <CANVenom.h>

using namespace LiftConstants;

LiftSubsystem::LiftSubsystem()
    : m_liftVenomMotor{kLiftCanId}
{
    /* Reset the motor revoltion counter to zero.  */
    m_liftVenomMotor.ResetPosition();

    /* Set the PID gains for use with holding on the chain.             */
    m_liftVenomMotor.SetPID(kLiftP, kLiftI, kLiftD, 0.0, 0.0);
}

// This method will be called once per scheduler run
void LiftSubsystem::Periodic() {}

   /* The following function is responsible for moving the lift hook up */
   /* and down.                                                         */
void LiftSubsystem::MoveLift(units::volt_t output)
{
   /* Simply pass the specified voltage to the lift motor to move it.   */
   m_liftVenomMotor.SetCommand(frc::CANVenom::ControlMode::kVoltageControl, output.value());
}

   /* The following function is responsible for stopping the movement of*/
   /* the lift hook.                                                    */
void LiftSubsystem::StopLift(void)
{
   /* Set the output to zero to stop the movement on the motor.         */
   m_liftVenomMotor.SetCommand(frc::CANVenom::ControlMode::kVoltageControl, 0.0);
}

   /* The following function is responsible for setting the lift hook to*/
   /* be in hold mode.                                                  */
void LiftSubsystem::HoldPosition(void)
{
   /* Reset the motor revoltion counter to zero.  */
   m_liftVenomMotor.ResetPosition();

   /* Send a command to the motor to perform position contol and keep   */
   /* the position at zero.                                             */
// change final parameter to the minimum number of volts required to counter act gravity..
//  needs to be proportional value between 2.0 and -2.0.....  it is a duty cycle offset.
   m_liftVenomMotor.SetCommand(frc::CANVenom::ControlMode::kPositionControl, 0.0, 0.0, 0.0);
}
