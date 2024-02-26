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
}

// This method will be called once per scheduler run
void LiftSubsystem::Periodic() {}
