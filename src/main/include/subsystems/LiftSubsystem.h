// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>

#include <CANVenom.h>

class LiftSubsystem : public frc2::SubsystemBase
{
 public:
  LiftSubsystem();

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Function to move the lift hook up and down.
   *
   *  @param output  The voltage to use to move the lift
   *                 hook up and down.  Positive values up.
   *
   */
  void MoveLift(units::volt_t output);

  /* Function to stop the moving of the lift hook.                      */
  void StopLift(void);

  /**
   * Function to set the lift into hold mode.
   */
  void HoldPosition(void);

 private:

  // The Lift Motor Controller.
  frc::CANVenom m_liftVenomMotor;

};
