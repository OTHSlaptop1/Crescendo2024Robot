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

  void LiftUp(void);

  void LiftDown(void);

  double GetLiftPosition(void);

 private:
  
  // The Lift Motor Controller.
  frc::CANVenom m_liftVenomMotor;

};
