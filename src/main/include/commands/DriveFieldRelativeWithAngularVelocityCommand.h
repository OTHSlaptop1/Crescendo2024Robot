// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/filter/SlewRateLimiter.h>
#include <units/angle.h>
#include <units/velocity.h>

#include <functional>

#include "Constants.h"

#include "subsystems/DriveSubsystem.h"
#include "subsystems/OdometrySubsystem.h"

/**
 * <p>Note that this extends CommandHelper, rather extending Command
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class DriveFieldRelativeWithAngularVelocityCommand : public frc2::CommandHelper<frc2::Command, DriveFieldRelativeWithAngularVelocityCommand>
{
 public:
  DriveFieldRelativeWithAngularVelocityCommand(DriveSubsystem *driveSubsystem, OdometrySubsystem *odometrySubsystem, std::function<units::meters_per_second_t()> xSpeedSupplier, std::function<units::meters_per_second_t()> ySpeedSupplier, std::function<units::radians_per_second_t()> rotationSpeedSupplier);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

private:
  /* The following variables are pointer to the various subsystem to be */
  /* used by this command.                                              */
  DriveSubsystem    *m_driveSubsystemPtr;
  OdometrySubsystem *m_odometrySubsystemPtr;

  /* The following variables hold the functions to be called to supply  */
  /* the various speed values used by this command.                     */
  std::function<units::meters_per_second_t(void)> m_xSpeedSupplier;
  std::function<units::meters_per_second_t(void)> m_ySpeedSupplier;
  std::function<units::radians_per_second_t(void)> m_rotationSpeedSupplier;

  // Slew rate filter variables for controlling lateral acceleration
  double m_currentRotation       = 0.0;
  double m_currentTranslationDir = 0.0;
  double m_currentTranslationMag = 0.0;

  frc::SlewRateLimiter<units::scalar> m_magLimiter{DriveConstants::kMagnitudeSlewRate/1_s};
  frc::SlewRateLimiter<units::scalar> m_rotLimiter{DriveConstants::kRotationalSlewRate/1_s};
  double m_prevTime = wpi::Now() * 1e-6; // in microseconds
};
