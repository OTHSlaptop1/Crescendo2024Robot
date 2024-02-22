// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>

#include "subsystems/DriveSubsystem.h"

class SwerveSendable : public wpi::Sendable {
 public:
  SwerveSendable(DriveSubsystem *driveSubsystemPtr);

/**
   * Initializes this Sendable object.
   *
   * @param builder sendable builder
   */
  void InitSendable(wpi::SendableBuilder& builder);

 private:
   // Drive Subsystem this object uses to get the swerve state information.
   DriveSubsystem *m_driveSubsystemPtr;  

};
