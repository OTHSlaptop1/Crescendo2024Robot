// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/TurnToAngleProfiled.h"

#include <units/angle.h>
#include <units/angular_velocity.h>

#include <cmath>

TurnToAngleProfiled::TurnToAngleProfiled(units::degree_t targetAngleDegrees, DriveSubsystem *driveSubsystemPtr)
   : CommandHelper{
          // The ProfiledPIDController that the command will use
          frc::ProfiledPIDController<units::radians>{
                                                      // The PID gains
                                                      1.0, // P
                                                      0,   // I
                                                      0,   // D
                                                      // The motion profile constraints
                                                      {180_deg_per_s, 360_deg_per_s_sq}
                                                    },
          // This should return the measurement
          [driveSubsystemPtr]{
                                 /* Get the current angle of the gyro. */
                                 units::radian_t currentAngle = units::degree_t{std::fmod(driveSubsystemPtr->GetHeading().value(), 360.0)};

                                 /* Map the angle to the range [-180.0,*/
                                 /* 180.0].                            */
                                 if(currentAngle > units::radian_t{180.0_deg})
                                    currentAngle = currentAngle - units::radian_t{360.0_deg};

                                 /* Simply return the current Gyro     */
                                 /* Angle.                             */
                                 return(currentAngle);
                             },
          // Set reference to target
          targetAngleDegrees,
          // This uses the output and current trajectory setpoint
          [driveSubsystemPtr](double output, frc::TrapezoidProfile<units::radians>::State setpoint)
          {
             driveSubsystemPtr->SetChassisSpeed({0_mps, 0_mps, units::radians_per_second_t{output}});
          },
          {driveSubsystemPtr}}
{
  // Set the controller to be continuous (because it is an angle controller)
  GetController().EnableContinuousInput(-180_deg, 180_deg);

  // Set the controller tolerance - the delta tolerance ensures the robot is
  // stationary at the setpoint before it is considered as having reached the
  // reference
  GetController().SetTolerance(5_deg, 10_deg_per_s);

  AddRequirements(driveSubsystemPtr);
}

// Returns true once we have reached the angle.
bool TurnToAngleProfiled::IsFinished()
{
   return(GetController().AtGoal());
}
