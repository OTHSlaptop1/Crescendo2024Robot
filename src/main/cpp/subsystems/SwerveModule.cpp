// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/geometry/Rotation2d.h>

#include <numbers>

#include "Constants.h"

using namespace ModuleConstants;

SwerveModule::SwerveModule(const int drivingCANId, const int turningCANId)
    : m_drivingSparkMax(drivingCANId, rev::CANSparkMax::MotorType::kBrushless),
      m_turningSparkMax(turningCANId, rev::CANSparkMax::MotorType::kBrushless)
{
   // Factory reset, so we get the SPARKS MAX to a known state before configuring
   // them. This is useful in case a SPARK MAX is swapped out.
   m_drivingSparkMax.RestoreFactoryDefaults();
   m_turningSparkMax.RestoreFactoryDefaults();

   // Apply position and velocity conversion factors for the driving encoder. The
   // native units for position and velocity are rotations and RPM, respectively,
   // but we want meters and meters per second to use with WPILib's swerve APIs.
   m_drivingEncoder.SetPositionConversionFactor(kDrivingEncoderPositionFactor);
   m_drivingEncoder.SetVelocityConversionFactor(kDrivingEncoderVelocityFactor);

   // Apply position and velocity conversion factors for the turning encoder. We
   // want these in radians and radians per second to use with WPILib's swerve
   // APIs.
   m_turningAbsoluteEncoder.SetPositionConversionFactor(kTurningEncoderPositionFactor);
   m_turningAbsoluteEncoder.SetVelocityConversionFactor(kTurningEncoderVelocityFactor);

   // Set the turning encoder inverted state based on the constant expression.
   m_turningAbsoluteEncoder.SetInverted(kTurningEncoderInverted);

   // Set the turning motor inverted state based on the constant expression.
   m_turningSparkMax.SetInverted(kTurningMotorInverted);

   // Set the Drive PID Controller to use the internal encoder.
   m_drivingPIDController.SetFeedbackDevice(m_drivingEncoder);

   // Set the PID gains for the driving motor. Note these are example gains, and
   // you may need to tune them for your own robot!
   m_drivingPIDController.SetP(kDrivingP);
   m_drivingPIDController.SetI(kDrivingI);
   m_drivingPIDController.SetD(kDrivingD);
   m_drivingPIDController.SetFF(kDrivingFF);
   m_drivingPIDController.SetOutputRange(kDrivingMinOutput, kDrivingMaxOutput);

   // Enable PID wrap around for the turning motor. This will allow the PID
   // controller to go through 0 to get to the setpoint i.e. going from 350
   // degrees to 10 degrees will go through 0 rather than the other direction
   // which is a longer route.
   m_turningPIDController.SetPositionPIDWrappingEnabled(true);
   m_turningPIDController.SetPositionPIDWrappingMinInput(kTurningEncoderPositionPIDMinInput.value());
   m_turningPIDController.SetPositionPIDWrappingMaxInput(kTurningEncoderPositionPIDMaxInput.value());

   // Set the Turn PID Controller to use the duty cycle encoder on the swerve
   // module instead of the built in NEO encoder.
   m_turningPIDController.SetFeedbackDevice(m_turningAbsoluteEncoder);

   // Set the PID gains for the turning motor.
   m_turningPIDController.SetP(kTurningP);
   m_turningPIDController.SetI(kTurningI);
   m_turningPIDController.SetD(kTurningD);
   m_turningPIDController.SetFF(kTurningFF);
   m_turningPIDController.SetOutputRange(kTurningMinOutput, kTurningMaxOutput);

   m_drivingSparkMax.SetIdleMode(kDrivingMotorIdleMode);
   m_turningSparkMax.SetIdleMode(kTurningMotorIdleMode);
   m_drivingSparkMax.SetSmartCurrentLimit(kDrivingMotorCurrentLimit.value());
   m_turningSparkMax.SetSmartCurrentLimit(kTurningMotorCurrentLimit.value());

//XXX ADDED - RTS
   /* Enable voltage compensation on the driving and turning spark max  */
   /* motor controllers.                                                */
   m_drivingSparkMax.EnableVoltageCompensation(12.0);
   m_turningSparkMax.EnableVoltageCompensation(12.0);

   /* *** NOTE *** Check out the below resources for more information on*/
   /*            what the below does.  If you increase the Odometry     */
   /*            Periodic Rate you might need to change the values      */
   /*            below.                                                 */
// see https://github.com/bovlb/frc-tips/blob/main/can-bus/README.md
// see https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames for details.

   /* Set the spark max periodic frame periods for each of the driving  */
   /* spark max motor controllers.                                      */
   m_drivingSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus0, 55);      // Applied *** Output, Faults, Stick Faults, Is Follower
   m_drivingSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus1, 15);      // Motor Velocity, Motor Tempature, Motor Voltage, Motor Current
   m_drivingSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 15);      // Motor Position
   m_drivingSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus3, 65535);   // Analog Sensor Voltage, Velocity, Position
   m_drivingSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus4, 65535);   // Alternate Encoder Velocity, Position
   m_drivingSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus5, 65535);   // Duty Cycle Absolute Encoder Position, Absolute Angle
   m_drivingSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus6, 65565);   // Duty Cycle Absolute Encoder Velocity, Frequency

   /* Set the spark max periodic frame periods for each of the turning  */
   /* spark max motor controllers.                                      */
   m_turningSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus0, 55);      // Applied *** Output, Faults, Stick Faults, Is Follower
   m_turningSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus1, 15);      // Motor Velocity, Motor Tempature, Motor Voltage, Motor Current
   m_turningSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 65535);   // Motor Position
   m_turningSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus3, 65535);   // Analog Sensor Voltage, Velocity, Position
   m_turningSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus4, 65535);   // Alternate Encoder Velocity, Position
   m_turningSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus5, 15);      // Duty Cycle Absolute Encoder Position, Absolute Angle
   m_turningSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus6, 65565);   // Duty Cycle Absolute Encoder Velocity, Frequency


#if 0 //xxx mechanical advantage team uses, not sure what it does....
   driveSparkMax.setCANTimeout(0);
   turnSparkMax.setCANTimeout(0);
#endif

   // Save the SPARK MAX configurations. If a SPARK MAX browns out during
   // operation, it will maintain the above configurations.
   m_drivingSparkMax.BurnFlash();
   m_turningSparkMax.BurnFlash();

   /* Initialize the desired state angle to the current turning encoder */
   /* positions (this hopefully should be zero).                        */
   m_desiredState.angle = frc::Rotation2d(units::radian_t{m_turningAbsoluteEncoder.GetPosition()});

   /* Reset the driver encoder position value to zero.                  */
   m_drivingEncoder.SetPosition(0);
}

   /* This function gets the current state for this swerve module.  It  */
   /* returns a swerve module state (which is wheel velocity and angle).*/
frc::SwerveModuleState SwerveModule::GetState() const
{
   return {units::meters_per_second_t{m_drivingEncoder.GetVelocity()}, units::radian_t{m_turningAbsoluteEncoder.GetPosition()}};
}

   /* This function gets the current position for this swerve module.   */
   /* It returns a swerve module state (which is wheel position and     */
   /* angle).                                                           */
frc::SwerveModulePosition SwerveModule::GetPosition() const
{
   return {units::meter_t{m_drivingEncoder.GetPosition()}, units::radian_t{m_turningAbsoluteEncoder.GetPosition()}};
}

   /* This function sets the desired state for this swerve module.      */
frc::SwerveModuleState SwerveModule::SetDesiredState(const frc::SwerveModuleState& desiredState)
{
//xxx currently using the simple version, can try testing the motor stop version sometime if there is time.
#if 1
   // Optimize the reference state to avoid spinning further than 90 degrees.
   frc::SwerveModuleState optimizedDesiredState{frc::SwerveModuleState::Optimize(desiredState, frc::Rotation2d(units::radian_t{m_turningAbsoluteEncoder.GetPosition()}))};

//xxx WPI swerve module lib does this....  test on carpet.  I tested it some, didn't notice much difference
#if 0
  // Scale speed by cosine of angle error. This scales down movement
  // perpendicular to the desired direction of travel that can occur when
  // modules change directions. This results in smoother driving.
  optimizedDesiredState.speed *= (optimizedDesiredState.angle - units::radian_t{m_turningAbsoluteEncoder.GetPosition()}).Cos();
#endif

   // Command driving and turning SPARKS MAX towards their respective setpoints.
   m_drivingPIDController.SetReference((double)optimizedDesiredState.speed, rev::CANSparkMax::ControlType::kVelocity);
   m_turningPIDController.SetReference(optimizedDesiredState.angle.Radians().value(), rev::CANSparkMax::ControlType::kPosition);

   m_desiredState = desiredState;
#else

   /* ** NOTE * Maybe consider using this.  This might help keeping the */
   /*          wheels from going back to zero when the velocity is zero */
   /*          (or near zero).  I tested it some but it didn't seem to  */
   /*          work well, I think the ~0 velocity I was using might have*/
   /*          been to large still.                                     */

   /* Initialize the optimized desired state with the current state.    */
   frc::SwerveModuleState optimizedDesiredState = GetState();

//xxx i think for this to work, maybe the 0 velocity number needs to be larger...
//xxx currently seems to stop command wheels to turn, but still moves them some i think...
   /* Check to see if the desired speed is very small (stopped).        */
   if(units::math::abs(desiredState.speed) < 0.0001_mps)
   {
      /* The desired speed is very small.  In this case just stop the   */
      /* motors instead of correcting for the angle.                    */

      /* ** NOTE ** This keeps the wheels from returning to a zero      */
      /*            position when there is now speed being commanded.   */
      /*            see https://youtu.be/0Xi9yb1IMyA?t=590 for details. */

      // Command driving and turning SPARKS MAX to stop (Velocity of driving 0, Turn Position stays same as current state)
      m_drivingPIDController.SetReference(0.0, rev::CANSparkMax::ControlType::kVelocity);
      m_turningPIDController.SetReference(optimizedDesiredState.angle.Radians().value(), rev::CANSparkMax::ControlType::kPosition);
   }
   else
   {
      /* The speed is not stopped.  In this case we can command the     */
      /* motors normally.                                               */

      // Optimize the reference state to avoid spinning further than 90 degrees.
      optimizedDesiredState = frc::SwerveModuleState::Optimize(desiredState, frc::Rotation2d(units::radian_t{m_turningAbsoluteEncoder.GetPosition()}));

//xxx WPI swerve module lib does this....  test on carpet.  I tested it some, didn't notice much difference
#if 0
     // Scale speed by cosine of angle error. This scales down movement
     // perpendicular to the desired direction of travel that can occur when
     // modules change directions. This results in smoother driving.
     state.speed *= (state.angle - units::radian_t{m_turningAbsoluteEncoder.GetPosition()).Cos();
#endif

      // Command driving and turning SPARKS MAX towards their respective setpoints.
      m_drivingPIDController.SetReference((double)optimizedDesiredState.speed, rev::CANSparkMax::ControlType::kVelocity);
      m_turningPIDController.SetReference(optimizedDesiredState.angle.Radians().value(), rev::CANSparkMax::ControlType::kPosition);

      m_desiredState = desiredState;
   }
#endif

   return(optimizedDesiredState);
}

   /* Get the current being used by the module drive motor.             */
double SwerveModule::GetDriveCurrent(void)
{
   /* Simply return the output current for the drive motor.             */
   return(m_drivingSparkMax.GetOutputCurrent());
}

   /* Get the current being used by the module turning motor.           */
double SwerveModule::GetTurnCurrent(void)
{
   /* Simply return the output current for the turn motor.              */
   return(m_turningSparkMax.GetOutputCurrent());
}

   /* This function resets the drive encoders position.                 */
void SwerveModule::ResetEncoders()
{
   m_drivingEncoder.SetPosition(0);
}
