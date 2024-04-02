// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/SubsystemBase.h>
#include <networktables/DoubleTopic.h>
#include <frc/Servo.h>

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

  /**
   * Function to move the lift up.
   *
   *  @param output The voltage to use to move the lift hook up.
   *
  */
  void MoveLiftUp(units::volt_t output);

  /**
   * Function to move the lift down.
   *
   *  @param output The voltage to use to move the lift hook down.
  */
  void MoveLiftDown(units::volt_t output);

  /* Function to stop the moving of the lift hook.                      */
  void StopLift(void);

  /* Function to reset the position encoder.                            */
  void ResetLiftEncoder(void);
  
   /* Function to get the current lift encoder value.                    */
 double GetLiftEncoderValue(void);

  /**
   * Function to set the servo controlled latch.
   */
  void SetLatch(void);

  /**
   * Function to release the servo controlled latch.
   */
  void ReleaseLatch(void);

 private:

  // The Lift Motor Controller.
  frc::CANVenom m_liftVenomMotor;

  // The servo used to control the latching mechanism
  frc::Servo m_latchServo{1};

  // The following enumerated type holds all possible lift motion state.
  typedef enum
  {
    lmsMovingUp,
    lmsMovingDown,
    lmsStopped,
    lmsFreeMove
  } LiftMotionState_t;

  // The following variable holds the current lift motion state.
  LiftMotionState_t m_liftMotionState;

  // Publisher variables for the lift.
  nt::DoublePublisher m_encoderPositionPublisher;
  nt::DoublePublisher m_liftServoAnglePublisher;
};
