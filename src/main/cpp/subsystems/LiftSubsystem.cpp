// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/LiftSubsystem.h"
#include <networktables/NetworkTableInstance.h>

#include "Constants.h"

#include <CANVenom.h>

using namespace LiftConstants;


LiftSubsystem::LiftSubsystem()
    : m_liftVenomMotor{kLiftCanId},
      m_latchServo{kLiftServoChannel}
{
    /* Reset the motor revoltion counter to zero.                       */
    m_liftVenomMotor.ResetPosition();

    /* Set the brake/coast mode for the motor when its stopped.         */
    m_liftVenomMotor.SetBrakeCoastMode(frc::CANVenom::BrakeCoastMode::kBrake);

    /* Set the servo position to the last released position.            */
    m_latchServo.SetAngle(kLiftServoReleaseAngle);

    // Initialize the lift motion state to stopped.
    m_liftMotionState = lmsStopped;

    // Start publishing lift state information with the "/Lift" key
    m_encoderPositionPublisher  = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("/Lift/EncoderPosition").Publish();
    m_liftServoAnglePublisher   = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("/Lift/ServoAngle").Publish();
}

// This method will be called once per scheduler run
void LiftSubsystem::Periodic()
{
   double encoderValue;

   /* Get the current encoder position value.                           */
   encoderValue = m_liftVenomMotor.GetPosition();

   /* Log some of the values we want to log continuously.               */
   m_encoderPositionPublisher.Set(encoderValue);
   m_liftServoAnglePublisher.Set(m_latchServo.GetAngle());

   /* Determine the current lift motion state.    */
   switch(m_liftMotionState)
   {
      case lmsMovingUp:
         /* Currently moving up. */
         if(encoderValue >= kLiftMaximumValue)
         {
            /* The encoder value indicates we are at the maximum        */
            /* position.  Stop the motor.                               */
            StopLift();
         }
         break;
      case lmsMovingDown:
         /* Currently moving down. */
         if(encoderValue <= 0.0)
         {
            /* The encoder value indcates we are at the minimum         */
            /* position.  Stop the motor.                               */
            StopLift();
         }
         break;
   }
}

   /* The following function is responsible for moving the lift hook up */
   /* and down.                                                         */
void LiftSubsystem::MoveLift(units::volt_t output)
{
   /* Set the lift motion state to allow free movement.                 */
   m_liftMotionState = lmsFreeMove;

   /* Simply pass the specified voltage to the lift motor to move it.   */
   m_liftVenomMotor.SetCommand(frc::CANVenom::ControlMode::kVoltageControl, output.value());
}

  /**
   * Function to move the lift up.
   *
   *  @param output The voltage to use to move the lift hook up.
   *
  */
void LiftSubsystem::MoveLiftUp(units::volt_t output)
{
   /* First check to make sure the parameter appears to be at least     */
   /* semi-valid.                                                       */
   if((output >= 0_V) && (output <= 12_V))
   {
      /* The specified output voltage appears to be at lease semi-valid.*/
      /* Now let check the current encoder value and make sure it is ok */
      /* to move the lift up.                                           */
      if(m_liftVenomMotor.GetPosition() <= kLiftMaximumValue)
      {
         /* The encoder indicates we are in a position in which we can  */
         /* go up.                                                      */

         /* Set the lift motion state to indicate we are going to be    */
         /* moving up.                                                  */
         m_liftMotionState = lmsMovingUp;

         /* Start moving the lift up.  */
         m_liftVenomMotor.SetCommand(frc::CANVenom::ControlMode::kVoltageControl, output.value());
      }
   }
}

  /**
   * Function to move the lift down.
   *
   *  @param output The voltage to use to move the lift hook down.
  */
void LiftSubsystem::MoveLiftDown(units::volt_t output)
{
   /* First check to make sure the parameter appears to be at least     */
   /* semi-valid.                                                       */
   if((output >= 0_V) && (output <= 12_V))
   {
      /* The specified output voltage appears to be at lease semi-valid.*/
      /* Now let check the current encoder value and make sure it is ok */
      /* to move the lift up.                                           */
      if(m_liftVenomMotor.GetPosition() >= 0.0)
      {
         /* The encoder indicates we are in a position in which we can  */
         /* go up.                                                      */

         /* Set the lift motion state to indicate we are going to be    */
         /* moving up.                                                  */
         m_liftMotionState = lmsMovingDown;

         /* Start moving the lift up.  */
         m_liftVenomMotor.SetCommand(frc::CANVenom::ControlMode::kVoltageControl, -output.value());
      }
   }
}

   /* The following function is responsible for stopping the movement of*/
   /* the lift hook.                                                    */
void LiftSubsystem::StopLift(void)
{
   /* Set the lift move state to stopped. */
   m_liftMotionState = lmsStopped;

   /* Set the output to zero to stop the movement on the motor.         */
   m_liftVenomMotor.SetCommand(frc::CANVenom::ControlMode::kVoltageControl, 0.0);
}

  /* Function to reset the position encoder.                            */
void LiftSubsystem::ResetLiftEncoder(void)
{
   /* Reset the lift encoder.                                           */
   m_liftVenomMotor.ResetPosition();
}

  /* Function to set the servo controlled latch.                        */
void LiftSubsystem::SetLatch(void)
{
   /* Set the servo to the latching angle.                              */
   m_latchServo.SetAngle(kLiftServoLatchAngle);
}

  /* Function to release the servo controlled latch.                    */
void LiftSubsystem::ReleaseLatch(void)
{
   /* Set the servo to the latching angle.                              */
   m_latchServo.SetAngle(kLiftServoReleaseAngle);
}

