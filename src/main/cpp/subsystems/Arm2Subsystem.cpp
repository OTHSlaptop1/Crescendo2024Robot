// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/Arm2Subsystem.h"
#include "Constants.h"

#include <networktables/NetworkTableInstance.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>

   /* The following constant defines the threshold percentage of the    */
   /* targeted Arm Position that the Arm must attain to complete this   */
   /* command.                                                          */
#define ARM_UP_THRESHOLD_PERCENT                                     (0.98)

   /* The following constant defines the threshold percentage when using*/
   /* the set position command.                                         */
#define ARM_SET_POSITION_THRESHOLD_PERCENT                           (0.925)

Arm2Subsystem::Arm2Subsystem()
   :  m_leaderSparkMax(kArmLeaderCanId, rev::CANSparkMax::MotorType::kBrushless),
      m_followerSparkMax(kArmFollwerCanId, rev::CANSparkMax::MotorType::kBrushless),
      m_feedForward{kS, kG, kV, kA}
{
   /* Set the arm to state to initializing.                             */
   m_ArmState = asInitializing;

   /* Initialize the initialization timer used to wait before determine */
   /* our initial angle.                                                */
   m_initializationTimer = 500_ms;

   // Factory reset, so we get the SPARKS MAX to a known state before configuring
   // them. This is useful in case a SPARK MAX is swapped out.
   m_leaderSparkMax.RestoreFactoryDefaults();
   m_followerSparkMax.RestoreFactoryDefaults();

   /* Set the follower spark max to follow the leader and have its      */
   /* output inverted.                                                  */
   m_followerSparkMax.Follow(m_leaderSparkMax, true);

   // Apply position and velocity conversion factors for the leader encoder. We
   // want these in radians and radians per second to use with the ARM.
   m_armAbsoluteEncoder.SetPositionConversionFactor(kArmEncoderPositionFactor);
   m_armAbsoluteEncoder.SetVelocityConversionFactor(kArmEncoderVelocityFactor);

   /* Invert the output of the absolute encoder.                        */
   m_armAbsoluteEncoder.SetInverted(true);

   /* Change the average depth of the encoder to be less than the       */
   /* default.  If you see alot of noise on the encoder consider        */
   /* increasing this.                                                  */
   m_armAbsoluteEncoder.SetAverageDepth(32);

   /* Set limits in the spark max for the controller to not move past  */
   /* the maximum angles of motion.                                    */
   m_leaderSparkMax.SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kReverse, units::radian_t{kArmSoftLimitMinimumAngle}.value());
   m_leaderSparkMax.SetSoftLimit(rev::CANSparkBase::SoftLimitDirection::kForward, units::radian_t{kArmSoftLimitMaximumAngle}.value());

   m_leaderSparkMax.EnableSoftLimit(rev::CANSparkBase::SoftLimitDirection::kReverse, true);
   m_leaderSparkMax.EnableSoftLimit(rev::CANSparkBase::SoftLimitDirection::kForward, true);

   /* Force the motors to be stopped before setting up the PID.         */
   m_leaderSparkMax.StopMotor();
   m_followerSparkMax.StopMotor();

   // Enable PID wrap around for the turning motor. This will allow the PID
   // controller to go through 0 to get to the setpoint i.e. going from 350
   // degrees to 10 degrees will go through 0 rather than the other direction
   // which is a longer route.
   m_leaderPIDController.SetPositionPIDWrappingEnabled(true);
   m_leaderPIDController.SetPositionPIDWrappingMinInput(kArmEncoderPositionPIDMinInput.value());
   m_leaderPIDController.SetPositionPIDWrappingMaxInput(kArmEncoderPositionPIDMaxInput.value());

   // Set the leader PID Controller to use the duty cycle encoder on the swerve
   // module instead of the built in NEO encoder.
   m_leaderPIDController.SetFeedbackDevice(m_armAbsoluteEncoder);

   // Set the PID gains for the arm motor.
   m_leaderPIDController.SetP(kArmP);
   m_leaderPIDController.SetI(kArmI);
   m_leaderPIDController.SetD(kArmD);
   m_leaderPIDController.SetFF(kArmFF);
   m_leaderPIDController.SetOutputRange(kArmMinOutput, kArmMaxOutput);

   m_leaderSparkMax.SetIdleMode(kArmMotorIdleMode);
   m_followerSparkMax.SetIdleMode(kArmMotorIdleMode);

   m_leaderSparkMax.SetSmartCurrentLimit(kLeaderMotorCurrentLimit.value());
   m_followerSparkMax.SetSmartCurrentLimit(kFollowerMotorCurrentLimit.value());

   /* Enable voltage compensation on the driving and turning spark max  */
   /* motor controllers.                                                */
   m_leaderSparkMax.EnableVoltageCompensation(12.0);
   m_followerSparkMax.EnableVoltageCompensation(12.0);

   /* *** NOTE *** Check out the below resources for more information on*/
   /*            what the below does.  If you increase the Odometry     */
   /*            Periodic Rate you might need to change the values      */
   /*            below.                                                 */
// see https://github.com/bovlb/frc-tips/blob/main/can-bus/README.md
// see https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces#periodic-status-frames for details.

   /* Set the spark max periodic frame periods for each of the arm spark*/
   /* max motor controllers.                                            */
   /* ** NOTE ** Periodic Frame Status 0 is used by the follower to set */
   /*            its output.                                            */
   m_leaderSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus0, 10);      // Applied *** Output, Faults, Stick Faults, Is Follower
   m_leaderSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus1, 20);      // Motor Velocity, Motor Tempature, Motor Voltage, Motor Current
   m_leaderSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus2, 20);      // Motor Position
   m_leaderSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus3, 65535);   // Analog Sensor Voltage, Velocity, Position
   m_leaderSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus4, 65535);   // Alternate Encoder Velocity, Position
   m_leaderSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus5, 20);      // Duty Cycle Absolute Encoder Position, Absolute Angle
   m_leaderSparkMax.SetPeriodicFramePeriod(rev::CANSparkLowLevel::PeriodicFrame::kStatus6, 65565);   // Duty Cycle Absolute Encoder Velocity, Frequency

   // Save the SPARK MAX configurations. If a SPARK MAX browns out during
   // operation, it will maintain the above configurations.
   m_leaderSparkMax.BurnFlash();
   m_followerSparkMax.BurnFlash();

   // Start publishing arm state information with the "/Arm" key
   m_armMeasuredPositionPublisher  = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("/Arm/MeasuredPosition").Publish();
   m_armSetpointPositionPublisher  = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("/Arm/SetpointPosition").Publish();
   m_armSetpointVelocityPublisher  = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("/Arm/SetpointVelocity").Publish();
   m_armOutputFeedForwardPublisher = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("/Arm/OutputFeedForward").Publish();
   m_armCurrentPublisher           = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("/Arm/Current").Publish();
   m_armAppliedOutputPublisher     = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("/Arm/AppliedOutput").Publish();
}

// This method will be called once per scheduler run
void Arm2Subsystem::Periodic()
{
   units::volt_t feedforward = 0_V;

   /* Log some of the values we want to log continuously.               */
   m_armMeasuredPositionPublisher.Set(GetArmAngle().value());
   m_armCurrentPublisher.Set(m_leaderSparkMax.GetOutputCurrent());
   m_armAppliedOutputPublisher.Set(m_leaderSparkMax.GetAppliedOutput()*12);

   /* Determine the current arm state.                                  */
   switch(m_ArmState)
   {
      case asInitializing:
         /* Currently in the initializing state.                        */

         /* Adjust the initialization timer by a period.                */
         m_initializationTimer = m_initializationTimer - 20_ms;

         /* Now check to see if the initialization timer has expired.   */
         if(m_initializationTimer <= 0_s)
         {
            /* The initialization time has expired.  Set the initial    */
            /* position and goal position for the trapozoidal profile   */
            /* state.                                                   */
            m_goal    = {units::radian_t{GetArmAngle()}, units::radians_per_second_t{0.0}};
            m_current = {units::radian_t{GetArmAngle()}, units::radians_per_second_t{0.0}};

            /* Now set the state of the arm to the stopped state.       */
            m_ArmState = asStopped;
         }
         break;
      case asMovingUp:
         /* Currently in the moving up state.                           */

         /* Using the Up Trapazoid Profile calculate the the next step  */
         /* to get to the goal position.                                */
         m_current = m_upProfile.Calculate(20_ms, m_current, m_goal);

         /* Calculate the feed forward from the specified setpoint.     */
         feedforward = m_feedForward.Calculate((m_current.position-kArmFeedforwardOffsetAngle), m_current.velocity);

         /* Set the position for the motor with the calculated feed     */
         /* forward value.                                              */
         m_leaderPIDController.SetReference(m_current.position.value(), rev::CANSparkMax::ControlType::kPosition, 0/*PID Slot*/, feedforward.value(), rev::SparkMaxPIDController::ArbFFUnits::kVoltage);
         break;
      case asMovingDown:
         /* Currently in the moving down state.                         */

         /* Using the Down Trapazoid Profile calculate the the next step*/
         /* to get to the goal position.                                */
         m_current = m_downProfile.Calculate(20_ms, m_current, m_goal);

         /* Calculate the feed forward from the specified setpoint.     */
         feedforward = m_feedForward.Calculate((m_current.position-kArmFeedforwardOffsetAngle), m_current.velocity);

         /* Set the position for the motor with the calculated feed     */
         /* forward value.                                              */
         m_leaderPIDController.SetReference(m_current.position.value(), rev::CANSparkMax::ControlType::kPosition, 0/*PID Slot*/, feedforward.value(), rev::SparkMaxPIDController::ArbFFUnits::kVoltage);
         break;
      case asStopped:
         /* Currently in the stopped state.                             */
         break;
      case asDisabled:
         /* Currently in the disabled state.                            */

         /* Do nothing.                                                 */
         break;
   }

   /* Log the current arm output values.                          */
   m_armSetpointPositionPublisher.Set(units::degree_t(m_current.position).value());
   m_armSetpointVelocityPublisher.Set(m_current.velocity.value());
   m_armOutputFeedForwardPublisher.Set(feedforward.value());
}

   /* Returns the current angle of the arm in degress.                  */
   /* ** NOTE ** The output range of the absolute encoder is mapped to  */
   /*            be between -180 to 180 degrees.                        */
units::degree_t Arm2Subsystem::GetArmAngle(void)
{
   units::radian_t ret_val;

   /* Get the current angle of the arm.                                 */
   ret_val = units::radian_t{std::fmod(m_armAbsoluteEncoder.GetPosition(), units::radian_t{360.0_deg}.value())};

   /* Map the angle to the range [-180.0, 180.0].                       */
   if(ret_val > units::radian_t{180.0_deg})
      ret_val = ret_val - units::radian_t{360.0_deg};

   /* Simply return the current Arm Angle.                              */
   return(units::degree_t(ret_val));
}

   /* Set the arm to the specified position.                            */
void Arm2Subsystem::SetArmPosition(units::degree_t setpoint)
{
   /* Get the current angle of the arm.                                 */
   units::degree_t currentAngle = GetArmAngle();

   /* Clamp the angle to the minimum and maximum.                       */
   setpoint = std::clamp(setpoint, kArmMinimumAngle, kArmMaximumAngle);

   /* Now check to see if the desired angle is up or down from the      */
   /* current angle.                                                    */
   if(setpoint > currentAngle)
   {
     /* The desired angle is up from the current angle.                 */

     /* Set the arm state to indicate we want to go up from our current */
     /* position.                                                       */
     m_ArmState = asMovingUp;

     /* Set the current position to be the current measured angle at the*/
     /* last calculated velocity.                                       */
     m_current = {units::radian_t{GetArmAngle()}, m_current.velocity};

     /* Set the goal for movement to the specified position and ending  */
     /* at a 0 velocity.                                                */
     m_goal = {units::radian_t{setpoint}, units::radians_per_second_t{0.0}};
   }
   else
   {
     /* The desired angle is down from the current angle.               */

     /* Set the arm state to indicate we want to go down from our       */
     /* current position.                                               */
     m_ArmState = asMovingDown;

     /* Set the current position to be the current measured angle at the*/
     /* last calculated velocity.                                       */
     m_current = {units::radian_t{GetArmAngle()}, m_current.velocity};

     /* Set the goal for movement to the specified position and ending  */
     /* at a 0 velocity.                                                */
     m_goal = {units::radian_t{setpoint}, units::radians_per_second_t{0.0}};
   }
}

   /* Generates a command to set the arm position.                      */
frc2::CommandPtr Arm2Subsystem::SetArmPositionCommand(units::degree_t setpoint)
{
   /* Create a command to run the arm to the specified setpoint.        */
   return(frc2::cmd::RunOnce([this, setpoint] { this->SetArmPosition(setpoint); }, {this}));
}

  /* Generates a command to set the arm position and wait until the     */
  /* motion profile has completed.                                      */
frc2::CommandPtr Arm2Subsystem::SetArmPositionAndWaitUntilCompleteCommand(units::degree_t setpoint)
{
   /* Generated a command that enable and sets the goal to the specified*/
   /* angle on initialization, does nothing while executing, stops the  */
   /* arm and sets it position to the current position when interrupted.*/
   /* Uses a motion profile velocity of zero to stop the commmand.      */
   frc2::CommandPtr ret_val = frc2::FunctionalCommand(
                                                      [this, setpoint] {
                                                               /* Set the position for moving the arm subsystem to the specified set point.          */
                                                               this->SetArmPosition(setpoint);
                                                             },
                                                      [] {},
                                                      [this](bool interrupted){
                                                                                /* Check to see if the reason this command ended was because it was  */
                                                                                /* interrupts.                                                       */
                                                                                if(interrupted)
                                                                                {
                                                                                   /* The reason this command ended was because it was interrupts.   */
                                                                                   /* Set the goal angle for the arm to be the current angle to stop */
                                                                                   /* its motion.                                                    */
                                                                                   this->SetArmPosition(units::radian_t{this->GetArmAngle()});
                                                                                }
                                                                              },
                                                      [this, setpoint]{
                                                               /* Now check to see if the motion profile has completed.*/
                                                               if((m_current.velocity == 0_deg_per_s) || ((m_ArmState == asMovingDown) && (this->GetArmAngle() <= 5.5_deg)) || ((m_ArmState == asMovingUp) && (this->GetArmAngle() >= setpoint*ARM_SET_POSITION_THRESHOLD_PERCENT)))
                                                                  return(true);
                                                               else
                                                                  return(false);
                                                            },
                                                      {this} ).ToPtr();

   /* Give the command a readable name.                                 */
   ret_val.get()->SetName("SetArmPositionAndWaitUntilCompleteCommand()");

   /* Return the command to the caller.                                 */
   return(ret_val);
}

  /* Disable the Arms motors.                                           */
void Arm2Subsystem::DisableArm(void)
{
   /* Disable the arms motors and set the state to indicate we are      */
   /* currently disabled.                                               */
   m_leaderSparkMax.StopMotor();
   m_followerSparkMax.StopMotor();

   /* Set the Arm State to indicate that we are currently disabled.     */
   m_ArmState = asDisabled;
}

   /* Generates a command to move the arm up.                           */
frc2::CommandPtr Arm2Subsystem::ArmUpCommand(void)
{
   /* Generated a command that enable and sets the goal to the maximum  */
   /* angle on initialization, does nothing while executing, stops the  */
   /* arm and sets it position to the current position when interrupted.*/
   /* Uses reaching the maximum up angle to stop the command.           */
   frc2::CommandPtr ret_val = frc2::FunctionalCommand(
                                                      [this] {
                                                               /* Set the position for moving the arm subsystem to be maximum angle supported by     */
                                                               /* the arm.                                                                           */
                                                               this->SetArmPosition(kArmMaximumAngle);
                                                             },
                                                      [] {},
                                                      [this](bool interrupted){
                                                                                /* Check to see if the reason this command ended was because it was  */
                                                                                /* interrupts.                                                       */
                                                                                if(interrupted)
                                                                                {
                                                                                   /* The reason this command ended was because it was interrupts.   */
                                                                                   /* Set the goal angle for the arm to be the current angle to stop */
                                                                                   /* its motion.                                                    */
                                                                                   this->SetArmPosition(units::radian_t{this->GetArmAngle()});
                                                                                }
                                                                              },
                                                      [this]{
                                                               /* Now check to see if we are at the maximum angle.                  */
                                                               if(this->GetArmAngle() >= (kArmMaximumAngle * ARM_UP_THRESHOLD_PERCENT))
                                                                  return(true);
                                                               else
                                                                  return(false);
                                                            },
                                                      {this} ).ToPtr();

   /* Give the command a readable name.                                 */
   ret_val.get()->SetName("ArmUpCommmand()");

   /* Return the command to the caller.                                 */
   return(ret_val);
}

   /* Generates a command to move the arm down.                         */
frc2::CommandPtr Arm2Subsystem::ArmDownCommand(void)
{
   /* Generated a command that enable and sets the goal to the minimum  */
   /* angle on initialization, does nothing while executing, stops the  */
   /* arm and sets it position to the current position when interrupted.*/
   /* Uses reaching the maximum up angle to stop the command.           */
   frc2::CommandPtr ret_val = frc2::FunctionalCommand(
                                                      [this] {
                                                               /* Set the position for moving the arm subsystem to be minimum angle     */
                                                               /* supported by the arm.                                                 */
                                                               this->SetArmPosition(kArmMinimumAngle);
                                                             },
                                                      [] {},
                                                      [this](bool interrupted){
                                                                                /* Check to see if the reason this command ended was because it was  */
                                                                                /* interrupts.                                                       */
                                                                                if(interrupted)
                                                                                {
                                                                                   /* The reason this command ended was because it was interrupts.   */
                                                                                   /* Set the goal angle for the arm to be the current angle to stop */
                                                                                   /* its motion.                                                    */
                                                                                   this->SetArmPosition(units::radian_t{this->GetArmAngle()});
                                                                                }
                                                                                else
                                                                                {
                                                                                   /* The reason this command ended was because we have reached the  */
                                                                                   /* down position.  In this case disable the arm movement so it    */
                                                                                   /* relaxes to the lowest possible state.                           */
                                                                                   this->DisableArm();
                                                                                }
                                                                              },
                                                      [this]{
                                                               /* Now check to see if we are at the minimum angle.                  */
                                                               if(this->GetArmAngle() <= 5.5_deg)
                                                                  return(true);
                                                               else
                                                                  return(false);
                                                            },
                                                      {this} ).ToPtr();

   /* Give the command a readable name.                                 */
   ret_val.get()->SetName("ArmDownCommmand()");

   /* Return the command to the caller.                                 */
   return(ret_val);
}
