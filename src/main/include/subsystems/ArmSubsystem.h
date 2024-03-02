// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <networktables/DoubleTopic.h>

#include <rev/CANSparkMax.h>
#include <rev/SparkMaxAbsoluteEncoder.h>

#include <frc2/command/Commands.h>
#include <frc/controller/ArmFeedforward.h>
#include <frc2/command/TrapezoidProfileSubsystem.h>
#include <units/angle.h>

class ArmSubsystem : public frc2::TrapezoidProfileSubsystem<units::radians>
{
 public:

   /* Arm Subsystem contructor.                                         */
   ArmSubsystem();

   /**
    * Get the current angle of the arm.
    *
    * @return The current angle of the Arm in degrees. The range is mapped
    *         to -180 to 180 degrees.
    *
    */
   units::degree_t GetArmAngle(void);

   /**
    * Set the arm position.
    *
    *  @param setpoint  Set point position to move the arm to.
    *
    */
   void SetArmPosition(units::degree_t setpoint);

   /**
    * Generates a command to set the arm position.
    *
    *  @param setpoint  Set point position to move the arm to.
    *
    *  @return A command pointer that runs the arm to the specified position.
    */
   frc2::CommandPtr SetArmPositionCommand(units::degree_t setpoint);

   /**
    * Generates a command to move the arm up.
    *
    *  @return A command pointer that runs the arm up.
    */
   frc2::CommandPtr ArmUpCommmand(void);

   /**
    * Generates a command to move the arm down.
    *
    *  @return A command pointer that runs the arm down.
    */
   frc2::CommandPtr ArmDownCommand(void);

 protected:

   /* The following function consumes the output of the trapezoid       */
   /* profile controller and the current setpoint state (which is used  */
   /* for computing a feedforward value).  The trapezoid profile (that  */
   /* is part of this object) automatically calls this method from its  */
   /* perodic() block and passes this function the computed output from */
   /* the control loop.                                                 */
   void UseState(frc::TrapezoidProfile<units::radians>::State setpoint) override;

 private:
   /* The following variable hold the controller objects for each of the*/
   /* spark max modules.                                                */
   rev::CANSparkMax m_leaderSparkMax;
   rev::CANSparkMax m_followerSparkMax;

   /* The following variable holds the arm absolute encoder.            */
   rev::SparkAbsoluteEncoder m_armAbsoluteEncoder = m_leaderSparkMax.GetAbsoluteEncoder(rev::SparkAbsoluteEncoder::Type::kDutyCycle);

   /* The following variable holds the arm leader PID controller.       */
   rev::SparkPIDController m_leaderPIDController = m_leaderSparkMax.GetPIDController();

   // The Arm Feed Forward Controller.
   frc::ArmFeedforward m_feedForward;

   // Publisher variables for the arm.
   nt::DoublePublisher m_armMeasuredPositionPublisher;
   nt::DoublePublisher m_armSetpointPositionPublisher;
   nt::DoublePublisher m_armSetpointVelocityPublisher;
   nt::DoublePublisher m_armOutputFeedForwardPublisher;
   nt::DoublePublisher m_armCurrentPublisher;
   nt::DoublePublisher m_armAppliedOutputPublisher;

};
