// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/CANSparkMax.h>
#include <ctre/phoenix6/core/CoreTalonFX.hpp>
#include <ctre/phoenix6/configs/Configs.hpp>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/angle.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>

#include <numbers>

#pragma once

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants.  This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace DriveConstants {
// Driving Parameters - Note that these are not the maximum capable speeds of
// the robot, rather the allowed maximum speeds
constexpr units::meters_per_second_t kDefaultMaxSpeed = 4.8_mps;
constexpr units::radians_per_second_t kDefaultMaxAngularSpeed{2 * std::numbers::pi};

constexpr double kDirectionSlewRate  = 1.2;   // radians per second
constexpr double kMagnitudeSlewRate  = 1.8;   // percent per second (1 = 100%)
constexpr double kRotationalSlewRate = 2.0;   // percent per second (1 = 100%)

// Chassis configuration
   /* ** NOTE ** If you change these values, make sure to also check the*/
   /*            kHolonomicPathFollowerConfigDriveBaseRadius constant.  */
   /*            It is defined to be "The radius of the drive base in   */
   /*            meters.  For swerve drive, this is the distance from   */
   /*            the center of the robot to the furthest module."       */
constexpr units::meter_t kTrackWidth = 0.530225_m;  // Distance between centers of right and left wheels on robot
constexpr units::meter_t kWheelBase  = 0.52705_m;   // Distance between centers of front and back wheels on robot

// SPARK MAX CAN IDs
constexpr int kFrontLeftDrivingCanId  = 2;
constexpr int kFrontRightDrivingCanId = 4;
constexpr int kRearRightDrivingCanId  = 6;
constexpr int kRearLeftDrivingCanId   = 8;

constexpr int kFrontLeftTurningCanId  = 1;
constexpr int kFrontRightTurningCanId = 3;
constexpr int kRearRightTurningCanId  = 5;
constexpr int kRearLeftTurningCanId   = 7;

constexpr int kPigeonGyroCanId = 10;
constexpr const char kPigeonBusName[] = "rio";

// Constants used with the Aiming PID Controller used to lock rotation to point at the speaker.
constexpr double kSpeakerAimP = 1.0;
constexpr double kSpeakerAimI = 0.0;
constexpr double kSpeakerAimD = 0.0;

constexpr units::radian_t kSpeakerAimTolerance = units::radian_t{5_deg};

}  // namespace DriveConstants

namespace ModuleConstants {

// Invert the turning encoder.
constexpr bool kTurningEncoderInverted = false;

// Invert the turning motor direction.
constexpr bool kTurningMotorInverted = true;

// Calculations required for driving motor conversion factors and feed forward
constexpr double kDrivingMotorFreeSpeedRps = 5676.0 / 60;  // NEO free speed is 5676 RPM

constexpr units::meter_t kWheelDiameter      = 0.1016_m;  // 4 inches in meters
constexpr units::meter_t kWheelCircumference = kWheelDiameter * std::numbers::pi;

// Swerve Drive Specialist Driving Gear Ratio is 6.12:1
constexpr double kDrivingMotorReduction  = 6.12;
constexpr double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumference.value()) / kDrivingMotorReduction;

constexpr double kDrivingEncoderPositionFactor = (kWheelDiameter.value() * std::numbers::pi) / kDrivingMotorReduction;  // meters
constexpr double kDrivingEncoderVelocityFactor = ((kWheelDiameter.value() * std::numbers::pi) / kDrivingMotorReduction) / 60.0;  // meters per second

constexpr double kTurningEncoderPositionFactor = (2 * std::numbers::pi);  // radians
constexpr double kTurningEncoderVelocityFactor =  (2 * std::numbers::pi) / 60.0;  // radians per second

constexpr units::radian_t kTurningEncoderPositionPIDMinInput = 0_rad;
constexpr units::radian_t kTurningEncoderPositionPIDMaxInput = units::radian_t{kTurningEncoderPositionFactor};

constexpr double kDrivingP = 0.04;
constexpr double kDrivingI = 0;
constexpr double kDrivingD = 0;
constexpr double kDrivingFF = (1 / kDriveWheelFreeSpeedRps);
constexpr double kDrivingMinOutput = -1;
constexpr double kDrivingMaxOutput = 1;

constexpr double kTurningP = 0.75;  // was originally 1, seem to make it jittery when turning wildly...
constexpr double kTurningI = 0;
constexpr double kTurningD = 0.01;  // was orignally 0, could probably be larger..
constexpr double kTurningFF = 0;
constexpr double kTurningMinOutput = -1;
constexpr double kTurningMaxOutput = 1;

constexpr rev::CANSparkMax::IdleMode kDrivingMotorIdleMode = rev::CANSparkMax::IdleMode::kBrake;
constexpr rev::CANSparkMax::IdleMode kTurningMotorIdleMode = rev::CANSparkMax::IdleMode::kBrake;

constexpr units::ampere_t kDrivingMotorCurrentLimit = 60_A;
constexpr units::ampere_t kTurningMotorCurrentLimit = 40_A;
}  // namespace ModuleConstants

namespace IntakeConstants {

// Intake motor spark max can id.
constexpr int kIntakeMotorCanId = 9;

// Settings for the intake motor.

// Intake motor idle mode state.
constexpr rev::CANSparkMax::IdleMode kIntakeMotorIdleMode = rev::CANSparkMax::IdleMode::kBrake;

// Intake motor current limit.
constexpr units::ampere_t kInakeMotorCurrentLimit = 35_A;

// Calculations required for intake motor conversion factors and feed forward
constexpr double kIntakeMotorFreeSpeedRps = 11000.0 / 60;

/* The maximum and minimum speed of the intake in RPMs.*/
constexpr units::revolutions_per_minute_t kIntakeMinimumSpeed = -9000_rpm;
constexpr units::revolutions_per_minute_t kIntakeMaximumSpeed = 9000_rpm;   // NEO 550 free speed is 11000 RPM so less than that is good.

   /* This is the reduction between the motor output and the encoder    */
   /* input.  Since using hall effect in the motor it is 1:1.  The speed*/
   /* at the intake wheels is 1/4 this value.                           */
constexpr double kIntakeMotorReduction  = 1.0;
constexpr double kIntakeEncoderPositionFactor = ((1.0)/(kIntakeMotorReduction));  // rotations
constexpr double kIntakeEncoderVelocityFactor = ((1.0)/(kIntakeMotorReduction));  // rotations per minute

// Intake motor PID constants.
constexpr double kIntakeP = 1.9161e-05;  // seems like some examples just make this an arbitrary number, i have see 1.0, 0.04, 0.03???
constexpr double kIntakeI = 0;
constexpr double kIntakeD = 0;
constexpr double kIntakeFF = 0;
constexpr double kIntakeMinOutput = -1;
constexpr double kIntakeMaxOutput = 1;

// Intake Simple Motor Feedforward constants.  These were captured using SysId
constexpr units::volt_t kIntakekS = 0.52013_V;                        // Voltage need to over come static friction
constexpr auto kIntakekV          = (0.059500_V * 1_s)/ 1_tr;         // Voltage need to hold/cruise at a constant voltage, while overcoming counter-electromotive force
constexpr auto kIntakekA          = (0.0097603_V * 1_s * 1_s)/ 1_tr;  // voltage need to induce a given acceleration in the motor shaft

// Intake note detection digital input channel.
constexpr int kNoteDetectInputChannel = 0;
} // namespace IntakeConstants

namespace ShooterConstants {

// Shooter motor can ids for the leader and follower motors.
constexpr int kShooterLeftMotorCanId  = 12;
constexpr int kShooterRightMotorCanId = 11;

constexpr const char kShooterBusName[] = "rio";

/* The maximum and minimum speed of the intake in RPMs.*/
constexpr units::revolutions_per_minute_t kShooterMinimumSpeed = -6250_rpm;
constexpr units::revolutions_per_minute_t kShooterMaximumSpeed = 6250_rpm;   // Falcon 500 free speed is 6380 RPM so less than that is good.

// Shooter motors current limits.
constexpr double kShooterMotorCurrentLimit = 80.0;

// Shooter left motor PID constants.
constexpr double kShooterLeftP = 0.12884;
constexpr double kShooterLeftI = 0;
constexpr double kShooterLeftD = 0;

// Shooter left Simple Motor Feedforward constants.  These were captured using SysId
constexpr units::volt_t kShooterLeftkS = 0.021881_V;
constexpr auto kShooterLeftkV          = (0.12500_V * 1_s)/ 1_tr;
constexpr auto kShooterLeftkA          = (0.037387_V * 1_s * 1_s)/ 1_tr;

// Shooter right motor PID constants.
constexpr double kShooterRightP = 0.12884;
constexpr double kShooterRightI = 0;
constexpr double kShooterRightD = 0;

// Shooter right Simple Motor Feedforward constants.  These were captured using SysId
constexpr units::volt_t  kShooterRightkS = 0.021881_V;
constexpr auto kShooterRightkV           = (0.12500_V * 1_s)/ 1_tr;
constexpr auto kShooterRightkA           = (0.037387_V * 1_s * 1_s)/ 1_tr;

} // namespace ShooterConstants

namespace OdometryConstants {

   /**
    * Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
    * matrix is in the form [x, y, theta], with units in meters and radians, then meters.
    */
   constexpr units::meter_t  kXStateStdDev       = 0.1_m;
   constexpr units::meter_t  kYStateStdDev       = 0.1_m;
   constexpr units::degree_t kHeadingStateStdDev = 4.5_deg;  // this is converted to radians in the odometry subsystem

   /**
    * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
    * less. This matrix is in the form [x, y, theta], with units in meters and radians.
    */
   constexpr units::meter_t  kXVisionStdDev       = 1.0_m;
   constexpr units::meter_t  kYVisionStdDev       = 1.0_m;
   constexpr units::degree_t kHeadingVisionStdDev = 30_deg; // this is converted to radians in the odometry subsystem


   constexpr double kHolonomicPathFollowerConfigTranslationP = 5.0;
   constexpr double kHolonomicPathFollowerConfigTranslationI = 0.0;
   constexpr double kHolonomicPathFollowerConfigTranslationD = 0.0;

   constexpr double kHolonomicPathFollowerConfigRotationP = 5.0;
   constexpr double kHolonomicPathFollowerConfigRotationI = 0.0;
   constexpr double kHolonomicPathFollowerConfigRotationD = 0.0;

   constexpr units::meters_per_second_t kHolonomicPathFollowerConfigMaxSpeed        = 3.25_mps;
   constexpr units::meter_t             kHolonomicPathFollowerConfigDriveBaseRadius = 0.745_m / 2;

} // namespace OdometryConstants


namespace ArmConstants {

   /* Conversion factor for the Arm encoder to convert to Radians and   */
   /* Radians per second.                                               */
   constexpr double kArmEncoderPositionFactor = (2 * std::numbers::pi);  // radians
   constexpr double kArmEncoderVelocityFactor =  (2 * std::numbers::pi) / 60.0;  // radians per second

   constexpr units::degree_t kArmMinimumAngle = -1.0_deg;
   constexpr units::degree_t kArmMaximumAngle = 90_deg;
   constexpr units::degree_t kArmFeedforwardOffsetAngle = 13.5_deg;

   constexpr units::radian_t kArmEncoderPositionPIDMinInput = 0_rad;
   constexpr units::radian_t kArmEncoderPositionPIDMaxInput = units::radian_t{kArmEncoderPositionFactor};

   /* Arm PID constants.            */
   constexpr double kArmP  = 0.785;
   constexpr double kArmI  = 0.0;
   constexpr double kArmD  = 0.1675;//0.155;
   constexpr double kArmFF = 0.0;
   constexpr double kArmMinOutput = -1;
   constexpr double kArmMaxOutput = 1;

   constexpr units::radians_per_second_t kArmMaxVelocity             = units::radians_per_second_t{60_deg_per_s};  // 65
   constexpr units::radians_per_second_squared_t kArmMaxAcceleration = units::radians_per_second_squared_t{75_deg_per_s_sq};

   constexpr int kArmLeaderCanId  = 13;
   constexpr int kArmFollwerCanId = 15;

   constexpr rev::CANSparkMax::IdleMode kArmMotorIdleMode = rev::CANSparkMax::IdleMode::kBrake;

   constexpr units::ampere_t kLeaderMotorCurrentLimit   = 60_A;
   constexpr units::ampere_t kFollowerMotorCurrentLimit = 60_A;

//xxx need to figure out these values???  maybe start with initial values form recalc...
   constexpr units::volt_t kS = 0.100_V;
   constexpr units::volt_t kG = 0.785_V;
   constexpr auto kV = 1.30_V * 1_s / 1_rad;
   constexpr auto kA = 0.05_V * 1_s * 1_s / 1_rad;

} // namespace ArmContrants

namespace LiftConstants {

   constexpr int kLiftCanId  = 14;

} // namespace LiftContrants

namespace OIConstants
{
   constexpr int kDriverControllerPort = 0;
   constexpr double kDriveDeadband = 0.1;
}  // namespace OIConstants
