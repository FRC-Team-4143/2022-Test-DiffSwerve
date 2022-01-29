// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/acceleration.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/time.h>
#include <units/velocity.h>
#include <units/voltage.h>
#include <wpi/numbers>

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
constexpr int kFrontLeftDriveMotorPort = 22;
constexpr int kRearLeftDriveMotorPort = 24;
constexpr int kFrontRightDriveMotorPort = 28;
constexpr int kRearRightDriveMotorPort = 26;

constexpr int kFrontLeftTurningMotorPort = 21;
constexpr int kRearLeftTurningMotorPort = 23;
constexpr int kFrontRightTurningMotorPort = 27;
constexpr int kRearRightTurningMotorPort = 25;

constexpr int kFrontLeftPot = 1;
constexpr int kFrontRightPot = 4;
constexpr int kRearLeftPot = 2;
constexpr int kRearRightPot = 3;

/*
constexpr int kFrontLeftTurningEncoderPorts[2]{0, 1};
constexpr int kRearLeftTurningEncoderPorts[2]{2, 3};
constexpr int kFrontRightTurningEncoderPorts[2]{4, 5};
constexpr int kRearRightTurningEncoderPorts[2]{6, 7};

constexpr bool kFrontLeftTurningEncoderReversed = false;
constexpr bool kRearLeftTurningEncoderReversed = true;
constexpr bool kFrontRightTurningEncoderReversed = false;
constexpr bool kRearRightTurningEncoderReversed = true;

constexpr int kFrontLeftDriveEncoderPorts[2]{8, 9};
constexpr int kRearLeftDriveEncoderPorts[2]{10, 11};
constexpr int kFrontRightDriveEncoderPorts[2]{12, 13};
constexpr int kRearRightDriveEncoderPorts[2]{14, 15};

constexpr bool kFrontLeftDriveEncoderReversed = false;
constexpr bool kRearLeftDriveEncoderReversed = true;
constexpr bool kFrontRightDriveEncoderReversed = false;
constexpr bool kRearRightDriveEncoderReversed = true;
*/


//constexpr auto ks = 0.64263_V;
constexpr auto ks = .64263_V;
constexpr auto kv = 2.5 * 1_V * 1_s / 1_m;
constexpr auto ka = 0 * 1_V * 1_s * 1_s / 1_m;

// Example value only - as above, this must be tuned for your drive!
//constexpr double kPFrontLeftVel = 0.5;
// constexpr double kPRearLeftVel = 0.5;
// constexpr double kPFrontRightVel = 0.5;
// constexpr double kPRearRightVel = 0.5;
}  // namespace DriveConstants

namespace ModuleConstants {
//constexpr int kEncoderCPR = 1024;
constexpr double kWheelDiameterMeters = 0.1143;
//constexpr double kDriveEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    //(kWheelDiameterMeters * wpi::numbers::pi) /
    //static_cast<double>(kEncoderCPR);

//stexpr double kTurningEncoderDistancePerPulse =
    // Assumes the encoders are directly mounted on the wheel shafts
    //(wpi::numbers::pi * 2) / static_cast<double>(kEncoderCPR);

constexpr double kPModuleTurningController = 0.55;
constexpr double kPModuleDriveController = 0.025;
}  // namespace ModuleConstants

namespace AutoConstants {
using radians_per_second_squared_t =
    units::compound_unit<units::radians,
                         units::inverse<units::squared<units::second>>>;

constexpr auto kMaxSpeed = units::meters_per_second_t(4.5);
constexpr auto kMaxAcceleration = units::meters_per_second_squared_t(3);
constexpr auto kMaxAngularSpeed = units::radians_per_second_t(8);
constexpr auto kMaxAngularAcceleration =
    units::unit_t<radians_per_second_squared_t>(3.142);

constexpr double kPXController = 0.5;
constexpr double kPYController = 0.5;
constexpr double kPThetaController = 0.5;

//

extern const frc::TrapezoidProfile<units::radians>::Constraints
    kThetaControllerConstraints;

}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
}  // namespace OIConstants
