// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/Encoder.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <wpi/numbers>
#include <ctre/Phoenix.h>
#include <string>
#include <frc/Preferences.h>
#include "Constants.h"

class SwerveModule {
  using radians_per_second_squared_t =
      units::compound_unit<units::radians,
      units::inverse<units::squared<units::second>>>;

 public:
  SwerveModule(int driveMotorChannel, int turningMotorChannel, std::string name);

  frc::SwerveModuleState GetState();

  void SetDesiredState(const frc::SwerveModuleState& state);

  void ResetEncoders();

  void SetWheelOffset();
  void LoadWheelOffset();

 private:
  // We have to use meters here instead of radians due to the fact that
  // ProfiledPIDController's constraints only take in meters per second and
  // meters per second squared.

  static constexpr units::radians_per_second_t kModuleMaxAngularVelocity =
      units::radians_per_second_t(wpi::numbers::pi);  // radians per second
  static constexpr units::unit_t<radians_per_second_squared_t>
      kModuleMaxAngularAcceleration =
        units::unit_t<radians_per_second_squared_t>(
        wpi::numbers::pi * 2.0);  // radians per second squared

  WPI_TalonSRX m_driveMotor;
  WPI_TalonSRX m_turningMotor;

  std::string m_name;

  frc2::PIDController m_drivePIDController{
      ModuleConstants::kPModuleDriveController, 0, 0};
  frc::ProfiledPIDController<units::radians> m_turningPIDController{
      ModuleConstants::kPModuleTurningController,
      0.0,
      0.0,
      {kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}};

  double m_offset;

};
