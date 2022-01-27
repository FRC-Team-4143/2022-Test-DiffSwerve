// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <iostream>
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : m_gyro{new AHRS(frc::I2C::Port::kMXP)},
      m_frontLeft{kFrontLeftDriveMotorPort, kFrontLeftTurningMotorPort, "frontLeft"},

      m_rearLeft{kRearLeftDriveMotorPort, kRearLeftTurningMotorPort, "rearLeft"},

      m_frontRight{kFrontRightDriveMotorPort, kFrontRightTurningMotorPort, "frontRight"},

      m_rearRight{kRearRightDriveMotorPort, kRearRightTurningMotorPort, "rearRight"},

      m_odometry{kDriveKinematics, units::degree_t(-m_gyro->GetYaw()), frc::Pose2d()} {
        LoadWheelOffsets();
        
        //m_gyro->Calibrate();
        //m_gyro->SetYawAxis(frc::ADIS16470_IMU::IMUAxis::kZ);
      }

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(units::degree_t(-m_gyro->GetYaw()), m_frontLeft.GetState(),
                    m_rearLeft.GetState(), m_frontRight.GetState(),
                    m_rearRight.GetState());
  frc::SmartDashboard::PutNumber ("Gyro", m_gyro->GetYaw());
  

}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {
  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, units::degree_t(-m_gyro->GetYaw()))
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kDriveKinematics.DesaturateWheelSpeeds(&states, AutoConstants::kMaxSpeed);


  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);

  frc::SmartDashboard::PutBoolean ("FieldCentric", fieldRelative);
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         AutoConstants::kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearRight.ResetEncoders();
}

units::degree_t DriveSubsystem::GetHeading() const {
  return units::degree_t(-m_gyro->GetYaw());
}

void DriveSubsystem::ZeroHeading() {
  m_gyro->ZeroYaw();
}

double DriveSubsystem::GetTurnRate() {
  return -m_gyro->GetRate();
}

frc::Pose2d DriveSubsystem::GetPose() {
  return m_odometry.GetPose();
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(pose, frc::Rotation2d(units::degree_t(GetHeading())));
}

// ================================================================

void DriveSubsystem::SetWheelOffsets() {
	m_frontLeft.SetWheelOffset();
	m_rearLeft.SetWheelOffset();
	m_frontRight.SetWheelOffset();
	m_rearRight.SetWheelOffset();
  std::cout << "SetWheelOffsets Complete " << std::endl;
		std::cout.flush();
}

// ================================================================

void DriveSubsystem::LoadWheelOffsets() {
	m_frontLeft.LoadWheelOffset();
	m_rearLeft.LoadWheelOffset();
	m_frontRight.LoadWheelOffset();
	m_rearRight.LoadWheelOffset();
}

// ================================================================

