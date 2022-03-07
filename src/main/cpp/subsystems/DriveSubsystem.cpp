// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
//#include <iostream>
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotController.h>

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : m_gyro{new AHRS(frc::I2C::Port::kMXP)},
      m_frontLeft{kFrontLeftDriveMotorPort, kFrontLeftTurningMotorPort, kFrontLeftPot, "frontLeft", "Default Name"},

      m_rearLeft{kRearLeftDriveMotorPort, kRearLeftTurningMotorPort, kRearLeftPot, "rearLeft", "Default Name"},

      m_frontRight{kFrontRightDriveMotorPort, kFrontRightTurningMotorPort, kFrontRightPot, "frontRight", "Default Name"},

      m_rearRight{kRearRightDriveMotorPort, kRearRightTurningMotorPort, kRearRightPot, "rearRight", "Default Name"},

      m_odometry{kDriveKinematics, units::degree_t(-m_gyro->GetYaw()), frc::Pose2d()},
      m_fieldCentric{false} {
        LoadWheelOffsets();
        frc::SmartDashboard::PutData("Field", &m_field);
        //m_gyro->Calibrate();
        //m_gyro->SetYawAxis(frc::ADIS16470_IMU::IMUAxis::kZ);
      }

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  m_odometry.Update(units::degree_t(-m_gyro->GetYaw()), m_frontLeft.GetState(),
                    m_rearLeft.GetState(), m_frontRight.GetState(),
                    m_rearRight.GetState());
  frc::SmartDashboard::PutNumber ("Gyro", -m_gyro->GetYaw());
  frc::SmartDashboard::PutBoolean ("FieldCentric", m_fieldCentric);
  m_field.SetRobotPose(m_odometry.GetPose());
  //Wheel Offset Code;
		if (frc::RobotController::GetUserButton() == 1 && m_counter == 0) {
			SetWheelOffsets();
			m_counter = 100;
			//std::cout << "User Button Pressed" << std::endl;
			//std::cout.flush();
		}

		if (m_counter > 0) {
			m_counter -= 1;
		}
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot) {
  auto states = kDriveKinematics.ToSwerveModuleStates(
      m_fieldCentric ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, units::degree_t(-m_gyro->GetYaw()))
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kDriveKinematics.DesaturateWheelSpeeds(&states, AutoConstants::kMaxSpeed);


  auto [fl, fr, bl, br] = states;

  double flMax = m_frontLeft.SetDesiredState(fl);
  double frMax = m_frontRight.SetDesiredState(fr);
  double blMax = m_rearLeft.SetDesiredState(bl);
  double brMax = m_rearRight.SetDesiredState(br);
  
  double driveMax = std::max(std::max(blMax,brMax),std::max(flMax,frMax));
  if(driveMax>DriveConstants::driveMaxVoltage)
    driveMax=DriveConstants::driveMaxVoltage/driveMax;
  else
    driveMax=1;
  
  m_frontLeft.SetVoltage(driveMax);
  m_frontRight.SetVoltage(driveMax);
  m_rearLeft.SetVoltage(driveMax);
  m_rearRight.SetVoltage(driveMax);
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         AutoConstants::kMaxSpeed);
  double flMax = m_frontLeft.SetDesiredState(desiredStates[0]);
  double frMax = m_frontRight.SetDesiredState(desiredStates[1]);
  double blMax = m_rearLeft.SetDesiredState(desiredStates[2]);
  double brMax = m_rearRight.SetDesiredState(desiredStates[3]);

  double driveMax = std::max(std::max(blMax,brMax),std::max(flMax,frMax));
  if(driveMax>DriveConstants::driveMaxVoltage)
    driveMax=DriveConstants::driveMaxVoltage/driveMax;
  else
    driveMax=1;
  
  m_frontLeft.SetVoltage(driveMax);
  m_frontRight.SetVoltage(driveMax);
  m_rearLeft.SetVoltage(driveMax);
  m_rearRight.SetVoltage(driveMax);
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
  //m_odometry.ResetPosition(pose, m_gyro->GetRotation2d());
}

void DriveSubsystem::ToggleFieldCentric(){
  m_fieldCentric=!m_fieldCentric;
}

// ================================================================

void DriveSubsystem::SetWheelOffsets() {
	m_frontLeft.SetWheelOffset();
	m_rearLeft.SetWheelOffset();
	m_frontRight.SetWheelOffset();
	m_rearRight.SetWheelOffset();
  //std::cout << "SetWheelOffsets Complete " << std::endl;
  //std::cout.flush();
}

// ================================================================

void DriveSubsystem::LoadWheelOffsets() {
	m_frontLeft.LoadWheelOffset();
	m_rearLeft.LoadWheelOffset();
	m_frontRight.LoadWheelOffset();
	m_rearRight.LoadWheelOffset();
  //std::cout << "LoadWheelOffsets Complete " << std::endl;
  //std::cout.flush();
}

// ================================================================

