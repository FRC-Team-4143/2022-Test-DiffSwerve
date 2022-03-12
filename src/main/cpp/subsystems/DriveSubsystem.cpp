// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotController.h>

#define CANIVORE "Default Name"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : m_frontLeft{kFrontLeftDriveMotorPort, kFrontLeftTurningMotorPort, kFrontLeftPot, "frontLeft", "roborio"},

      m_rearLeft{kRearLeftDriveMotorPort, kRearLeftTurningMotorPort, kRearLeftPot, "rearLeft", CANIVORE},

      m_frontRight{kFrontRightDriveMotorPort, kFrontRightTurningMotorPort, kFrontRightPot, "frontRight", "roborio"},

      m_rearRight{kRearRightDriveMotorPort, kRearRightTurningMotorPort, kRearRightPot, "rearRight", CANIVORE},

      m_odometry{kDriveKinematics, GetHeading(), frc::Pose2d()},
      m_fieldCentric{false} {
        LoadWheelOffsets();
        frc::SmartDashboard::PutData("Field", &m_field);
      }

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.
  currentYaw = m_pidgey.GetYaw();

  m_odometry.Update(GetHeading(), m_frontLeft.GetState(),
                    m_rearLeft.GetState(), m_frontRight.GetState(),
                    m_rearRight.GetState());
  frc::SmartDashboard::PutNumber ("Gyro", GetHeading().value());
  frc::SmartDashboard::PutBoolean ("FieldCentric", m_fieldCentric);
  m_field.SetRobotPose(m_odometry.GetPose());
  //Wheel Offset Code;
		if (frc::RobotController::GetUserButton() == 1 && m_counter == 0) {
			SetWheelOffsets();
			m_counter = 100;
      fmt::print("ERRROR: User Button Pressed\n");
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
                          xSpeed, ySpeed, rot, GetHeading())
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot});

  kDriveKinematics.DesaturateWheelSpeeds(&states, ModuleConstants::kMaxSpeed);


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
                                         ModuleConstants::kMaxSpeed);
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
  return units::degree_t(currentYaw);  // was negated
}

void DriveSubsystem::ZeroHeading() {
  m_pidgey.SetYaw(0,30);
}

double DriveSubsystem::GetTurnRate() {
  return m_pidgey.GetRate();
}

frc::Pose2d DriveSubsystem::GetPose() {
  return m_odometry.GetPose();
}

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(pose, frc::Rotation2d(units::degree_t(GetHeading())));
}

void DriveSubsystem::ToggleFieldCentric(){
  m_fieldCentric=!m_fieldCentric;
}

void DriveSubsystem::motorsOff(){
  m_frontLeft.motorsOff();
  m_rearLeft.motorsOff();
  m_frontRight.motorsOff();
  m_rearRight.motorsOff();
}

void DriveSubsystem::GyroCrab(double x, double y, double desiredAngle) {
	double currentAngle = GetHeading().value();
	auto twist = -(desiredAngle - currentAngle);

	while (twist > 180.0) {
		twist -= 360.0;
	}
	while (twist < -180.0) {
		twist += 360.0;
	}

	constexpr double GYRO_P = 0.028*6; //original is 0.007
	constexpr double GYRO_MAX = 0.3*6;

	twist = std::clamp(twist*GYRO_P, -GYRO_MAX, GYRO_MAX);
	Drive(units::meters_per_second_t(x), units::meters_per_second_t(y), units::radians_per_second_t(twist));
}

double DriveSubsystem::GyroRotate() {
	auto yaw = GetHeading().value();
	float desiredangle = 0;

	if (yaw > 45 && yaw < 135) {
		desiredangle = 90;
	}
	else if ((yaw > 135 && yaw < 179) || (yaw < -135 && yaw > -179)) {
		desiredangle = 180;
	}
	else if (yaw > -135 && yaw < -45) {
		desiredangle = -90;
	}
	else if (yaw < 45 && yaw > -45) {
		desiredangle = 0;
	}

	auto twist = desiredangle - yaw;
	while (twist > 180.0) {
		twist -= 360.0;
	}
	while (twist < -180.0) {
		twist += 360.0;
	}

	return twist;
}

// ================================================================

void DriveSubsystem::SetWheelOffsets() {
	m_frontLeft.SetWheelOffset();
	m_rearLeft.SetWheelOffset();
	m_frontRight.SetWheelOffset();
	m_rearRight.SetWheelOffset();
  fmt::print("ERROR: SetWheelOffsets Complete\n");
}

// ================================================================

void DriveSubsystem::LoadWheelOffsets() {
	m_frontLeft.LoadWheelOffset();
	m_rearLeft.LoadWheelOffset();
	m_frontRight.LoadWheelOffset();
	m_rearRight.LoadWheelOffset();
  fmt::print("ERROR: LoadWheelOffsets Complete\n");
}

// ================================================================

