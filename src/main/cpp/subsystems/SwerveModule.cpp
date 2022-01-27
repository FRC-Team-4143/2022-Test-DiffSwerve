// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/SwerveModule.h"
#include "subsystems/DriveSubsystem.h"
#include <frc/geometry/Rotation2d.h>
#include <wpi/numbers>
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include "RobotContainer.h"

SwerveModule::SwerveModule(int driveMotorChannel, int turningMotorChannel, int encoderChannel, std::string name)
    : m_driveMotor(driveMotorChannel),
      m_turningMotor(turningMotorChannel),
      m_encoder(encoderChannel),
      m_name(name){

  m_turningPIDController.EnableContinuousInput(
      units::radian_t(0), units::radian_t(2*wpi::numbers::pi));
}

frc::SwerveModuleState SwerveModule::GetState() {
  return {units::meters_per_second_t{GetDriveMotorSpeed()},
          //frc::Rotation2d(units::radian_t(m_turningEncoder.Get()))};
          frc::Rotation2d(units::radian_t((m_encoder.GetPosition()-m_offset)/360*2*wpi::numbers::pi))};
}
float SwerveModule::GetDriveMotorSpeed(){
    return ((m_driveMotor.GetSelectedSensorVelocity() - m_turningMotor.GetSelectedSensorVelocity()) / 2.0) 
        * (10.0 / 2048) /*Revs per second*/ * ((10 / 88.0) * (54 / 14.0) * (1 / 3.0)) /*Gear Ratios*/ * (4.5 * 0.0254 * wpi::numbers::pi) /*Axle Revs per Second*/;
}

void SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState) {
    double encoderValue = (m_encoder.GetPosition()-m_offset)/360*2*wpi::numbers::pi;
  // Optimize the reference state to avoid spinning further than 90 degrees
  const auto state = frc::SwerveModuleState::Optimize(
      referenceState, units::radian_t(encoderValue));

  // Calculate the drive output from the drive PID controller.
  const auto driveOutput = m_drivePIDController.Calculate(GetDriveMotorSpeed(), state.speed.value());

  // Calculate the turning motor output from the turning PID controller.
  auto turnOutput = m_turningPIDController.Calculate(
      units::radian_t(encoderValue), state.angle.Radians());

  // Set the motor outputs.
  m_driveMotor.Set(driveOutput/(AutoConstants::kMaxSpeed.value())+turnOutput);
  m_turningMotor.Set(-driveOutput/(AutoConstants::kMaxSpeed.value())+turnOutput);

  frc::SmartDashboard::PutNumber (m_name +" Encoder1", encoderValue);
  frc::SmartDashboard::PutNumber (m_name + " Drive Power",driveOutput/AutoConstants::kMaxSpeed.value());
  frc::SmartDashboard::PutNumber (m_name + " Turn Power",turnOutput);
  
}

void SwerveModule::ResetEncoders() {

}

// =========================Wheel Offsets=======================================

void SwerveModule::SetWheelOffset() {
	auto steerPosition = m_encoder.GetPosition();
	frc::Preferences::SetDouble(m_name, steerPosition);
    m_offset = steerPosition;
}

void SwerveModule::LoadWheelOffset() {
	auto steerPosition = frc::Preferences::GetDouble(m_name);
	m_offset = steerPosition;
}

// ================================================================
