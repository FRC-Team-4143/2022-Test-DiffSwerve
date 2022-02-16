#include "subsystems/SwerveModule.h"
//#include "subsystems/DriveSubsystem.h"
//#include <frc/geometry/Rotation2d.h>
//#include <wpi/numbers>
//#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <iostream>
//#include "RobotContainer.h"

// ============================================================================

SwerveModule::SwerveModule(int driveMotorChannel, int turningMotorChannel, int encoderChannel, std::string name)
:   m_driveMotor(driveMotorChannel),
    m_turningMotor(turningMotorChannel),
    m_encoder(encoderChannel),
    m_name(name)
{          
    //Reset motors and encoders
    m_driveMotor.ConfigFactoryDefault();
	m_turningMotor.ConfigFactoryDefault();
    m_encoder.ConfigFactoryDefault();
    m_encoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180,20);
	m_encoder.SetStatusFramePeriod(CANCoderStatusFrame_SensorData, 5, 20);
    
    constexpr double MAX_CURRENT = 40.0;

	SupplyCurrentLimitConfiguration supply{true, MAX_CURRENT, MAX_CURRENT, 10};
	//m_driveMotor.ConfigSupplyCurrentLimit(supply);
	//m_turningMotor.ConfigSupplyCurrentLimit(supply);

	StatorCurrentLimitConfiguration stator{true, MAX_CURRENT, MAX_CURRENT, 10};
	//m_driveMotor.ConfigStatorCurrentLimit(stator);
	//m_turningMotor.ConfigStatorCurrentLimit(stator);

    m_turningPIDController.EnableContinuousInput(
        units::radian_t{-wpi::numbers::pi}, units::radian_t(wpi::numbers::pi));
    m_driveMotor.SetNeutralMode(NeutralMode::Coast);
    m_turningMotor.SetNeutralMode(NeutralMode::Coast);
}

// ============================================================================

frc::SwerveModuleState SwerveModule::GetState() {
    return {units::meters_per_second_t{GetDriveMotorSpeed()},
        //frc::Rotation2d(units::radian_t(m_turningEncoder.Get()))};
        frc::Rotation2d(units::radian_t((m_encoder.GetAbsolutePosition() - m_offset) / 360 * 2 * wpi::numbers::pi))
    };
}

// ============================================================================

float SwerveModule::GetDriveMotorSpeed() {
    return ((m_driveMotor.GetSelectedSensorVelocity() - m_turningMotor.GetSelectedSensorVelocity()) / 2.0) 
        * (10.0 / 2048) /*Revs per second*/ * ((10 / 88.0) * (54 / 14.0) * (1 / 3.0)) /*Gear Ratios*/ * (4.5 * 0.0254 * wpi::numbers::pi) /*Axle Revs per Second*/;
}

// ============================================================================

double SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState) {
    double encoderValue{(m_encoder.GetAbsolutePosition()-m_offset)/360*2*wpi::numbers::pi};
 

    // Optimize the reference state to avoid spinning further than 90 degrees
    const auto state{frc::SwerveModuleState::Optimize(
        referenceState, units::radian_t(encoderValue)
    )};

    // Calculate the drive output from the drive PID controller.
    const auto driveOutput{m_drivePIDController.Calculate(GetDriveMotorSpeed(), state.speed.value())};

    // Calculate the turning motor output from the turning PID controller.
    auto turnOutput{m_turningPIDController.Calculate(units::radian_t(encoderValue), state.angle.Radians())};

    

/*
    auto angle = fmod(m_encoder.GetPosition(), 360);
    if (angle > 180){angle -= 360;}
    if (angle < -180){angle += 360;}

    // Calculate turning output from steer error
    auto turn{angle - state.angle.Radians().value()*(180/wpi::numbers::pi)};

    if (std::fabs(turn) < 1) {
	    turn = 0;
	}
    else {
        //turn *= 2000.0 / 180; // simple gain on steering error
	    turn *= 1. / 180.;
	}
  
    turnOutput = turn;
*/

    const auto driveFeedforward{m_driveFeedforward.Calculate(state.speed)};

    // Set the motor outputs

    driveVoltage =
        DriveConstants::driveMaxVoltage * (driveOutput / AutoConstants::kMaxSpeed.value())
        + driveFeedforward.value()
        + DriveConstants::driveMaxVoltage * turnOutput;

     turnVoltage =
        -DriveConstants::driveMaxVoltage * (driveOutput / AutoConstants::kMaxSpeed.value())
        - driveFeedforward.value()
        + DriveConstants::driveMaxVoltage * turnOutput;

    if(!(fabs(state.angle.Radians().value()-encoderValue) < wpi::numbers::pi/4) &&
       !(fabs(state.angle.Radians().value()-encoderValue+(wpi::numbers::pi*2)) < wpi::numbers::pi/4) &&
       !(fabs(state.angle.Radians().value()-encoderValue-(wpi::numbers::pi*2)) < wpi::numbers::pi/4)){
        driveVoltage = 0;
    }

    frc::SmartDashboard::PutNumber(m_name + " Current Angle", encoderValue);
    frc::SmartDashboard::PutNumber(m_name + " Drive Power", driveOutput / AutoConstants::kMaxSpeed.value());
    frc::SmartDashboard::PutNumber(m_name + " Drive Feedforward", driveFeedforward.value());
    frc::SmartDashboard::PutNumber(m_name + " Turn Power", turnOutput);
    frc::SmartDashboard::PutNumber(m_name + " Wanted Angle", state.angle.Radians().value()*(180/wpi::numbers::pi));
    frc::SmartDashboard::PutNumber(m_name + " Angle Error", state.angle.Radians().value()-encoderValue);

  
    frc::SmartDashboard::PutNumber (m_name + " SetVoltage", driveVoltage);
    //frc::SmartDashboard::PutNumber (m_name + " driveFeedForward",driveFeedforward.value());

    return std::max(driveVoltage,turnVoltage);
}

void SwerveModule::SetVoltage(double driveMax){
    m_driveMotor.SetVoltage(units::voltage::volt_t{driveVoltage*driveMax});
    m_turningMotor.SetVoltage(units::voltage::volt_t{turnVoltage*driveMax});
}

// ============================================================================

void SwerveModule::ResetEncoders() {
}

// =========================Wheel Offsets======================================

void SwerveModule::SetWheelOffset() {

    if(m_encoder.ConfigMagnetOffset(0,20)) {
        std::cout << m_name << " config mag offset 0 failed" << std::endl;
		std::cout.flush();
    };
	auto steerPosition{-m_encoder.GetAbsolutePosition()};
    std::cout << m_name << " steerPosition " << steerPosition << std::endl;
	frc::Preferences::SetDouble(m_name, steerPosition);
    if(m_encoder.ConfigMagnetOffset(steerPosition,20)) {
        std::cout << m_name << " config mag offset steerPosition failed" << std::endl;
		std::cout.flush();
    };
    m_offset = 0;
}

// ============================================================================

void SwerveModule::LoadWheelOffset() {
	auto steerPosition{frc::Preferences::GetDouble(m_name)};
	m_encoder.ConfigMagnetOffset(steerPosition,20);
    m_offset = 0;
}

// ============================================================================
