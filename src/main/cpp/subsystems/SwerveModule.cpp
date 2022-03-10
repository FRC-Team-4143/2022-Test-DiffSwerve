#include "subsystems/SwerveModule.h"
#include <frc/smartdashboard/SmartDashboard.h>

// ============================================================================

SwerveModule::SwerveModule(int driveMotorChannel, int turningMotorChannel, int encoderChannel, std::string name, std::string CANbus)
:   m_driveMotor(driveMotorChannel, CANbus),
    m_turningMotor(turningMotorChannel, CANbus),
    m_encoder(encoderChannel, CANbus),
    m_name(name)
{          
    //Reset motors and encoders
    m_driveMotor.ConfigFactoryDefault();
	m_turningMotor.ConfigFactoryDefault();
    m_encoder.ConfigFactoryDefault();
    m_encoder.ConfigAbsoluteSensorRange(AbsoluteSensorRange::Signed_PlusMinus180,20);
	//m_encoder.SetStatusFramePeriod(CANCoderStatusFrame_SensorData, 5, 20);

    m_driveMotor.ConfigVoltageCompSaturation(DriveConstants::driveMaxVoltage); 
    m_driveMotor.EnableVoltageCompensation(true);
    m_turningMotor.ConfigVoltageCompSaturation(DriveConstants::driveMaxVoltage); 
    m_turningMotor.EnableVoltageCompensation(true);
 
    
    //constexpr double MAX_CURRENT = 40.0;

	//SupplyCurrentLimitConfiguration supply{true, MAX_CURRENT, MAX_CURRENT, 10};
	//m_driveMotor.ConfigSupplyCurrentLimit(supply);
	//m_turningMotor.ConfigSupplyCurrentLimit(supply);

	//StatorCurrentLimitConfiguration stator{true, MAX_CURRENT, MAX_CURRENT, 10};
	//m_driveMotor.ConfigStatorCurrentLimit(stator);
	//m_turningMotor.ConfigStatorCurrentLimit(stator);

    m_turningPIDController.EnableContinuousInput(
        units::radian_t{-wpi::numbers::pi}, units::radian_t(wpi::numbers::pi));
    m_driveMotor.SetNeutralMode(NeutralMode::Coast);
    m_turningMotor.SetNeutralMode(NeutralMode::Coast);
}

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
	//m_encoder.SetStatusFramePeriod(CANCoderStatusFrame_SensorData, 5, 20);

    m_driveMotor.ConfigVoltageCompSaturation(DriveConstants::driveMaxVoltage); 
    m_driveMotor.EnableVoltageCompensation(true);
    m_turningMotor.ConfigVoltageCompSaturation(DriveConstants::driveMaxVoltage); 
    m_turningMotor.EnableVoltageCompensation(true);
    
    
    //constexpr double MAX_CURRENT = 40.0;

	//SupplyCurrentLimitConfiguration supply{true, MAX_CURRENT, MAX_CURRENT, 10};
	//m_driveMotor.ConfigSupplyCurrentLimit(supply);
	//m_turningMotor.ConfigSupplyCurrentLimit(supply);

	//StatorCurrentLimitConfiguration stator{true, MAX_CURRENT, MAX_CURRENT, 10};
	//m_driveMotor.ConfigStatorCurrentLimit(stator);
	//m_turningMotor.ConfigStatorCurrentLimit(stator);

    m_turningPIDController.EnableContinuousInput(
        units::radian_t{-wpi::numbers::pi}, units::radian_t(wpi::numbers::pi));
    m_driveMotor.SetNeutralMode(NeutralMode::Coast);
    m_turningMotor.SetNeutralMode(NeutralMode::Coast);
}

// ============================================================================

//called from DriveSubsystem Periodic

frc::SwerveModuleState SwerveModule::GetState() {
    m_driveSpeed = GetDriveMotorSpeed();
    m_moduleAngle = (m_encoder.GetAbsolutePosition() - m_offset) / 360 * 2 * wpi::numbers::pi;
    return {units::meters_per_second_t{m_driveSpeed},
        //frc::Rotation2d(units::radian_t(m_turningEncoder.Get()))};
        frc::Rotation2d(units::radian_t(m_moduleAngle))
    };
}

// ============================================================================

double SwerveModule::GetDriveMotorSpeed() {
    double speed = ((m_driveMotor.GetSelectedSensorVelocity() - m_turningMotor.GetSelectedSensorVelocity()) / 2.0) 
    * (10.0 / 2048) /*Revs per second*/ * ((10  / 88.0) * (54 / 14.0) * (1 / 3.0)) /*Gear Ratios*/ * (4 * 0.0254 * wpi::numbers::pi);

    frc::SmartDashboard::PutNumber(m_name + " Wheel Speed ", speed);
    
    return speed;
}

// ============================================================================

double SwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState) {

    // Optimize the reference state to avoid spinning further than 90 degrees
    const auto state{frc::SwerveModuleState::Optimize(
        referenceState, units::radian_t(m_moduleAngle)
    )};

    // Calculate the drive output from the drive PID controller.
    const auto driveOutput{m_drivePIDController.Calculate(m_driveSpeed, state.speed.value())};

    // Calculate the turning motor output from the turning PID controller.
    auto turnOutput{m_turningPIDController.Calculate(units::radian_t(m_moduleAngle), state.angle.Radians())};

    turnOutput = std::clamp(turnOutput,-ModuleConstants::kmaxTurnOutput,ModuleConstants::kmaxTurnOutput);

    const auto driveFeedforward{m_driveFeedforward.Calculate(state.speed)};

    // Set the motor outputs
    /*if(!(fabs(state.angle.Radians().value()-encoderValue) < wpi::numbers::pi/4) &&
       !(fabs(state.angle.Radians().value()-encoderValue+(wpi::numbers::pi*2)) < wpi::numbers::pi/4) &&
       !(fabs(state.angle.Radians().value()-encoderValue-(wpi::numbers::pi*2)) < wpi::numbers::pi/4)){
        driveVoltage = DriveConstants::driveMaxVoltage * turnOutput;
        turnVoltage = DriveConstants::driveMaxVoltage * turnOutput;
    } else */ {
        m_driveVoltage =
            driveOutput
            + driveFeedforward.value()
            + DriveConstants::driveMaxVoltage * turnOutput;

        m_turnVoltage =
            -driveOutput
            - driveFeedforward.value()
            + DriveConstants::driveMaxVoltage * turnOutput;
    }



    frc::SmartDashboard::PutNumber(m_name + " m_moduleAngle", m_moduleAngle);
    frc::SmartDashboard::PutNumber(m_name + " driveFeedforward", driveFeedforward.value());
    frc::SmartDashboard::PutNumber(m_name + " turnOutput", turnOutput * DriveConstants::driveMaxVoltage);
    frc::SmartDashboard::PutNumber(m_name + " m_driveVoltage", m_driveVoltage);
    frc::SmartDashboard::PutNumber(m_name + " driveOutput", driveOutput);

    return std::max(m_driveVoltage,m_turnVoltage);
}

void SwerveModule::SetVoltage(double driveMax){
    m_driveMotor.Set(ControlMode::PercentOutput, m_driveVoltage*driveMax/DriveConstants::driveMaxVoltage);
    m_turningMotor.Set(ControlMode::PercentOutput, m_turnVoltage*driveMax/DriveConstants::driveMaxVoltage);
    //m_driveMotor.SetVoltage(units::voltage::volt_t{m_driveVoltage*driveMax});
    //m_turningMotor.SetVoltage(units::voltage::volt_t{m_turnVoltage*driveMax});
}

// ============================================================================

void SwerveModule::ResetEncoders() {
}

// =========================Wheel Offsets======================================

void SwerveModule::SetWheelOffset() {
	auto steerPosition{m_encoder.GetAbsolutePosition()};
    fmt::print("ERROR: {} steerPosition {}\n", m_name, steerPosition);
	frc::Preferences::SetDouble(m_name, steerPosition);
    m_offset = steerPosition;
}

// ============================================================================

void SwerveModule::LoadWheelOffset() {
	auto steerPosition{frc::Preferences::GetDouble(m_name)};
    fmt::print("ERROR: {} steerPosition {}\n", m_name, steerPosition);
    m_offset = steerPosition;
}

// ============================================================================
