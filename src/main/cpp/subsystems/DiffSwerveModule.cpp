#include "subsystems/DiffSwerveModule.h"
#include <frc/smartdashboard/SmartDashboard.h>

// ============================================================================

DiffSwerveModule::DiffSwerveModule(int driveMotorChannel, int turningMotorChannel, int encoderChannel, std::string name, std::string CANbus, wpi::log::DataLog& log)
:   m_driveMotor(driveMotorChannel, CANbus),
    m_turningMotor(turningMotorChannel, CANbus),
    m_encoder(encoderChannel, CANbus),
    m_name(name),
    m_log(log)
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
    
    constexpr double MAX_CURRENT = 80.0;

	SupplyCurrentLimitConfiguration supply{true, MAX_CURRENT, MAX_CURRENT, 10};
	m_driveMotor.ConfigSupplyCurrentLimit(supply);
	m_turningMotor.ConfigSupplyCurrentLimit(supply);

	//StatorCurrentLimitConfiguration stator{true, MAX_CURRENT, MAX_CURRENT, 10};
	//m_driveMotor.ConfigStatorCurrentLimit(stator);
	//m_turningMotor.ConfigStatorCurrentLimit(stator);

    m_turningPIDController.EnableContinuousInput(
        units::radian_t{-wpi::numbers::pi}, units::radian_t(wpi::numbers::pi));
    m_driveMotor.SetNeutralMode(NeutralMode::Coast);
    m_turningMotor.SetNeutralMode(NeutralMode::Coast);

    m_topMotorCurrent = wpi::log::DoubleLogEntry(log, "/"+m_name+"/topMotorCurrent");
    m_bottomMotorCurrent = wpi::log::DoubleLogEntry(log, "/"+m_name+"/bottomMotorCurrent");
    m_wheelSpeed = wpi::log::DoubleLogEntry(log, "/"+m_name+"/wheelSpeed");
    m_topMotorRPM = wpi::log::DoubleLogEntry(log, "/"+m_name+"/topMotorRPM");
    m_bottomMotorRPM = wpi::log::DoubleLogEntry(log, "/"+m_name+"/bottomMotorRPM");
    m_moduleAngleLog = wpi::log::DoubleLogEntry(log, "/"+m_name+"/moduleAngle");
    m_expectedSpeed = wpi::log::DoubleLogEntry(log, "/"+m_name+"/expectedSpeed");
    m_expectedAngle = wpi::log::DoubleLogEntry(log, "/"+m_name+"/expectedAngle");

}

DiffSwerveModule::DiffSwerveModule(int driveMotorChannel, int turningMotorChannel, int encoderChannel, std::string name, wpi::log::DataLog& log)
:   m_driveMotor(driveMotorChannel),
    m_turningMotor(turningMotorChannel),
    m_encoder(encoderChannel),
    m_name(name),
    m_log(log)
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
    
    
    constexpr double MAX_CURRENT = 100.0;

	SupplyCurrentLimitConfiguration supply{true, MAX_CURRENT, MAX_CURRENT, 10};
	m_driveMotor.ConfigSupplyCurrentLimit(supply);
	m_turningMotor.ConfigSupplyCurrentLimit(supply);

	//StatorCurrentLimitConfiguration stator{true, MAX_CURRENT, MAX_CURRENT, 10};
	//m_driveMotor.ConfigStatorCurrentLimit(stator);
	//m_turningMotor.ConfigStatorCurrentLimit(stator);

    m_turningPIDController.EnableContinuousInput(
        units::radian_t{-wpi::numbers::pi}, units::radian_t(wpi::numbers::pi));
    m_driveMotor.SetNeutralMode(NeutralMode::Coast);
    m_turningMotor.SetNeutralMode(NeutralMode::Coast);

    m_topMotorCurrent = wpi::log::DoubleLogEntry(log, "/"+m_name+"/topMotorCurrent");
    m_bottomMotorCurrent = wpi::log::DoubleLogEntry(log, "/"+m_name+"/bottomMotorCurrent");
    m_wheelSpeed = wpi::log::DoubleLogEntry(log, "/"+m_name+"/wheelSpeed");
    m_topMotorRPM = wpi::log::DoubleLogEntry(log, "/"+m_name+"/topMotorRPM");
    m_bottomMotorRPM = wpi::log::DoubleLogEntry(log, "/"+m_name+"/bottomMotorRPM");
    m_moduleAngleLog = wpi::log::DoubleLogEntry(log, "/"+m_name+"/moduleAngle");
}

// ============================================================================

//called from DriveSubsystem Periodic

frc::SwerveModuleState DiffSwerveModule::GetState() {

    double topMotorSpeed = m_driveMotor.GetSelectedSensorVelocity();
    double bottomMotorSpeed = m_turningMotor.GetSelectedSensorVelocity();

    m_driveSpeed = GetDriveMotorSpeed(topMotorSpeed, bottomMotorSpeed);
    m_moduleAngle = (m_encoder.GetAbsolutePosition() - m_offset) / 360 * 2 * wpi::numbers::pi;
    return {units::meters_per_second_t{m_driveSpeed},
        //frc::Rotation2d(units::radian_t(m_turningEncoder.Get()))};
        frc::Rotation2d(units::radian_t(m_moduleAngle))
    };

    //data logging
    m_topMotorCurrent.Append(m_driveMotor.GetOutputCurrent());
    m_bottomMotorCurrent.Append(m_turningMotor.GetOutputCurrent());
    m_wheelSpeed.Append(m_driveSpeed);
    m_topMotorRPM.Append(topMotorSpeed);
    m_bottomMotorRPM.Append(bottomMotorSpeed);
    m_moduleAngleLog.Append(m_moduleAngle);


}

// ============================================================================

double DiffSwerveModule::GetDriveMotorSpeed(double topSpeed, double bottomSpeed) {
    double speed = ((topSpeed - bottomSpeed) / 2.0) 
    * (10.0 / 2048) /*Revs per second*/ * ((10  / 88.0) * (54 / 14.0) * (1 / 3.0)) /*Gear Ratios*/ * (4 * 0.0254 * wpi::numbers::pi * 1.10); //1.1 worn wheels 3/24/22

    frc::SmartDashboard::PutNumber(m_name + " Wheel Speed ", speed);
    
    return speed;
}

// ============================================================================

double DiffSwerveModule::SetDesiredState(const frc::SwerveModuleState& referenceState) {

    // Optimize the reference state to avoid spinning further than 90 degrees
    const auto state{frc::SwerveModuleState::Optimize(
        referenceState, units::radian_t(m_moduleAngle)
    )};

    // Calculate the drive output from the drive PID controller.
    const auto driveOutput{m_drivePIDController.Calculate(m_driveSpeed, state.speed.value())};
    m_expectedSpeed.Append(state.speed.value());

    // Calculate the turning motor output from the turning PID controller.
    auto turnOutput{m_turningPIDController.Calculate(units::radian_t(m_moduleAngle), state.angle.Radians())};
    m_expectedAngle.Append(state.angle.Radians().value());

    turnOutput = std::clamp(turnOutput,-ModuleConstants::kmaxTurnOutput,ModuleConstants::kmaxTurnOutput);

    const auto driveFeedforward{m_driveFeedforward.Calculate(state.speed)};

    auto feedForward = driveFeedforward.value();

    m_driveVoltage =
        driveOutput
        + driveFeedforward.value()
        + DriveConstants::driveMaxVoltage * turnOutput;

    m_turnVoltage =
        -driveOutput
        - driveFeedforward.value()
        + DriveConstants::driveMaxVoltage * turnOutput;

    return std::max(m_driveVoltage,m_turnVoltage);
}

void DiffSwerveModule::SetVoltage(double driveMax){
    m_driveMotor.Set(ControlMode::PercentOutput, m_driveVoltage*driveMax/DriveConstants::driveMaxVoltage);
    m_turningMotor.Set(ControlMode::PercentOutput, m_turnVoltage*driveMax/DriveConstants::driveMaxVoltage);
    //m_driveMotor.SetVoltage(units::voltage::volt_t{m_driveVoltage*driveMax});
    //m_turningMotor.SetVoltage(units::voltage::volt_t{m_turnVoltage*driveMax});
}

// ============================================================================

void DiffSwerveModule::ResetEncoders() {
}

void DiffSwerveModule::motorsOff () {
    m_driveMotor.Set(ControlMode::PercentOutput, 0);
    m_turningMotor.Set(ControlMode::PercentOutput, 0);
}

// =========================Wheel Offsets======================================

void DiffSwerveModule::SetWheelOffset() {
	auto steerPosition{m_encoder.GetAbsolutePosition()};
    fmt::print("ERROR: {} steerPosition {}\n", m_name, steerPosition);
	frc::Preferences::SetDouble(m_name, steerPosition);
    m_offset = steerPosition;
}

// ============================================================================

void DiffSwerveModule::LoadWheelOffset() {
	auto steerPosition{frc::Preferences::GetDouble(m_name)};
    fmt::print("ERROR: {} steerPosition {}\n", m_name, steerPosition);
    m_offset = steerPosition;
}

// ============================================================================
