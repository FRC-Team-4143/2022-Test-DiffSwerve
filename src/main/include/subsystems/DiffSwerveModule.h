#pragma once
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/Encoder.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/motorcontrol/Spark.h>
#include <frc/Preferences.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <wpi/numbers>
#include <ctre/Phoenix.h>
#include <string>
#include "Constants.h"
#include <frc/DataLogManager.h>
#include <wpi/DataLog.h>

class DiffSwerveModule {
	using radians_per_second_squared_t =
		units::compound_unit<
			units::radians,
			units::inverse<units::squared<units::second>>
		>;

public:

	DiffSwerveModule(int driveMotorChannel, int turningMotorChannel, int encoderChannel, std::string name, std::string CANbus, wpi::log::DataLog& log);
	DiffSwerveModule(int driveMotorChannel, int turningMotorChannel, int encoderChannel, std::string name, wpi::log::DataLog& log);

	frc::SwerveModuleState GetState();

	double SetDesiredState(const frc::SwerveModuleState& state);

	void ResetEncoders();

	void SetWheelOffset();
	void LoadWheelOffset();
	void motorsOff();
	double GetDriveMotorSpeed(double topSpeed, double bottomSpeed);

	void SetVoltage(double driveMax);

	//wpi::log::DataLog& GetDataLog(wpi::log::DataLog& log);


	wpi::log::DoubleLogEntry m_topMotorCurrent;
	wpi::log::DoubleLogEntry m_bottomMotorCurrent;
	wpi::log::DoubleLogEntry m_topSupplyCurrent;
	wpi::log::DoubleLogEntry m_bottomSupplyCurrent;
	wpi::log::DoubleLogEntry m_wheelSpeed;
	wpi::log::DoubleLogEntry m_topMotorRPM;
	wpi::log::DoubleLogEntry m_bottomMotorRPM;
	wpi::log::DoubleLogEntry m_moduleAngleLog;
	wpi::log::DoubleLogEntry m_expectedSpeed;
	wpi::log::DoubleLogEntry m_expectedAngle;
	wpi::log::DoubleLogEntry m_driveoutput;
	wpi::log::DoubleLogEntry m_drivefeedforward;
	wpi::log::DoubleLogEntry m_turnoutput;
private:

	// We have to use meters here instead of radians due to the fact that
	// ProfiledPIDController's constraints only take in meters per second and
	// meters per second squared.
	/*
	static constexpr units::radians_per_second_t kModuleMaxAngularVelocity =
	units::radians_per_second_t(wpi::numbers::pi * 100.0);  // radians per second
	static constexpr units::unit_t<radians_per_second_squared_t> kModuleMaxAngularAcceleration =
	units::unit_t<radians_per_second_squared_t>{
	wpi::numbers::pi * 2.0 * 100.0
	*/
	static constexpr units::radians_per_second_t kModuleMaxAngularVelocity =
		units::radians_per_second_t(wpi::numbers::pi * 100.0);  // radians per second

	static constexpr units::unit_t<radians_per_second_squared_t> kModuleMaxAngularAcceleration =
		units::unit_t<radians_per_second_squared_t>{
			wpi::numbers::pi * 2.0 * 100.0
		}; // radians per second squared

	WPI_TalonFX m_driveMotor;
	WPI_TalonFX m_turningMotor;
	WPI_CANCoder m_encoder;

	std::string m_name;
	wpi::log::DataLog& m_log;

	frc::SimpleMotorFeedforward<units::meters> m_driveFeedforward{ModuleConstants::ks, ModuleConstants::kv, ModuleConstants::ka};
	//frc::SimpleMotorFeedforward<units::radians> m_turnFeedforward{DriveConstants::kts, DriveConstants::ktv};

	frc2::PIDController m_drivePIDController{
		ModuleConstants::kPModuleDriveController, 0, 0
	};

	frc::ProfiledPIDController<units::radians> m_turningPIDController{
		ModuleConstants::kPModuleTurningController,
		0.0,
		0.0,
		{kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration}
		};

	double m_offset;

	double m_driveVoltage;
	double m_turnVoltage;

	double m_driveSpeed;
	double m_moduleAngle;
	double m_lastDriveVoltage = 0;
};
