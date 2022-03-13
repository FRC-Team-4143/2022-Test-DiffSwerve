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

class SwerveModule {
	using radians_per_second_squared_t =
		units::compound_unit<
			units::radians,
			units::inverse<units::squared<units::second>>
		>;

public:

	SwerveModule(int driveMotorChannel, int turningMotorChannel, int encoderChannel, std::string name, std::string CANbus);
	SwerveModule(int driveMotorChannel, int turningMotorChannel, int encoderChannel, std::string name);

	frc::SwerveModuleState GetState();

	double SetDesiredState(const frc::SwerveModuleState& state);

	void ResetEncoders();

	void SetWheelOffset();
	void LoadWheelOffset();
	void motorsOff();
	double GetDriveMotorSpeed();

	void SetVoltage(double driveMax);

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
};
