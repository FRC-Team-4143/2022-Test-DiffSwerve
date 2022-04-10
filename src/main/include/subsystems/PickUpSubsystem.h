#pragma once
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include <frc/motorcontrol/PWMTalonFX.h>
#include "Constants.h"
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/XboxController.h>
#include "DriveSubsystem.h"

class PickUpSubsystem : public frc2::SubsystemBase {
public:
	PickUpSubsystem(frc::XboxController* controller);

	void Periodic() override;

	void PickUpExtend();
	void PickUpExtendStart();
	void PickUpBounce();
	void PickUpRetract();
	void PickUpToggle();

	void RollerIn();
	void RollerOut();
	void RollerOff();

	void IndexerOn();
	void IndexerOff();
	void IndexerRev();
	void IndexerLoad();

	void ShooterOn();
	void ShooterOnManual();
	void ShooterOnLimeLight();
	void ShooterOnLimeLightAuto();
	void ShooterOff();
	void ShooterFaster();
	void ShooterSlower();
	void SetShooterSpeed(double shooterSpeed);

	void ShooterFar();
	void ShooterClose();
	void ShooterDistToggle();

	void SetDist(double value);
	void SetOffset(double value);

private:

	//TalonSRX m_roller;
	TalonSRX m_index1;
	TalonSRX m_index2;
	frc::PWMTalonFX m_rollerpwm;


	frc::DoubleSolenoid m_upperSolenoid;
	frc::DoubleSolenoid m_lowerSolenoid;
	frc::Compressor m_compressor;

	rev::CANSparkMax m_shooter1;
	rev::CANSparkMax m_shooter2;
	frc::MotorControllerGroup m_shooter;
	rev::CANSparkMax m_backSpinShooter;
	frc::DoubleSolenoid m_shooterSolenoid;

	double m_shooterSpeed=PickUpConstants::kLongSpeedSlow;
	double m_shooterSpeedShortFast=PickUpConstants::kShortSpeed;
	double m_shooterSpeedLongFast=PickUpConstants::kLongSpeed;
	double m_shooterSpeedShortSlow=PickUpConstants::kShortSpeedSlow;
	double m_shooterSpeedLongSlow=PickUpConstants::kLongSpeedSlow;

	frc::XboxController* m_controller;

	std::shared_ptr<nt::NetworkTable> m_limelightTable;

	rev::SparkMaxRelativeEncoder m_shooter1Encoder;
	rev::SparkMaxRelativeEncoder m_shooter2Encoder;
	rev::SparkMaxRelativeEncoder m_backSpinShooterEncoder;
	rev::SparkMaxPIDController m_shooter1PIDController;
	rev::SparkMaxPIDController m_shooter2PIDController;
	rev::SparkMaxPIDController m_backSpinPIDController;

	int counter = 0;

	double m_offset;
	double m_realDist;
};
