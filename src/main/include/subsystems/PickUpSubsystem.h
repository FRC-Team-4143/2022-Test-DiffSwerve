#pragma once
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
#include <frc/motorcontrol/PWMTalonFX.h>
#include "Constants.h"

class PickUpSubsystem : public frc2::SubsystemBase {
public:
	PickUpSubsystem();

	void Periodic() override;

	void PickUpExtend();
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
	void ShooterOff();
	void ShooterFaster();
	void ShooterSlower();
	void SetShooterSpeed(double shooterSpeed);

	void ShooterFar();
	void ShooterClose();
	void ShooterDistToggle();

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

	double m_shooterSpeed=PickUpConstants::kLongSpeed;
	double m_shooterSpeedShort=PickUpConstants::kShortSpeed;
	double m_shooterSpeedLong=PickUpConstants::kLongSpeed;
};
