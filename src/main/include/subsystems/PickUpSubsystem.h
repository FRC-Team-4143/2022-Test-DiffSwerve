#pragma once
#include <frc/Compressor.h>
#include <frc/DoubleSolenoid.h>
#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include <rev/CANSparkMax.h>
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

	void ShooterOn();
	void ShooterOff();
	void ShooterFaster();
	void ShooterSlower();

private:

	TalonSRX m_roller;
	TalonSRX m_index1;
	TalonSRX m_index2;

	frc::DoubleSolenoid m_upperSolenoid;
	frc::DoubleSolenoid m_lowerSolenoid;
	frc::Compressor m_compressor;

	rev::CANSparkMax m_shooter1;
	rev::CANSparkMax m_shooter2;
	frc::MotorControllerGroup m_shooter;
	rev::CANSparkMax m_backSpinShooter;

	double m_shooterSpeed;
};
