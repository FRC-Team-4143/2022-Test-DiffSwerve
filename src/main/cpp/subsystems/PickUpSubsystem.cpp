#include "subsystems/PickUpSubsystem.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>

// ============================================================================

PickUpSubsystem::PickUpSubsystem()
:	m_roller {PickUpConstants::kRollerPort},
	m_index1 {PickUpConstants::kIndex1Port},
	m_index2 {PickUpConstants::kIndex2Port},
	m_upperSolenoid {frc::PneumaticsModuleType::CTREPCM, PickUpConstants::kUpperForwardSolenoidPort, PickUpConstants::kUpperReverseSolenoidPort},
	m_lowerSolenoid {frc::PneumaticsModuleType::CTREPCM, PickUpConstants::kLowerForwardSolenoidPort, PickUpConstants::kLowerReverseSolenoidPort},
	m_compressor {frc::PneumaticsModuleType::CTREPCM},
	m_shooter1 {PickUpConstants::kShooter1Port,  rev::CANSparkMaxLowLevel::MotorType::kBrushless},
	m_shooter2 {PickUpConstants::kShooter2Port,  rev::CANSparkMaxLowLevel::MotorType::kBrushless},
	m_shooter{m_shooter1, m_shooter2},
	m_backSpinShooter{PickUpConstants::kBackSpinShooterPort,  rev::CANSparkMaxLowLevel::MotorType::kBrushless},
	m_shooterSolenoid{frc::PneumaticsModuleType::CTREPCM, PickUpConstants::kShooterForwardSolenoidPort, PickUpConstants::kShooterReverseSolenoidPort},
	m_shooterSpeed{1.0}
	
{
	m_shooter2.SetInverted(true);
}

// ============================================================================

void PickUpSubsystem::Periodic() {
	frc::SmartDashboard::PutNumber ("Shooter Speed", m_shooterSpeed);
}

// ============================================================================

void PickUpSubsystem::PickUpBounce() {
	m_upperSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
	m_lowerSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
}

// ============================================================================

void PickUpSubsystem::PickUpRetract() {
	m_upperSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
	m_lowerSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}

// ============================================================================

void PickUpSubsystem::PickUpExtend() {
	m_upperSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
	m_lowerSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}

// ============================================================================

void PickUpSubsystem::PickUpToggle() {
	if (m_lowerSolenoid.Get() == frc::DoubleSolenoid::Value::kForward) {
		PickUpExtend();
	}
	else {
		PickUpBounce();
	}
}

// ============================================================================

void PickUpSubsystem::RollerIn() {
	m_roller.Set(TalonSRXControlMode::PercentOutput, -1.0);
	PickUpExtend();
}

// ============================================================================

void PickUpSubsystem::RollerOut() {
	m_roller.Set(TalonSRXControlMode::PercentOutput, 0.75);
}

// ============================================================================

void PickUpSubsystem::RollerOff() {
	m_roller.Set(TalonSRXControlMode::PercentOutput, 0);
	PickUpRetract();
}

// ============================================================================

void PickUpSubsystem::IndexerOn() {
	m_index1.Set(TalonSRXControlMode::PercentOutput, 0.5);
	m_index2.Set(TalonSRXControlMode::PercentOutput, 0.5);
}

void PickUpSubsystem::IndexerLoad() {
	m_index1.Set(TalonSRXControlMode::PercentOutput, 0);
	m_index2.Set(TalonSRXControlMode::PercentOutput, 0.5);
}


void PickUpSubsystem::IndexerRev() {
	m_index1.Set(TalonSRXControlMode::PercentOutput, -0.5);
	m_index2.Set(TalonSRXControlMode::PercentOutput, -0.5);
}

// ============================================================================

void PickUpSubsystem::IndexerOff() {
	m_index1.Set(TalonSRXControlMode::PercentOutput, 0);
	m_index2.Set(TalonSRXControlMode::PercentOutput, 0);
}

// ============================================================================

void PickUpSubsystem::ShooterOn() {
	m_shooter.Set(m_shooterSpeed);
	m_backSpinShooter.Set(-m_shooterSpeed);
}

// ============================================================================

void PickUpSubsystem::ShooterOff() {
	m_shooter.Set(0);
	m_backSpinShooter.Set(0);
}

// ============================================================================

void PickUpSubsystem::ShooterFaster() {
	m_shooterSpeed = std::min(1.0, m_shooterSpeed + 0.05);
}

// ============================================================================

void PickUpSubsystem::ShooterSlower() {
	m_shooterSpeed = std::max(0.0, m_shooterSpeed - 0.05);
}

// ============================================================================

void PickUpSubsystem::ShooterFar() {
	m_shooterSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
}

void PickUpSubsystem::ShooterClose() {
	m_shooterSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}

void PickUpSubsystem::ShooterDistToggle() {
	if (m_shooterSolenoid.Get() == frc::DoubleSolenoid::Value::kReverse) {
		ShooterFar();
	}
	else {
		ShooterClose();
	}
}