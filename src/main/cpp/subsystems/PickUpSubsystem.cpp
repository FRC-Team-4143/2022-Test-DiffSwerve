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
	m_shooter1 {PickUpConstants::kShooter1Port, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
	m_shooter2 {PickUpConstants::kShooter2Port, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
	m_shooter{m_shooter1, m_shooter2},
	m_backSpinShooter{PickUpConstants::kBackSpinShooterPort, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
	m_shooterSolenoid{frc::PneumaticsModuleType::CTREPCM, PickUpConstants::kShooterForwardSolenoidPort, PickUpConstants::kShooterReverseSolenoidPort},
	m_shooterSpeed{0.55}, m_shooterSpeedLong{0.55}, m_shooterSpeedShort{0.42}
{
	m_index1.SetNeutralMode(NeutralMode::Brake);
	m_index2.SetNeutralMode(NeutralMode::Brake);
	m_shooter2.SetInverted(true);
	m_shooter1.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 100); 
    m_shooter1.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 500); 
    m_shooter1.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 500);
	m_shooter2.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 100); 
    m_shooter2.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 500); 
    m_shooter2.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 500);
	m_backSpinShooter.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus0, 100); 
    m_backSpinShooter.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus1, 500); 
    m_backSpinShooter.SetPeriodicFramePeriod(rev::CANSparkMaxLowLevel::PeriodicFrame::kStatus2, 500);
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
	m_index1.Set(TalonSRXControlMode::PercentOutput, 0.75);
	m_index2.Set(TalonSRXControlMode::PercentOutput, 0.5);
}

// ============================================================================

void PickUpSubsystem::IndexerLoad() {
	m_index1.Set(TalonSRXControlMode::PercentOutput, 0);
	m_index2.Set(TalonSRXControlMode::PercentOutput, 0.5);
}

// ============================================================================

void PickUpSubsystem::IndexerRev() {
	m_index1.Set(TalonSRXControlMode::PercentOutput, -0.5);
	m_index2.Set(TalonSRXControlMode::PercentOutput, -0.75);
}

// ============================================================================

void PickUpSubsystem::IndexerOff() {
	m_index1.Set(TalonSRXControlMode::PercentOutput, 0);
	m_index2.Set(TalonSRXControlMode::PercentOutput, 0);
}

// ============================================================================

void PickUpSubsystem::ShooterOn() {
	m_shooter.SetVoltage(units::voltage::volt_t{m_shooterSpeed*12});
	m_backSpinShooter.SetVoltage(units::voltage::volt_t{-12});
}

// ============================================================================

void PickUpSubsystem::SetShooterSpeed(double shooterSpeed) {
	m_shooterSpeed = shooterSpeed;
}

// ============================================================================

void PickUpSubsystem::ShooterOff() {
	m_shooter.Set(0);
	m_backSpinShooter.Set(0);
}

// ============================================================================

void PickUpSubsystem::ShooterFaster() {
	m_shooterSpeedLong = std::min(1.0, m_shooterSpeedLong + 0.025);
}

// ============================================================================

void PickUpSubsystem::ShooterSlower() {
	m_shooterSpeedLong = std::max(0.0, m_shooterSpeedLong - 0.025);
}

// ============================================================================

void PickUpSubsystem::ShooterFar() {
	m_shooterSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
}

// ============================================================================

void PickUpSubsystem::ShooterClose() {
	m_shooterSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}

// ============================================================================

void PickUpSubsystem::ShooterDistToggle() {
	if (m_shooterSolenoid.Get() == frc::DoubleSolenoid::Value::kReverse) {
		ShooterFar();
		m_shooterSpeed = m_shooterSpeedLong;
	}
	else {
		ShooterClose();
		m_shooterSpeed = m_shooterSpeedShort;
	}
}

// ============================================================================
