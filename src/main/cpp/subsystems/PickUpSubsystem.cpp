#include "subsystems/PickUpSubsystem.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>

// ============================================================================

PickUpSubsystem::PickUpSubsystem(frc::XboxController* controller)
:	//m_roller {PickUpConstants::kRollerPort},
	m_index1 {PickUpConstants::kIndex1Port},
	m_index2 {PickUpConstants::kIndex2Port},
	m_rollerpwm {0},
	m_upperSolenoid {frc::PneumaticsModuleType::CTREPCM, PickUpConstants::kUpperForwardSolenoidPort, PickUpConstants::kUpperReverseSolenoidPort},
	m_lowerSolenoid {frc::PneumaticsModuleType::CTREPCM, PickUpConstants::kLowerForwardSolenoidPort, PickUpConstants::kLowerReverseSolenoidPort},
	m_compressor {frc::PneumaticsModuleType::CTREPCM},
	m_shooter1 {PickUpConstants::kShooter1Port, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
	m_shooter2 {PickUpConstants::kShooter2Port, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
	m_shooter{m_shooter1, m_shooter2},
	m_backSpinShooter{PickUpConstants::kBackSpinShooterPort, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
	m_shooterSolenoid{frc::PneumaticsModuleType::CTREPCM, PickUpConstants::kShooterForwardSolenoidPort, PickUpConstants::kShooterReverseSolenoidPort},
	m_controller{controller}
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
	m_limelightTable= nt::NetworkTableInstance::GetDefault().GetTable("limelight");
	m_limelightTable->PutNumber("ledMode", 1);
}

// ============================================================================

void PickUpSubsystem::Periodic() {
	frc::SmartDashboard::PutNumber ("Shooter Speed", m_shooterSpeed);
	frc::SmartDashboard::PutNumber ("ShooterSpeedshortSlow", m_shooterSpeedShortSlow);
	frc::SmartDashboard::PutNumber ("ShooterSpeedlongSlow", m_shooterSpeedLongSlow);
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
	//m_roller.Set(TalonSRXControlMode::PercentOutput, -1.0);
	m_rollerpwm.Set(-.75);
	PickUpExtend();
}

// ============================================================================

void PickUpSubsystem::RollerOut() {
	//m_roller.Set(TalonSRXControlMode::PercentOutput, 0.75);
	m_rollerpwm.Set(.5);
}

// ============================================================================

void PickUpSubsystem::RollerOff() {
	//m_roller.Set(TalonSRXControlMode::PercentOutput, 0);
	m_rollerpwm.Set(0);
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

	double triggerAxis = m_controller->GetRightTriggerAxis();
	auto solenoidState = m_shooterSolenoid.Get();
	auto isForward = frc::DoubleSolenoid::Value::kForward == solenoidState;

	if (triggerAxis > .8 && !isForward) {
		m_shooterSpeed = m_shooterSpeedShortSlow;
		m_shooter.SetVoltage(units::voltage::volt_t{m_shooterSpeed*12});
	}
	else if (triggerAxis > 0 && triggerAxis <= .8 && !isForward) {
		m_shooterSpeed = m_shooterSpeedShortFast;
		m_shooter.SetVoltage(units::voltage::volt_t{m_shooterSpeed*12});
	}
	else if (triggerAxis > .8 && isForward) {
		m_shooterSpeed = m_shooterSpeedLongSlow;
		m_shooter.SetVoltage(units::voltage::volt_t{m_shooterSpeed*12});
	}
	else if (triggerAxis > 0 && triggerAxis <= .8 && isForward) {
		m_shooterSpeed = m_shooterSpeedLongFast;
		m_shooter.SetVoltage(units::voltage::volt_t{m_shooterSpeed*12});
	}
	else {
		m_shooter.SetVoltage(units::voltage::volt_t{m_shooterSpeed*12});

	}
	m_backSpinShooter.SetVoltage(units::voltage::volt_t{-12});
	m_limelightTable->PutNumber("ledMode", 0);
}

// ============================================================================

void PickUpSubsystem::SetShooterSpeed(double shooterSpeed) {
	m_shooterSpeed = shooterSpeed;
}

// ============================================================================

void PickUpSubsystem::ShooterOff() {
	m_shooter.Set(0);
	m_backSpinShooter.Set(0);
	m_limelightTable->PutNumber("ledMode", 1);

}

// ============================================================================

void PickUpSubsystem::ShooterFaster() {
	if (m_shooterSolenoid.Get() == frc::DoubleSolenoid::Value::kForward)
		m_shooterSpeed = m_shooterSpeedLongSlow = std::min(1.0, m_shooterSpeedLongSlow + 0.025);
	else
		m_shooterSpeed = m_shooterSpeedShortSlow = std::min(1.0, m_shooterSpeedShortSlow + 0.025);
}

// ============================================================================

void PickUpSubsystem::ShooterSlower() {
	if (m_shooterSolenoid.Get() == frc::DoubleSolenoid::Value::kForward)
		m_shooterSpeed = m_shooterSpeedLongSlow = std::max(0.0, m_shooterSpeedLongSlow - 0.025);
	else
		m_shooterSpeed = m_shooterSpeedShortSlow = std::max(0.0, m_shooterSpeedShortSlow - 0.025);
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
		m_shooterSpeed = m_shooterSpeedLongSlow;
	}
	else {
		ShooterClose();
		m_shooterSpeed = m_shooterSpeedShortSlow;
	}
}

// ============================================================================
