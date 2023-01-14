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
	m_controller{controller},
	m_shooter1Encoder{m_shooter1.GetEncoder()},
	m_shooter2Encoder{m_shooter2.GetEncoder()},
	m_backSpinShooterEncoder{m_backSpinShooter.GetEncoder()},
	m_shooter1PIDController{m_shooter1.GetPIDController()},
	m_shooter2PIDController{m_shooter2.GetPIDController()},
	m_backSpinPIDController{m_backSpinShooter.GetPIDController()}
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
	m_limelightTable->PutNumber("ledMode", 0);
	m_shooter1PIDController.SetP(6e-5);
	m_shooter2PIDController.SetP(6e-5);
	m_backSpinPIDController.SetP(6e-5);
	m_shooter1PIDController.SetFF(1.5e-5);
	m_shooter2PIDController.SetFF(1.5e-5);
	m_backSpinPIDController.SetFF(1.5e-5);
	//frc::SmartDashboard::PutNumber("shooter constant", frc::SmartDashboard::GetNumber("shooter constant", 0.26));
	frc::SmartDashboard::PutNumber("shooter constant", frc::SmartDashboard::GetNumber("shooter constant", 0.));

}

// ============================================================================

void PickUpSubsystem::Periodic() {
	frc::SmartDashboard::PutNumber ("Shooter Speed", m_shooterSpeed);
	frc::SmartDashboard::PutNumber ("ShooterSpeedshortSlow", m_shooterSpeedShortSlow);
	frc::SmartDashboard::PutNumber ("ShooterSpeedlongSlow", m_shooterSpeedLongSlow);
	frc::SmartDashboard::PutNumber ("Shooter1 RPM", m_shooter1Encoder.GetVelocity());
	//frc::SmartDashboard::PutNumber ("Shooter2 RPM", m_shooter2Encoder.GetVelocity());
	frc::SmartDashboard::PutNumber ("BackSpinShooter RPM", m_backSpinShooterEncoder.GetVelocity());
	frc::SmartDashboard::PutNumber("ty LimeLight", m_limelightTable->GetNumber("ty",0));
	frc::SmartDashboard::PutNumber("realDist", m_realDist);
}

// ============================================================================

void PickUpSubsystem::PickUpBounce() {
	//m_upperSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
	m_lowerSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
}

// ============================================================================

void PickUpSubsystem::PickUpRetract() {
	//m_upperSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
	m_lowerSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}

// ============================================================================

void PickUpSubsystem::PickUpExtend() {
	//m_upperSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
	m_lowerSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
}

void PickUpSubsystem::PickUpExtendStart() {
	//m_upperSolenoid.Set(frc::DoubleSolenoid::Value::kReverse);
	m_lowerSolenoid.Set(frc::DoubleSolenoid::Value::kForward);
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
	m_rollerpwm.Set(-1.0);  //.75
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
	if(m_controller->GetLeftTriggerAxis() <= 0.5 && m_controller->GetLeftTriggerAxis() > 0.05){
		m_index1.Set(TalonSRXControlMode::PercentOutput, 0.85);
		m_index2.Set(TalonSRXControlMode::PercentOutput, 0);
	} else{
		m_index1.Set(TalonSRXControlMode::PercentOutput, 0.85);
		m_index2.Set(TalonSRXControlMode::PercentOutput, 0.7);
	}
}

// ============================================================================

void PickUpSubsystem::IndexerLoad() {
	m_index1.Set(TalonSRXControlMode::PercentOutput, 0);
	m_index2.Set(TalonSRXControlMode::PercentOutput, 0.7);
}

// ============================================================================

void PickUpSubsystem::IndexerRev() {
	m_index1.Set(TalonSRXControlMode::PercentOutput, -0.7);
	m_index2.Set(TalonSRXControlMode::PercentOutput, -0.85);
}

// ============================================================================

void PickUpSubsystem::IndexerOff() {
	m_index1.Set(TalonSRXControlMode::PercentOutput, 0);
	m_index2.Set(TalonSRXControlMode::PercentOutput, 0);
}

// ============================================================================

void PickUpSubsystem::ShooterOnManual() {
	counter++;

	double triggerAxis = m_controller->GetRightTriggerAxis();
	bool aButton = m_controller->GetAButton();
	auto solenoidState = m_shooterSolenoid.Get();
	auto isForward = frc::DoubleSolenoid::Value::kForward == solenoidState;
	auto tx = m_limelightTable->GetNumber("tx", 0);

	//if (fabs(tx) < 2 && tx != 0&& !frc::SmartDashboard::GetBoolean("Disable Limelight", 0) && counter > 24) {
	//	IndexerOn();
	//}

	/*
	auto ty = m_limelightTable->GetNumber("ty",0)
	m_shooterSpeed = f(ty);
	*/

	if (triggerAxis > 0 && !aButton && !isForward) {
		m_shooterSpeed = m_shooterSpeedShortSlow;
		m_shooter.SetVoltage(units::voltage::volt_t{m_shooterSpeed*12});
	}
	else if (triggerAxis > 0 && (aButton || triggerAxis <= .8) && !isForward) {
		m_shooterSpeed = m_shooterSpeedShortFast;
		m_shooter.SetVoltage(units::voltage::volt_t{m_shooterSpeed*12});
	}
	else if (triggerAxis > 0 && !aButton && isForward) {
		m_shooterSpeed = m_shooterSpeedLongSlow;
		m_shooter.SetVoltage(units::voltage::volt_t{m_shooterSpeed*12});
	}
	else if (triggerAxis > 0 && (aButton || triggerAxis <= .8) && isForward) {
		m_shooterSpeed = m_shooterSpeedLongFast;
		m_shooter.SetVoltage(units::voltage::volt_t{m_shooterSpeed*12});
	}
	else {
		m_shooter.SetVoltage(units::voltage::volt_t{m_shooterSpeed*12});

	}
	m_backSpinShooter.SetVoltage(units::voltage::volt_t{-10});
	//m_limelightTable->PutNumber("ledMode", 0);
	}

// ============================================================================

void PickUpSubsystem::ShooterOnLimeLight() {
	if (counter > 0) counter--;

	auto tx = m_limelightTable->GetNumber("tx", 0);
	double ty = m_limelightTable->GetNumber("ty", 0);
	auto tv = m_limelightTable->GetNumber("tv",0);

	frc::SmartDashboard::PutNumber("LL xerr", tx - m_offset);
	double tolerance = frc::SmartDashboard::GetNumber("limelightTolerance", 3);

	if (fabs(tx - m_offset) < tolerance && fabs(m_offset) < 25. && tv != 0 && !frc::SmartDashboard::GetBoolean("Disable Limelight", 0) && (m_controller->GetRightTriggerAxis() > .1 || frc::DriverStation::IsAutonomousEnabled())) {
		counter2++;
		if(counter2 > 2) {  // must be locked on for 3 cycles
			IndexerOn();
			counter = 30;
		}	
	}
	else counter2 = 0;

	if ( counter == 1 ) 
		IndexerOff();

    //double shooterconstant = frc::SmartDashboard::GetNumber("shooter constant", .26);
    double shooterconstant = frc::SmartDashboard::GetNumber("shooter constant", 0.);

	//m_shooterSpeed = (0.378515 - 0.00009270941*ty + 0.0005572375*pow(ty,2));
	
	//m_shooterSpeed = shooterconstant + 0.0204*m_realDist + .00867*pow(m_realDist, 2);
	m_shooterSpeed = shooterconstant + .26 + -.00383*m_realDist + .00867*pow(m_realDist, 2);
	//m_shooterSpeed = shooterconstant + 0.0345 + 0.0862*m_realDist + -2.81E-03*pow(m_realDist,2); // newest sheet


	m_shooter.SetVoltage(units::voltage::volt_t{m_shooterSpeed*12});
	m_backSpinShooter.SetVoltage(units::voltage::volt_t{-10});
}

void PickUpSubsystem::ShooterOn() {
	auto solenoidState = m_shooterSolenoid.Get();
	auto isForward = frc::DoubleSolenoid::Value::kForward == solenoidState;

	if (m_controller->GetAButton()) { 
		ShooterOff(); 
	} else if (frc::SmartDashboard::GetBoolean("Disable Limelight", 0) || !isForward) {
		ShooterOnManual();
	} else if (!frc::SmartDashboard::GetBoolean("Disable Limelight", 0)) {
		ShooterOnLimeLight();
	}
}

// ============================================================================

void PickUpSubsystem::SetShooterSpeed(double shooterSpeed) {
	m_shooterSpeed = shooterSpeed;
}

// ============================================================================

void PickUpSubsystem::ShooterOff() {
	counter = 0;
	m_shooter.Set(0);
	m_backSpinShooter.Set(0);
	//m_limelightTable->PutNumber("ledMode", 1);
	IndexerOff();

}

// ============================================================================

void PickUpSubsystem::ShooterFaster() {
	if (m_shooterSolenoid.Get() == frc::DoubleSolenoid::Value::kForward)
		m_shooterSpeed = m_shooterSpeedLongSlow = std::min(1.0, m_shooterSpeedLongSlow + 0.010);
	else
		m_shooterSpeed = m_shooterSpeedShortSlow = std::min(1.0, m_shooterSpeedShortSlow + 0.010);
}

// ============================================================================

void PickUpSubsystem::ShooterSlower() {
	if (m_shooterSolenoid.Get() == frc::DoubleSolenoid::Value::kForward)
		m_shooterSpeed = m_shooterSpeedLongSlow = std::max(0.0, m_shooterSpeedLongSlow - 0.010);
	else
		m_shooterSpeed = m_shooterSpeedShortSlow = std::max(0.0, m_shooterSpeedShortSlow - 0.010);
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

void PickUpSubsystem::SetDist(double value){
		m_realDist = value;
}

void PickUpSubsystem::SetOffset(double value){
		m_offset = value;
}

bool PickUpSubsystem::HasShot() {
	return (counter == 1);
}