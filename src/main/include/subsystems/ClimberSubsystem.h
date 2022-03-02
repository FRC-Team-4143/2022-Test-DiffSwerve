#pragma once
#include <frc2/command/SubsystemBase.h>
#include <rev/CANSparkMax.h>
#include "Constants.h"
#include <frc/XboxController.h>

class ClimberSubsystem : public frc2::SubsystemBase {
public:
	ClimberSubsystem(frc::XboxController* controller);

	void Periodic() override;

	void climberEnabled();
	void climberDisabled();

	void ZeroClimber();
	double GetLeftRotationPosition();
	double GetRightRotationPosition();

private:
	frc::XboxController* m_controller;

	rev::CANSparkMax m_rotateLeft;
	rev::CANSparkMax m_rotateRight;
	rev::CANSparkMax m_extendLeft;
	rev::CANSparkMax m_extendRight;

	rev::SparkMaxRelativeEncoder m_rotateLeftEncoder;
	rev::SparkMaxRelativeEncoder m_rotateRightEncoder;
	rev::SparkMaxRelativeEncoder m_extendLeftEncoder;
	rev::SparkMaxRelativeEncoder m_extendRightEncoder;

	rev::SparkMaxPIDController m_rotateLeftPidController;
	rev::SparkMaxPIDController m_rotateRightPidController;
	rev::SparkMaxPIDController m_extendLeftPidController;
	rev::SparkMaxPIDController m_extendRightPidController;
	
};