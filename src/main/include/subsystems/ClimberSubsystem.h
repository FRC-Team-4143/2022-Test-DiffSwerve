#pragma once
#include <frc2/command/SubsystemBase.h>
#include <frc/Solenoid.h>
#include <frc/XboxController.h>
#include <rev/CANSparkMax.h>

class ClimberSubsystem : public frc2::SubsystemBase {
public:

	ClimberSubsystem(frc::XboxController* controller);

	void Periodic() override;

	void IndexStep();
	void BackStep();
	void ZeroClimber();

private:

	double _GetLeftRotationPosition();
	double _GetRightRotationPosition();

	bool slowSafeMode;
	bool fastSafeMode;

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

	rev::SparkMaxLimitSwitch m_rotateLeftForwardLimit;
	rev::SparkMaxLimitSwitch m_rotateLeftReverseLimit;

	frc::Solenoid m_brakeSolenoidRght;
	frc::Solenoid m_brakeSolenoidLeft;
	
	double m_rightPosition = 0;
	double m_leftPosition = 0;
	double m_rightExtensionPos = 0;
	double m_leftExtensionPos = 0;
	int m_numSteps = 14;

	double m_climbSteps[15][4] = {
		{0, 0, 0, 0},
		//{22, 0, 265*9/16, 265*9/16},
		{32, 0, 265*9/16, 265*9/16},
		{32, 0, -10*9/16, 265*9/16},
		{17, 0, -10*9/16, 265*9/16}, // was 22
 		{17, 0, 130*9/16, 165*9/16}, // was 22
		{0, -22, 100*9/16, 165*9/16},
		{0, -32, 265*9/16, 0*9/16},
		{0, -22, 265*9/16, 0*9/16},
		{0, -22, 0*9/16, 165*9/16},
		{0, -22, 50*9/16, 0},
		{10, 0, 50*9/16, 0},
		{10, 0, 0, 50*9/16},
		{0, 0, 0, 50*9/16},
		{0, 0, 0, 0}
	};

	int m_step = 0;
	bool m_newStep = false;
};
