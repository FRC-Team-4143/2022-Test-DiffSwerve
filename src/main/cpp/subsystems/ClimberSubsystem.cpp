#include "subsystems/ClimberSubsystem.h"
#include "Constants.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/MathUtil.h>

// ============================================================================

ClimberSubsystem::ClimberSubsystem(frc::XboxController* controller)
:	m_controller(controller),
	m_rotateLeft{ClimberConstants::kRotateLeftPort, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
	m_rotateRight{ClimberConstants::kRotateRightPort, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
	m_extendLeft{ClimberConstants::kExtendLeftPort, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
	m_extendRight{ClimberConstants::kExtendRightPort, rev::CANSparkMaxLowLevel::MotorType::kBrushless},

	m_rotateLeftEncoder{m_rotateLeft.GetEncoder()},
	m_rotateRightEncoder{m_rotateRight.GetEncoder()},
	m_extendLeftEncoder{m_extendLeft.GetEncoder()},
	m_extendRightEncoder{m_extendRight.GetEncoder()},

	m_rotateLeftPidController{m_rotateLeft.GetPIDController()},
	m_rotateRightPidController{m_rotateRight.GetPIDController()},
	m_extendLeftPidController{m_extendLeft.GetPIDController()},
	m_extendRightPidController{m_extendRight.GetPIDController()},
    m_rotateLeftForwardLimit {m_rotateLeft.GetForwardLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyClosed)},
    m_rotateLeftReverseLimit {m_rotateLeft.GetReverseLimitSwitch(rev::SparkMaxLimitSwitch::Type::kNormallyClosed)},
    m_brakeSolenoidRght {frc::PneumaticsModuleType::CTREPCM, ClimberConstants::kBrakeSolenoidPortRght},
    m_brakeSolenoidLeft {frc::PneumaticsModuleType::CTREPCM, ClimberConstants::kBrakeSolenoidPortLeft}
{
    m_rotateLeft.RestoreFactoryDefaults();
    m_rotateRight.RestoreFactoryDefaults();
    m_extendLeft.RestoreFactoryDefaults();
    m_extendRight.RestoreFactoryDefaults();

    m_rotateLeftEncoder.SetPositionConversionFactor(90/17.57);
    m_rotateRightEncoder.SetPositionConversionFactor(90/17.57);
    m_extendLeftEncoder.SetPositionConversionFactor(100/260);
    m_extendRightEncoder.SetPositionConversionFactor(100/260);

    //m_rotateLeft.SetInverted(true);
    //m_extendRight.SetInverted(true);

	m_rotateLeft.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	m_rotateRight.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	m_extendLeft.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
	m_extendRight.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

	m_rotateLeftPidController.SetP(ClimberConstants::kP);
    m_rotateLeftPidController.SetI(ClimberConstants::kI);
    m_rotateLeftPidController.SetD(ClimberConstants::kD);
    m_rotateLeftPidController.SetIZone(ClimberConstants::kIz);
    m_rotateLeftPidController.SetFF(ClimberConstants::kFF);
    m_rotateLeftPidController.SetOutputRange(ClimberConstants::kMinOutput, ClimberConstants::kMaxOutput);

	m_rotateRightPidController.SetP(ClimberConstants::kP);
    m_rotateRightPidController.SetI(ClimberConstants::kI);
    m_rotateRightPidController.SetD(ClimberConstants::kD);
    m_rotateRightPidController.SetIZone(ClimberConstants::kIz);
    m_rotateRightPidController.SetFF(ClimberConstants::kFF);
    m_rotateRightPidController.SetOutputRange(ClimberConstants::kMinOutput, ClimberConstants::kMaxOutput);

	m_extendLeftPidController.SetP(ClimberConstants::kP);
    m_extendLeftPidController.SetI(ClimberConstants::kI);
    m_extendLeftPidController.SetD(ClimberConstants::kD);
    m_extendLeftPidController.SetIZone(ClimberConstants::kIz);
    m_extendLeftPidController.SetFF(ClimberConstants::kFF);
    m_extendLeftPidController.SetOutputRange(ClimberConstants::kMinOutput, ClimberConstants::kMaxOutput);

	m_extendRightPidController.SetP(ClimberConstants::kP);
    m_extendRightPidController.SetI(ClimberConstants::kI);
    m_extendRightPidController.SetD(ClimberConstants::kD);
    m_extendRightPidController.SetIZone(ClimberConstants::kIz);
    m_extendRightPidController.SetFF(ClimberConstants::kFF);
    m_extendRightPidController.SetOutputRange(ClimberConstants::kMinOutput, ClimberConstants::kMaxOutput);


	//m_rotateLeftPidController.SetSmartMotionMaxVelocity(ClimberConstants::kMaxVel);
    //m_rotateLeftPidController.SetSmartMotionMinOutputVelocity(ClimberConstants::kMinVel);
    //m_rotateLeftPidController.SetSmartMotionMaxAccel(ClimberConstants::kMaxAcc);
    //m_rotateLeftPidController.SetSmartMotionAllowedClosedLoopError(ClimberConstants::kAllErr);

	//m_rotateRightPidController.SetSmartMotionMaxVelocity(ClimberConstants::kMaxVel);
    //m_rotateRightPidController.SetSmartMotionMinOutputVelocity(ClimberConstants::kMinVel);
    //m_rotateRightPidController.SetSmartMotionMaxAccel(ClimberConstants::kMaxAcc);
    //m_rotateRightPidController.SetSmartMotionAllowedClosedLoopError(ClimberConstants::kAllErr);

	m_extendLeftPidController.SetSmartMotionMaxVelocity(ClimberConstants::kMaxVel);
    m_extendLeftPidController.SetSmartMotionMinOutputVelocity(ClimberConstants::kMinVel);
    m_extendLeftPidController.SetSmartMotionMaxAccel(ClimberConstants::kMaxAcc);
    m_extendLeftPidController.SetSmartMotionAllowedClosedLoopError(ClimberConstants::kAllErr);

	m_extendRightPidController.SetSmartMotionMaxVelocity(ClimberConstants::kMaxVel);
    m_extendRightPidController.SetSmartMotionMinOutputVelocity(ClimberConstants::kMinVel);
    m_extendRightPidController.SetSmartMotionMaxAccel(ClimberConstants::kMaxAcc);
    m_extendRightPidController.SetSmartMotionAllowedClosedLoopError(ClimberConstants::kAllErr);

    m_rotateLeftForwardLimit.EnableLimitSwitch(false);
    m_rotateLeftReverseLimit.EnableLimitSwitch(false);

    m_rotateLeft.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, false);
    m_rotateLeft.EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);

	// display PID coefficients on SmartDashboard
    frc::SmartDashboard::PutNumber("P Gain", ClimberConstants::kP);
    frc::SmartDashboard::PutNumber("I Gain", ClimberConstants::kI);
    frc::SmartDashboard::PutNumber("D Gain", ClimberConstants::kD);
    frc::SmartDashboard::PutNumber("I Zone", ClimberConstants::kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", ClimberConstants::kFF);
    frc::SmartDashboard::PutNumber("Max Output", ClimberConstants::kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", ClimberConstants::kMinOutput);

    // display Smart Motion coefficients
    frc::SmartDashboard::PutNumber("Max Velocity", ClimberConstants::kMaxVel);
    frc::SmartDashboard::PutNumber("Min Velocity", ClimberConstants::kMinVel);
    frc::SmartDashboard::PutNumber("Max Acceleration", ClimberConstants::kMaxAcc);
    frc::SmartDashboard::PutNumber("Allowed Closed Loop Error", ClimberConstants::kAllErr);
    frc::SmartDashboard::PutNumber("Set Left Position", 0);
    frc::SmartDashboard::PutNumber("Set Right Position", 0);

    m_brakeSolenoidRght.Set(true);
    m_brakeSolenoidLeft.Set(true);
}
// ============================================================================

void ClimberSubsystem::ZeroClimber(){
    m_rotateLeftEncoder.SetPosition(0);
    m_rotateRightEncoder.SetPosition(0);
    m_extendLeftEncoder.SetPosition(0);
    m_extendRightEncoder.SetPosition(0);
}

double ClimberSubsystem::GetLeftRotationPosition(){
    return m_rotateLeftEncoder.GetPosition();
}

double ClimberSubsystem::GetRightRotationPosition(){
    return m_rotateRightEncoder.GetPosition();
}

void ClimberSubsystem::Periodic() {
	if(m_controller->GetRightBumper()){
        if(m_controller->GetStartButton()) { m_rightPosition = -30.0; m_leftPosition = 30.0;}
        if(m_controller->GetBackButton()) { m_rightPosition = 0.0; m_leftPosition = 0.0;}
        if(m_controller->GetYButton()) m_leftPosition+=.5;
        if(m_controller->GetXButton()) m_leftPosition-=.5;
        if(m_leftPosition < 0.) m_leftPosition = 0.;
        if(m_leftPosition > 45.) m_leftPosition = 45.;
        if(m_controller->GetAButton()) m_rightPosition+=.5;
        if(m_controller->GetBButton()) m_rightPosition-=.5;
        if(m_rightPosition > 0.) m_rightPosition = 0.;
        if(m_rightPosition < -45.) m_rightPosition = -45.;

		//m_rotateLeft.Set(frc::ApplyDeadband(m_controller->GetLeftY(),.3)*ClimberConstants::kMaxRotatePower);
        //m_rotateLeftPidController.SetReference(frc::SmartDashboard::GetNumber("Set Left Position", 0), rev::ControlType::kPosition);
        m_rotateLeftPidController.SetReference(m_leftPosition, rev::CANSparkMax::ControlType::kPosition);
		//m_rotateRight.Set(frc::ApplyDeadband(m_controller->GetRightY(),.3)*ClimberConstants::kMaxRotatePower);
        //m_rotateRightPidController.SetReference(frc::SmartDashboard::GetNumber("Set Right Position", 0), rev::ControlType::kPosition);
        m_rotateRightPidController.SetReference(m_rightPosition, rev::CANSparkMax::ControlType::kPosition);
		m_extendLeft.Set(frc::ApplyDeadband(m_controller->GetLeftY(),.3) * ClimberConstants::kMaxExtendPower);
		m_extendRight.Set(frc::ApplyDeadband(-m_controller->GetRightY(),.3) * ClimberConstants::kMaxExtendPower);
	} else {
		m_rotateLeft.Set(0);
		m_rotateRight.Set(0);
		m_extendLeft.Set(0);
		m_extendRight.Set(0);
	}

	frc::SmartDashboard::PutNumber("Rotate Left Pos", GetLeftRotationPosition());
	frc::SmartDashboard::PutNumber("Rotate Right Pos", GetRightRotationPosition());
	frc::SmartDashboard::PutNumber("Extend Left Pos", m_extendLeftEncoder.GetPosition());
	frc::SmartDashboard::PutNumber("Extend Right Pos", m_extendRightEncoder.GetPosition());

	// display PID coefficients on SmartDashboard
    frc::SmartDashboard::PutNumber("P Gain", ClimberConstants::kP);
    frc::SmartDashboard::PutNumber("I Gain", ClimberConstants::kI);
    frc::SmartDashboard::PutNumber("D Gain", ClimberConstants::kD);
    frc::SmartDashboard::PutNumber("I Zone", ClimberConstants::kIz);
    frc::SmartDashboard::PutNumber("Feed Forward", ClimberConstants::kFF);
    frc::SmartDashboard::PutNumber("Max Output", ClimberConstants::kMaxOutput);
    frc::SmartDashboard::PutNumber("Min Output", ClimberConstants::kMinOutput);

    // display Smart Motion coefficients
    frc::SmartDashboard::PutNumber("Max Velocity", ClimberConstants::kMaxVel);
    frc::SmartDashboard::PutNumber("Min Velocity", ClimberConstants::kMinVel);
    frc::SmartDashboard::PutNumber("Max Acceleration", ClimberConstants::kMaxAcc);
    frc::SmartDashboard::PutNumber("Allowed Closed Loop Error", ClimberConstants::kAllErr);

}
