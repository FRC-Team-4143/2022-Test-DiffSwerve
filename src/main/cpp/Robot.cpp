#include "Robot.h"
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/DataLogManager.h>
#include <wpi/DataLog.h>

void Robot::RobotInit() {
	frc::DataLogManager::Start();

	wpi::log::DataLog& log = frc::DataLogManager::GetLog();

	//front f, back b, left l, right r, top t, bottom b

	fltMotorCurrent = wpi::log::DoubleLogEntry(log, "/fltMotorCurrent/double");
	flbMotorCurrent = wpi::log::DoubleLogEntry(log, "/flbMotorCurrent/double");
}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */

void Robot::RobotPeriodic() {
	frc2::CommandScheduler::GetInstance().Run();
	m_container.m_pickUp.SetDist(m_container.m_drive.GetDist() );
	m_container.m_pickUp.SetOffset(m_container.m_drive.GetOffset() );

	//data logging
	//wanna try to only append when enabled
	if (true){
		fltMotorCurrent.Append(m_container.m_drive.m_frontLeft.topMotorCurrent);
		flbMotorCurrent.Append(m_container.m_drive.m_frontLeft.bottomMotorCurrent);
	}
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */
void Robot::DisabledInit() {
}

void Robot::DisabledPeriodic() {
}

/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() {
	m_autonomousCommand = m_container.GetAutonomousCommand();
	if (m_autonomousCommand) {
		m_autonomousCommand->Schedule();
	}
}

void Robot::AutonomousPeriodic() {
}

void Robot::TeleopInit() {
	// This makes sure that the autonomous stops running when
	// teleop starts running. If you want the autonomous to
	// continue until interrupted by another command, remove
	// this line or comment it out.
	if (m_autonomousCommand) {
		m_autonomousCommand->Cancel();
		m_autonomousCommand = nullptr;
	}
	m_container.m_pickUp.ShooterFar();
	m_container.m_pickUp.ShooterOff();
	m_container.m_pickUp.PickUpRetract();
	m_container.m_pickUp.RollerOff();
	m_container.m_pickUp.IndexerOff();
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() {
	m_container.m_pickUp.ShooterOn();
}

/**
 * This function is called periodically during test mode.
 */
void Robot::TestPeriodic() {
}

#ifndef RUNNING_FRC_TESTS
int main() {
	return frc::StartRobot<Robot>();
}
#endif
