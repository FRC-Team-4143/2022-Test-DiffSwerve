#pragma once
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/Filesystem.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/trajectory/TrajectoryUtil.h>
#include <frc/XboxController.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/Command.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/RunCommand.h>
#include "Scripting/PathManager.h"
#include "Scripting/ValidateScriptCmd.h"
#include "subsystems/ClimberSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/PickUpSubsystem.h"
#include "Constants.h"
#include <memory>
#include <pathplanner/lib/PathPlanner.h>
#include <wpi/fs.h>

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */

class RobotContainer {
public:

	RobotContainer();

	//frc2::Command* GetAutonomousCommandOld();
	std::unique_ptr<frc2::Command> GetAutonomousCommand();

	DriveSubsystem m_drive;
	PickUpSubsystem m_pickUp;
	ClimberSubsystem m_climber;

private:

	frc2::RunCommand m_driveCommand;

	frc::XboxController m_driverController{OIConstants::kDriverControllerPort};
	frc::XboxController m_climberController{OIConstants::kClimberControllerPort};

	// The robot's subsystems
	frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{.5 / 1_s};
	frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{.5 / 1_s};
	frc::SlewRateLimiter<units::scalar> m_rotLimiter{6 / 1_s};
	frc::SlewRateLimiter<units::scalar> m_powerLimiter{1. / 1_s};

	frc2::JoystickButton *m_rb;
	frc2::JoystickButton *m_lb;

	frc::Trajectory m_testTrajectory;
	pathplanner::PathPlannerTrajectory m_ppTrajectory;

	PathManager m_pathManager;

	frc4143::ValidateScriptCmd m_validateScriptCmd;

	std::unique_ptr<frc2::Command> _GetDrivePathCommand();

	void _ConfigureButtonBindings();
	void _ConfigureDashboardControls();
	void _InitializeScriptEngine();

	//bool IsNearWaypoint(Pose2d waypoint, double within);
};
