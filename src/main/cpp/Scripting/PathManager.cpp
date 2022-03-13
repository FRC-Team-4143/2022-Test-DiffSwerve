#include "Scripting/PathManager.h"
#include <frc2/command/PrintCommand.h>
#include "commands/PPSwerveControllerCommand.h"

// ==========================================================================

PathManager::PathManager(units::meters_per_second_t maxVel, units::meters_per_second_squared_t maxAccel)
:	m_maxVel(maxVel), m_maxAccel(maxAccel), m_loadedPaths{}
{
}

// ==========================================================================

bool PathManager::AddPath(std::string pathFileName) {
	m_loadedPaths.push_back(pathplanner::PathPlanner::loadPath(pathFileName, m_maxVel, m_maxAccel));
}

// ==========================================================================

std::unique_ptr<frc2::Command> PathManager::GetPathCommand(DriveSubsystem *driveSubsystem, int pathIndex, bool resetOdometry) {
	if (pathIndex < 0 || pathIndex >= m_loadedPaths.size()) {
		return std::make_unique<frc2::PrintCommand>("Path index out of bounds.");
	}

    frc::ProfiledPIDController<units::radians> thetaController{
		AutoConstants::kPThetaController, 0, 0,
		AutoConstants::kThetaControllerConstraints
	};

	thetaController.EnableContinuousInput(
		units::radian_t(-wpi::numbers::pi),
		units::radian_t(wpi::numbers::pi)
	);

	auto& trajectory{m_loadedPaths.at(pathIndex)};

	auto cmd{
		std::make_unique<PPSwerveControllerCommand<4>>(
			trajectory,
			[driveSubsystem]() { return driveSubsystem->GetPose(); },
			driveSubsystem->kDriveKinematics,
			frc2::PIDController(AutoConstants::kPXController, 0, 0),
			frc2::PIDController(AutoConstants::kPYController, 0, 0),
			thetaController,
			[this](auto moduleStates) { driveSubsystem.SetModuleStates(moduleStates); },
			std::initializer_list<frc2::Subsystem*>{driveSubsystem}
		)
	};

	// TODO - This resets odometry at the time the command is _created_.
	// It should instead reset odometry when the command _initializes_.
	// Probably OK for now, as long as the pose is correct at the start
	// of autonomous.

	if (resetOdometry) {
		frc::Pose2d pose(trajectory.getInitialState()->pose.Translation(), trajectory.getInitialState()->holonomicRotation);
		ResetOdometry(driveSubsystem, pose);
	}

	return cmd;
}

// ==========================================================================

void PathManager::ResetOdometry(DriveSubsystem *driveSubsystem, frc::Pose2d pose) {
	driveSubsystem->ResetOdometry(pose);
}

// ==========================================================================
