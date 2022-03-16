#include "Scripting/PathManager.h"
#include "Scripting/CommandGroupBuilder.h"
#include <frc2/command/PrintCommand.h>
#include "commands/PPSwerveControllerCommand.h"

// ==========================================================================

PathManager::PathManager(units::meters_per_second_t maxVel, units::meters_per_second_squared_t maxAccel)
:	m_maxVel(maxVel), m_maxAccel(maxAccel), m_loadedPaths{}
{
}

// ==========================================================================

void PathManager::AddPath(std::string pathFileName) {
	m_loadedPaths.push_back(pathplanner::PathPlanner::loadPath(pathFileName, m_maxVel, m_maxAccel));
}

// ==========================================================================

std::unique_ptr<frc2::Command> PathManager::GetPathCommand(
	DriveSubsystem *driveSubsystem,
	frc2::PIDController xController,
	frc2::PIDController yController,
	frc::ProfiledPIDController<units::radians> thetaController,
	std::size_t pathIndex, bool resetOdometry
)
{
	if (pathIndex >= m_loadedPaths.size()) {
		// In normal usage, we'd throw an exception. But we know this will
		// be used for scripting, so let's fail a bit more gracefully.
		return std::make_unique<frc2::PrintCommand>("Path index out of bounds.");
	}

	// Get the trajectory.
	auto& trajectory{m_loadedPaths.at(pathIndex)};



	// Build the PathPlanner command.
	auto pathCmd{
	
		std::make_unique<PPSwerveControllerCommand<4>>(
			trajectory,
			[driveSubsystem]() { return driveSubsystem->GetPose(); },
			driveSubsystem->kDriveKinematics,
			xController, yController, thetaController,
			[driveSubsystem](auto moduleStates) { driveSubsystem->SetModuleStates(moduleStates); },
			std::initializer_list<frc2::Subsystem*>{driveSubsystem}
		)
	};

	if (resetOdometry) {
		frc::Pose2d pose(trajectory.getInitialState()->pose.Translation(), trajectory.getInitialState()->holonomicRotation);
	
		auto resetOdometryCmd{
			std::make_unique<frc2::InstantCommand>(
				[driveSubsystem, pose]() {
					driveSubsystem->ResetOdometry(pose);
				}
			)
		};

		frc4143::CommandGroupBuilder builder{};
		builder.AddSequential(std::move(resetOdometryCmd));
		builder.AddSequential(std::move(pathCmd));
		return builder.Create();
	}

	return pathCmd;
}

// ==========================================================================