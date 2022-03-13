#pragma	once
#include <frc/geometry/Pose2d.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/controller/PIDController.h>
#include <pathplanner/lib/PathPlanner.h>
#include <units/velocity.h>
#include <units/acceleration.h>
#include <memory>
#include <string>
#include <vector>
#include "subsystems/DriveSubsystem.h"

// ==========================================================================

class PathManager{
public:

	PathManager(units::meters_per_second_t maxVel, units::meters_per_second_squared_t maxAccel);

	void AddPath(std::string pathFileName);
	std::unique_ptr<frc2::Command> GetPathCommand(
		DriveSubsystem *driveSubsystem,
		frc2::PIDController xController,
		frc2::PIDController yController,
		frc::ProfiledPIDController<units::radians> thetaController,
		std::size_t pathIndex, bool resetOdometry
	);
	void ResetOdometry(DriveSubsystem *driveSubsystem, frc::Pose2d pose);

private:

	units::meters_per_second_t m_maxVel;
	units::meters_per_second_squared_t m_maxAccel;
	std::vector<pathplanner::PathPlannerTrajectory> m_loadedPaths;
};

// ==========================================================================
