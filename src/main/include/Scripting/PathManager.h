#pragma	once
#include <frc/geometry/Pose2d.h>
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

	bool AddPath(std::string pathFileName);
	std::unique_ptr<frc2::Command> GetPathCommand(DriveSubsystem *driveSubsystem, int pathIndex, bool resetOdometry);
	void ResetOdometry(DriveSubsystem *driveSubsystem, frc::Pose2d pose);

private:

	units::meters_per_second_t m_maxVel;
	units::meters_per_second_squared_t m_maxAccel;
	std::vector<pathplanner::PathPlannerTrajectory> m_loadedPaths;
};

// ==========================================================================
