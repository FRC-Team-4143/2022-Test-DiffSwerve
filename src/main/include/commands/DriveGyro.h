#pragma once
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/DriveSubsystem.h"


class DriveGyro : public frc2::CommandHelper<frc2::CommandBase, DriveGyro> {
public:

	explicit DriveGyro(DriveSubsystem* subsystem, double x, double y, double angle);

	void Initialize() override;
	void Execute() override;
	void End(bool) override;
	bool IsFinished() override;

private:
	DriveSubsystem* m_drive;
	double m_x;
	double m_y;
	double m_angle;
};