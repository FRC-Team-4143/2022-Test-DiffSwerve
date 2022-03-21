#pragma once
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/DriveSubsystem.h"


class DriveLime : public frc2::CommandHelper<frc2::CommandBase, DriveLime> {
public:

	explicit DriveLime(DriveSubsystem* subsystem);

	void Initialize() override;
	void Execute() override;
	void End(bool) override;
	bool IsFinished() override;

private:
	DriveSubsystem* m_drive;
};