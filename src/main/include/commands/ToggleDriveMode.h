#pragma once
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/DriveSubsystem.h"

class ToggleDriveMode : public frc2::CommandHelper<frc2::CommandBase, ToggleDriveMode> {
public:

	explicit ToggleDriveMode(DriveSubsystem* subsystem);

	void Initialize() override;
	bool IsFinished() override;
	bool RunsWhenDisabled() const override;

private:

	DriveSubsystem* m_drive;
};
