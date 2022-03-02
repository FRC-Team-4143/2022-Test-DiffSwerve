#pragma once
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/ClimberSubsystem.h"

class ZeroClimber : public frc2::CommandHelper<frc2::CommandBase, ZeroClimber> {
public:
	explicit ZeroClimber(ClimberSubsystem* subsystem);

	void Initialize() override;
	bool IsFinished() override;
	bool RunsWhenDisabled() const override;

private:

	ClimberSubsystem* m_climber;
};
