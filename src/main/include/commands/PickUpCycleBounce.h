#pragma once
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include "subsystems/PickUpSubsystem.h"
#include <frc2/command/button/JoystickButton.h>
#include <frc/XboxController.h>


class PickUpCycleBounce : public frc2::CommandHelper<frc2::CommandBase, PickUpCycleBounce> {
public:

	explicit PickUpCycleBounce(PickUpSubsystem* subsystem, frc::XboxController* controller);

	void Initialize() override;
	void Execute() override;
	void End(bool) override;
	bool IsFinished() override;

private:
	PickUpSubsystem* m_pickUp;
	int counter;
	frc::XboxController* m_controller;
};