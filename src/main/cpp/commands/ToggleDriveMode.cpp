#include "commands/ToggleDriveMode.h"

ToggleDriveMode::ToggleDriveMode(DriveSubsystem* subsystem) : m_drive(subsystem) {
	AddRequirements(subsystem);
}

void ToggleDriveMode::Initialize() {
	m_drive->ToggleFieldCentric();
}

bool ToggleDriveMode::IsFinished() {
	return true;
}

bool ToggleDriveMode::RunsWhenDisabled() const {
	return true;
}