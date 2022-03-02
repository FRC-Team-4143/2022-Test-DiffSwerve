#include "commands/ZeroClimber.h"

ZeroClimber::ZeroClimber(ClimberSubsystem* subsystem) : m_climber(subsystem) {
	AddRequirements(subsystem);
}

void ZeroClimber::Initialize() {
	m_climber->ZeroClimber();
}

bool ZeroClimber::IsFinished() {
	return true;
}

bool ZeroClimber::RunsWhenDisabled() const {
	return true;
}