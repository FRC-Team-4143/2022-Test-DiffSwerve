#include "commands/ZeroYaw.h"

ZeroYaw::ZeroYaw(DriveSubsystem* subsystem) : m_drive(subsystem) {
  AddRequirements(subsystem);
}

void ZeroYaw::Initialize() {
  m_drive->ZeroHeading();
}

bool ZeroYaw::IsFinished() {
  return true;
}

bool ZeroYaw::RunsWhenDisabled() const {
  return true;
}