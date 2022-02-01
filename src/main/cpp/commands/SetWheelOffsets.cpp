#include "commands/SetWheelOffsets.h"

SetWheelOffsets::SetWheelOffsets(DriveSubsystem* subsystem) : m_drive(subsystem) {
  AddRequirements(subsystem);
}

void SetWheelOffsets::Initialize() {
  m_drive->SetWheelOffsets();
}

bool SetWheelOffsets::IsFinished() {
  return true;
}

bool SetWheelOffsets::RunsWhenDisabled() const {
  return true;
}