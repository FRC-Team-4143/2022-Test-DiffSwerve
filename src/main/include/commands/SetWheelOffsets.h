#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveSubsystem.h"

class SetWheelOffsets : public frc2::CommandHelper<frc2::CommandBase, SetWheelOffsets> {
 public:
  explicit SetWheelOffsets(DriveSubsystem* subsystem);

  void Initialize() override;

  bool IsFinished() override;

  bool RunsWhenDisabled() const override;

 private:
  DriveSubsystem* m_drive;
};