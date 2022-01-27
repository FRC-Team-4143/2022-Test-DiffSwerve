// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::Command* GetAutonomousCommand();

  DriveSubsystem m_drive;
  frc2::InstantCommand m_SetWheelOffsets{[this] {m_drive.SetWheelOffsets(); }, {&m_drive}};
  frc2::InstantCommand m_ZeroYaw{[this] {m_drive.ZeroHeading(); }, {&m_drive}};
  frc2::RunCommand m_FieldCentricMode{[this] {
        m_drive.Drive(
            units::meters_per_second_t(m_driverController.GetLeftY()*AutoConstants::kMaxSpeed),
            units::meters_per_second_t(m_driverController.GetLeftX()*AutoConstants::kMaxSpeed),
            units::radians_per_second_t(m_driverController.GetRightX()*8), true);
      },
      {&m_drive}
  };
  frc2::RunCommand m_CrabMode{[this] {
        m_drive.Drive(
            units::meters_per_second_t(m_driverController.GetLeftY()*AutoConstants::kMaxSpeed),
            units::meters_per_second_t(m_driverController.GetLeftX()*AutoConstants::kMaxSpeed),
            units::radians_per_second_t(m_driverController.GetRightX()*8), false);
      },
      {&m_drive}
  };

 private:
  // The driver's controller
  frc::XboxController m_driverController{OIConstants::kDriverControllerPort};

  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  void ConfigureButtonBindings();
};
