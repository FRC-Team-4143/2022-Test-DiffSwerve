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
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc/filter/SlewRateLimiter.h>
#include "Subsystems/PickUpSubsystem.h"
#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include <frc2/command/button/JoystickButton.h>
#include "subsystems/ClimberSubsystem.h"

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
	PickUpSubsystem m_pickUp;
	ClimberSubsystem m_climber;
	frc2::RunCommand m_DriveCommand;

private:

	// The driver's controller
	frc::XboxController m_driverController{OIConstants::kDriverControllerPort};
	frc::XboxController m_climberController{OIConstants::kClimberControllerPort};

	// The robot's subsystems and commands are defined here...

	// The robot's subsystems
	frc::SlewRateLimiter<units::scalar> m_xspeedLimiter{2 / 1_s};
	frc::SlewRateLimiter<units::scalar> m_yspeedLimiter{2 / 1_s};
	frc::SlewRateLimiter<units::scalar> m_rotLimiter{2 / 1_s};

	// The chooser for the autonomous routines
	//frc::SendableChooser<frc2::Command*> m_chooser;
	frc2::JoystickButton *m_rb;
	frc2::JoystickButton *m_lb;

	void ConfigureButtonBindings();
};
