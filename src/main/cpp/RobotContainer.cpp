#include "RobotContainer.h"
#include <frc/geometry/Translation2d.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/button/Trigger.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include <frc2/command/WaitCommand.h>
#include <frc2/command/WaitUntilCommand.h>
#include <networktables/NetworkTableEntry.h>
#include <units/angle.h>
#include <units/velocity.h>
#include <utility>
#include "commands/DriveGyro.h"
#include "commands/DriveLime.h"
#include "commands/PickUpCycle.h"
#include "commands/PickUpCycleBounce.h"

#undef PATHWEAVER

#ifdef PATHWEAVER
#include "commands/PWSwerveControllerCommand.h"
#else
#include "commands/PPSwerveControllerCommand.h"
#endif

#include "commands/SetWheelOffsets.h"
#include "commands/ToggleDriveMode.h"
#include "commands/ZeroClimber.h"
#include "commands/ZeroYaw.h"
#include "Scripting/ScriptEngine.h"
#include "Scripting/ScriptParser.h"
#include "Scripting/ScriptParserElement.h"

using namespace DriveConstants;

// ==========================================================================

RobotContainer::RobotContainer()
:	m_log{frc::DataLogManager::GetLog()}, m_drive{&m_driverController, m_log}, m_pickUp{&m_driverController}, m_climber{&m_climberController},
	m_powerDistributionPanel{},
	m_driveCommand{
		[this] {
			//auto x = -m_xspeedLimiter.Calculate(frc::ApplyDeadband(m_driverController.GetLeftY(), DriveConstants::stickDeadBand));
			//auto y = -m_yspeedLimiter.Calculate(frc::ApplyDeadband(m_driverController.GetLeftX(),DriveConstants::stickDeadBand));
			auto x = -frc::ApplyDeadband(m_driverController.GetLeftY(), DriveConstants::stickDeadBand);
			auto y = -frc::ApplyDeadband(m_driverController.GetLeftX(),DriveConstants::stickDeadBand);
			auto rot = -m_rotLimiter.Calculate(frc::ApplyDeadband(m_driverController.GetRightX(), DriveConstants::stickDeadBand));

			double stickMagnitude = std::clamp(sqrt(pow(x, 2) + pow(y, 2)), -1., 1.);
			double power = 0;
			if(stickMagnitude != 0)
				power = m_powerLimiter.Calculate(stickMagnitude) / stickMagnitude;
			else
				m_powerLimiter.Reset(0);

			//reset on stop
			//if(fabs(x.value()) > DriveConstants::stickDeadBand && fabs(m_driverController.GetLeftY()) < DriveConstants::stickDeadBand) m_xspeedLimiter.Reset(0);
			//if(fabs(y.value()) > DriveConstants::stickDeadBand && fabs(m_driverController.GetLeftX()) < DriveConstants::stickDeadBand) m_yspeedLimiter.Reset(0);
			if(fabs(rot.value()) > DriveConstants::stickDeadBand && fabs(m_driverController.GetRightX()) < DriveConstants::stickDeadBand) m_rotLimiter.Reset(0);
			
			//auto rotMod = (fabs(x.value())>.3 || fabs(y.value()) > .3) ? .5 : 1.0;
			auto rotMod = 1.0;

			if(m_driverController.GetRightTriggerAxis()) {x = std::clamp(x,-.75,.75); y = std::clamp(y,-.75,.75); }

			m_drive.Drive(
				units::meters_per_second_t(x * DriveConstants::kMaxSpeed * power),
				units::meters_per_second_t(y * DriveConstants::kMaxSpeed * power),
				units::radians_per_second_t(rot * DriveConstants::kMaxAngularSpeed * rotMod)
			);
		},
		{&m_drive}
	},
	m_testTrajectory{},
	m_ppTrajectory{},
	m_pathManager{AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration},
	m_validateScriptCmd{}
{
	// Initialize all of your commands and subsystems here

	_InitializeScriptEngine();
	_ConfigureDashboardControls();
	_ConfigureButtonBindings();

	frc::DataLogManager::Start();

	frc::LiveWindow::DisableAllTelemetry();

	// Set up default drive command
	// The left stick controls translation of the robot.
	// Turning is controlled by the X axis of the right stick.
	m_drive.SetDefaultCommand(m_driveCommand);

#ifdef PATHWEAVER
	fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
	deployDirectory = deployDirectory / "output" / "Test1.wpilib.json";
	m_testTrajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
#else
	// This will load the file "Example Path.path" and generate it
	m_ppTrajectory = pathplanner::PathPlanner::loadPath("gethumanplayerball", AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);

	m_pathManager.AddPath("gethumanplayerball");  	//0
	m_pathManager.AddPath("rightsideball"); 	 	//1
	m_pathManager.AddPath("rightsideball2");  		//2
	m_pathManager.AddPath("rightsideball3");  		//3
	m_pathManager.AddPath("leftsideball");  		//4
	m_pathManager.AddPath("cornerRightBall"); 		//5
	m_pathManager.AddPath("bigballs1");  			//6
	m_pathManager.AddPath("bigballs2");  			//7
	m_pathManager.AddPath("bigballs3");  			//8
	m_pathManager.AddPath("leftsideballDG");		//9
	m_pathManager.AddPath("4Ball2");				//10
	m_pathManager.AddPath("BackUp");				//11
	m_pathManager.AddPath("devious1");				//12
	m_pathManager.AddPath("devious2");				//13
	m_pathManager.AddPath("devious3");				//14


	//m_pathManager.AddPath("midBall");
	//m_pathManager.AddPath("topBall");

#endif

	m_totalCurrent = wpi::log::DoubleLogEntry(m_log, "/robot/totalCurrent");
    m_batteryVoltage = wpi::log::DoubleLogEntry(m_log, "/robot/batteryVoltage");
}

// ==========================================================================

	//m_powerDistributionPanel.GetTotalCurrent();

void RobotContainer::LogData() {
	m_totalCurrent.Append(m_powerDistributionPanel.GetTotalCurrent());
	m_batteryVoltage.Append(m_powerDistributionPanel.GetVoltage());

}

// ==========================================================================

std::unique_ptr<frc2::Command> RobotContainer::GetAutonomousCommand() {
	// Get the script.
	auto script{
		nt::NetworkTableInstance::GetDefault()
		.GetTable("Shuffleboard/Autonomous")->GetEntry("Script")
		.GetString("")
	};

	// Build a command from the script.
	return frc4143::ScriptEngine::CreateCommand(script);
}

// ==========================================================================

void RobotContainer::_ConfigureButtonBindings() {
	frc2::InstantCommand pickUpBounceCommand{
		[this]() { m_pickUp.PickUpBounce(); },
		{&m_pickUp}
	};

	frc2::InstantCommand pickUpExtendCommand{
		[this]() { m_pickUp.PickUpExtend(); },
		{&m_pickUp}
	};

	frc2::InstantCommand pickUpRetractCommand{
		[this]() { m_pickUp.PickUpRetract(); },
		{&m_pickUp}
	};

	frc2::InstantCommand pickUpToggleCommand{
		[this]() { m_pickUp.PickUpToggle(); },
		{&m_pickUp}
	};

	frc2::FunctionalCommand rollerInCommand{
		[]() {},
		[this]() { m_pickUp.RollerIn(); },
		[this](bool) { m_pickUp.RollerOff(); },
		[]() { return false; },
		{&m_pickUp}
	};

	frc2::FunctionalCommand rollerOutCommand{
		[]() {},
		[this]() { m_pickUp.RollerOut(); },
		[this](bool) { m_pickUp.RollerOff(); },
		[]() { return false; },
		{&m_pickUp}
	};

	frc2::FunctionalCommand indexerOnCommand{
		[]() {},
		[this]() { m_pickUp.IndexerOn(); },
		[this](bool) { m_pickUp.IndexerOff(); },
		[]() { return false; }
	};

	frc2::FunctionalCommand indexerRevCommand{
		[]() {},
		[this]() { m_pickUp.IndexerRev(); },
		[this](bool) { m_pickUp.IndexerOff(); },
		[]() { return false; }
	};

	frc2::FunctionalCommand shooterOnLimeLightCommand{
		[]() {},
		[this]() { m_pickUp.ShooterOnLimeLight(); },
		[this](bool) { m_pickUp.ShooterOff(); },
		[]() { return false; },
	};

	frc2::FunctionalCommand shooterOnCommand{
		[]() {},
		[this]() { m_pickUp.ShooterOn(); },
		[this](bool) { m_pickUp.ShooterOff(); },
		[]() { return false; },
	};

	frc2::InstantCommand shooterFasterCommand{
		[this]() { m_pickUp.ShooterFaster(); },
	};

	frc2::InstantCommand shooterSlowerCommand{
		[this]() { m_pickUp.ShooterSlower(); },
	};

	frc2::InstantCommand shooterDistToggleCommand{
		[this]() { m_pickUp.ShooterDistToggle(); },
	};

	frc2::InstantCommand nextClimberStepCommand{
		[this]() { m_climber.IndexStep(); },
	};

	frc2::InstantCommand previousClimberStepCommand{
		[this]() { m_climber.BackStep(); },
	};

	//(new frc2::JoystickButton(&m_driverController, JOYSTICK_BUTTON_A))->WhileHeld(rollerInCommand);
	//(new frc2::JoystickButton(&m_driverController, JOYSTICK_BUTTON_START))->WhenPressed(pickUpRetractCommand);
	//(new frc2::JoystickButton(&m_driverController, JOYSTICK_BUTTON_RB))->WhenPressed(shooterFasterCommand);
	//(new frc2::JoystickButton(&m_driverController, JOYSTICK_BUTTON_X))->WhileHeld(indexerOnCommand);

	frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kY)
		.WhenPressed(shooterDistToggleCommand);
	frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kLeftStick)
		.WhenPressed(ToggleDriveMode{&m_drive});
	frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kB)
		.WhileHeld(indexerRevCommand);
	frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kStart)
		.WhenPressed(shooterSlowerCommand);
	frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kBack)
		.WhenPressed(shooterFasterCommand);
	frc2::JoystickButton(&m_climberController, frc::XboxController::Button::kLeftBumper)
		.WhenPressed(nextClimberStepCommand);
	frc2::JoystickButton(&m_climberController, frc::XboxController::Button::kBack)
		.WhenPressed(previousClimberStepCommand);

	frc2::Trigger leftTrigger{
		[this]() {
			return m_driverController.GetLeftTriggerAxis() != 0;
		}
	};
	leftTrigger.WhileActiveOnce(indexerOnCommand);

	frc2::Trigger rightTrigger{
		[this]() {
			return m_driverController.GetRightTriggerAxis() != 0;
		}
	};
	//rightTrigger.WhileActiveContinous(shooterOnCommand);

	frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kRightBumper).WhenPressed(PickUpCycle{&m_pickUp,&m_driverController});
	frc2::JoystickButton(&m_driverController, frc::XboxController::Button::kLeftBumper).WhenPressed(PickUpCycleBounce{&m_pickUp,&m_driverController});

	frc::SmartDashboard::PutData("Zero Climber", new ZeroClimber(&m_climber));
	frc::SmartDashboard::PutData("Set WheelOffsets", new SetWheelOffsets(&m_drive));
	frc::SmartDashboard::PutData("Zero Yaw", new ZeroYaw(&m_drive));
}

// ==========================================================================
#if 0
frc2::Command* RobotContainer::GetAutonomousCommandOld() {
	// Set up config for trajectory
	frc::TrajectoryConfig config(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);

	// Add kinematics to ensure max speed is actually obeyed
	config.SetKinematics(m_drive.kDriveKinematics);

	frc::ProfiledPIDController<units::radians> thetaController{
		AutoConstants::kPThetaController, 0, 0,
		AutoConstants::kThetaControllerConstraints
	};

	thetaController.EnableContinuousInput(
		units::radian_t(-wpi::numbers::pi),
		units::radian_t(wpi::numbers::pi)
	);

	// Reset odometry to the starting pose of the trajectory.
	m_drive.ZeroHeading();

#ifdef PATHWEAVER
	//frc2::SwerveControllerCommand<4> swerveControllerCommand();
	PWSwerveControllerCommand<4> swerveControllerCommand{
		m_testTrajectory,
		[this]() { return m_drive.GetPose(); },
		m_drive.kDriveKinematics,
		frc2::PIDController(AutoConstants::kPXController, 0, 0),
		frc2::PIDController(AutoConstants::kPYController, 0, 0),
		thetaController,
		[this](auto moduleStates) {
			m_drive.SetModuleStates(moduleStates);
		},
		{&m_drive}
	};
	m_drive.ResetOdometry(m_testTrajectory.InitialPose());
#else
	PPSwerveControllerCommand<4> swerveControllerCommand{
		m_ppTrajectory,
		[this]() { return m_drive.GetPose(); },
		m_drive.kDriveKinematics,
		frc2::PIDController(AutoConstants::kPXController, 0, 0),
		frc2::PIDController(AutoConstants::kPYController, 0, 0),
		thetaController,
		[this](auto moduleStates) {
			m_drive.SetModuleStates(moduleStates);
		},
		{&m_drive}
	};
	m_drive.ResetOdometry(m_ppTrajectory.getInitialState()->pose);
#endif



	/*
	bool IsNearWaypoint(Pose2d waypoint, double within) {
		double distance = (-m_drivetrain.getPose()).getTranslation().getDistance(waypoint.getTranslation());
		return (distance <= within);
	};

	frc2::WaitUntilCommand pickUpBall1Command(isNearWaypoint(Pose2d(5.675_m, 2.215_m, Rotation2d(-2.56521743299861_rad)), .25_m)) {
	}
	*/

	return new frc2::SequentialCommandGroup{
		frc2::InstantCommand{
			[this]() {
				m_pickUp.PickUpExtend();
				m_pickUp.RollerIn();
				m_pickUp.IndexerLoad();
				m_pickUp.ShooterClose();
				m_pickUp.SetShooterSpeed(.4);
				m_pickUp.ShooterOn();
			},
			{&m_pickUp}
		},

		frc2::WaitCommand{.1_s},

		std::move(swerveControllerCommand),

		frc2::InstantCommand{
			[this]() {
				m_drive.Drive(
					units::meters_per_second_t(0),
					units::meters_per_second_t(0),
					units::radians_per_second_t(0)
				);
			},
			{}
		},

		frc2::InstantCommand{
			[this]() { m_pickUp.IndexerOn(); },
			{&m_pickUp}
		},

		frc2::WaitCommand{2_s},

		frc2::InstantCommand{
			[this]() {
				m_pickUp.PickUpRetract();
				m_pickUp.RollerOff();
				m_pickUp.IndexerOff();
				m_pickUp.ShooterOff();
			},
			{&m_pickUp}
		}
	};
}
#endif
// ==========================================================================

std::unique_ptr<frc2::Command> RobotContainer::_GetDrivePathCommand() {
	// Set up config for trajectory
	frc::TrajectoryConfig config(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);

	// Add kinematics to ensure max speed is actually obeyed
	config.SetKinematics(m_drive.kDriveKinematics);

	frc::ProfiledPIDController<units::radians> thetaController{
		AutoConstants::kPThetaController, 0, 0,
		AutoConstants::kThetaControllerConstraints
	};

	thetaController.EnableContinuousInput(
		units::radian_t(-wpi::numbers::pi),
		units::radian_t(wpi::numbers::pi)
	);

#ifdef PATHWEAVER
	//frc2::SwerveControllerCommand<4> swerveControllerCommand();
	auto cmd{
		std::make_unique<PWSwerveControllerCommand<4>>(
			m_testTrajectory,
			[this]() { return m_drive.GetPose(); },
			m_drive.kDriveKinematics,
			frc2::PIDController(AutoConstants::kPXController, 0, 0),
			frc2::PIDController(AutoConstants::kPYController, 0, 0),
			thetaController,
			[this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },
			std::initializer_list<frc2::Subsystem*>{&m_drive}
		)
	};

	// Reset odometry to the starting pose of the trajectory.
	//m_drive.ZeroHeading();
	m_drive.ResetOdometry(m_testTrajectory.InitialPose());
#else
	auto cmd{
		std::make_unique<PPSwerveControllerCommand<4>>(
			m_ppTrajectory,
			[this]() { return m_drive.GetPose(); },
			m_drive.kDriveKinematics,
			frc2::PIDController(AutoConstants::kPXController, 0, 0),
			frc2::PIDController(AutoConstants::kPYController, 0, 0),
			thetaController,
			[this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },
			std::initializer_list<frc2::Subsystem*>{&m_drive}
		)
	};
	frc::Pose2d pose(m_ppTrajectory.getInitialState()->pose.Translation(), m_ppTrajectory.getInitialState()->holonomicRotation);
	m_drive.ResetOdometry(pose);
	//m_drive.ResetOdometry(m_ppTrajectory.getInitialState());
#endif

	return cmd;
}

// ==========================================================================

void RobotContainer::_ConfigureDashboardControls() {
	// Add a button on the dashboard for validating the script.
	frc::Shuffleboard::GetTab("Autonomous").Add(m_validateScriptCmd);
}

// ==========================================================================

void RobotContainer::_InitializeScriptEngine() {
	auto parser{frc4143::ScriptParser::Create()};

	// TODO - Add your script commands here

	parser->Add(
		frc4143::ScriptParserElement{
			"Sleep", {"S"},
			[](std::vector<float> parameters) {
				parameters.resize(1);
				units::time::second_t duration{parameters[0]};
				return std::make_unique<frc2::WaitCommand>(duration);
			}
		}
	);

	parser->Add(
		frc4143::ScriptParserElement{
			"StartUp", {"SU"},
			[this](std::vector<float> parameters) {
				parameters.resize(2);
				auto speed{parameters[0]};
				auto far {!!parameters[1]};
				return std::make_unique<frc2::InstantCommand>(
					[this, speed, far]() {
						//m_pickUp.PickUpExtend();
						//m_pickUp.RollerIn();
						m_pickUp.IndexerLoad();
						if (far) {
							m_pickUp.ShooterFar();
						}
						else {
							m_pickUp.ShooterClose();
						}
						m_pickUp.SetShooterSpeed(speed);
						m_pickUp.ShooterOnManual();
					}
				);
			}
		}
	);

	parser->Add(
		frc4143::ScriptParserElement{
			"ShooterLimeLight", {"SL"},
			[this](std::vector<float> parameters) {
				return std::make_unique<frc2::FunctionalCommand>(
					[]() {},
					[this]() { m_pickUp.ShooterOnLimeLight();
							   m_drive.DriveLime();},
					[](bool) {},
					[this]() { return m_pickUp.HasShot(); }
				);
			}
		}
	);

	parser->Add(
		frc4143::ScriptParserElement{
			"Stop", {},
			[this](std::vector<float> parameters) {
				return std::make_unique<frc2::InstantCommand>(
					[this]() {
						m_pickUp.PickUpRetract();
						m_pickUp.RollerOff();
						m_pickUp.IndexerOff();
						m_pickUp.ShooterOff();
					}
				);
			}
		}
	);

	parser->Add(
		frc4143::ScriptParserElement{
			"IndexerOn", {"iOn"},
			[this](std::vector<float> parameters) {
				return std::make_unique<frc2::FunctionalCommand>(
					[]() {},
					[this]() { m_pickUp.IndexerOn();},
					[this](bool) { m_pickUp.IndexerOff(); },
					[]() { return false; }
				);
			}
		}
	);

	frc2::FunctionalCommand shooterOnCommand{
		[]() {},
		[this]() { m_pickUp.ShooterOn(); },
		[this](bool) { m_pickUp.ShooterOff(); },
		[]() { return false; },
	};

	/*parser->Add(
		frc4143::ScriptParserElement{
			"PickUpRetract", {"PR"},
			[this](std::vector<float> parameters) {
				return std::make_unique<frc2::InstantCommand>(
					[this]() {
						m_pickUp.PickUpRetract();
					}
				);
			}
		}
	);*/

	parser->Add(
		frc4143::ScriptParserElement{
			"PickUpRetract", {"PR"},
			[this](std::vector<float> parameters) {
				return std::make_unique<frc2::FunctionalCommand>(
					[this]() { m_pickUp.PickUpRetract(); m_pickUp.IndexerLoad();},
					[]() {},
					[this](bool) { m_pickUp.IndexerOff(); m_pickUp.RollerOff();},
					[]() {return false;}
				);
			}
		}
	);

	parser->Add(
		frc4143::ScriptParserElement{
			"PickUpExtend", {"PE"},
			[this](std::vector<float> parameters) {
				return std::make_unique<frc2::InstantCommand>(
					[this]() {
						m_pickUp.PickUpExtend();
						m_pickUp.RollerIn();
						m_pickUp.IndexerLoad();
					}
				);
			}
		}
	);

	parser->Add(
		frc4143::ScriptParserElement{
			"PickUpExtendNoFeed", {"PENF"},
			[this](std::vector<float> parameters) {
				return std::make_unique<frc2::InstantCommand>(
					[this]() {
						m_pickUp.PickUpExtend();
						m_pickUp.RollerIn();
						m_pickUp.IndexerOff();
					}
				);
			}
		}
	);


	parser->Add(
		frc4143::ScriptParserElement{
			"DrivePath", {"DP"},
			[this](std::vector<float> parameters) {
				return _GetDrivePathCommand();
			}
		}
	);

	// DrivePath2
	// DP2(index, resetOdometry)
	parser->Add(
		frc4143::ScriptParserElement{
			"DrivePath2", {"DP2"},
			[this](std::vector<float> parameters) {
				parameters.resize(2);
				auto index{parameters[0]};
				auto resetOdometry{!!parameters[1]};

				frc::ProfiledPIDController<units::radians> thetaController{
					AutoConstants::kPThetaController, 0, 0,
					AutoConstants::kThetaControllerConstraints
				};

				thetaController.EnableContinuousInput(
					units::radian_t(-wpi::numbers::pi),
					units::radian_t(wpi::numbers::pi)
				);

				return m_pathManager.GetPathCommand(
					&m_drive,
					frc2::PIDController(AutoConstants::kPXController, 0, 0),
					frc2::PIDController(AutoConstants::kPYController, 0, 0),
					thetaController, index, resetOdometry
				);
			}
		}
	);

	// ResetOdometry
	// RO(xMeters, yMeters, degrees)
	parser->Add(
		frc4143::ScriptParserElement{
			"ResetOdometry", {"RO"},
			[this](std::vector<float> parameters) {
				parameters.resize(3);
				auto x{units::meter_t{parameters[0]}};
				auto y{units::meter_t{parameters[1]}};
				auto theta{units::degree_t{parameters[2]}};

				frc::Pose2d pose{x, y, frc::Rotation2d{theta}};

				return std::make_unique<frc2::InstantCommand>(
					[this, pose]() {
						m_drive.ResetOdometry(pose);
					}
				);
			}
		}
	);

	parser->Add(
		frc4143::ScriptParserElement{
			"ZeroClimber", {"ZC"},
			[this](std::vector<float> parameters) {
				return std::make_unique<ZeroClimber>(&m_climber);
			}
		}
	);

	parser->Add(
		frc4143::ScriptParserElement{
			"ZeroGyro", {"ZG"},
			[this](std::vector<float> parameters) {
				return std::make_unique<ZeroYaw>(&m_drive);
			}
		}
	);

	parser->Add(
		frc4143::ScriptParserElement{
			"DriveStop", {"DS"},
			[this](std::vector<float> parameters) {
				return std::make_unique<frc2::InstantCommand>(
					[this]() {
						m_drive.MotorsOff();
					}
				);
			}	
		}
	);

	parser->Add(
		frc4143::ScriptParserElement{
			"DriveGyro", {"DG"},
			[this](std::vector<float> parameters) {
				parameters.resize(3);
				auto x{parameters[0]};
				auto y{parameters[1]};
				auto angle{parameters[2]};
				return std::make_unique<DriveGyro>(&m_drive, x, y, angle);
			}
		}
	);

	parser->Add(
		frc4143::ScriptParserElement{
			"DriveLime", {"DL"},
			[this](std::vector<float> parameters) {
				return std::make_unique<DriveLime>(&m_drive);
			}
		}
	);

	parser->Add(
		frc4143::ScriptParserElement{
			"RollerOff", {"rOff"},
			[this](std::vector<float> parameters) {
				parameters.resize(1);
				units::time::second_t duration{parameters[0]};
				return std::make_unique<frc2::SequentialCommandGroup>(
					*std::make_unique<frc2::WaitCommand>(duration),
					*std::make_unique<frc2::InstantCommand>(
						[this]() {
						m_pickUp.RollerOff();
						}
					)
				);
			}
		}
	);

	parser->Add(
		frc4143::ScriptParserElement{
			"EjectBall", {"EB"},
			[this](std::vector<float> parameters) {
				return std::make_unique<frc2::InstantCommand>(
					[this]() {
						m_pickUp.PickUpBounce();
						m_pickUp.IndexerRev();
						m_pickUp.RollerOut();
					}
				);
			}
		}
	);



	// Do an initial parse to build all the regular expressions.
	parser->IsValid("S(0)");

	frc4143::ScriptEngine::SetParser(parser);
}

// ==========================================================================
