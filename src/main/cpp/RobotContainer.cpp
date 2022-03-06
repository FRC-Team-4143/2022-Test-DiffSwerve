// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include <frc/controller/PIDController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryGenerator.h>
#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/SwerveControllerCommand.h>
#include "commands/PWSwerveControllerCommand.h"
#include <units/angle.h>
#include <units/velocity.h>
#include <utility>
#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "commands/SetWheelOffsets.h"
#include "commands/ZeroYaw.h"
#include "commands/ToggleDriveMode.h"
#include <frc2/command/WaitCommand.h>
#include "commands/PickUpCycle.h"
#include "commands/PickUpCycleBounce.h"
#include <frc2/command/button/Trigger.h>
#include <frc2/command/WaitUntilCommand.h>
#include "commands/ZeroClimber.h"
#include "Scripting/ScriptEngine.h"
#include "Scripting/ScriptParser.h"
#include "Scripting/ScriptParserElement.h"
#include <networktables/NetworkTableEntry.h>

using namespace DriveConstants;

const uint32_t JOYSTICK_LX_AXIS = 0;
const uint32_t JOYSTICK_LY_AXIS = 1;
const uint32_t JOYSTICK_LTRIG_AXIS = 2;
const uint32_t JOYSTICK_RTRIG_AXIS = 3;
const uint32_t JOYSTICK_RX_AXIS = 4;
const uint32_t JOYSTICK_RY_AXIS = 5;

const uint32_t JOYSTICK_BUTTON_A = 1;
const uint32_t JOYSTICK_BUTTON_B = 2;
const uint32_t JOYSTICK_BUTTON_X = 3;
const uint32_t JOYSTICK_BUTTON_Y = 4;
const uint32_t JOYSTICK_BUTTON_LB = 5;
const uint32_t JOYSTICK_BUTTON_RB = 6;
const uint32_t JOYSTICK_BUTTON_BACK = 7;
const uint32_t JOYSTICK_BUTTON_START = 8;
const uint32_t JOYSTICK_BUTTON_LEFT = 9;
const uint32_t JOYSTICK_BUTTON_RIGHT = 10;

RobotContainer::RobotContainer()
:  m_pickUp{}, m_climber{&m_climberController},
m_DriveCommand{[this] {
        m_drive.Drive(
            units::meters_per_second_t(-m_xspeedLimiter.Calculate(frc::ApplyDeadband(m_driverController.GetLeftY(), 0.2))*AutoConstants::kMaxSpeed),
            units::meters_per_second_t(-m_yspeedLimiter.Calculate(frc::ApplyDeadband(m_driverController.GetLeftX(),0.2))*AutoConstants::kMaxSpeed),
            units::radians_per_second_t(-m_rotLimiter.Calculate(frc::ApplyDeadband(m_driverController.GetRightX(), 0.2))*AutoConstants::kMaxAngularSpeed)*.5);
      },
      {&m_drive}
},
    _validateScriptCmd{},
    m_testTrajectory{}
 {
  // Initialize all of your commands and subsystems here
 
  // Configure the button bindings
  
  _InitializeScriptEngine();
  _ConfigureDashboardControls();
  ConfigureButtonBindings();
  frc::LiveWindow::DisableAllTelemetry();
  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(m_DriveCommand);

  fs::path deployDirectory = frc::filesystem::GetDeployDirectory();
  deployDirectory = deployDirectory / "output" / "Test1.wpilib.json";
  m_testTrajectory = frc::TrajectoryUtil::FromPathweaverJson(deployDirectory.string());
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

void RobotContainer::ConfigureButtonBindings() {

// ============================================================================
    
    
    frc2::InstantCommand pickUpBounceCommand{
        [this]() {m_pickUp.PickUpBounce();},
        {&m_pickUp}};
    
    frc2::InstantCommand pickUpExtendCommand{
        [this]() {m_pickUp.PickUpExtend();},
        {&m_pickUp}};
    

    frc2::InstantCommand pickUpRetractCommand{
        [this]() {m_pickUp.PickUpRetract();},
        {&m_pickUp}};

    frc2::InstantCommand pickUpToggleCommand{
        [this]() {m_pickUp.PickUpToggle();},
        {&m_pickUp}};    
    
// ============================================================================

    frc2::FunctionalCommand rollerInCommand{
        [this]() {m_pickUp.RollerIn();},
        []() {},
        [this](bool) {m_pickUp.RollerOff();},
        []() {return false;}, 
        {&m_pickUp}};
    
    frc2::FunctionalCommand rollerOutCommand{
        [this]() {m_pickUp.RollerOut();},
        []() {},
        [this](bool) {m_pickUp.RollerOff();},
        []() {return false;}, 
        {&m_pickUp}};

// ============================================================================

    frc2::FunctionalCommand indexerOnCommand{
        [this]() {m_pickUp.IndexerOn();},
        []() {},
        [this](bool) {m_pickUp.IndexerOff();},
        []() {return false;}
        };

    frc2::FunctionalCommand indexerRevCommand{
        [this]() {m_pickUp.IndexerRev();},
        []() {},
        [this](bool) {m_pickUp.IndexerOff();},
        []() {return false;}
        };
        
// ============================================================================

    frc2::FunctionalCommand shooterOnCommand{
        [this]() {m_pickUp.ShooterOn();},
        []() {},
        [this](bool) {m_pickUp.ShooterOff();},
        []() {return false;}, 
        };

    frc2::InstantCommand shooterFasterCommand{
        [this]() {m_pickUp.ShooterFaster();},
        };

    frc2::InstantCommand shooterSlowerCommand{
        [this]() {m_pickUp.ShooterSlower();},
        };

    frc2::InstantCommand shooterDistToggleCommand{
        [this]() {m_pickUp.ShooterDistToggle();},
        };

// ============================================================================

    //(new frc2::JoystickButton(&m_driverController, JOYSTICK_BUTTON_A))->WhileHeld(rollerInCommand);
    //(new frc2::JoystickButton(&m_driverController, JOYSTICK_BUTTON_START))->WhenPressed(pickUpRetractCommand);
    (new frc2::JoystickButton(&m_driverController, JOYSTICK_BUTTON_BACK))->WhenPressed(shooterFasterCommand);
    //(new frc2::JoystickButton(&m_driverController, JOYSTICK_BUTTON_RB))->WhenPressed(shooterFasterCommand);
    (new frc2::JoystickButton(&m_driverController, JOYSTICK_BUTTON_START))->WhenPressed(shooterSlowerCommand);

    (new frc2::JoystickButton(&m_driverController, JOYSTICK_BUTTON_B))->WhileHeld(indexerRevCommand);
    //(new frc2::JoystickButton(&m_driverController, JOYSTICK_BUTTON_X))->WhileHeld(indexerOnCommand);
    (new frc2::JoystickButton(&m_driverController, JOYSTICK_BUTTON_LEFT))->WhenPressed(ToggleDriveMode{&m_drive});
    (new frc2::JoystickButton(&m_driverController, JOYSTICK_BUTTON_Y))->WhenPressed(shooterDistToggleCommand);


    frc2::Trigger RT{
        [this]() 
        {return m_driverController.GetRightTriggerAxis() != 0;}
    };

    RT.WhileActiveContinous(shooterOnCommand);

    frc2::Trigger LT{
        [this]() 
        {return m_driverController.GetLeftTriggerAxis() != 0;}
    };

    LT.WhileActiveContinous(indexerOnCommand);


    m_rb = new frc2::JoystickButton(&m_driverController, JOYSTICK_BUTTON_RB);
    m_lb = new frc2::JoystickButton(&m_driverController, JOYSTICK_BUTTON_LB);
    m_rb->WhenPressed(PickUpCycle{&m_pickUp,&m_driverController});
    m_lb->WhenPressed(PickUpCycleBounce{&m_pickUp,&m_driverController});
  

    frc::SmartDashboard::PutData("Zero Climber", new ZeroClimber(&m_climber));
    frc::SmartDashboard::PutData("Set WheelOffsets", new SetWheelOffsets(&m_drive));
    frc::SmartDashboard::PutData("Zero Yaw", new ZeroYaw(&m_drive));
}

frc2::Command* RobotContainer::GetAutonomousCommand2() {
  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);


  frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t(-wpi::numbers::pi),
                                        units::radian_t(wpi::numbers::pi));

  //frc2::SwerveControllerCommand<4> swerveControllerCommand();
  PWSwerveControllerCommand<4> swerveControllerCommand(
      m_testTrajectory, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0), thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  // Reset odometry to the starting pose of the trajectory.
  m_drive.ZeroHeading();
  m_drive.ResetOdometry(m_testTrajectory.InitialPose());
/*
  bool IsNearWaypoint(Pose2d waypoint, double within) {
  		double distance = (-m_drivetrain.getPose()).getTranslation().getDistance(waypoint.getTranslation());
  		return (distance <= within);
	};

  frc2::WaitUntilCommand pickUpBall1Command(isNearWaypoint(Pose2d(5.675_m, 2.215_m, Rotation2d(-2.56521743299861_rad)), .25_m)){

      }
*/
  // no auto
  return new frc2::SequentialCommandGroup(
      frc2::InstantCommand{
        [this]() {
            m_pickUp.PickUpExtend();
            m_pickUp.RollerIn();
            m_pickUp.IndexerLoad();
            m_pickUp.ShooterClose();
            m_pickUp.SetShooterSpeed(.4);
            m_pickUp.ShooterOn();
        },
        {&m_pickUp}},

      frc2::WaitCommand{.1_s},

      std::move(swerveControllerCommand),

      frc2::InstantCommand(
        [this]() {
        m_drive.Drive(units::meters_per_second_t(0),
                      units::meters_per_second_t(0),
                      units::radians_per_second_t(0));
        },
        {}),

      frc2::InstantCommand{
        [this]() {m_pickUp.IndexerOn();},
        {&m_pickUp}},
        
      frc2::WaitCommand{2_s},      

      frc2::InstantCommand{
        [this]() {
            m_pickUp.PickUpRetract();
            m_pickUp.RollerOff();
            m_pickUp.IndexerOff();
            m_pickUp.ShooterOff();
            },
        {&m_pickUp}});
}

std::unique_ptr<frc2::Command> RobotContainer::GetSwerveCommand(){
      // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);


  frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t(-wpi::numbers::pi),
                                        units::radian_t(wpi::numbers::pi));


  //frc2::SwerveControllerCommand<4> swerveControllerCommand();
  auto cmd{ std::make_unique<PWSwerveControllerCommand<4>>(
      m_testTrajectory, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0), thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },
      std::initializer_list<frc2::Subsystem*>{&m_drive})};

  // Reset odometry to the starting pose of the trajectory.
  //m_drive.ZeroHeading();
  m_drive.ResetOdometry(m_testTrajectory.InitialPose());

  return cmd;

}


// ==========================================================================

void RobotContainer::_ConfigureDashboardControls() {
	// Add a button on the dashboard for validating the script.
	frc::Shuffleboard::GetTab("Autonomous").Add(_validateScriptCmd);
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
				parameters.resize(1);
				double speed{parameters[0]};
				return  std::make_unique<frc2::InstantCommand>(
                            [this, speed]() {
                            m_pickUp.PickUpExtend();
                            m_pickUp.RollerIn();
                            m_pickUp.IndexerLoad();
                            m_pickUp.ShooterClose();
                            m_pickUp.SetShooterSpeed(speed);
                            m_pickUp.ShooterOn();
                        });
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
                            });
                }
		}
	);

    parser->Add(
		frc4143::ScriptParserElement{
			"IndexerOn", {"iOn"},
			[this](std::vector<float> parameters) {
				return std::make_unique<frc2::InstantCommand>(
                        [this]() {
                            m_pickUp.IndexerOn();
                            });
                }
		}
	);

    parser->Add(
		frc4143::ScriptParserElement{
			"DrivePathWeaver", {"DP"},
			[this](std::vector<float> parameters) {
				return GetSwerveCommand();
                }
		}
	);

	// Do an initial parse to build all the regular expressions.
	parser->IsValid("S(0)");

	frc4143::ScriptEngine::SetParser(parser);
}

