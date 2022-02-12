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
#include <units/angle.h>
#include <units/velocity.h>
#include <utility>
#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "commands/SetWheelOffsets.h"
#include "commands/ZeroYaw.h"

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
:  m_pickUp{},
m_FieldCentricMode{[this] {
        m_drive.Drive(
            units::meters_per_second_t(-m_xspeedLimiter.Calculate(frc::ApplyDeadband(m_driverController.GetLeftY(), 0.2))*AutoConstants::kMaxSpeed),
            units::meters_per_second_t(-m_yspeedLimiter.Calculate(frc::ApplyDeadband(m_driverController.GetLeftX(),0.2))*AutoConstants::kMaxSpeed),
            units::radians_per_second_t(m_rotLimiter.Calculate(frc::ApplyDeadband(m_driverController.GetRightX(), 0.2))*AutoConstants::kMaxAngularSpeed), true);
      },
      {&m_drive}
  },
    m_CrabMode{[this] {
        m_drive.Drive(
          units::meters_per_second_t(-m_xspeedLimiter.Calculate(frc::ApplyDeadband(m_driverController.GetLeftY(), 0.2))*AutoConstants::kMaxSpeed),
          units::meters_per_second_t(-m_yspeedLimiter.Calculate(frc::ApplyDeadband(m_driverController.GetLeftX(), 0.2))*AutoConstants::kMaxSpeed),
          units::radians_per_second_t(m_rotLimiter.Calculate(frc::ApplyDeadband(m_driverController.GetRightX(), 0.2))*AutoConstants::kMaxAngularSpeed), false);
      },
      {&m_drive}
  }
 {
  // Initialize all of your commands and subsystems here

  // Configure the button bindings
  ConfigureButtonBindings();
  frc::LiveWindow::DisableAllTelemetry();
  // Set up default drive command
  // The left stick controls translation of the robot.
  // Turning is controlled by the X axis of the right stick.
  m_drive.SetDefaultCommand(m_FieldCentricMode);

}

void RobotContainer::ConfigureButtonBindings() {

// ============================================================================
    
    /*
    frc2::InstantCommand pickUpBounceCommand{
        [this]() {m_pickUp.PickUpBounce();},
        {&m_pickUp}};
    
    frc2::InstantCommand pickUpExtendCommand{
        [this]() {m_pickUp.PickUpExtend();},
        {&m_pickUp}};
    */

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
        
// ============================================================================

    frc2::FunctionalCommand shooterOnCommand{
        [this]() {m_pickUp.ShooterOn();},
        []() {},
        [this](bool) {m_pickUp.ShooterOff();},
        []() {return false;}, 
        {&m_pickUp}};

    frc2::InstantCommand shooterFasterCommand{
        [this]() {m_pickUp.ShooterFaster();},
        {&m_pickUp}};

    frc2::InstantCommand shooterSlowerCommand{
        [this]() {m_pickUp.ShooterSlower();},
        {&m_pickUp}};

// ============================================================================

    (new frc2::JoystickButton(&m_driverController, JOYSTICK_BUTTON_A))->WhileHeld(rollerInCommand);
    (new frc2::JoystickButton(&m_driverController, JOYSTICK_BUTTON_B))->WhileHeld(rollerOutCommand);
    (new frc2::JoystickButton(&m_driverController, JOYSTICK_BUTTON_START))->WhenPressed(pickUpRetractCommand);
    (new frc2::JoystickButton(&m_driverController, JOYSTICK_BUTTON_BACK))->WhenPressed(pickUpToggleCommand);
    (new frc2::JoystickButton(&m_driverController, JOYSTICK_BUTTON_X))->WhileHeld(indexerOnCommand);
    (new frc2::JoystickButton(&m_driverController, JOYSTICK_BUTTON_LEFT))->ToggleWhenPressed(m_CrabMode);

    (new frc2::JoystickButton(&m_driverController, JOYSTICK_BUTTON_Y))->WhileHeld(shooterOnCommand);
    (new frc2::JoystickButton(&m_driverController, JOYSTICK_BUTTON_RB))->WhenPressed(shooterFasterCommand);
    (new frc2::JoystickButton(&m_driverController, JOYSTICK_BUTTON_LB))->WhenPressed(shooterSlowerCommand);

    frc::SmartDashboard::PutData("Set WheelOffsets", new SetWheelOffsets(&m_drive));
    frc::SmartDashboard::PutData("Zero Yaw", new ZeroYaw(&m_drive));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  // Set up config for trajectory
  frc::TrajectoryConfig config(AutoConstants::kMaxSpeed, AutoConstants::kMaxAcceleration);
  // Add kinematics to ensure max speed is actually obeyed
  config.SetKinematics(m_drive.kDriveKinematics);

  // An example trajectory to follow.  All units in meters.
  auto exampleTrajectory = frc::TrajectoryGenerator::GenerateTrajectory(
      // Start at the origin facing the +X direction
      frc::Pose2d(0_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass through these two interior waypoints, making an 's' curve path
      {frc::Translation2d(1_m, 1_m), frc::Translation2d(2_m, -1_m)},
      // End 3 meters straight ahead of where we started, facing forward
      frc::Pose2d(3_m, 0_m, frc::Rotation2d(0_deg)),
      // Pass the config
      config);

  frc::ProfiledPIDController<units::radians> thetaController{
      AutoConstants::kPThetaController, 0, 0,
      AutoConstants::kThetaControllerConstraints};

  thetaController.EnableContinuousInput(units::radian_t(-wpi::numbers::pi),
                                        units::radian_t(wpi::numbers::pi));

  frc2::SwerveControllerCommand<4> swerveControllerCommand(
      exampleTrajectory, [this]() { return m_drive.GetPose(); },

      m_drive.kDriveKinematics,

      frc2::PIDController(AutoConstants::kPXController, 0, 0),
      frc2::PIDController(AutoConstants::kPYController, 0, 0), thetaController,

      [this](auto moduleStates) { m_drive.SetModuleStates(moduleStates); },

      {&m_drive});

  // Reset odometry to the starting pose of the trajectory.
  m_drive.ResetOdometry(exampleTrajectory.InitialPose());

  // no auto
  return new frc2::SequentialCommandGroup(
      std::move(swerveControllerCommand),
      frc2::InstantCommand(
          [this]() {
            m_drive.Drive(units::meters_per_second_t(0),
                          units::meters_per_second_t(0),
                          units::radians_per_second_t(0), false);
          },
          {}));
}
