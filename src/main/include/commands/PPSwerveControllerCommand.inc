// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <memory>
#include <utility>

#include "PPSwerveControllerCommand.h"

template <size_t NumModules>
PPSwerveControllerCommand<NumModules>::PPSwerveControllerCommand(
    pathplanner::PathPlannerTrajectory trajectory, std::function<frc::Pose2d()> pose,
    frc::SwerveDriveKinematics<NumModules> kinematics,
    frc2::PIDController xController, frc2::PIDController yController,
    frc::ProfiledPIDController<units::radians> thetaController,
    std::function<void(std::array<frc::SwerveModuleState, NumModules>)> output,
    std::initializer_list<frc2::Subsystem*> requirements)
    : m_trajectory(std::move(trajectory)),
      m_pose(std::move(pose)),
      m_kinematics(kinematics),
      m_controller(xController, yController, thetaController),
      m_outputStates(output) {
  this->AddRequirements(requirements);
}

template <size_t NumModules>
PPSwerveControllerCommand<NumModules>::PPSwerveControllerCommand(
    pathplanner::PathPlannerTrajectory trajectory, std::function<frc::Pose2d()> pose,
    frc::SwerveDriveKinematics<NumModules> kinematics,
    frc2::PIDController xController, frc2::PIDController yController,
    frc::ProfiledPIDController<units::radians> thetaController,
    std::function<void(std::array<frc::SwerveModuleState, NumModules>)> output,
    wpi::span<frc2::Subsystem* const> requirements)
    : m_trajectory(std::move(trajectory)),
      m_pose(std::move(pose)),
      m_kinematics(kinematics),
      m_controller(xController, yController, thetaController),
      m_outputStates(output) {
  this->AddRequirements(requirements);
}

template <size_t NumModules>
void PPSwerveControllerCommand<NumModules>::Initialize() {
  m_timer.Reset();
  m_timer.Start();
}

template <size_t NumModules>
void PPSwerveControllerCommand<NumModules>::Execute() {
  auto curTime = units::second_t(m_timer.Get());
  pathplanner::PathPlannerTrajectory::PathPlannerState  m_desiredState = m_trajectory.sample(curTime);
  frc::Trajectory::State wpiState;

  wpiState.t = m_desiredState.time;
  wpiState.pose = m_desiredState.pose;
  wpiState.velocity = m_desiredState.velocity;
  wpiState.acceleration = m_desiredState.acceleration;
  wpiState.curvature = m_desiredState.curvature;

  auto targetChassisSpeeds =
      m_controller.Calculate(m_pose(), wpiState, m_desiredState.holonomicRotation);
  auto targetModuleStates =
      m_kinematics.ToSwerveModuleStates(targetChassisSpeeds);

  m_outputStates(targetModuleStates);
}

template <size_t NumModules>
void PPSwerveControllerCommand<NumModules>::End(bool interrupted) {
  m_timer.Stop();
}

template <size_t NumModules>
bool PPSwerveControllerCommand<NumModules>::IsFinished() {
  return m_timer.HasElapsed(m_trajectory.getTotalTime());
}

