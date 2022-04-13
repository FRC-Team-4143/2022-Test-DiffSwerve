// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"
//#include <frc/geometry/Rotation2d.h>
#include <frc/RobotController.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include "Constants.h"
#include <wpi/numbers>
#include <math.h>

#define CANIVORE "Default Name"

using namespace DriveConstants;

// ==========================================================================

DriveSubsystem::DriveSubsystem(frc::XboxController* controller)
:	m_frontLeft{kFrontLeftDriveMotorPort, kFrontLeftTurningMotorPort, kFrontLeftPot, "frontLeft", "roborio"},
	m_rearLeft{kRearLeftDriveMotorPort, kRearLeftTurningMotorPort, kRearLeftPot, "rearLeft", CANIVORE},
	m_frontRight{kFrontRightDriveMotorPort, kFrontRightTurningMotorPort, kFrontRightPot, "frontRight", "roborio"},
	m_rearRight{kRearRightDriveMotorPort, kRearRightTurningMotorPort, kRearRightPot, "rearRight", CANIVORE},
	m_odometry{kDriveKinematics, GetHeading(), frc::Pose2d()},
	m_fieldCentric{false},
	m_controller(controller),
	m_lastPose{m_odometry.GetPose()}
{
	LoadWheelOffsets();
	frc::SmartDashboard::PutData("Field", &m_field);
	m_limelightTable = nt::NetworkTableInstance::GetDefault().GetTable("limelight");
	frc::SmartDashboard::PutBoolean("Disable Limelight", false);
}
 
// ==========================================================================

void DriveSubsystem::Periodic() {
	auto frontLeftState = m_frontLeft.GetState();
	auto rearLeftState = m_rearLeft.GetState();
	auto frontRightState = m_frontRight.GetState();
	auto rearRightState = m_rearRight.GetState();

	auto [vx, vy, vr] = kDriveKinematics.ToChassisSpeeds(frontLeftState, frontRightState, rearLeftState, rearRightState);

	double ty = m_limelightTable->GetNumber("ty",0);
	double tx = m_limelightTable->GetNumber("tx",0);
	double tv = m_limelightTable->GetNumber("tv",0);


/*
	double airTime = 0.8149467 + 0.003095104*ty + 0.001401973*pow(ty,2);  //seconds
	//converts dist to meters from inches
	double dist = 0.0254*(67.77761 - 1.716064*ty + 0.2305231*pow(ty,2));  //meters
*/
	double dist = 2.62 + -0.177*ty + .00823*pow(ty,2);
	double airTime = 0.671 + 0.0418*dist + 0.0186*pow(dist,2);

	if (tv == 0) {dist = 3; airTime = 1;}

	double totDist = dist + -vx.value()*airTime;  //meters

	m_expectedOffset = 1/.02 * atan2(-vy.value()*.02, dist);  //Radians per Second
	m_expectedOffset *= .65;   /// adjust to change lag of shot

	m_offset = atan2(-vy.value()*airTime, totDist) * 1.;   //radians    // test constant

	m_offset *= 180./wpi::numbers::pi;  // degrees

	m_offset = std::clamp(m_offset, -25., 25.);

	m_realDist = totDist;
	//double m_realDist = -vy.value()*airTime/sin(m_offset);

	m_currentYaw = m_pidgey.GetYaw() - m_zero;

	m_odometry.Update(
		GetHeading(),
		frontLeftState,
		frontRightState,
		rearLeftState,
		rearRightState
	);
/*
	m_poseEstimator.Update(
		GetHeading(),
		frontLeftState,
		frontRightState,
		rearLeftState,
		rearRightState
	);
*/
/*
	frc::Pose2d currentPose = m_odometry.GetPose();
	auto ds = m_lastPose - currentPose;
	auto heading = currentPose.Rotation().Radians().value();
	auto xVel = ds.X().value()/.02;
	auto yVel = ds.Y().value()/.02;
	auto rVel = ds.Rotation().Radians().value()/.02;

	auto lxVel = xVel * sin(heading) + yVel * cos(wpi::numbers::pi - heading);
	auto lyVel = xVel * cos(heading) + yVel * sin(90 - heading);
*/
	//m per s
	

	//auto rsPosition{_GetPositionFromRealSense()};
	//auto rsYaw{_GetYawFromRealSense()};
/*
	m_poseEstimator.AddVisionMeasurement(
		frc::Pose2d{rsPosition, rsYaw},
		frc::Timer::GetFPGATimestamp() - 0.05_s
	);
*/
/*
	frc::SmartDashboard::PutNumber("m_odometry_x", m_odometry.GetPose().X().value());
	frc::SmartDashboard::PutNumber("m_odometry_y", m_odometry.GetPose().Y().value());
	frc::SmartDashboard::PutNumber("m_odometry_r", m_odometry.GetPose().Rotation().Degrees().value());

	frc::SmartDashboard::PutNumber("m_poseEstimator_x", m_poseEstimator.GetEstimatedPosition().X().value());
	frc::SmartDashboard::PutNumber("m_poseEstimator_y", m_poseEstimator.GetEstimatedPosition().Y().value());
	frc::SmartDashboard::PutNumber("m_poseEstimator_r", m_poseEstimator.GetEstimatedPosition().Rotation().Degrees().value());
*/
	frc::SmartDashboard::PutNumber("Gyro", GetHeading().value());
	frc::SmartDashboard::PutBoolean("FieldCentric", m_fieldCentric);
	//frc::SmartDashboard::PutNumber("RSYaw", rsYaw.value());
	frc::SmartDashboard::PutNumber("vx" ,vx.value());
	frc::SmartDashboard::PutNumber("vy", vy.value());
	frc::SmartDashboard::PutNumber("vr", vr.value());
	//frc::SmartDashboard::PutNumber("vxbug" ,vxbug.value());
	//frc::SmartDashboard::PutNumber("vybug", vybug.value());
	//frc::SmartDashboard::PutNumber("vrbug", vrbug.value());
	frc::SmartDashboard::PutNumber("m_offset", m_offset);
	frc::SmartDashboard::PutNumber("actualrealDist", m_realDist);
	frc::SmartDashboard::PutNumber("expectedOFfset", m_expectedOffset);
	frc::SmartDashboard::PutNumber("airTime", airTime);

	

	m_field.SetRobotPose(m_odometry.GetPose());

	//Wheel Offset Code;
	if (frc::RobotController::GetUserButton() == 1 && m_counter == 0) {
		SetWheelOffsets();
		m_counter = 100;
		fmt::print("ERRROR: User Button Pressed\n");
	}

	if (m_counter > 0) {
		m_counter -= 1;
	}
}

// ==========================================================================

void DriveSubsystem::Drive(
	units::meters_per_second_t xSpeed,
	units::meters_per_second_t ySpeed,
	units::radians_per_second_t rot)
{
	auto limeLightTX = m_limelightTable->GetNumber("tx", 0.0);
	auto limeLightTV = m_limelightTable->GetNumber("tv", 0.0);
    double limeLightP = frc::SmartDashboard::GetNumber("limeLightP", -.03);
	double tolerance = frc::SmartDashboard::GetNumber("limelightTolerance", 3);

	double ffoffset = .1;
	//                
	double newTX = 0;                                                                                                                                                  double txOffset = 0.7223274*m_offset + 1.029443;
	if(limeLightTV > 0)
		newTX = limeLightTX - m_offset;

	
	if (m_controller->GetRightTriggerAxis() != 0 && !frc::SmartDashboard::GetBoolean("Disable Limelight", 0) && limeLightTV != 0)  {
		if (fabs(newTX) > tolerance)
			rot =  units::radians_per_second_t(newTX*(limeLightP) + m_expectedOffset);
		else if(fabs(m_expectedOffset) > .1) 
			rot = units::radians_per_second_t(m_expectedOffset);
		else
			rot = units::radians_per_second_t(0);
	}
	auto states = kDriveKinematics.ToSwerveModuleStates(
		m_fieldCentric ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
			xSpeed, ySpeed, rot, GetHeading())
		: frc::ChassisSpeeds{xSpeed, ySpeed, rot}
	);

	kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

	auto [fl, fr, bl, br] = states;

	double flMax = m_frontLeft.SetDesiredState(fl);
	double frMax = m_frontRight.SetDesiredState(fr);
	double blMax = m_rearLeft.SetDesiredState(bl);
	double brMax = m_rearRight.SetDesiredState(br);

	double driveMax = std::max(std::max(blMax, brMax), std::max(flMax, frMax));

	if (driveMax > DriveConstants::driveMaxVoltage)
		driveMax = DriveConstants::driveMaxVoltage/driveMax;
	else
		driveMax = 1;

	m_frontLeft.SetVoltage(driveMax);
	m_frontRight.SetVoltage(driveMax);
	m_rearLeft.SetVoltage(driveMax);
	m_rearRight.SetVoltage(driveMax);
}

// ==========================================================================

void DriveSubsystem::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates) {
	kDriveKinematics.DesaturateWheelSpeeds(&desiredStates, DriveConstants::kMaxSpeed);

	double flMax = m_frontLeft.SetDesiredState(desiredStates[0]);
	double frMax = m_frontRight.SetDesiredState(desiredStates[1]);
	double blMax = m_rearLeft.SetDesiredState(desiredStates[2]);
	double brMax = m_rearRight.SetDesiredState(desiredStates[3]);

	double driveMax = std::max(std::max(blMax, brMax),std::max(flMax, frMax));

	if (driveMax > DriveConstants::driveMaxVoltage)
		driveMax = DriveConstants::driveMaxVoltage / driveMax;
	else
		driveMax = 1;

	m_frontLeft.SetVoltage(driveMax);
	m_frontRight.SetVoltage(driveMax);
	m_rearLeft.SetVoltage(driveMax);
	m_rearRight.SetVoltage(driveMax);
}

// ==========================================================================

void DriveSubsystem::ResetEncoders() {
	m_frontLeft.ResetEncoders();
	m_rearLeft.ResetEncoders();
	m_frontRight.ResetEncoders();
	m_rearRight.ResetEncoders();
}

// ==========================================================================

units::degree_t DriveSubsystem::GetHeading() const {
	return units::degree_t(m_currentYaw);  // was negated
}

// ==========================================================================

void DriveSubsystem::ZeroHeading() {
//		m_pidgey.SetYaw(0,8);
m_zero = m_pidgey.GetYaw();
}

// ==========================================================================

void DriveSubsystem::SetOffsetHeading(int heading){
//	   m_pidgey.SetYaw(heading, 8);
m_zero = m_pidgey.GetYaw() - heading;
}

// ==========================================================================

double DriveSubsystem::GetTurnRate() {
	return m_pidgey.GetRate();
}

// ==========================================================================

frc::Pose2d DriveSubsystem::GetPose() {
	return m_odometry.GetPose();
}

// ==========================================================================

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {

	m_currentYaw = pose.Rotation().Degrees().value();
	SetOffsetHeading(m_currentYaw);

	m_odometry.ResetPosition(pose, frc::Rotation2d(units::degree_t(GetHeading())));
	m_poseEstimator.ResetPosition(pose, frc::Rotation2d(units::degree_t(GetHeading())));

	m_resetRSx = m_xEntry.GetDouble(0);
	m_resetRSz = m_zEntry.GetDouble(0);
	m_resetRSrw = m_rwEntry.GetDouble(0);
	m_resetRSrx = m_rxEntry.GetDouble(0);
	m_resetRSry = m_ryEntry.GetDouble(0);
	m_resetRSrz = m_rzEntry.GetDouble(0);

}

// ==========================================================================

void DriveSubsystem::MotorsOff() {
	m_frontLeft.motorsOff();
	m_rearLeft.motorsOff();
	m_frontRight.motorsOff();
	m_rearRight.motorsOff();
}

// ==========================================================================

void DriveSubsystem::ToggleFieldCentric() {
	m_fieldCentric = !m_fieldCentric;
}

// ==========================================================================

void DriveSubsystem::GyroCrab(double x, double y, double desiredAngle) {
	double currentAngle = GetHeading().value();
	while ( currentAngle > 180.) currentAngle -= 360.;
	while ( currentAngle < -180.) currentAngle += 360.;

	auto twist = (desiredAngle - currentAngle);

	while (twist > 180.0) {
		twist -= 360.0;
	}
	while (twist < -180.0) {
		twist += 360.0;
	}

	constexpr double GYRO_P = 0.01*6; //original is 0.007
	constexpr double GYRO_MAX = 0.6*6;

	twist = std::clamp(twist*GYRO_P, -GYRO_MAX, GYRO_MAX);

	Drive(units::meters_per_second_t(x), units::meters_per_second_t(y), units::radians_per_second_t(twist));
}

// ==========================================================================

void DriveSubsystem::DriveLime() {
	auto limeLightTX = m_limelightTable->GetNumber("tx", 0.0);
	double rot = 0;
	if (fabs(limeLightTX) >= 1) 
			rot = (limeLightTX/(-30)*1);

	Drive(units::meters_per_second_t(0), units::meters_per_second_t(0), units::radians_per_second_t(rot));
}


// ==========================================================================

double DriveSubsystem::GyroRotate() {
	auto yaw = GetHeading().value();
	float desiredangle = 0;

	if (yaw > 45 && yaw < 135) {
		desiredangle = 90;
	}
	else if ((yaw > 135 && yaw < 179) || (yaw < -135 && yaw > -179)) {
		desiredangle = 180;
	}
	else if (yaw > -135 && yaw < -45) {
		desiredangle = -90;
	}
	else if (yaw < 45 && yaw > -45) {
		desiredangle = 0;
	}

	auto twist = desiredangle - yaw;
	while (twist > 180.0) {
		twist -= 360.0;
	}
	while (twist < -180.0) {
		twist += 360.0;
	}

	return twist;
}

// ================================================================

void DriveSubsystem::SetWheelOffsets() {
	m_frontLeft.SetWheelOffset();
	m_rearLeft.SetWheelOffset();
	m_frontRight.SetWheelOffset();
	m_rearRight.SetWheelOffset();
	fmt::print("ERROR: SetWheelOffsets Complete\n");
}

// ================================================================

void DriveSubsystem::LoadWheelOffsets() {
	m_frontLeft.LoadWheelOffset();
	m_rearLeft.LoadWheelOffset();
	m_frontRight.LoadWheelOffset();
	m_rearRight.LoadWheelOffset();
	fmt::print("ERROR: LoadWheelOffsets Complete\n");
}

// ================================================================

frc::Translation2d DriveSubsystem::_GetPositionFromRealSense() {
	return frc::Translation2d{
		units::meter_t(m_xEntry.GetDouble(0) - m_resetRSx),
		units::meter_t(m_zEntry.GetDouble(0) - m_resetRSz)
	};
}

// ================================================================

units::degree_t DriveSubsystem::_GetYawFromRealSense() {
	// Get quaternion from RealSense.
	auto qw{m_rwEntry.GetDouble(0) - m_resetRSrw};
	auto qx{m_rxEntry.GetDouble(0) - m_resetRSrx};
	auto qy{m_ryEntry.GetDouble(0) - m_resetRSry};
	auto qz{m_rzEntry.GetDouble(0) - m_resetRSrz};

	// Convert quaternion to angle in degrees.
	auto yawRadians{atan2(2.0 * (qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz)};
	auto yawDegrees{yawRadians * 180.0 / wpi::numbers::pi};

	return units::degree_t{yawDegrees};
}

// ================================================================

double DriveSubsystem::GetDist(){
	return m_realDist;
}

double DriveSubsystem::GetOffset(){
	return m_offset;
}