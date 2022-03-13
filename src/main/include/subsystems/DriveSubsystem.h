#pragma once
#include <frc/Encoder.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/interfaces/Gyro.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <ctre/Phoenix.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include "SwerveModule.h"

class DriveSubsystem : public frc2::SubsystemBase {
public:

	DriveSubsystem();

	/**
	 * Will be called periodically whenever the CommandScheduler runs.
	 */
	void Periodic() override;

	// Subsystem methods go here.

	/**
	 * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
	 * and the linear speeds have no effect on the angular speed.
	 *
	 * @param xSpeed        Speed of the robot in the x direction
	 *                      (forward/backwards).
	 * @param ySpeed        Speed of the robot in the y direction (sideways).
	 * @param rot           Angular rate of the robot.
	 * @param fieldRelative Whether the provided x and y speeds are relative to
	 *                      the field.
	 */
	void Drive(
		units::meters_per_second_t xSpeed,
		units::meters_per_second_t ySpeed,
		units::radians_per_second_t rot
	);

	/**
	 * Resets the drive encoders to currently read a position of 0.
	 */
	void ResetEncoders();

	/**
	 * Sets the drive MotorControllers to a power from -1 to 1.
	 */
	void SetModuleStates(wpi::array<frc::SwerveModuleState, 4> desiredStates);

	/**
	 * Returns the heading of the robot.
	 *
	 * @return the robot's heading in degrees, from 180 to 180
	 */
	units::degree_t GetHeading() const;

	/**
	 * Zeroes the heading of the robot.
	 */
	void ZeroHeading();

	/**
	 * Returns the turn rate of the robot.
	 *
	 * @return The turn rate of the robot, in degrees per second
	 */
	double GetTurnRate();

	/**
	 * Returns the currently-estimated pose of the robot.
	 *
	 * @return The pose.
	 */
	frc::Pose2d GetPose();

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	void ResetOdometry(frc::Pose2d pose);

	void ToggleFieldCentric();

	// 2020 robot
	//units::meter_t kTrackWidth = 0.52_m; // Distance between centers of right and left wheels on robot
	//units::meter_t kWheelBase = 0.78_m; // Distance between centers of front and back wheels on robot

	// 2022 robot
	units::meter_t kTrackWidth = 0.432_m; // Distance between centers of right and left wheels on robot
	units::meter_t kWheelBase = 0.686_m; // Distance between centers of front and back wheels on robot

	frc::SwerveDriveKinematics<4> kDriveKinematics{
		frc::Translation2d(kWheelBase / 2, kTrackWidth / 2),
		frc::Translation2d(kWheelBase / 2, -kTrackWidth / 2),
		frc::Translation2d(-kWheelBase / 2, kTrackWidth / 2),
		frc::Translation2d(-kWheelBase / 2, -kTrackWidth / 2)
	};

	void SetWheelOffsets();
	void LoadWheelOffsets();
	void motorsOff();

	void GyroCrab(double x, double y, double desiredAngle);
	double GyroRotate();

	// The gyro sensor
	WPI_Pigeon2 m_pidgey{0, "Default Name"};

private:

	// Components (e.g. motor controllers and sensors) should generally be
	// declared private and exposed only through public methods.

	SwerveModule m_frontLeft;
	SwerveModule m_rearLeft;
	SwerveModule m_frontRight;
	SwerveModule m_rearRight;

	double currentYaw = 0;

	int m_counter = 0;

	// Odometry class for tracking robot pose
	// 4 defines the number of modules
	frc::SwerveDriveOdometry<4> m_odometry;

	frc::SwerveDrivePoseEstimator<4> m_poseEstimator{
      frc::Rotation2d(), frc::Pose2d(), kDriveKinematics,
      {0.1, 0.1, 0.1},   {0.05},        {0.1, 0.1, 0.1}};

	bool m_fieldCentric;
	frc::Field2d m_field;

    std::shared_ptr<nt::NetworkTable> table = nt::NetworkTableInstance::GetDefault().GetTable("/RealSensePose");
    nt::NetworkTableEntry xEntry = table->GetEntry("x");
    nt::NetworkTableEntry ryEntry = table->GetEntry("ry");
    nt::NetworkTableEntry zEntry = table->GetEntry("z");

	double m_resetRSx;
	double m_resetRSz;
	double m_resetRSry;
};
