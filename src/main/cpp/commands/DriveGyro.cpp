#include "commands/DriveGyro.h"

DriveGyro::DriveGyro(DriveSubsystem* subsystem, double x, double y, double angle)
:	m_drive(subsystem), m_x(x), m_y(y), m_angle(angle)
{
	AddRequirements(subsystem);
}

void DriveGyro::Initialize() {

}

void DriveGyro::Execute() {
	m_drive->GyroCrab(m_x, m_y, m_angle);
}

void DriveGyro::End(bool) {
	m_drive->MotorsOff();
}

bool DriveGyro::IsFinished() {

	if (fabs(m_drive->GetHeading().value() - m_angle) < 2 )
		return true;

	return false;
}
