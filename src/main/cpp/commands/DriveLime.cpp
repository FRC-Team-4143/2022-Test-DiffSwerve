#include "commands/DriveLime.h"

DriveLime::DriveLime(DriveSubsystem* subsystem)
:	m_drive(subsystem)
{
	AddRequirements(subsystem);
}

void DriveLime::Initialize() {

}

void DriveLime::Execute() {
	m_drive->DriveLime();
}

void DriveLime::End(bool) {
	m_drive->MotorsOff();
}

bool DriveLime::IsFinished() {
	return false;
}
