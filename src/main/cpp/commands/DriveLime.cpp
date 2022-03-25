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
	
	if (fabs(m_drive->m_limelightTable->GetNumber("tx",3)) < 2){
		return true;
	}

	return false;
}
