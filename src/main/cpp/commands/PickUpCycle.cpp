#include "commands/PickUpCycle.h"

PickUpCycle::PickUpCycle(PickUpSubsystem* subsystem,frc::XboxController* controller) : m_pickUp(subsystem), m_controller(controller) {
	AddRequirements(subsystem);
}

void PickUpCycle::Initialize() {
	m_pickUp->PickUpExtend();
	m_pickUp->RollerIn();
	m_pickUp->IndexerLoad();
	counter = 0;
}

void PickUpCycle::Execute() {
	if(m_controller->GetRightBumper() == false){
		counter++;
		m_pickUp->PickUpRetract();
	} else {
		m_pickUp->PickUpExtend();
		counter = 0;
	}
}

void PickUpCycle::End(bool) {
	m_pickUp->PickUpRetract();
	m_pickUp->RollerOff();
	m_pickUp->IndexerOff();
}

bool PickUpCycle::IsFinished() {
	if(counter >= 50)
		return true;
	else
		return false;
}