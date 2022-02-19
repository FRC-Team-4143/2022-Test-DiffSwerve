#include "commands/PickUpCycle.h"

PickUpCycle::PickUpCycle(PickUpSubsystem* subsystem,frc::XboxController* controller) : m_pickUp(subsystem), m_controller(controller) {
	AddRequirements(subsystem);
}

void PickUpCycle::Initialize() {
	m_pickUp->PickUpExtend();
	m_pickUp->RollerIn();
	counter = 0;
}

void PickUpCycle::Execute() {
	if(m_controller->GetRightBumper() == false){
		counter++;
		m_pickUp->PickUpRetract();
	}
}

void PickUpCycle::End(bool) {
	m_pickUp->PickUpRetract();
	m_pickUp->RollerOff();
}

bool PickUpCycle::IsFinished() {
	if(counter >= 50)
		return true;
	else
		return false;
}