#include "commands/PickUpCycle.h"

PickUpCycle::PickUpCycle(PickUpSubsystem* subsystem,frc::XboxController* controller) : m_pickUp(subsystem), m_controller(controller) {
	AddRequirements(subsystem);
}

void PickUpCycle::Initialize() {
	m_pickUp->RollerIn();
	m_pickUp->PickUpExtend();
	//m_pickUp->PickUpBounce(); // this line does the scoop pickup motion
	m_pickUp->IndexerLoad();
	counter = 0;
	counter2 = 0;
}

void PickUpCycle::Execute() {
	if(m_controller->GetRightBumper() == false){
		counter++;
		m_pickUp->PickUpRetract();
	} else {
		counter = 0;
		if (counter2 > 8) m_pickUp->PickUpExtend();
	}
	counter2++;
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