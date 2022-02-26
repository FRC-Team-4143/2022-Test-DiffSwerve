#include "commands/PickUpCycleBounce.h"

PickUpCycleBounce::PickUpCycleBounce(PickUpSubsystem* subsystem,frc::XboxController* controller) : m_pickUp(subsystem), m_controller(controller) {
	AddRequirements(subsystem);
}

void PickUpCycleBounce::Initialize() {
	m_pickUp->PickUpBounce();
	m_pickUp->RollerIn();
	m_pickUp->IndexerLoad();
	counter = 0;
}

void PickUpCycleBounce::Execute() {
	if(m_controller->GetLeftBumper() == false){
		counter++;
	}
	else counter=0;

	if(counter == 0)
		m_pickUp->PickUpBounce();
	else if(counter < 50)
		m_pickUp->PickUpExtend();
    else
		m_pickUp->PickUpRetract();
}

void PickUpCycleBounce::End(bool) {
	m_pickUp->RollerOff();
	m_pickUp->IndexerOff();
}

bool PickUpCycleBounce::IsFinished() {
	if(counter >= 100)
		return true;
	else
		return false;
}