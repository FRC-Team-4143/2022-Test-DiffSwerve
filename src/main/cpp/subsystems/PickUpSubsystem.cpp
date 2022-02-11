
#include "subsystems/PickUpSubsystem.h"
#include "Constants.h"

PickUpSubsystem::PickUpSubsystem()
: m_roller {PickUpConstants::kRollerPort}{}


  void PickUpSubsystem::Periodic() {

  }

  void PickUpSubsystem::RollerIn(){
      m_roller.Set(TalonSRXControlMode::PercentOutput, 0.75);
  }

  void PickUpSubsystem::RollerOut(){
      m_roller.Set(TalonSRXControlMode::PercentOutput, -0.75);
  }
  
  void PickUpSubsystem::RollerOff(){
      m_roller.Set(TalonSRXControlMode::PercentOutput, 0);
  }