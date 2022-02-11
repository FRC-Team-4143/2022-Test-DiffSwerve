
#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/Phoenix.h>
#include "Constants.h"


class PickUpSubsystem : public frc2::SubsystemBase {
 public:
  PickUpSubsystem();


  
  void Periodic() override;

  void RollerIn();

  void RollerOut();

  void RollerOff();

  
 private:
  
TalonSRX m_roller;



};
