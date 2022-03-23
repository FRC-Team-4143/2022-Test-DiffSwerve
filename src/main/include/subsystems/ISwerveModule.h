#if 0

#pragma once

class ISwerveModule
{
private:
    
public:
    ISwerveModule(/* args */);
    ~ISwerveModule();

	virtual double SetDesiredState(const frc::SwerveModuleState& state);

	virtual void ResetEncoders();

	virtual void SetWheelOffset();
	virtual void LoadWheelOffset();
	virtual void motorsOff();
	virtual double GetDriveMotorSpeed();

	virtual void SetVoltage(double driveMax);
};

ISwerveModule::ISwerveModule(/* args */)
{
}

ISwerveModule::~ISwerveModule()
{
}

#endif