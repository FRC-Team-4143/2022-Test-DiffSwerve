// ==========================================================================
// ValidateScriptCmd class
//
// FRC 4143: MARS/WARS
// ==========================================================================
// 2022-02-26 JKSalmon - Initial release
// ==========================================================================

#pragma once
#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <networktables/NetworkTableEntry.h>
#include <string>

// ==========================================================================

namespace frc4143 {

class ValidateScriptCmd : public frc2::CommandHelper<frc2::CommandBase, ValidateScriptCmd> {
public:

	ValidateScriptCmd();
	ValidateScriptCmd(std::string_view tabName, std::string_view scriptName, std::string_view resultName);

	// Command methods
	virtual void Initialize() override;
	virtual bool IsFinished() override;
	virtual bool RunsWhenDisabled() const override;

private:

	void _ValidateScript();

	nt::NetworkTableEntry _script;
	nt::NetworkTableEntry _result;
};

} // END namespace

// ==========================================================================
