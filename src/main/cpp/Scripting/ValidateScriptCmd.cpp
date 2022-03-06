// ==========================================================================
// ValidateScriptCmd class
//
// FRC 4143: MARS/WARS
// ==========================================================================
// 2022-02-26 JKSalmon - Initial release
// ==========================================================================

#include "Scripting/ValidateScriptCmd.h"
#include "Scripting/ScriptEngine.h"
#include <frc/shuffleboard/Shuffleboard.h>

using frc4143::ValidateScriptCmd;

// ==========================================================================

ValidateScriptCmd::ValidateScriptCmd()
:	ValidateScriptCmd("Autonomous", "Script", "Valid") {
}

// ==========================================================================

ValidateScriptCmd::ValidateScriptCmd(std::string_view tabName, std::string_view scriptName, std::string_view resultName)
:	frc2::CommandHelper<frc2::CommandBase, ValidateScriptCmd>{},
	_script{
		frc::Shuffleboard::GetTab(tabName)
		.AddPersistent(scriptName, std::string{})
		.WithWidget(frc::BuiltInWidgets::kTextView)
		.GetEntry()
	},
	_result{
		frc::Shuffleboard::GetTab(tabName)
		.Add(resultName, false)
		.GetEntry()
	}
{
	SetName("Validate Script");
}

// ==========================================================================

void ValidateScriptCmd::Initialize() {
	_ValidateScript();
}

// ==========================================================================

bool ValidateScriptCmd::IsFinished() {
	return true;
}

// ==========================================================================

bool ValidateScriptCmd::RunsWhenDisabled() const {
	return true;
}

// ==========================================================================

void ValidateScriptCmd::_ValidateScript() {
	// Get the script.
	auto script{_script.GetString("")};

	// Is it valid?
	auto valid{frc4143::ScriptEngine::ScriptIsValid(script)};

	// Update the dashboard.
	_result.SetBoolean(valid);
}

// ==========================================================================
