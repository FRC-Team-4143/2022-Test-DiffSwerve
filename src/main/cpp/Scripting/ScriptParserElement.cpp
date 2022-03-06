// ==========================================================================
// ScriptParserElement class
//
// FRC 4143: MARS/WARS
// ==========================================================================
// 2022-02-27 JKSalmon - Initial release
// ==========================================================================

#include "Scripting/ScriptParserElement.h"
#include "Scripting/StringUtils.h"
#include <stdexcept>

using frc4143::ScriptParserElement;

// ==========================================================================

ScriptParserElement::ScriptParserElement(
	std::string_view name,
	std::vector<std::string> aliases,
	CreateCommandFunction createCommand
)
:	_name{name},
	_aliases{aliases},
	_createCommand{createCommand}
{
	if (!name.length()) throw std::invalid_argument("name must not be empty.");
	if (!createCommand) throw std::invalid_argument("createCommand must not be null.");
}

// ==========================================================================

std::string ScriptParserElement::GetName() const {
	return _name;
}

// ==========================================================================

std::vector<std::string> ScriptParserElement::GetAliases() const {
	return _aliases;
}

// ==========================================================================

bool ScriptParserElement::IsMatch(std::string s) const {
	s = StringUtils::ToUpper(s);

	if (StringUtils::ToUpper(GetName()) == s) {
		return true;
	}

	for (const auto &a : GetAliases()) {
		if (StringUtils::ToUpper(a) == s) {
			return true;
		}
	}

	return false;
}

// ==========================================================================

std::unique_ptr<frc2::Command> ScriptParserElement::CreateCommand(std::vector<float> parameters) const {
	return _createCommand(parameters);
}

// ==========================================================================
