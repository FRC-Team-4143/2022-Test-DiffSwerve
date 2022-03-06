// ==========================================================================
// ScriptParserElement class
//
// FRC 4143: MARS/WARS
// ==========================================================================
// 2022-02-27 JKSalmon - Initial release
// ==========================================================================

#pragma once
#include <frc2/command/Command.h>
#include <functional>
#include <memory>
#include <string>
#include <units/time.h>
#include <vector>

// ==========================================================================

namespace frc4143 {

class ScriptParserElement {
public:

	using CreateCommandFunction = std::function<
		std::unique_ptr<frc2::Command>(
			std::vector<float> parameters
		)
	>;

	ScriptParserElement(
		std::string_view name,
		std::vector<std::string> aliases,
		CreateCommandFunction createCommand
	);

	~ScriptParserElement() = default;

	ScriptParserElement(const ScriptParserElement& other) = default;
	ScriptParserElement& operator=(const ScriptParserElement& other) = default;

	std::string GetName() const;
	std::vector<std::string> GetAliases() const;
	bool IsMatch(std::string s) const;

	std::unique_ptr<frc2::Command> CreateCommand(std::vector<float> parameters) const;

private:

	std::string _name;
	std::vector<std::string> _aliases;
	CreateCommandFunction _createCommand;
};

} // END namespace

// ==========================================================================
