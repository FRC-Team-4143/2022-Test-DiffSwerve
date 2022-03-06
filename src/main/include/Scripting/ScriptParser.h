// ==========================================================================
// ScriptParser class
//
// FRC 4143: MARS/WARS
// ==========================================================================
// 2022-02-27 JKSalmon - Initial release
// ==========================================================================

#pragma once
#include "Scripting/IScriptParser.h"
#include "Scripting/ScriptParserElement.h"
#include <functional>
#include <memory>
#include <regex>
#include <string>
#include <units/time.h>
#include <vector>

// ==========================================================================

namespace frc4143 {

class ScriptParser : public frc4143::IScriptParser {
public:

	static std::shared_ptr<ScriptParser> Create();

	ScriptParser();
	virtual ~ScriptParser() = default;

	// Interface methods
	virtual bool IsValid(std::string script) override;
	virtual void Parse(std::string script, frc4143::ICommandGroupBuilder& builder) override;

	bool Add(const frc4143::ScriptParserElement& pe);

private:

	std::vector<float> _ParseParameterList(std::string s);
	std::shared_ptr<std::regex> _GetCommandListRx();
	std::shared_ptr<std::regex> _GetParameterListRx();
	std::string _GetCommandName(std::string alias) const;
	frc4143::ScriptParserElement _GetParserElement(std::string alias) const;
	std::string _BuildDurationRx(bool capturing) const;
	std::string _BuildModeRx(bool capturing) const;
	std::string _BuildNameRx(bool capturing) const;
	std::string _BuildParametersRx(bool capturing) const;
	bool _IsDuplicateName(std::string name) const;
	bool _IsValidName(std::string name) const;

	bool _debug;
	std::vector<frc4143::ScriptParserElement> _parseElements;
	std::shared_ptr<std::regex> _rxCommandList;
	std::shared_ptr<std::regex> _rxParameterList;
	std::regex _rxValidName;
};

} // END namespace

// ==========================================================================
