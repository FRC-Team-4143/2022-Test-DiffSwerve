// ==========================================================================
// ScriptParser class
//
// FRC 4143: MARS/WARS
// ==========================================================================
// 2022-02-27 JKSalmon - Initial release
// ==========================================================================

#include "Scripting/ScriptParser.h"
#include "Scripting/StringUtils.h"
#include <iostream>

using frc4143::ScriptParser;

// ==========================================================================

std::shared_ptr<ScriptParser> ScriptParser::Create() {
	return std::make_shared<ScriptParser>();
}

// ==========================================================================

ScriptParser::ScriptParser()
:	_debug{false},
	_parseElements{},
	_rxCommandList{},
	_rxParameterList{},
	_rxValidName{"^[A-Z][0-9A-Z]*$", std::regex_constants::icase}
{
	// Initialize the parameter list RegEx. The RegEx is not needed here,
	// but this caches it for better performance in the Parse method.
	_GetParameterListRx();
}

// ==========================================================================

bool ScriptParser::IsValid(std::string script) {
	auto rx{_GetCommandListRx()};
	return std::regex_match(std::string{script}, *rx);
}

// ==========================================================================

void ScriptParser::Parse(std::string script, frc4143::ICommandGroupBuilder& builder) {
	script = StringUtils::ToUpper(script);

	auto rx{_GetCommandListRx()};
	std::smatch m;

	while (std::regex_match(script.cbegin(), script.cend(), m, *rx)) {
		auto mode{m.str(1)};
		auto alias{m.str(2)};
		auto parameters{m.str(3)};
		units::time::second_t duration{(m.str(4).length() > 0) ? std::stof(m.str(4)) : 0.0f};
		auto remainder{m.str(5)};

		auto values{_ParseParameterList(parameters)};
		auto pe{_GetParserElement(alias)};

		if ("P" == mode || "PARALLEL" == mode) {
			builder.AddParallel(pe.CreateCommand(values), duration);
		}
		else if ("PD" == mode || "PARALLELDEADLINE" == mode) {
			builder.AddParallelDeadline(pe.CreateCommand(values), duration);
		}
		else if ("PR" == mode || "PARALLELRACE" == mode) {
			builder.AddParallelRace(pe.CreateCommand(values), duration);
		}
		else {
			builder.AddSequential(pe.CreateCommand(values), duration);
		}

		script = remainder;
	}
}

// ==========================================================================

bool ScriptParser::Add(const frc4143::ScriptParserElement& pe) {
	if (!_IsValidName(pe.GetName()) || _IsDuplicateName(pe.GetName())) {
		throw std::invalid_argument("Command name must be unique.");
	}

	for (const auto &a : pe.GetAliases()) {
		if (!_IsValidName(a) || _IsDuplicateName(a)) {
			throw std::invalid_argument("Command aliases must be unique.");
		}
	}

	_parseElements.push_back(pe);
	_rxCommandList.reset();
	return true;
}

// ==========================================================================

std::vector<float> ScriptParser::_ParseParameterList(std::string s) {
	std::vector<float> result;
	auto rx{_GetParameterListRx()};
	std::smatch m;

	while (std::regex_match(s.cbegin(), s.cend(), m, *rx)) {
		auto parameter{m.str(1)};
		auto remainder{m.str(2)};

		result.push_back(std::stof(parameter));
		s = remainder;
	}

	return result;
}

// ==========================================================================

std::shared_ptr<std::regex> ScriptParser::_GetCommandListRx() {
	if (!_rxCommandList) {
		std::string s;

		s += "^";
		s += _BuildModeRx(true);		// Parallel vs. Sequential (optional, group 1)
		s += _BuildNameRx(true);		// Command name (required, group 2)
		s += _BuildParametersRx(true);	// Parameters (optional, group 3)
		s += _BuildDurationRx(true);	// Duration (optional, group 4)
		s += "(";						// Remainder (optional, group 5)
		s += "(?:";						// open group
		s += R"(\s)";					// required white space
		s += _BuildModeRx(false);		// Parallel vs. Sequential (optional)
		s += _BuildNameRx(false);		// Command name (required)
		s += _BuildParametersRx(false);	// Parameters (optional)
		s += _BuildDurationRx(false);	// Duration (optional, group 4)
		s += ")*";						// close group, zero or more instances
		s += ")";						// close capture group
		s += R"(\s*)";					// optional white space
		s += "$";

		_rxCommandList = std::make_shared<std::regex>(s, std::regex_constants::icase);
	}

	return _rxCommandList;
}

// ==========================================================================

std::shared_ptr<std::regex> ScriptParser::_GetParameterListRx() {
	if (!_rxParameterList) {
		std::string s;

		s += R"(\s*)"; // optional white space
		s += "(";      // open capture group
		s += "[-+]?";  // optional sign
		s += "(?:";
		s += R"(\d+(?:\.\d*)?)"; // accept formats: ##, ##., and ##.##
		s += "|";
		s += R"(\.\d+)"; // accept formats: .##
		s += ")";
		s += ")";      // close capture group
		s += R"(\s*)"; // optional white space

		s += "(?:";     // open group
		s += R"(,\s*)"; // comma, optional white space

		s += "("; // open capture group

		s += "[-+]?"; // optional sign
		s += "(?:";
		s += R"(\d+(?:\.\d*)?)"; // accept formats: ##, ##., and ##.##
		s += "|";
		s += R"(\.\d+)"; // accept formats: .##
		s += ")";
		s += R"(\s*)"; // optional white space

		s += "(?:";     // open group
		s += R"(,\s*)"; // comma, optional white space

		s += "[-+]?"; // optional sign
		s += "(?:";
		s += R"(\d+(?:\.\d*)?)"; // accept formats: ##, ##., and ##.##
		s += "|";
		s += R"(\.\d+)"; // accept formats: .##
		s += ")";
		s += R"(\s*)"; // optional white space

		s += ")*"; // close group, zero or more instances
		s += ")";  // close capture group
		s += ")?"; // close group, zero or one instance

		_rxParameterList = std::make_shared<std::regex>(s);
	}

	return _rxParameterList;
}

// ==========================================================================

std::string ScriptParser::_GetCommandName(std::string alias) const {
	for (const auto & pe : _parseElements) {
		if (pe.IsMatch(alias)) {
			return pe.GetName();
		}
	}

	throw std::runtime_error("Unknown command alias.");
}

// ==========================================================================

frc4143::ScriptParserElement ScriptParser::_GetParserElement(std::string alias) const {
	for (const auto & pe : _parseElements) {
		if (pe.IsMatch(alias)) {
			return pe;
		}
	}

	throw std::runtime_error("Unknown command alias.");
}

// ==========================================================================

std::string ScriptParser::_BuildDurationRx(bool capturing) const {
	std::string s;

	s += "(?:";					// open group
	s += R"(\s*\[\s*)";			// optional white space, open bracket, optional white space
	s += capturing ? "(" : "";

	s += R"(\+?)";					// optional sign
	s += "(?:";
	s += R"(\d+(?:\.\d*)?)";	// accept formats: ##, ##., and ##.##
	s += "|";
	s += R"(\.\d+)";			// accept formats: .##
	s += ")";

	s += capturing ? ")" : "";
	s += R"(\s*\])";			// optional white space, close bracket
	s += ")?";					// close group, zero or one instance

	return s;
}

// ==========================================================================

std::string ScriptParser::_BuildModeRx(bool capturing) const {
	std::string s;

	s += "(?:";						// open group
	s += R"(\s*)";					// optional white space
	s += capturing ? "(" : "(?:";

	s += "P|Parallel|PD|ParallelDeadline|PR|ParallelRace|S|Sequential";

	s += ")";
	s += R"(\s*:)";					// optional white space, colon
	s += ")?";						// close group, zero or one instance

	return s;
}

// ==========================================================================

std::string ScriptParser::_BuildNameRx(bool capturing) const {
	std::vector<std::string> names;

	for (auto pe : _parseElements) {
		names.push_back(pe.GetName());
		for (auto s : pe.GetAliases()) {
			names.push_back(s);
		}
	}

	std::string s;

	s += R"(\s*)";					// optional white space
	s += capturing ? "(" : "(?:";	// open group
	s += frc4143::StringUtils::Join(names, "|");			// pipe-delimited names
	s += ")";						// close group

	return s;
}

// ==========================================================================

std::string ScriptParser::_BuildParametersRx(bool capturing) const {
	std::string s;

	s += "(?:"; // open group A
	s += R"(\s*\(\s*)"; // optional white space, open paren, optional white space
	s += capturing ? "(" : "";

	s += "(?:"; // open group B

	s += "[-+]?"; // optional sign
	s += "(?:";
	s += R"(\d+(?:\.\d*)?)"; // accept formats: ##, ##., and ##.##
	s += "|";
	s += R"(\.\d+)"; // accept formats: .##
	s += ")";
	s += R"(\s*)"; // optional white space

	s += "(?:";     // open group C
	s += R"(,\s*)"; // comma, optional white space

	s += "[-+]?"; // optional sign
	s += "(?:";
	s += R"(\d+(?:\.\d*)?)"; // accept formats: ##, ##., and ##.##
	s += "|";
	s += R"(\.\d+)"; // accept formats: .##
	s += ")";
	s += R"(\s*)"; // optional white space

	s += ")*"; // close group C, zero or more instances

	s += ")?"; // close group B, zero or one instances

	s += capturing ? ")" : "";

	s += R"(\))"; // close paren

	s += ")?"; // close group A, zero or one instances

	return s;
}

// ==========================================================================

bool ScriptParser::_IsDuplicateName(std::string name) const {
	auto n{StringUtils::ToUpper(name)};

	for (const auto &c : _parseElements) {
		if (StringUtils::ToUpper(c.GetName()) == n) {
			return true;
		}

		for (const auto &a : c.GetAliases()) {
			if (StringUtils::ToUpper(a) == n) {
				return true;
			}
		} // END for each alias
	} // END for each supported command

	return false;
}

// ==========================================================================

bool ScriptParser::_IsValidName(std::string name) const {
	return std::regex_match(name, _rxValidName);
}

// ==========================================================================
