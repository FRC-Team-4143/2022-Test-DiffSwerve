// ==========================================================================
// ScriptEngine class
//
// FRC 4143: MARS/WARS
// ==========================================================================
// 2022-02-26 JKSalmon - Initial release
// ==========================================================================

#include "Scripting/ScriptEngine.h"
#include "Scripting/ScriptParser.h"
#include "Scripting/CommandGroupBuilder.h"
#include <frc2/command/WaitCommand.h>
#include <stdexcept>

using frc4143::ScriptEngine;

// ==========================================================================

std::function<std::unique_ptr<frc4143::ICommandGroupBuilder>()> ScriptEngine::_builderFactory{};
std::shared_ptr<frc4143::IScriptParser> ScriptEngine::_parser{};

// ==========================================================================

void ScriptEngine::SetBuilderFactory(std::function<std::unique_ptr<frc4143::ICommandGroupBuilder>()> builderFactory) {
	if (!builderFactory) throw std::invalid_argument("builderFactory cannot be null.");
	_builderFactory = builderFactory;
}

// ==========================================================================

void ScriptEngine::SetParser(std::shared_ptr<frc4143::IScriptParser> parser) {
	if (!parser) throw std::invalid_argument("parser cannot be null.");
	_parser = parser;
}

// ==========================================================================

bool ScriptEngine::ScriptIsValid(std::string script) {
	return _GetParser()->IsValid(script);
}

// ==========================================================================

std::unique_ptr<frc2::Command> ScriptEngine::CreateCommand(std::string script) {
	auto builder{_GetBuilder()};
	_GetParser()->Parse(script, *builder);
	return builder->Create();
}

// ==========================================================================

std::unique_ptr<frc4143::ICommandGroupBuilder> ScriptEngine::_GetBuilder() {
	return _GetBuilderFactory()();
}

// ==========================================================================

std::function<std::unique_ptr<frc4143::ICommandGroupBuilder>()> ScriptEngine::_GetBuilderFactory() {
	return _builderFactory ? _builderFactory : (_builderFactory = _CreateDefaultBuilderFactory());
}

// ==========================================================================

std::shared_ptr<frc4143::IScriptParser> ScriptEngine::_GetParser() {
	return _parser ? _parser : (_parser = _CreateDefaultParser());
}

// ==========================================================================

std::function<std::unique_ptr<frc4143::ICommandGroupBuilder>()> ScriptEngine::_CreateDefaultBuilderFactory() {
	return [](){ return std::make_unique<frc4143::CommandGroupBuilder>(); };
}

// ==========================================================================

std::shared_ptr<frc4143::IScriptParser> ScriptEngine::_CreateDefaultParser() {
	auto parser{frc4143::ScriptParser::Create()};

	parser->Add(
		frc4143::ScriptParserElement{
			"Sleep", {"S", "s"},
			[](std::vector<float> parameters) {
				parameters.resize(1);
				units::time::second_t duration{parameters[0]};
				return std::make_unique<frc2::WaitCommand>(duration);
			}
		}
	);

	return parser;
}

// ==========================================================================
