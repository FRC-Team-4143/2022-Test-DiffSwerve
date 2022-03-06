// ==========================================================================
// ScriptEngine class
//
// FRC 4143: MARS/WARS
// ==========================================================================
// 2022-02-26 JKSalmon - Initial release
// ==========================================================================

#pragma once
#include <frc2/command/Command.h>
#include <functional>
#include <memory>
#include <string>
#include "Scripting/ICommandGroupBuilder.h"
#include "Scripting/IScriptParser.h"

// ==========================================================================

namespace frc4143 {

class ScriptEngine {
public:

	// All functionality is static, so we don't need a ctor.
	ScriptEngine() = delete;

	static void SetBuilderFactory(std::function<std::unique_ptr<frc4143::ICommandGroupBuilder>()> builderFactory);
	static void SetParser(std::shared_ptr<frc4143::IScriptParser> parser);

	static bool ScriptIsValid(std::string script);
	static std::unique_ptr<frc2::Command> CreateCommand(std::string script);

private:

	static std::unique_ptr<frc4143::ICommandGroupBuilder> _GetBuilder();
	static std::function<std::unique_ptr<frc4143::ICommandGroupBuilder>()> _GetBuilderFactory();
	static std::shared_ptr<frc4143::IScriptParser> _GetParser();

	static std::function<std::unique_ptr<frc4143::ICommandGroupBuilder>()> _CreateDefaultBuilderFactory();
	static std::shared_ptr<frc4143::IScriptParser> _CreateDefaultParser();

	static std::function<std::unique_ptr<frc4143::ICommandGroupBuilder>()> _builderFactory;
	static std::shared_ptr<frc4143::IScriptParser> _parser;
};

} // END namespace

// ==========================================================================
