// ==========================================================================
// IScriptParser interface
//
// FRC 4143: MARS/WARS
// ==========================================================================
// 2022-02-26 JKSalmon - Initial release
// ==========================================================================

#pragma once
#include <string>
#include "Scripting/ICommandGroupBuilder.h"

// ==========================================================================

namespace frc4143 {

class IScriptParser {
public:

	virtual bool IsValid(std::string script) = 0;
	virtual void Parse(std::string script, frc4143::ICommandGroupBuilder& builder) = 0;
};

} // END namespace

// ==========================================================================
