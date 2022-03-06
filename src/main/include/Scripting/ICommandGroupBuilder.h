// ==========================================================================
// ICommandGroupBuilder interface
//
// FRC 4143: MARS/WARS
// ==========================================================================
// 2022-02-27 JKSalmon - Initial release
// ==========================================================================

#pragma once
#include <memory>
#include <frc2/command/Command.h>
#include <frc2/command/CommandBase.h>
#include <units/time.h>

// ==========================================================================

namespace frc4143 {

class ICommandGroupBuilder {
public:

	virtual void AddSequential(std::unique_ptr<frc2::Command>&& command, units::time::second_t duration = 0_s) = 0;
	virtual void AddParallel(std::unique_ptr<frc2::Command>&& command, units::time::second_t duration = 0_s) = 0;
	virtual void AddParallelDeadline(std::unique_ptr<frc2::Command>&& command, units::time::second_t duration = 0_s) = 0;
	virtual void AddParallelRace(std::unique_ptr<frc2::Command>&& command, units::time::second_t duration = 0_s) = 0;

	virtual std::unique_ptr<frc2::Command> Create() = 0;
};

} // END namespace

// ==========================================================================
