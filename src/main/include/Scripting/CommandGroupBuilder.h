// ==========================================================================
// CommandGroupBuilder class
//
// FRC 4143: MARS/WARS
// ==========================================================================
// 2022-02-27 JKSalmon - Initial release
// ==========================================================================

#pragma once
#include "Scripting/ICommandGroupBuilder.h"
#include <frc2/command/ParallelRaceGroup.h>

// ==========================================================================

namespace frc4143 {

class CommandGroupBuilder : public frc4143::ICommandGroupBuilder {
public:

	CommandGroupBuilder();
	virtual ~CommandGroupBuilder();

	// Interface methods
	virtual void AddSequential(std::unique_ptr<frc2::Command>&& command, units::time::second_t duration = 0_s) override;
	virtual void AddParallel(std::unique_ptr<frc2::Command>&& command, units::time::second_t duration = 0_s) override;
	virtual void AddParallelDeadline(std::unique_ptr<frc2::Command>&& command, units::time::second_t duration = 0_s) override;
	virtual void AddParallelRace(std::unique_ptr<frc2::Command>&& command, units::time::second_t duration = 0_s) override;
	virtual std::unique_ptr<frc2::Command> Create() override;

private:

	std::unique_ptr<frc2::ParallelRaceGroup> _ApplyTimeout(std::unique_ptr<frc2::Command>&& command, units::time::second_t duration);
	void _FlushParallel();
	void _FlushParallelDeadline();
	void _FlushParallelRace();

	std::vector<std::unique_ptr<frc2::Command>> _sequentialCommands;
	std::vector<std::unique_ptr<frc2::Command>> _parallelCommands;
	std::unique_ptr<frc2::Command> _parallelDeadlineCommand;
	std::vector<std::unique_ptr<frc2::Command>> _parallelDeadlineCommands;
	std::vector<std::unique_ptr<frc2::Command>> _parallelRaceCommands;
};

} // END namespace

// ==========================================================================
