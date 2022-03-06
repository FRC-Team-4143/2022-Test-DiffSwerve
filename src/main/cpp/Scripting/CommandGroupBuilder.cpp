// ==========================================================================
// CommandGroupBuilder class
//
// FRC 4143: MARS/WARS
// ==========================================================================
// 2022-02-27 JKSalmon - Initial release
// ==========================================================================

#include "Scripting/CommandGroupBuilder.h"
#include <utility>
#include "frc2/command/ParallelCommandGroup.h"
#include "frc2/command/ParallelDeadlineGroup.h"
#include "frc2/command/ParallelRaceGroup.h"
#include "frc2/command/SequentialCommandGroup.h"
#include "frc2/command/WaitCommand.h"

using frc4143::CommandGroupBuilder;

// ==========================================================================

CommandGroupBuilder::CommandGroupBuilder()
:	_sequentialCommands{},
	_parallelCommands{},
	_parallelDeadlineCommand{},
	_parallelDeadlineCommands{},
	_parallelRaceCommands{}
{
}

// ==========================================================================

CommandGroupBuilder::~CommandGroupBuilder() {
}

// ==========================================================================

void CommandGroupBuilder::AddSequential(
	std::unique_ptr<frc2::Command>&& command,
	units::time::second_t duration
)
{
	_FlushParallel();
	_FlushParallelDeadline();
	_FlushParallelRace();

	if (duration > 0_s) {
		command = _ApplyTimeout(std::move(command), duration);
	}

	_sequentialCommands.emplace_back(std::move(command));
}

// ==========================================================================

void CommandGroupBuilder::AddParallel(
	std::unique_ptr<frc2::Command>&& command,
	units::time::second_t duration
)
{
	_FlushParallelDeadline();
	_FlushParallelRace();

	if (duration > 0_s) {
		command = _ApplyTimeout(std::move(command), duration);
	}

	_parallelCommands.emplace_back(std::move(command));
}

// ==========================================================================

void CommandGroupBuilder::AddParallelDeadline(
	std::unique_ptr<frc2::Command>&& command,
	units::time::second_t duration
)
{
	_FlushParallel();
	_FlushParallelRace();

	if (duration > 0_s) {
		command = _ApplyTimeout(std::move(command), duration);
	}

	if (_parallelDeadlineCommand) {
		_parallelDeadlineCommands.emplace_back(std::move(command));
		}
	else {
		_parallelDeadlineCommand = std::move(command);
	}
}

// ==========================================================================

void CommandGroupBuilder::AddParallelRace(
	std::unique_ptr<frc2::Command>&& command,
	units::time::second_t duration
)
{
	_FlushParallel();
	_FlushParallelDeadline();

	if (duration > 0_s) {
		command = _ApplyTimeout(std::move(command), duration);
	}

	_parallelRaceCommands.emplace_back(std::move(command));
}

// ==========================================================================

std::unique_ptr<frc2::Command> CommandGroupBuilder::Create() {
	_FlushParallel();
	_FlushParallelDeadline();
	_FlushParallelRace();

	auto numCommands{_sequentialCommands.size()};

	if (0 == numCommands) {
		return nullptr;
	}

	if (1 == numCommands) {
		return std::move(_sequentialCommands[0]);
	}

	return std::make_unique<frc2::SequentialCommandGroup>(std::move(_sequentialCommands));
}

// ==========================================================================

std::unique_ptr<frc2::ParallelRaceGroup> CommandGroupBuilder::_ApplyTimeout(
	std::unique_ptr<frc2::Command>&& command,
	units::time::second_t duration
)
{
	std::vector<std::unique_ptr<frc2::Command>> temp;
	temp.emplace_back(std::make_unique<frc2::WaitCommand>(duration));
	temp.emplace_back(std::move(command));
	return std::make_unique<frc2::ParallelRaceGroup>(std::move(temp));
}

// ==========================================================================

void CommandGroupBuilder::_FlushParallel() {
	if (!_parallelCommands.empty()) {
		_sequentialCommands.emplace_back(std::make_unique<frc2::ParallelCommandGroup>(std::move(_parallelCommands)));
		_parallelCommands.clear();
	}
}

// ==========================================================================

void CommandGroupBuilder::_FlushParallelDeadline() {
	if (_parallelDeadlineCommand) {
		_sequentialCommands.emplace_back(std::make_unique<frc2::ParallelDeadlineGroup>(std::move(_parallelDeadlineCommand), std::move(_parallelDeadlineCommands)));
		_parallelDeadlineCommand.reset();
		_parallelDeadlineCommands.clear();
	}
}

// ==========================================================================

void CommandGroupBuilder::_FlushParallelRace() {
	if (!_parallelRaceCommands.empty()) {
		_sequentialCommands.emplace_back(std::make_unique<frc2::ParallelRaceGroup>(std::move(_parallelRaceCommands)));
		_parallelRaceCommands.clear();
	}
}

// ==========================================================================
