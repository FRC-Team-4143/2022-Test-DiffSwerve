// ==========================================================================
// Logger class
//
// FRC 4143: MARS/WARS
// ==========================================================================
// 2015-02-01 JKSalmon - Initial development
// 2017-01-28 JKSalmon - Minor code and performance improvements
// 2022-02-26 JKSalmon - Added namespace
// ==========================================================================

#pragma once
#include <mutex>
#include <string>

// ==========================================================================

namespace frc4143 {

class Logger {
public:

	Logger();

	static void Log(std::string msg);

private:

	static std::mutex m_mutex;
	static int m_counter;
};

} // END namespace

// ==========================================================================

#ifndef LOG
#define LOG(msg) frc4143::Logger::Log(msg);
#endif

// ==========================================================================
