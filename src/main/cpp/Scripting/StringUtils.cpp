// ==========================================================================
// StringUtils class
//
// FRC 4143: MARS/WARS
// ==========================================================================
// 2022-02-26 JKSalmon - Added namespace
// 2022-02-27 JKSalmon - Added ToLower and ToUpper
// ==========================================================================

#include "Scripting/StringUtils.h"
#include <algorithm>
#include <cctype>
#include <functional>

using frc4143::StringUtils;

// ==========================================================================

std::string StringUtils::LeftTrim(std::string s) {
	s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
	return s;
}

// ==========================================================================

std::string StringUtils::RightTrim(std::string s) {
	s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(),	s.end());
	return s;
}

// ==========================================================================

std::string StringUtils::Trim(std::string s) {
	return LeftTrim(RightTrim(s));
}

// ==========================================================================

std::string StringUtils::ToLower(std::string s) {
	std::transform(s.begin(), s.end(), s.begin(), ::tolower);
	return s;
}

// ==========================================================================

std::string StringUtils::ToUpper(std::string s) {
	std::transform(s.begin(), s.end(), s.begin(), ::toupper);
	return s;
}

// ==========================================================================
