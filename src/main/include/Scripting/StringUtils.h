// ==========================================================================
// StringUtils class
//
// FRC 4143: MARS/WARS
// ==========================================================================
// 2022-02-26 JKSalmon - Added namespace
// 2022-02-27 JKSalmon - Added ToLower and ToUpper
// ==========================================================================

#pragma once
#include <string>

// ==========================================================================

namespace frc4143 {

class StringUtils {
public:

	static std::string LeftTrim(std::string s);
	static std::string RightTrim(std::string s);
	static std::string Trim(std::string s);

	static std::string ToLower(std::string s);
	static std::string ToUpper(std::string s);

	template <typename T> static std::string Join(const T &v, const std::string_view &delim) {
		std::string s;
		for (const auto &i : v) {
			if (&i != &v[0]) {
				s += delim;
			}
			s += i;
		}
		return s;
	}
};

} // END namespace

// ==========================================================================
