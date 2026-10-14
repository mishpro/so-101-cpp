#pragma once

// Минимальный тест-раннер без внешних зависимостей: в проекте нет сети для
// FetchContent, а тащить Catch2 ради шести проверок незачем.

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

namespace so101_test {

inline int g_failures = 0;
inline int g_checks = 0;

inline void report(bool ok, const char* expression, const char* file, int line,
                   const std::string& detail) {
	++g_checks;
	if (ok) return;
	++g_failures;
	std::cerr << "FAIL " << file << ':' << line << ": " << expression;
	if (!detail.empty()) std::cerr << "  [" << detail << ']';
	std::cerr << '\n';
}

inline bool nearly_equal(double left, double right, double tolerance) {
	return std::abs(left - right) <= tolerance;
}

inline int finish(const char* suite) {
	std::cout << suite << ": " << (g_checks - g_failures) << '/' << g_checks
			  << " проверок пройдено\n";
	return g_failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}

}  // namespace so101_test

#define CHECK(expression)                                             \
	::so101_test::report(static_cast<bool>(expression), #expression,    \
	                     __FILE__, __LINE__, {})

#define CHECK_MSG(expression, detail)                                 \
	::so101_test::report(static_cast<bool>(expression), #expression,    \
	                     __FILE__, __LINE__, (detail))

#define CHECK_NEAR(actual, expected, tolerance)                                   \
	::so101_test::report(                                                          \
	    ::so101_test::nearly_equal((actual), (expected), (tolerance)),            \
	    #actual " ~= " #expected, __FILE__, __LINE__,                              \
	    std::to_string((actual)) + " vs " + std::to_string((expected)))
