#pragma once
#include <iostream>
#include <string>

// Simple ANSI color codes for terminal output
#define ANSI_GREEN "\033[32m"
#define ANSI_RED   "\033[31m"
#define ANSI_RESET "\033[0m"

static int g_tests_passed = 0;
static int g_tests_failed = 0;

#define TEST_ASSERT(cond) \
    do { \
        if (!(cond)) { \
            std::cout << ANSI_RED << "[FAIL] " << ANSI_RESET << #cond << " at " << __FILE__ << ":" << __LINE__ << std::endl; \
            g_tests_failed++; \
        } else { \
            g_tests_passed++; \
        } \
    } while(0)

#define TEST_CASE(name) \
    void name(); \
    static int dummy_##name = (std::cout << "Running " << #name << "..." << std::endl, name(), 0); \
    void name()

#define TEST_EQUAL(a, b) \
    do { \
        if ((a) != (b)) { \
            std::cout << ANSI_RED << "[FAIL] " << ANSI_RESET << #a << " (" << (a) << ") != " << #b << " (" << (b) << ") at " << __FILE__ << ":" << __LINE__ << std::endl; \
            g_tests_failed++; \
        } else { \
            g_tests_passed++; \
        } \
    } while(0)

inline void print_test_summary() {
    std::cout << "--------------------------------------------------" << std::endl;
    std::cout << "Tests Passed: " << g_tests_passed << std::endl;
    std::cout << "Tests Failed: " << g_tests_failed << std::endl;
    if (g_tests_failed == 0) {
        std::cout << ANSI_GREEN << "ALL TESTS PASSED" << ANSI_RESET << std::endl;
    } else {
        std::cout << ANSI_RED << "SOME TESTS FAILED" << ANSI_RESET << std::endl;
    }
}
