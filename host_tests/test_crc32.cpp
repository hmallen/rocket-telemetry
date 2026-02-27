#include <iostream>
#include <vector>
#include <string>
#include <cassert>
#include <iomanip>
#include "../src/crc32.h"

// Helper to print hex values
void print_hex(const std::string& label, uint32_t value) {
    std::cout << label << ": 0x" << std::hex << std::uppercase << std::setw(8) << std::setfill('0') << value << std::dec << std::endl;
}

// Test assertion helper
#define ASSERT_EQ_HEX(actual, expected, msg)     if ((actual) != (expected)) {         std::cerr << "[FAIL] " << msg << ": Expected 0x" << std::hex << (expected) << ", got 0x" << (actual) << std::dec << std::endl;         failures++;     } else {         std::cout << "[PASS] " << msg << std::endl;     }

int main() {
    std::cout << "Running CRC32 Tests..." << std::endl;
    int failures = 0;

    // Test Case 1: Standard "123456789"
    // Expected CRC32: 0xCBF43926
    {
        const uint8_t data[] = "123456789";
        uint32_t result = crc32_compute(data, 9);
        ASSERT_EQ_HEX(result, 0xCBF43926, "Standard check value '123456789'");
    }

    // Test Case 2: Empty string
    // Expected CRC32: 0x00000000
    {
        const uint8_t data[] = "";
        uint32_t result = crc32_compute(data, 0);
        ASSERT_EQ_HEX(result, 0x00000000, "Empty string");
    }

    // Test Case 3: "The quick brown fox jumps over the lazy dog"
    // Expected CRC32: 0x414FA339
    {
        const uint8_t data[] = "The quick brown fox jumps over the lazy dog";
        uint32_t result = crc32_compute(data, 43);
        ASSERT_EQ_HEX(result, 0x414FA339, "Fox string");
    }

    // Test Case 4: Single byte 0x00
    // Expected CRC32: 0xD202EF8D
    {
        const uint8_t data[] = {0x00};
        uint32_t result = crc32_compute(data, 1);
        ASSERT_EQ_HEX(result, 0xD202EF8D, "Single null byte");
    }

    // Test Case 5: Single byte 0xFF
    // Expected CRC32: 0xFF000000
    {
        const uint8_t data[] = {0xFF};
        uint32_t result = crc32_compute(data, 1);
        ASSERT_EQ_HEX(result, 0xFF000000, "Single 0xFF byte");
    }

    // Test Case 6: Split update (Chained)
    // "123456789" split into "12345" and "6789"
    {
        const uint8_t part1[] = "12345";
        const uint8_t part2[] = "6789";

        uint32_t crc1 = crc32_update(0, part1, 5);
        // Note: crc32_update expects the *previous* CRC.
        // BUT: our implementation's crc32_update takes 'crc' as the start value.
        // Standard crc32_update usually inverts input and output.
        // Let's check implementation again.
        // implementation: crc = ~crc; ... return ~crc;
        // So if we pass the result of first call (which is ~internal_crc) back to second call,
        // The second call will do ~(~internal_crc) = internal_crc. Correct.

        uint32_t final_crc = crc32_update(crc1, part2, 4);
        ASSERT_EQ_HEX(final_crc, 0xCBF43926, "Split update '12345' + '6789'");
    }

    if (failures == 0) {
        std::cout << "\nAll tests passed successfully!" << std::endl;
        return 0;
    } else {
        std::cerr << "\n" << failures << " tests failed." << std::endl;
        return 1;
    }
}
