#include "Arduino.h"
#include "../src/psram_spool.h"
#include <iostream>
#include <vector>
#include <cstring>
#include <cassert>

using namespace std;

// Helper to assert with message
void assert_eq_impl(long long a, long long b, const char* msg, int line) {
    if (a != b) {
        cerr << "FAIL: " << msg << " | Expected: " << b << ", Actual: " << a << " at line " << line << endl;
        exit(1);
    }
}

void assert_true_impl(bool cond, const char* msg, int line) {
    if (!cond) {
        cerr << "FAIL: " << msg << " at line " << line << endl;
        exit(1);
    }
}

#define ASSERT_EQ(a, b, msg) assert_eq_impl((long long)(a), (long long)(b), msg, __LINE__)
#define ASSERT_TRUE(cond, msg) assert_true_impl((cond), msg, __LINE__)

void test_initialization() {
    cout << "Running test_initialization..." << endl;
    uint8_t backing[1024];
    PsramSpool spool;
    spool.init(backing, 1024);

    ASSERT_EQ(spool.drops(), 0, "Drops should be 0 initially");
    ASSERT_EQ(spool.available(), 0, "Available should be 0 initially");
    ASSERT_EQ(spool.free_space(), 1023, "Free space should be capacity - 1");
    cout << "PASS" << endl;
}

void test_basic_write_read() {
    cout << "Running test_basic_write_read..." << endl;
    uint8_t backing[1024];
    PsramSpool spool;
    spool.init(backing, 1024);

    uint8_t data[] = {0xAA, 0xBB, 0xCC, 0xDD};
    ASSERT_TRUE(spool.write(data, 4), "Write should succeed");

    ASSERT_EQ(spool.available(), 4, "Available should be 4");
    ASSERT_EQ(spool.free_space(), 1023 - 4, "Free space should decrease");

    uint8_t read_buf[10];
    // Attempt to read more than available
    uint32_t read_count = spool.read(read_buf, 10);

    ASSERT_EQ(read_count, 4, "Should read only available bytes");
    ASSERT_EQ(memcmp(data, read_buf, 4), 0, "Read data should match written data");
    ASSERT_EQ(spool.available(), 0, "Buffer should be empty after read");
    cout << "PASS" << endl;
}

void test_wrap_around() {
    cout << "Running test_wrap_around..." << endl;
    // Small buffer: Capacity 10 means useful capacity is 9 bytes.
    uint8_t backing[10];
    PsramSpool spool;
    spool.init(backing, 10);

    // Fill partially: 6 bytes.
    // Data: 1, 2, 3, 4, 5, 6
    uint8_t data1[] = {1, 2, 3, 4, 5, 6};
    ASSERT_TRUE(spool.write(data1, 6), "First write failed");

    // Drain 4 bytes.
    // Remaining in buffer: 5, 6 (2 bytes)
    uint8_t sink[10];
    spool.read(sink, 4);

    // Write enough to wrap.
    // 5 new bytes: 10, 11, 12, 13, 14
    // Should fit (2 + 5 = 7 <= 9).
    uint8_t data2[] = {10, 11, 12, 13, 14};
    ASSERT_TRUE(spool.write(data2, 5), "Wrap write failed");

    // Total bytes in buffer: 7.
    // Expected order: 5, 6, 10, 11, 12, 13, 14
    ASSERT_EQ(spool.available(), 7, "Available count wrong after wrap");

    uint8_t read_buf[10];
    uint32_t n = spool.read(read_buf, 10);
    ASSERT_EQ(n, 7, "Read count wrong");

    uint8_t expected[] = {5, 6, 10, 11, 12, 13, 14};
    bool match = true;
    for(int i=0; i<7; i++) {
        if(read_buf[i] != expected[i]) {
            match = false;
            cout << "Mismatch at index " << i << ": expected " << (int)expected[i] << ", got " << (int)read_buf[i] << endl;
        }
    }
    ASSERT_TRUE(match, "Read data mismatch");
    cout << "PASS" << endl;
}

void test_buffer_full() {
    cout << "Running test_buffer_full..." << endl;
    uint8_t backing[10];
    PsramSpool spool;
    spool.init(backing, 10);

    // Capacity is 10, useful capacity is 9.
    uint8_t data[9];
    memset(data, 0xFF, 9);
    ASSERT_TRUE(spool.write(data, 9), "Fill to capacity-1 failed");

    ASSERT_EQ(spool.free_space(), 0, "Free space should be 0");
    ASSERT_EQ(spool.available(), 9, "Available should be 9");

    uint8_t one_byte = 0x00;
    // Attempt to write 1 more byte should fail
    ASSERT_TRUE(!spool.write(&one_byte, 1), "Write to full buffer should fail");
    ASSERT_EQ(spool.drops(), 1, "Drop count should increment");

    cout << "PASS" << endl;
}

int main() {
    test_initialization();
    test_basic_write_read();
    test_wrap_around();
    test_buffer_full();
    cout << "All tests passed!" << endl;
    return 0;
}
