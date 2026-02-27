#include "Arduino.h"
#include "../src/byte_ring.h"
#include <cassert>
#include <iostream>
#include <cstring>
#include <vector>

void test_init() {
    ByteRing br;
    const uint32_t cap = 16;
    uint8_t buffer[cap];
    br.init(buffer, cap);

    assert(br.available() == 0);
    assert(br.free_space() == cap - 1); // 1 byte reserved for full/empty distinction
    assert(br.drops() == 0);
    std::cout << "test_init passed" << std::endl;
}

void test_basic_rw() {
    ByteRing br;
    const uint32_t cap = 16;
    uint8_t buffer[cap];
    br.init(buffer, cap);

    const char* msg = "Hello";
    uint32_t len = strlen(msg);
    assert(br.write(msg, len));

    assert(br.available() == len);
    assert(br.free_space() == cap - 1 - len);

    char read_buf[16];
    uint32_t read_len = br.read(read_buf, len);
    read_buf[read_len] = '\0';

    assert(read_len == len);
    assert(strcmp(read_buf, msg) == 0);
    assert(br.available() == 0);
    std::cout << "test_basic_rw passed" << std::endl;
}

void test_wrap_around() {
    ByteRing br;
    const uint32_t cap = 10;
    uint8_t buffer[cap];
    br.init(buffer, cap);

    // Write almost full (capacity 10, usable 9)
    // Write 8 bytes
    uint8_t data1[] = {1, 2, 3, 4, 5, 6, 7, 8};
    assert(br.write(data1, 8));
    assert(br.available() == 8);
    // head=8, tail=0

    // Read 4 bytes to advance tail
    uint8_t read_buf[10];
    assert(br.read(read_buf, 4) == 4);
    assert(br.available() == 4);
    // head=8, tail=4 (remaining: 5, 6, 7, 8)

    // Write 4 more bytes.
    // Current state: head=8, tail=4, free=5.
    // Writing 4 bytes will wrap around: 8, 9, 0, 1.
    uint8_t data2[] = {9, 10, 11, 12};
    assert(br.write(data2, 4));

    assert(br.available() == 8); // 4 old + 4 new

    // Verify content by reading all 8 bytes
    // Expected: 5, 6, 7, 8, 9, 10, 11, 12
    uint8_t expected[] = {5, 6, 7, 8, 9, 10, 11, 12};
    uint8_t result[8];
    assert(br.read(result, 8) == 8);

    if (memcmp(result, expected, 8) != 0) {
        std::cerr << "Wrap around data mismatch!" << std::endl;
        for(int i=0; i<8; i++) std::cerr << (int)result[i] << " ";
        std::cerr << std::endl;
        assert(false);
    }
    std::cout << "test_wrap_around passed" << std::endl;
}

void test_full_buffer() {
    ByteRing br;
    const uint32_t cap = 5;
    uint8_t buffer[cap];
    br.init(buffer, cap);

    // Usable space is cap - 1 = 4
    uint8_t data[] = {1, 2, 3, 4};
    assert(br.write(data, 4));
    assert(br.free_space() == 0);
    assert(br.available() == 4);

    uint8_t extra = 5;
    assert(!br.write(&extra, 1)); // Should fail
    assert(br.drops() == 1);
    assert(br.available() == 4); // Still 4

    // Verify data integrity wasn't compromised
    uint8_t read_buf[4];
    br.read(read_buf, 4);
    assert(memcmp(read_buf, data, 4) == 0);
    std::cout << "test_full_buffer passed" << std::endl;
}

void test_peek() {
    ByteRing br;
    const uint32_t cap = 10;
    uint8_t buffer[cap];
    br.init(buffer, cap);

    uint8_t data[] = {10, 20, 30};
    br.write(data, 3);

    uint8_t peek_buf[3];
    assert(br.peek(peek_buf, 3) == 3);
    assert(memcmp(peek_buf, data, 3) == 0);

    // Ensure peek didn't consume data
    assert(br.available() == 3);

    // Now read it
    uint8_t read_buf[3];
    assert(br.read(read_buf, 3) == 3);
    assert(memcmp(read_buf, data, 3) == 0);
    assert(br.available() == 0);
    std::cout << "test_peek passed" << std::endl;
}

int main() {
    test_init();
    test_basic_rw();
    test_wrap_around();
    test_full_buffer();
    test_peek();

    std::cout << "All ByteRing tests passed!" << std::endl;
    return 0;
}
