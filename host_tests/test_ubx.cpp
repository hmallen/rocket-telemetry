#include <iostream>
#include <vector>
#include <cstring>
#include <cassert>

// Include the mock Arduino environment
#include "Arduino.h"

// Define globals required by cfg.h
HardwareSerial Serial;
HardwareSerial Serial1;
HardwareSerial Serial2;
TwoWire Wire;
SPIClass SPI;
SPIClass SPI1;

// Mock implementation of HardwareSerial for testing
class MockSerial : public HardwareSerial {
public:
    std::vector<uint8_t> rx_buffer;
    size_t read_pos = 0;

    void push_data(const std::vector<uint8_t>& data) {
        rx_buffer.insert(rx_buffer.end(), data.begin(), data.end());
    }

    virtual int available() override {
        return (int)(rx_buffer.size() - read_pos);
    }

    virtual int read() override {
        if (read_pos < rx_buffer.size()) {
            return rx_buffer[read_pos++];
        }
        return -1;
    }

    void clear() {
        rx_buffer.clear();
        read_pos = 0;
    }
};

// Include the source under test
#include "../src/gnss_ubx.h"

// Helper to calculate UBX checksum
void calculate_ubx_checksum(std::vector<uint8_t>& packet) {
    uint8_t ck_a = 0;
    uint8_t ck_b = 0;
    // Checksum covers CLS (index 2) to end of payload
    for (size_t i = 2; i < packet.size() - 2; ++i) {
        ck_a = ck_a + packet[i];
        ck_b = ck_b + ck_a;
    }
    packet[packet.size()-2] = ck_a;
    packet[packet.size()-1] = ck_b;
}

// Helper to build a UBX NAV-PVT packet (Class 0x01, ID 0x07)
std::vector<uint8_t> build_nav_pvt(uint32_t iTOW, uint8_t fixType, uint8_t flags) {
    std::vector<uint8_t> packet;
    packet.push_back(0xB5); // Sync 1
    packet.push_back(0x62); // Sync 2
    packet.push_back(0x01); // Class: NAV
    packet.push_back(0x07); // ID: PVT

    uint16_t payload_len = 92;
    packet.push_back(payload_len & 0xFF);
    packet.push_back((payload_len >> 8) & 0xFF);

    // Payload start
    // 0: iTOW (U4)
    packet.push_back(iTOW & 0xFF);
    packet.push_back((iTOW >> 8) & 0xFF);
    packet.push_back((iTOW >> 16) & 0xFF);
    packet.push_back((iTOW >> 24) & 0xFF);

    // 4..19: various fields (zero)
    for(int i=0; i<16; ++i) packet.push_back(0);

    // 20: fixType (U1)
    packet.push_back(fixType);

    // 21: flags (U1)
    packet.push_back(flags);

    // 22..91: rest of payload
    for(int i=22; i<92; ++i) packet.push_back(0);

    // Checksum placeholders
    packet.push_back(0);
    packet.push_back(0);

    calculate_ubx_checksum(packet);

    return packet;
}

void test_valid_packet() {
    std::cout << "Test: Valid Packet... ";
    MockSerial serial;
    GnssUbx gnss(serial);

    uint32_t test_tow = 12345678;
    // 3D fix (3), valid fix flag (1)
    auto packet = build_nav_pvt(test_tow, 3, 1);

    serial.push_data(packet);

    gnss.poll(nullptr, 1000000);

    const auto& t = gnss.time();
    if (t.tow_ms == test_tow && t.fix_type == 3 && t.fix_ok == true) {
        std::cout << "PASS" << std::endl;
    } else {
        std::cout << "FAIL (Got TOW=" << t.tow_ms << " Fix=" << (int)t.fix_type << ")" << std::endl;
        exit(1);
    }
}

void test_invalid_checksum() {
    std::cout << "Test: Invalid Checksum... ";
    MockSerial serial;
    GnssUbx gnss(serial);

    uint32_t test_tow = 87654321;
    auto packet = build_nav_pvt(test_tow, 3, 1);

    // Corrupt checksum (last byte)
    packet.back() = packet.back() + 1;

    serial.push_data(packet);

    gnss.poll(nullptr, 2000000);

    const auto& t = gnss.time();
    // Should remain default (0)
    if (t.tow_ms == 0 && t.fix_type == 0) {
        std::cout << "PASS" << std::endl;
    } else {
        std::cout << "FAIL (Updated despite invalid checksum)" << std::endl;
        exit(1);
    }
}

void test_corrupt_payload() {
    std::cout << "Test: Corrupt Payload... ";
    MockSerial serial;
    GnssUbx gnss(serial);

    uint32_t test_tow = 11111111;
    auto packet = build_nav_pvt(test_tow, 3, 1);

    // Corrupt payload byte (at index 10)
    // Packet structure: B5 62 CLS ID L1 L2 P0...
    // Index 0: B5
    // Index 1: 62
    // Index 2: 01
    // Index 3: 07
    // Index 4: 5C (92)
    // Index 5: 00
    // Index 6: P0 (Payload start)
    // Index 10 is payload byte 4.
    packet[10] = packet[10] + 1;

    // We do NOT recalculate checksum, so it is now invalid for the data.

    serial.push_data(packet);

    gnss.poll(nullptr, 3000000);

    const auto& t = gnss.time();
    if (t.tow_ms == 0) {
        std::cout << "PASS" << std::endl;
    } else {
        std::cout << "FAIL (Updated despite payload corruption)" << std::endl;
        exit(1);
    }
}

int main() {
    test_valid_packet();
    test_invalid_checksum();
    test_corrupt_payload();
    std::cout << "All tests passed!" << std::endl;
    return 0;
}
