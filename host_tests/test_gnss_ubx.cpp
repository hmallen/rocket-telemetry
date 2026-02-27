#include <vector>
#include <cstring>
#include <cstdio>
#include <algorithm>
#include "test_utils.h" // Includes <string>, <iostream>

// Include project headers AFTER standard headers
// and ensure Arduino.h is included after standard headers
#include "Arduino.h"
#include "gnss_ubx.h"

// Mock Serial implementation for test injection
class MockSerial : public HardwareSerial {
public:
    std::vector<uint8_t> rx_buffer;
    size_t read_pos = 0;

    void begin(unsigned long baud) override {}

    int available() override {
        return (int)(rx_buffer.size() - read_pos);
    }

    int read() override {
        if (read_pos < rx_buffer.size()) {
            return rx_buffer[read_pos++];
        }
        return -1;
    }

    size_t write(const uint8_t *buffer, size_t size) override {
        // Output spy could go here
        return size;
    }

    void feed(const uint8_t* data, size_t len) {
        rx_buffer.insert(rx_buffer.end(), data, data + len);
    }

    void reset() {
        rx_buffer.clear();
        read_pos = 0;
    }
};

MockSerial mockSerial;
// Define the globals required by Arduino.h
Serial_ Serial;
HardwareSerial Serial1;
HardwareSerial Serial2;


// Helper to build a UBX frame
void build_ubx_frame(uint8_t cls, uint8_t id, const uint8_t* payload, uint16_t len, std::vector<uint8_t>& out) {
    out.push_back(0xB5);
    out.push_back(0x62);
    out.push_back(cls);
    out.push_back(id);
    out.push_back(len & 0xFF);
    out.push_back((len >> 8) & 0xFF);

    for (uint16_t i = 0; i < len; i++) {
        out.push_back(payload[i]);
    }

    uint8_t ck_a = 0, ck_b = 0;
    for (size_t i = 2; i < out.size(); i++) {
        ck_a += out[i];
        ck_b += ck_a;
    }

    out.push_back(ck_a);
    out.push_back(ck_b);
}

void Test_Initialization() {
    GnssUbx gnss(mockSerial);
    TEST_ASSERT(gnss.begin());
    TEST_EQUAL(gnss.bytes_rx(), 0);
}

void Test_Parse_Valid_PVT() {
    mockSerial.reset();
    GnssUbx gnss(mockSerial);

    // Construct a valid UBX-NAV-PVT payload (92 bytes)
    // We only care about specific fields for the parser:
    // Offset 0: iTOW (U4)
    // Offset 20: fixType (U1)
    // Offset 21: flags (U1)
    // Offset 24: lon (I4)
    // Offset 28: lat (I4)
    // Offset 32: height (I4)

    uint8_t payload[92] = {0};
    uint32_t iTOW = 12345678;
    int32_t lon = 180000000;
    int32_t lat = 90000000;
    int32_t height = 1000;

    memcpy(&payload[0], &iTOW, 4);
    payload[20] = 3; // 3D fix
    payload[21] = 1; // Valid fix flag
    memcpy(&payload[24], &lon, 4);
    memcpy(&payload[28], &lat, 4);
    memcpy(&payload[32], &height, 4);

    std::vector<uint8_t> frame;
    build_ubx_frame(0x01, 0x07, payload, 92, frame); // NAV-PVT

    mockSerial.feed(frame.data(), frame.size());
    gnss.poll(nullptr, 1000000); // 1 sec

    const auto& time = gnss.time();
    TEST_EQUAL(time.tow_ms, iTOW);
    TEST_EQUAL(time.fix_type, 3);
    TEST_ASSERT(time.fix_ok);
    TEST_EQUAL(time.lon_e7, lon);
    TEST_EQUAL(time.lat_e7, lat);
    TEST_EQUAL(time.height_mm, height);
    TEST_ASSERT(gnss.fresh(1000000, 1000));
}

void Test_Parse_Checksum_Error() {
    mockSerial.reset();
    GnssUbx gnss(mockSerial);

    uint8_t payload[92] = {0};
    std::vector<uint8_t> frame;
    build_ubx_frame(0x01, 0x07, payload, 92, frame);

    // Corrupt checksum
    frame.back() = frame.back() ^ 0xFF;

    mockSerial.feed(frame.data(), frame.size());
    gnss.poll(nullptr, 1000000);

    // Should NOT have updated time
    TEST_EQUAL(gnss.time().tow_ms, 0);
}

void Test_Parse_Fragmented() {
    mockSerial.reset();
    GnssUbx gnss(mockSerial);

    uint8_t payload[92] = {0};
    uint32_t iTOW = 87654321;
    memcpy(&payload[0], &iTOW, 4);
    payload[20] = 3;
    payload[21] = 1;

    std::vector<uint8_t> frame;
    build_ubx_frame(0x01, 0x07, payload, 92, frame);

    // Feed byte by byte
    for (size_t i = 0; i < frame.size(); i++) {
        mockSerial.feed(&frame[i], 1);
        gnss.poll(nullptr, 1000000);
    }

    TEST_EQUAL(gnss.time().tow_ms, iTOW);
    TEST_EQUAL(gnss.bytes_rx(), frame.size());
}

void Test_Parse_Garbage_Recovery() {
    mockSerial.reset();
    GnssUbx gnss(mockSerial);

    // Valid frame
    uint8_t payload[92] = {0};
    uint32_t iTOW = 55555555;
    memcpy(&payload[0], &iTOW, 4);
    payload[20] = 3;
    payload[21] = 1;

    std::vector<uint8_t> valid_frame;
    build_ubx_frame(0x01, 0x07, payload, 92, valid_frame);

    // Garbage + Partial Sync + Valid Frame
    std::vector<uint8_t> stream;
    // Random garbage
    for(int i=0; i<10; i++) stream.push_back(rand() % 256);
    // Fake sync start 0xB5 but not followed by 0x62
    stream.push_back(0xB5);
    stream.push_back(0x00);
    // More garbage
    stream.push_back(0xAA);
    // Valid frame
    stream.insert(stream.end(), valid_frame.begin(), valid_frame.end());

    mockSerial.feed(stream.data(), stream.size());
    gnss.poll(nullptr, 1000000);

    TEST_EQUAL(gnss.time().tow_ms, iTOW);
}

int main() {
    Test_Initialization();
    Test_Parse_Valid_PVT();
    Test_Parse_Checksum_Error();
    Test_Parse_Fragmented();
    Test_Parse_Garbage_Recovery();

    print_test_summary();
    return (g_tests_failed == 0) ? 0 : 1;
}
