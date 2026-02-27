#pragma once

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define A2 0

inline uint32_t millis() { return 0; }
inline void delay(uint32_t ms) {}
inline uint32_t micros() { return 0; }
inline long random(long max) { return 0; }
inline long random(long min, long max) { return min; }

class MockSerial {
public:
    void begin(unsigned long) {}
    operator bool() { return true; }
    void print(const char*) {}
    void print(int) {}
    void print(double) {}
    void println(const char*) {}
    void println(int) {}
    void printf(const char* fmt, ...) {}
    int available() { return 0; }
    int read() { return -1; }
};

class HardwareSerial : public MockSerial {};

class TwoWire {
public:
    void begin() {}
    void setClock(uint32_t) {}
};

class SPIClass {
public:
    void begin() {}
};

extern MockSerial Serial;
extern HardwareSerial Serial1;
extern HardwareSerial Serial2;
extern TwoWire Wire;
extern SPIClass SPI1;

#include <algorithm>
using std::min;
using std::max;
