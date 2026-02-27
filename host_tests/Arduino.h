#pragma once
#include <cstdint>
#include <cstddef>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <string>
#include <algorithm>

using std::min;
using std::max;

typedef uint8_t byte;

class HardwareSerial {
public:
    virtual void begin(unsigned long baud) {}
    virtual void write(const uint8_t *buffer, size_t size) {}
    virtual size_t write(uint8_t c) { return 1; }
    virtual int available() { return 0; }
    virtual int read() { return -1; }
    virtual operator bool() { return true; }
};

inline void delay(unsigned long ms) {}
inline unsigned long micros() { return 0; }
inline unsigned long millis() { return 0; }

#define A0 14
#define A1 15
#define A2 16
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2
#define HIGH 1
#define LOW 0

inline void pinMode(uint8_t, uint8_t) {}
inline void digitalWrite(uint8_t, uint8_t) {}
inline int digitalRead(uint8_t) { return 0; }
inline int analogRead(uint8_t) { return 0; }

class TwoWire { public: void begin() {} };
class SPIClass { public: void begin() {} };

extern HardwareSerial Serial;
extern HardwareSerial Serial1;
extern HardwareSerial Serial2;
extern TwoWire Wire;
extern SPIClass SPI;
extern SPIClass SPI1;
