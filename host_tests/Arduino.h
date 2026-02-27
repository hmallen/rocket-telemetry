#pragma once
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <stdarg.h>
#include <algorithm>

// Define simple min/max macros that work for primitives
// In C++, we can use templates to be safer, but the source code likely expects macros
// or standard Arduino-style min/max

inline uint32_t millis() { return 0; }
inline void delay(uint32_t ms) {}

class Serial_ {
public:
    void begin(unsigned long baud) {}
    void print(const char* s) { printf("%s", s); }
    void println(const char* s) { printf("%s\n", s); }
    void printf(const char* fmt, ...) {
        va_list args;
        va_start(args, fmt);
        vprintf(fmt, args);
        va_end(args);
    }
    operator bool() const { return true; }
};

extern Serial_ Serial;

class HardwareSerial {
public:
    virtual void begin(unsigned long baud) {}
    virtual int available() { return 0; }
    virtual int read() { return -1; }
    virtual size_t write(const uint8_t *buffer, size_t size) { return size; }
    virtual size_t write(uint8_t c) { return 1; }
};

extern HardwareSerial Serial1;
extern HardwareSerial Serial2;

#define PROGMEM
#define PSTR(s) (s)
#define pgm_read_byte(p) (*(const uint8_t *)(p))

inline long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Add these at the end to avoid conflict with standard headers included before Arduino.h
// but still be available for code that includes Arduino.h
#ifndef min
#define min(a,b) ((a)<(b)?(a):(b))
#endif
#ifndef max
#define max(a,b) ((a)>(b)?(a):(b))
#endif
#ifndef abs
#define abs(x) ((x)>0?(x):-(x))
#endif

#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
