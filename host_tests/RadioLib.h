#pragma once

#include "Arduino.h"

// Define these to match cfg.h or just dummy values
#define LORA_CS 10
#define LORA_DIO0 9
#define LORA_RST 8
#define LORA_BUSY 255

class Module {
public:
    Module(int, int, int, int) {}
};

class SX1276 {
public:
    SX1276(Module*) {}
    int begin(float) { return 0; }
    void setPacketSentAction(void (*func)(void)) {}
    int setSpreadingFactor(uint8_t) { return 0; }
    int setBandwidth(float) { return 0; }
    int setCodingRate(uint8_t) { return 0; }
    int setOutputPower(int8_t) { return 0; }
    int setCRC(bool) { return 0; }
    int setSyncWord(uint8_t) { return 0; }
    int setPreambleLength(uint16_t) { return 0; }
    int startTransmit(uint8_t*, size_t) { return 0; }
    int finishTransmit() { return 0; }
    int startReceive() { return 0; }
    uint16_t getIRQFlags() { return 0; }
    size_t getPacketLength() { return 0; }
    int readData(uint8_t*, size_t) { return 0; }
};
