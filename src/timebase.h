#pragma once
#include <Arduino.h>

struct PpsLatch {
  volatile bool     pending;
  volatile uint32_t t_us_at_pps;
};

void timebase_init(uint8_t pps_pin);
uint32_t time_us();
bool time_pop_pps(uint32_t& t_us_at_pps);
