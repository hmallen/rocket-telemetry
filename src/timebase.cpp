#include "timebase.h"

static PpsLatch g_pps{false, 0};
static uint8_t g_pps_pin = 255;

static void IRAM_ATTR pps_isr() {
  g_pps.t_us_at_pps = micros();
  g_pps.pending = true;
}

void timebase_init(uint8_t pps_pin) {
  g_pps_pin = pps_pin;
  pinMode(g_pps_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(g_pps_pin), pps_isr, RISING);
}

uint32_t time_us() {
  return micros();
}

bool time_pop_pps(uint32_t& t_us_at_pps) {
  if (!g_pps.pending) return false;
  noInterrupts();
  t_us_at_pps = g_pps.t_us_at_pps;
  g_pps.pending = false;
  interrupts();
  return true;
}
