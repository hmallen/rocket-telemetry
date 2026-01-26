#pragma once
#include <Arduino.h>

struct ImuSample {
  int16_t ax, ay, az;
  int16_t gx, gy, gz;
  int16_t temp;
  uint16_t status;
};

struct BaroSample {
  int32_t press_pa_x10;
  int16_t temp_c_x100;
  uint16_t status;
};

class Sensors {
public:
  bool begin();
  bool read_imu(ImuSample& out);
  bool read_baro(BaroSample& out);
  bool read_baro2(BaroSample& out);
};
