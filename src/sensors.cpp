#include "sensors.h"
#include "cfg.h"

#include <SPI.h>
#include <Adafruit_BMP3XX.h>

static Adafruit_BMP3XX bmp;

bool Sensors::begin() {
  if (!bmp.begin_SPI(BMP_CS, &BMP_SPI_BUS, BMP_SPI_HZ)) return false;

  // Fastest / lowest-latency settings for high-rate logging
  bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
  bmp.setPressureOversampling(BMP3_NO_OVERSAMPLING);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);
  return true;
}

bool Sensors::read_imu(ImuSample& out) {
  // Framework stub:
  // Replace with your chosen IMU/10-DOF driver reads.
  // Keep this function non-blocking and fast.
  out = {};
  out.status = 0;
  return false;
}

bool Sensors::read_baro(BaroSample& out) {
  if (!bmp.performReading()) return false;
  // Convert to fixed-point
  float pa = bmp.pressure;       // Pa
  float tc = bmp.temperature;    // C
  out.press_pa_x10 = (int32_t)(pa * 10.0f);
  out.temp_c_x100  = (int16_t)(tc * 100.0f);
  out.status = 1;
  return true;
}
