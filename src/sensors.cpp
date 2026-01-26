#include "sensors.h"
#include "cfg.h"

#include <SPI.h>
#include <Adafruit_BMP3XX.h>

 #include <Wire.h>
 #include <Adafruit_Sensor.h>
 #include <Adafruit_L3GD20_U.h>
 #include <Adafruit_LSM303_U.h>
 #include <math.h>

static Adafruit_BMP3XX bmp;
static Adafruit_L3GD20_Unified gyro(20);
static Adafruit_LSM303_Accel_Unified accel(30301);
static bool imu_ok = false;

bool Sensors::begin() {
  if (GINT_PIN != 255) pinMode(GINT_PIN, INPUT);
  if (GRDY_PIN != 255) pinMode(GRDY_PIN, INPUT);
  if (LIN1_PIN != 255) pinMode(LIN1_PIN, INPUT);
  if (LIN2_PIN != 255) pinMode(LIN2_PIN, INPUT);
  if (LRDY_PIN != 255) pinMode(LRDY_PIN, INPUT);

  I2C_BUS.begin();
  I2C_BUS.setClock(I2C_HZ);

  const bool gyro_ok  = gyro.begin(GYRO_RANGE_2000DPS, &I2C_BUS);
  const bool accel_ok = accel.begin();
  imu_ok = gyro_ok && accel_ok;

  if (!bmp.begin_SPI(BMP_CS, &BMP_SPI_BUS, BMP_SPI_HZ)) return false;

  // Fastest / lowest-latency settings for high-rate logging
  bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
  bmp.setPressureOversampling(BMP3_NO_OVERSAMPLING);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);
  return true;
}

bool Sensors::read_imu(ImuSample& out) {
  out = {};
  if (!imu_ok) return false;

  const bool accel_ready = (LRDY_PIN == 255) ? true : (digitalRead(LRDY_PIN) != 0);
  const bool gyro_ready  = (GRDY_PIN == 255) ? true : (digitalRead(GRDY_PIN) != 0);

  if (!accel_ready && !gyro_ready) return false;

  sensors_event_t aev;
  sensors_event_t gev;

  const bool accel_read = accel_ready ? accel.getEvent(&aev) : false;
  const bool gyro_read  = gyro_ready ? gyro.getEvent(&gev) : false;

  uint16_t status = 0;

  if (accel_read) {
    // m/s^2 -> mg
    const float g = 9.80665f;
    out.ax = (int16_t)lrintf((aev.acceleration.x / g) * 1000.0f);
    out.ay = (int16_t)lrintf((aev.acceleration.y / g) * 1000.0f);
    out.az = (int16_t)lrintf((aev.acceleration.z / g) * 1000.0f);
    status |= 0x0001;
  }

  if (gyro_read) {
    // rad/s -> deg/s -> deci-deg/s (0.1 dps)
    const float r2d = 57.2957795f;
    out.gx = (int16_t)lrintf((gev.gyro.x * r2d) * 10.0f);
    out.gy = (int16_t)lrintf((gev.gyro.y * r2d) * 10.0f);
    out.gz = (int16_t)lrintf((gev.gyro.z * r2d) * 10.0f);
    status |= 0x0002;

    if (!isnan(gev.temperature)) {
      out.temp = (int16_t)lrintf(gev.temperature * 100.0f); // centi-degC
      status |= 0x0004;
    }
  }

  out.status = status;
  return status != 0;
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
