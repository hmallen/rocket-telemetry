#include "sensors.h"
#include "cfg.h"

#include <SPI.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BMP085.h>

 #include <Wire.h>
 #include <Adafruit_Sensor.h>
 #include <Adafruit_L3GD20_U.h>
 #include <Adafruit_LSM303_U.h>
 #include <math.h>

static Adafruit_BMP3XX bmp;
static Adafruit_BMP085 bmp180;
static Adafruit_L3GD20_Unified gyro(20);
static Adafruit_LSM303_Accel_Unified accel(30301);
static bool gyro_ok = false;
static bool accel_ok = false;
static bool baro2_ok = false;

bool Sensors::begin() {
  if (GINT_PIN != 255) pinMode(GINT_PIN, INPUT);
  if (GRDY_PIN != 255) pinMode(GRDY_PIN, INPUT);
  if (LIN1_PIN != 255) pinMode(LIN1_PIN, INPUT);
  if (LIN2_PIN != 255) pinMode(LIN2_PIN, INPUT);
  if (LRDY_PIN != 255) pinMode(LRDY_PIN, INPUT);

  I2C_BUS.begin();
  I2C_BUS.setClock(I2C_HZ);

#if DEBUG_MODE
  uint8_t i2c_found = 0;
  for (uint8_t addr = 0x08; addr < 0x78; ++addr) {
    I2C_BUS.beginTransmission(addr);
    const uint8_t err = I2C_BUS.endTransmission();
    if (err == 0) {
      DBG_PRINTF("sensors: i2c_found=0x%02x\n", (unsigned)addr);
      ++i2c_found;
    }
  }
  DBG_PRINTF("sensors: i2c_devices=%u\n", (unsigned)i2c_found);
#endif

  gyro_ok = gyro.begin(GYRO_RANGE_2000DPS, &I2C_BUS);
  accel_ok = accel.begin();

  DBG_PRINTF("sensors: gyro=%u accel=%u\n", (unsigned)gyro_ok, (unsigned)accel_ok);

  baro2_ok = bmp180.begin(BMP085_STANDARD, &I2C_BUS);

  DBG_PRINTF("sensors: bmp180=%u\n", (unsigned)baro2_ok);

  pinMode(BMP_CS, OUTPUT);
  digitalWrite(BMP_CS, HIGH);
  BMP_SPI_BUS.setMOSI(BMP_MOSI_PIN);
  BMP_SPI_BUS.setSCK(BMP_SCK_PIN);
  BMP_SPI_BUS.setMISO(BMP_MISO_PIN);
  BMP_SPI_BUS.begin();
  const bool bmp_ok = bmp.begin_SPI(BMP_CS, &BMP_SPI_BUS, BMP_SPI_HZ);
  DBG_PRINTF("sensors: bmp3xx_spi=%u\n", (unsigned)bmp_ok);
  if (!bmp_ok) return false;

  // Fastest / lowest-latency settings for high-rate logging
  bmp.setTemperatureOversampling(BMP3_NO_OVERSAMPLING);
  bmp.setPressureOversampling(BMP3_NO_OVERSAMPLING);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_DISABLE);
  bmp.setOutputDataRate(BMP3_ODR_200_HZ);
  return true;
}

bool Sensors::read_imu(ImuSample& out) {
  out = {};
  if (!gyro_ok && !accel_ok) return false;

  const bool accel_ready = (LRDY_PIN == 255) ? true : (digitalRead(LRDY_PIN) != 0);
  const bool gyro_ready  = (GRDY_PIN == 255) ? true : (digitalRead(GRDY_PIN) != 0);

  if (!accel_ready && !gyro_ready) return false;

  sensors_event_t aev;
  sensors_event_t gev;

  const bool accel_read = (accel_ok && accel_ready) ? accel.getEvent(&aev) : false;
  const bool gyro_read  = (gyro_ok && gyro_ready) ? gyro.getEvent(&gev) : false;

  uint16_t status = 0;

  if (accel_read) {
    // m/s^2 -> mg
    const float g = 9.80665f;
    out.ax = (int16_t)lrintf((aev.acceleration.x / g) * 1000.0f);
    out.ay = (int16_t)lrintf((aev.acceleration.z / g) * 1000.0f);
    out.az = (int16_t)lrintf((aev.acceleration.y / g) * 1000.0f);
    status |= 0x0001;
  }

  if (gyro_read) {
    // rad/s -> deg/s -> deci-deg/s (0.1 dps)
    const float r2d = 57.2957795f;
    out.gx = (int16_t)lrintf((gev.gyro.x * r2d) * 10.0f);
    out.gy = (int16_t)lrintf((gev.gyro.z * r2d) * 10.0f);
    out.gz = (int16_t)lrintf((gev.gyro.y * r2d) * 10.0f);
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

bool Sensors::read_baro2(BaroSample& out) {
  if (!baro2_ok) return false;
  const int32_t pa = (int32_t)bmp180.readPressure();
  const float tc = bmp180.readTemperature();
  out.press_pa_x10 = (int32_t)(pa * 10);
  out.temp_c_x100  = (int16_t)(tc * 100.0f);
  out.status = 1;
  return true;
}
