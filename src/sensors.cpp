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
static int16_t imu_ax_bias_mg = 0;
static int16_t imu_ay_bias_mg = 0;
static int16_t imu_az_bias_mg = 0;
static int16_t imu_gx_bias_dps10 = 0;
static int16_t imu_gy_bias_dps10 = 0;
static int16_t imu_gz_bias_dps10 = 0;

static inline uint8_t imu_ready_pin_mode(bool pullup) {
  return pullup ? INPUT_PULLUP : INPUT;
}

static inline bool imu_ready_level(uint8_t pin, bool active_low) {
  if (pin == 255) return true;
  const bool high = digitalRead(pin) != 0;
  return active_low ? !high : high;
}

static inline int16_t clamp_i16(int32_t v) {
  if (v > 32767) return 32767;
  if (v < -32768) return -32768;
  return static_cast<int16_t>(v);
}

static bool read_imu_raw_sample(ImuSample& out) {
  out = {};
  if (!gyro_ok && !accel_ok) return false;

  const bool accel_ready = imu_ready_level(LRDY_PIN, ACCEL_RDY_ACTIVE_LOW);
  const bool gyro_ready = imu_ready_level(GRDY_PIN, GYRO_RDY_ACTIVE_LOW);
  if (!accel_ready && !gyro_ready) return false;

  sensors_event_t aev;
  sensors_event_t gev;
  const bool accel_read = (accel_ok && accel_ready) ? accel.getEvent(&aev) : false;
  const bool gyro_read = (gyro_ok && gyro_ready) ? gyro.getEvent(&gev) : false;

  uint16_t status = 0;
  if (accel_read) {
    // m/s^2 -> mg
    const float g = 9.80665f;
    out.ax = static_cast<int16_t>(lrintf((aev.acceleration.y / g) * 1000.0f));
    out.ay = static_cast<int16_t>(lrintf((aev.acceleration.z / g) * 1000.0f));
    out.az = static_cast<int16_t>(lrintf((aev.acceleration.x / g) * 1000.0f));
    status |= 0x0001;
  }

  if (gyro_read) {
    // rad/s -> deg/s -> deci-deg/s (0.1 dps)
    const float r2d = 57.2957795f;
    out.gx = static_cast<int16_t>(lrintf((gev.gyro.y * r2d) * 10.0f));
    out.gy = static_cast<int16_t>(lrintf((gev.gyro.z * r2d) * 10.0f));
    out.gz = static_cast<int16_t>(lrintf((gev.gyro.x * r2d) * 10.0f));
    status |= 0x0002;

    if (!isnan(gev.temperature)) {
      out.temp = static_cast<int16_t>(lrintf(gev.temperature * 100.0f));
      status |= 0x0004;
    }
  }

  out.status = status;
  return status != 0;
}

bool Sensors::begin() {
  if (GINT_PIN != 255) pinMode(GINT_PIN, INPUT);
  if (GRDY_PIN != 255) pinMode(GRDY_PIN, imu_ready_pin_mode(GYRO_RDY_PULLUP));
  if (LIN1_PIN != 255) pinMode(LIN1_PIN, INPUT);
  if (LIN2_PIN != 255) pinMode(LIN2_PIN, INPUT);
  if (LRDY_PIN != 255) pinMode(LRDY_PIN, imu_ready_pin_mode(ACCEL_RDY_PULLUP));

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

  // Max out IMU ranges at startup to reduce saturation risk during high dynamics.
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

bool Sensors::calibrate_imu() {
  if (!gyro_ok && !accel_ok) {
    return false;
  }

  int64_t sum_ax = 0;
  int64_t sum_ay = 0;
  int64_t sum_az = 0;
  int64_t sum_gx = 0;
  int64_t sum_gy = 0;
  int64_t sum_gz = 0;
  uint16_t accel_used = 0;
  uint16_t gyro_used = 0;

  const uint32_t start_ms = millis();
  while ((uint32_t)(millis() - start_ms) < IMU_CAL_TIMEOUT_MS) {
    if ((!accel_ok || accel_used >= IMU_CAL_SAMPLES) && (!gyro_ok || gyro_used >= IMU_CAL_SAMPLES)) {
      break;
    }

    ImuSample raw{};
    if (!read_imu_raw_sample(raw)) {
      if (IMU_CAL_SAMPLE_DELAY_MS > 0) {
        delay(IMU_CAL_SAMPLE_DELAY_MS);
      }
      continue;
    }

    if (accel_ok && accel_used < IMU_CAL_SAMPLES && ((raw.status & 0x0001u) != 0u)) {
      sum_ax += raw.ax;
      sum_ay += raw.ay;
      sum_az += raw.az;
      accel_used++;
    }
    if (gyro_ok && gyro_used < IMU_CAL_SAMPLES && ((raw.status & 0x0002u) != 0u)) {
      sum_gx += raw.gx;
      sum_gy += raw.gy;
      sum_gz += raw.gz;
      gyro_used++;
    }

    if (IMU_CAL_SAMPLE_DELAY_MS > 0) {
      delay(IMU_CAL_SAMPLE_DELAY_MS);
    }
  }

  if ((accel_ok && accel_used < IMU_CAL_MIN_SAMPLES) || (gyro_ok && gyro_used < IMU_CAL_MIN_SAMPLES)) {
    DBG_PRINTF("sensors: imu_cal failed accel_used=%u gyro_used=%u\n",
               static_cast<unsigned>(accel_used),
               static_cast<unsigned>(gyro_used));
    return false;
  }

  if (gyro_ok) {
    imu_gx_bias_dps10 = clamp_i16(static_cast<int32_t>(sum_gx / static_cast<int64_t>(gyro_used)));
    imu_gy_bias_dps10 = clamp_i16(static_cast<int32_t>(sum_gy / static_cast<int64_t>(gyro_used)));
    imu_gz_bias_dps10 = clamp_i16(static_cast<int32_t>(sum_gz / static_cast<int64_t>(gyro_used)));
  }

  if (accel_ok) {
    const float avg_ax = static_cast<float>(sum_ax) / static_cast<float>(accel_used);
    const float avg_ay = static_cast<float>(sum_ay) / static_cast<float>(accel_used);
    const float avg_az = static_cast<float>(sum_az) / static_cast<float>(accel_used);
    const float norm = sqrtf(avg_ax * avg_ax + avg_ay * avg_ay + avg_az * avg_az);

    if (norm > 100.0f) {
      const float scale = 1000.0f / norm;
      const float expected_ax = avg_ax * scale;
      const float expected_ay = avg_ay * scale;
      const float expected_az = avg_az * scale;
      imu_ax_bias_mg = clamp_i16(static_cast<int32_t>(lrintf(avg_ax - expected_ax)));
      imu_ay_bias_mg = clamp_i16(static_cast<int32_t>(lrintf(avg_ay - expected_ay)));
      imu_az_bias_mg = clamp_i16(static_cast<int32_t>(lrintf(avg_az - expected_az)));
    } else {
      imu_ax_bias_mg = 0;
      imu_ay_bias_mg = 0;
      imu_az_bias_mg = 0;
    }
  }

  DBG_PRINTF("sensors: imu_cal ok accel_used=%u gyro_used=%u acc_bias=%d,%d,%d gyro_bias=%d,%d,%d\n",
             static_cast<unsigned>(accel_used),
             static_cast<unsigned>(gyro_used),
             static_cast<int>(imu_ax_bias_mg),
             static_cast<int>(imu_ay_bias_mg),
             static_cast<int>(imu_az_bias_mg),
             static_cast<int>(imu_gx_bias_dps10),
             static_cast<int>(imu_gy_bias_dps10),
             static_cast<int>(imu_gz_bias_dps10));
  return true;
}

bool Sensors::read_imu(ImuSample& out) {
  ImuSample raw{};
  if (!read_imu_raw_sample(raw)) {
    out = {};
    return false;
  }

  out = raw;
  if ((raw.status & 0x0001u) != 0u) {
    out.ax = clamp_i16(static_cast<int32_t>(raw.ax) - static_cast<int32_t>(imu_ax_bias_mg));
    out.ay = clamp_i16(static_cast<int32_t>(raw.ay) - static_cast<int32_t>(imu_ay_bias_mg));
    out.az = clamp_i16(static_cast<int32_t>(raw.az) - static_cast<int32_t>(imu_az_bias_mg));
  }
  if ((raw.status & 0x0002u) != 0u) {
    out.gx = clamp_i16(static_cast<int32_t>(raw.gx) - static_cast<int32_t>(imu_gx_bias_dps10));
    out.gy = clamp_i16(static_cast<int32_t>(raw.gy) - static_cast<int32_t>(imu_gy_bias_dps10));
    out.gz = clamp_i16(static_cast<int32_t>(raw.gz) - static_cast<int32_t>(imu_gz_bias_dps10));
  }
  return true;
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
