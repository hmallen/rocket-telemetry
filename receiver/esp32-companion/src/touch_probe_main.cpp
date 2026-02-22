#if defined(TOUCH_PROBE_TEST)

#include <Arduino.h>
#include <SPI.h>

namespace {

struct SpiBusConfig {
  const char* name;
  int8_t sclk;
  int8_t miso;
  int8_t mosi;
};

constexpr SpiBusConfig kBuses[] = {
    {"shared-miso33", 14, 33, 13},
    {"shared-miso12", 14, 12, 13},
    {"rnt-lvgl", 25, 39, 32},
    {"vspi", 18, 19, 23},
};

constexpr int8_t kCsPins[] = {12, 33, 15, 5, 2, 21, 22, 27, 32};
constexpr int8_t kIrqPins[] = {36, 39, 34, 35};

int16_t peakZ[sizeof(kBuses) / sizeof(kBuses[0])][sizeof(kCsPins) / sizeof(kCsPins[0])] = {{0}};
uint32_t sampleCount = 0;

int16_t clampZ(int32_t z) {
  if (z < 0) {
    return 0;
  }
  if (z > 4095) {
    return 4095;
  }
  return static_cast<int16_t>(z);
}

void deselectAllKnownCs() {
  for (size_t i = 0; i < (sizeof(kCsPins) / sizeof(kCsPins[0])); ++i) {
    pinMode(kCsPins[i], OUTPUT);
    digitalWrite(kCsPins[i], HIGH);
  }
}

void readRawSample(const SpiBusConfig& bus, int8_t cs, int16_t& outX, int16_t& outY, int16_t& outZ,
                   int16_t& outZ1, int16_t& outZ2) {
  SPI.end();
  SPI.begin(bus.sclk, bus.miso, bus.mosi, cs);
  deselectAllKnownCs();

  SPI.beginTransaction(SPISettings(500000, MSBFIRST, SPI_MODE0));
  digitalWrite(cs, LOW);

  SPI.transfer(0xB1);                               // Z1 command
  const int16_t z1 = SPI.transfer16(0xC1) >> 3;    // Z2 command shifted in
  const int16_t z2 = SPI.transfer16(0x91) >> 3;    // next command while reading prior
  const int32_t z = static_cast<int32_t>(z1) + 4095 - static_cast<int32_t>(z2);

  // One quick X/Y read pair for probe purposes.
  const int16_t y = SPI.transfer16(0xD1) >> 3;
  const int16_t x = SPI.transfer16(0x91) >> 3;

  // Power down sequence.
  SPI.transfer16(0xD0);
  SPI.transfer16(0);

  digitalWrite(cs, HIGH);
  SPI.endTransaction();

  outX = x;
  outY = y;
  outZ = clampZ(z);
  outZ1 = z1;
  outZ2 = z2;
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(300);

  deselectAllKnownCs();
  for (size_t i = 0; i < (sizeof(kIrqPins) / sizeof(kIrqPins[0])); ++i) {
    pinMode(kIrqPins[i], INPUT);
  }

  Serial.println();
  Serial.println("=== XPT2046 Touch Pin Probe (DIS05035H) ===");
  Serial.println("Watch at rest, then touch and hold during multiple samples.");
  Serial.println("Look for a bus/cs pair where x/y change with touch.");
  Serial.println("Columns: bus sclk/miso/mosi cs | z x y z1 z2 | peakZ");
  Serial.println();
}

void loop() {
  ++sampleCount;
  Serial.print("sample ");
  Serial.println(sampleCount);

  Serial.print("  irq");
  for (size_t i = 0; i < (sizeof(kIrqPins) / sizeof(kIrqPins[0])); ++i) {
    Serial.print(" ");
    Serial.print(kIrqPins[i]);
    Serial.print("=");
    Serial.print(digitalRead(kIrqPins[i]) == LOW ? "LOW" : "HIGH");
  }
  Serial.println();

  for (size_t busIdx = 0; busIdx < (sizeof(kBuses) / sizeof(kBuses[0])); ++busIdx) {
    for (size_t csIdx = 0; csIdx < (sizeof(kCsPins) / sizeof(kCsPins[0])); ++csIdx) {
      int16_t x = 0;
      int16_t y = 0;
      int16_t z = 0;
      int16_t z1 = 0;
      int16_t z2 = 0;

      readRawSample(kBuses[busIdx], kCsPins[csIdx], x, y, z, z1, z2);
      if (z > peakZ[busIdx][csIdx]) {
        peakZ[busIdx][csIdx] = z;
      }

      Serial.print("  ");
      Serial.print(kBuses[busIdx].name);
      Serial.print(" ");
      Serial.print(kBuses[busIdx].sclk);
      Serial.print("/");
      Serial.print(kBuses[busIdx].miso);
      Serial.print("/");
      Serial.print(kBuses[busIdx].mosi);
      Serial.print(" cs");
      Serial.print(kCsPins[csIdx]);
      Serial.print(" | z=");
      Serial.print(z);
      Serial.print(" x=");
      Serial.print(x);
      Serial.print(" y=");
      Serial.print(y);
      Serial.print(" z1=");
      Serial.print(z1);
      Serial.print(" z2=");
      Serial.print(z2);
      Serial.print(" | peak=");
      Serial.println(peakZ[busIdx][csIdx]);

      delay(6);
    }
  }

  Serial.println();
  delay(450);
}

#endif  // TOUCH_PROBE_TEST
