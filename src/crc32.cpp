#include "crc32.h"
#include <string.h> // For memcpy

// Teensy 4.1 has 1MB RAM, so 8KB for CRC table is acceptable (0.8%).
// Other ARM platforms (STM32, ESP32) also typically have enough RAM.
// AVR devices (Uno, Nano) have only 2KB RAM, so we must avoid 8KB static allocation.
// Also enable on host platforms (Linux, macOS, Windows) for testing/benchmarking.
#if defined(TEENSYDUINO) || defined(ESP32) || defined(ARDUINO_ARCH_RP2040) || \
    (defined(__arm__) && !defined(__AVR__)) || \
    defined(__linux__) || defined(__APPLE__) || defined(_WIN32)
  #define CRC32_USE_SLICING_BY_8 1
#else
  #define CRC32_USE_SLICING_BY_8 0
#endif

#if CRC32_USE_SLICING_BY_8
// Slicing-by-8 optimization: 8 tables * 256 entries * 4 bytes = 8 KB
static uint32_t crc32_table[8][256];
#else
// Standard table: 1 table * 256 entries * 4 bytes = 1 KB
static uint32_t crc32_table[256];
#endif

static bool table_init = false;

static void init_table() {
#if CRC32_USE_SLICING_BY_8
  // Initialize the first table (standard CRC32 table)
  for (uint32_t i = 0; i < 256; i++) {
    uint32_t c = i;
    for (int k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
    crc32_table[0][i] = c;
  }

  // Initialize the remaining 7 slicing tables
  for (int j = 1; j < 8; j++) {
    for (int i = 0; i < 256; i++) {
      uint32_t c = crc32_table[j-1][i];
      // Logic for generating higher-order tables:
      // table[j][i] = (table[j-1][i] >> 8) ^ table[0][table[j-1][i] & 0xFF]
      crc32_table[j][i] = (c >> 8) ^ crc32_table[0][c & 0xFF];
    }
  }
#else
  // Standard initialization for single table
  for (uint32_t i = 0; i < 256; i++) {
    uint32_t c = i;
    for (int k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
    crc32_table[i] = c;
  }
#endif
  table_init = true;
}

uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len) {
  if (!table_init) init_table();
  crc = ~crc;

#if CRC32_USE_SLICING_BY_8
  // Align to 4-byte boundary for word access optimization
  while (len > 0 && ((uintptr_t)data & 3)) {
    crc = (crc >> 8) ^ crc32_table[0][(crc ^ *data++) & 0xFF];
    len--;
  }

  // Process 8 bytes at a time
  while (len >= 8) {
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    uint32_t one;
    uint32_t two;
    // Use memcpy to avoid strict aliasing violations and handle unaligned access safely
    // Modern compilers optimize this to a single load instruction on capable archs
    memcpy(&one, data, 4);
    memcpy(&two, data + 4, 4);

    one ^= crc;
    crc = crc32_table[7][one & 0xFF] ^
          crc32_table[6][(one >> 8) & 0xFF] ^
          crc32_table[5][(one >> 16) & 0xFF] ^
          crc32_table[4][(one >> 24) & 0xFF] ^
          crc32_table[3][two & 0xFF] ^
          crc32_table[2][(two >> 8) & 0xFF] ^
          crc32_table[1][(two >> 16) & 0xFF] ^
          crc32_table[0][(two >> 24) & 0xFF];
    data += 8;
    len -= 8;
#else
    // Fallback for Big Endian (unlikely on supported platforms but safe)
    // Process 8 bytes byte-by-byte using standard table logic but unrolled
    crc = (crc >> 8) ^ crc32_table[0][(crc ^ *data++) & 0xFF];
    crc = (crc >> 8) ^ crc32_table[0][(crc ^ *data++) & 0xFF];
    crc = (crc >> 8) ^ crc32_table[0][(crc ^ *data++) & 0xFF];
    crc = (crc >> 8) ^ crc32_table[0][(crc ^ *data++) & 0xFF];
    crc = (crc >> 8) ^ crc32_table[0][(crc ^ *data++) & 0xFF];
    crc = (crc >> 8) ^ crc32_table[0][(crc ^ *data++) & 0xFF];
    crc = (crc >> 8) ^ crc32_table[0][(crc ^ *data++) & 0xFF];
    crc = (crc >> 8) ^ crc32_table[0][(crc ^ *data++) & 0xFF];
    data += 8;
    len -= 8;
#endif
  }

  // Process remaining bytes
  while (len > 0) {
    crc = (crc >> 8) ^ crc32_table[0][(crc ^ *data++) & 0xFF];
    len--;
  }

#else
  // Standard byte-by-byte implementation
  for (size_t i = 0; i < len; i++) {
    const uint32_t prev = crc;
    crc = crc32_table[(prev ^ data[i]) & 0xFF] ^ (prev >> 8);
  }
#endif

  return ~crc;
}

uint32_t crc32_compute(const uint8_t* data, size_t len) {
  return crc32_update(0u, data, len);
}
