#include "crc32.h"

static uint32_t crc32_table[256];
static bool table_init = false;

static void init_table() {
  for (uint32_t i = 0; i < 256; i++) {
    uint32_t c = i;
    for (int k = 0; k < 8; k++) c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
    crc32_table[i] = c;
  }
  table_init = true;
}

uint32_t crc32_update(uint32_t crc, const uint8_t* data, size_t len) {
  if (!table_init) init_table();
  crc = ~crc;
  for (size_t i = 0; i < len; i++) crc = crc32_table[(crc ^ data[i]) & 0xFF] ^ (crc >> 8);
  return ~crc;
}

uint32_t crc32_compute(const uint8_t* data, size_t len) {
  return crc32_update(0u, data, len);
}
