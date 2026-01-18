#include "gnss_ubx.h"
#include "cfg.h"
#include "records.h"
#include <stddef.h>

static void write_gnss_chunk(ByteRing& ring, uint32_t t_us, const uint8_t* data, uint16_t n) {
  while (n) {
    uint16_t take = (n > GNSS_CHUNK_MAX) ? GNSS_CHUNK_MAX : n;
    RecGnssChunk r{};
    r.h.type = REC_GNSS_CHUNK;
    r.h.ver = 1;
    r.h.len = (uint16_t)(offsetof(RecGnssChunk, data) + take);
    r.t_us = t_us;
    r.n = take;
    memcpy(r.data, data, take);
    ring.write(&r, r.h.len);
    data += take;
    n -= take;
  }
}

bool GnssUbx::begin() {
  GNSS_SERIAL.begin(GNSS_BAUD);
  delay(50);
  configure();
  return true;
}

void GnssUbx::send_ubx(const uint8_t* msg, uint16_t n) {
  GNSS_SERIAL.write(msg, n);
}

void GnssUbx::configure() {
  // Framework stub:
  // - disable NMEA
  // - enable UBX as needed
  // - set rate (GNSS_HZ)
  // Implement by sending UBX-CFG messages for your exact module/firmware.
}

void GnssUbx::poll(ByteRing& ring, uint32_t now_us) {
  while (GNSS_SERIAL.available()) {
    int c = GNSS_SERIAL.read();
    if (c < 0) break;
    chunk_[chunk_n_++] = (uint8_t)c;
    if (chunk_n_ == sizeof(chunk_)) {
      write_gnss_chunk(ring, now_us, chunk_, chunk_n_);
      chunk_n_ = 0;
    }
  }
  if (chunk_n_ && GNSS_SERIAL.available() == 0) {
    write_gnss_chunk(ring, now_us, chunk_, chunk_n_);
    chunk_n_ = 0;
  }
}
