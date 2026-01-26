#include "gnss_ubx.h"
#include "cfg.h"
#include "records.h"
#include <stddef.h>
#include <string.h>

static void ubx_checksum(const uint8_t* p, uint16_t n, uint8_t* ck_a, uint8_t* ck_b) {
  uint8_t a = 0;
  uint8_t b = 0;
  for (uint16_t i = 0; i < n; ++i) {
    a = (uint8_t)(a + p[i]);
    b = (uint8_t)(b + a);
  }
  *ck_a = a;
  *ck_b = b;
}

static void ubx_build(uint8_t* out, uint16_t* out_n, uint8_t cls, uint8_t id, const void* payload, uint16_t payload_n) {
  out[0] = 0xB5;
  out[1] = 0x62;
  out[2] = cls;
  out[3] = id;
  out[4] = (uint8_t)(payload_n & 0xFF);
  out[5] = (uint8_t)((payload_n >> 8) & 0xFF);
  if (payload_n && payload) memcpy(out + 6, payload, payload_n);
  uint8_t ck_a = 0, ck_b = 0;
  ubx_checksum(out + 2, (uint16_t)(4 + payload_n), &ck_a, &ck_b);
  out[6 + payload_n] = ck_a;
  out[7 + payload_n] = ck_b;
  *out_n = (uint16_t)(8 + payload_n);
}

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
  serial_.begin(GNSS_BAUD);
  delay(50);
  configure();
  return true;
}

void GnssUbx::send_ubx(const uint8_t* msg, uint16_t n) {
  serial_.write(msg, n);
}

void GnssUbx::configure() {
  auto send = [&](uint8_t cls, uint8_t id, const void* payload, uint16_t payload_n) {
    uint8_t buf[64];
    if (payload_n > (uint16_t)(sizeof(buf) - 8)) return;
    uint16_t n = 0;
    ubx_build(buf, &n, cls, id, payload, payload_n);
    send_ubx(buf, n);
    delay(20);
  };

  {
    struct {
      uint8_t portID;
      uint8_t reserved0;
      uint16_t txReady;
      uint32_t mode;
      uint32_t baudRate;
      uint16_t inProtoMask;
      uint16_t outProtoMask;
      uint16_t flags;
      uint16_t reserved5;
    } p{};
    p.portID = 1;
    p.mode = 0x08D0;
    p.baudRate = GNSS_BAUD;
    p.inProtoMask = 0x0001;
    p.outProtoMask = 0x0001;
    send(0x06, 0x00, &p, (uint16_t)sizeof(p));
  }

  {
    uint8_t p[8] = {0};
    p[0] = 0xF0;
    for (uint8_t id = 0; id <= 0x0A; ++id) {
      p[1] = id;
      p[2] = 0;
      p[3] = 0;
      p[4] = 0;
      p[5] = 0;
      p[6] = 0;
      p[7] = 0;
      send(0x06, 0x01, p, (uint16_t)sizeof(p));
    }
  }

  {
    uint8_t p[8] = {0};
    p[0] = 0x01;
    p[1] = 0x07;
    p[3] = 1;
    send(0x06, 0x01, p, (uint16_t)sizeof(p));
  }

  {
    if (GNSS_HZ == 0) return;
    uint16_t meas_ms = (uint16_t)((1000UL + (GNSS_HZ / 2)) / GNSS_HZ);
    struct {
      uint16_t measRate;
      uint16_t navRate;
      uint16_t timeRef;
    } p{};
    p.measRate = meas_ms;
    p.navRate = 1;
    p.timeRef = 1;
    send(0x06, 0x08, &p, (uint16_t)sizeof(p));
  }
}

void GnssUbx::poll(ByteRing* ring, uint32_t now_us) {
  while (serial_.available()) {
    int c = serial_.read();
    if (c < 0) break;
    last_rx_us_ = now_us;
    chunk_[chunk_n_++] = (uint8_t)c;
    if (chunk_n_ == sizeof(chunk_)) {
      if (ring) write_gnss_chunk(*ring, now_us, chunk_, chunk_n_);
      chunk_n_ = 0;
    }
  }
  if (chunk_n_ && serial_.available() == 0) {
    if (ring) write_gnss_chunk(*ring, now_us, chunk_, chunk_n_);
    chunk_n_ = 0;
  }
}
