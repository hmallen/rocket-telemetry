#include "companion_uart.h"

#include <string.h>

namespace companion_uart {
namespace {

static constexpr uint8_t SOF1 = 0xA5;
static constexpr uint8_t SOF2 = 0x5A;
static constexpr uint8_t VERSION = 0x01;
static constexpr uint8_t MSG_TELEM_SNAPSHOT = 0x01;
static constexpr uint8_t MSG_CMD = 0x10;
static constexpr uint8_t MSG_CMD_ACK = 0x11;

#pragma pack(push, 1)
struct CmdV1 {
  uint8_t cmd;
  uint8_t arg;
};

struct CmdAckV1 {
  uint8_t cmd;
  uint8_t ok;
  uint8_t err;
};
#pragma pack(pop)

HardwareSerial* g_serial = nullptr;
uint16_t g_seq = 0;

uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
    }
  }
  return crc;
}

void write_frame(uint8_t type, const uint8_t* payload, uint16_t len) {
  if (!g_serial) return;

  uint8_t hdr[8];
  hdr[0] = SOF1;
  hdr[1] = SOF2;
  hdr[2] = VERSION;
  hdr[3] = type;
  hdr[4] = static_cast<uint8_t>(g_seq & 0xFF);
  hdr[5] = static_cast<uint8_t>((g_seq >> 8) & 0xFF);
  hdr[6] = static_cast<uint8_t>(len & 0xFF);
  hdr[7] = static_cast<uint8_t>((len >> 8) & 0xFF);

  uint8_t crcBuf[6 + sizeof(Snapshot)];
  crcBuf[0] = hdr[2];
  crcBuf[1] = hdr[3];
  crcBuf[2] = hdr[4];
  crcBuf[3] = hdr[5];
  crcBuf[4] = hdr[6];
  crcBuf[5] = hdr[7];
  if (len > 0) memcpy(crcBuf + 6, payload, len);
  uint16_t crc = crc16_ccitt(crcBuf, 6 + len);

  g_serial->write(hdr, sizeof(hdr));
  if (len > 0) g_serial->write(payload, len);
  g_serial->write(static_cast<uint8_t>(crc & 0xFF));
  g_serial->write(static_cast<uint8_t>((crc >> 8) & 0xFF));
  ++g_seq;
}

}  // namespace

void init(HardwareSerial& serial, uint32_t baud) {
  g_serial = &serial;
  g_serial->begin(baud);
}

void send_snapshot(const Snapshot& s) {
  write_frame(MSG_TELEM_SNAPSHOT, reinterpret_cast<const uint8_t*>(&s), sizeof(Snapshot));
}

Command poll_cmd() {
  // Minimal skeleton parser for fixed-size command frames only.
  // Production: replace with full state-machine parser + CRC check + resync.
  Command out{};
  if (!g_serial || g_serial->available() < 12) return out;

  if (g_serial->read() != SOF1) return out;
  if (g_serial->read() != SOF2) return out;

  uint8_t ver = g_serial->read();
  uint8_t type = g_serial->read();
  (void)g_serial->read();  // seq lo
  (void)g_serial->read();  // seq hi
  uint8_t lenLo = g_serial->read();
  uint8_t lenHi = g_serial->read();
  uint16_t len = static_cast<uint16_t>(lenLo | (lenHi << 8));

  if (ver != VERSION || type != MSG_CMD || len != sizeof(CmdV1) || g_serial->available() < 4) {
    return out;
  }

  CmdV1 c{};
  g_serial->readBytes(reinterpret_cast<char*>(&c), sizeof(CmdV1));
  (void)g_serial->read();  // crc lo (TODO: validate)
  (void)g_serial->read();  // crc hi (TODO: validate)

  out.available = true;
  out.cmd = c.cmd;
  out.arg = c.arg;
  return out;
}

void send_cmd_ack(uint8_t cmd, bool ok, uint8_t err) {
  CmdAckV1 ack{cmd, static_cast<uint8_t>(ok ? 1 : 0), err};
  write_frame(MSG_CMD_ACK, reinterpret_cast<const uint8_t*>(&ack), sizeof(ack));
}

}  // namespace companion_uart
