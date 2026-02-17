#include "uart_proto.h"

#include <string.h>

namespace companion_proto {

uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; ++i) {
    crc ^= static_cast<uint16_t>(data[i]) << 8;
    for (int b = 0; b < 8; ++b) {
      if (crc & 0x8000) {
        crc = (crc << 1) ^ 0x1021;
      } else {
        crc <<= 1;
      }
    }
  }
  return crc;
}

bool FrameParser::feed(uint8_t b, Frame& outFrame) {
  switch (state_) {
    case WAIT_SOF1:
      if (b == SOF1) state_ = WAIT_SOF2;
      return false;
    case WAIT_SOF2:
      state_ = (b == SOF2) ? READ_VER : WAIT_SOF1;
      return false;
    case READ_VER:
      frame_ = Frame{};
      frame_.version = b;
      state_ = READ_TYPE;
      return false;
    case READ_TYPE:
      frame_.type = b;
      state_ = READ_SEQ_LO;
      return false;
    case READ_SEQ_LO:
      frame_.seq = b;
      state_ = READ_SEQ_HI;
      return false;
    case READ_SEQ_HI:
      frame_.seq |= static_cast<uint16_t>(b) << 8;
      state_ = READ_LEN_LO;
      return false;
    case READ_LEN_LO:
      frame_.len = b;
      state_ = READ_LEN_HI;
      return false;
    case READ_LEN_HI:
      frame_.len |= static_cast<uint16_t>(b) << 8;
      if (frame_.len > MAX_PAYLOAD) {
        state_ = WAIT_SOF1;
        return false;
      }
      payloadIdx_ = 0;
      state_ = (frame_.len == 0) ? READ_CRC_LO : READ_PAYLOAD;
      return false;
    case READ_PAYLOAD:
      frame_.payload[payloadIdx_++] = b;
      if (payloadIdx_ >= frame_.len) state_ = READ_CRC_LO;
      return false;
    case READ_CRC_LO:
      crcRx_ = b;
      state_ = READ_CRC_HI;
      return false;
    case READ_CRC_HI: {
      crcRx_ |= static_cast<uint16_t>(b) << 8;

      uint8_t crcBuf[6 + MAX_PAYLOAD];
      crcBuf[0] = frame_.version;
      crcBuf[1] = frame_.type;
      crcBuf[2] = static_cast<uint8_t>(frame_.seq & 0xFF);
      crcBuf[3] = static_cast<uint8_t>((frame_.seq >> 8) & 0xFF);
      crcBuf[4] = static_cast<uint8_t>(frame_.len & 0xFF);
      crcBuf[5] = static_cast<uint8_t>((frame_.len >> 8) & 0xFF);
      memcpy(crcBuf + 6, frame_.payload, frame_.len);
      uint16_t calc = crc16_ccitt(crcBuf, 6 + frame_.len);

      state_ = WAIT_SOF1;
      if (calc != crcRx_ || frame_.version != VERSION) return false;

      outFrame = frame_;
      return true;
    }
  }

  state_ = WAIT_SOF1;
  return false;
}

size_t encodeFrame(const Frame& frame, uint8_t* out, size_t outCap) {
  const size_t total = 2 + 1 + 1 + 2 + 2 + frame.len + 2;
  if (outCap < total || frame.len > MAX_PAYLOAD) return 0;

  out[0] = SOF1;
  out[1] = SOF2;
  out[2] = frame.version;
  out[3] = frame.type;
  out[4] = static_cast<uint8_t>(frame.seq & 0xFF);
  out[5] = static_cast<uint8_t>((frame.seq >> 8) & 0xFF);
  out[6] = static_cast<uint8_t>(frame.len & 0xFF);
  out[7] = static_cast<uint8_t>((frame.len >> 8) & 0xFF);
  memcpy(out + 8, frame.payload, frame.len);

  uint16_t crc = crc16_ccitt(out + 2, 6 + frame.len);
  out[8 + frame.len] = static_cast<uint8_t>(crc & 0xFF);
  out[9 + frame.len] = static_cast<uint8_t>((crc >> 8) & 0xFF);
  return total;
}

const char* phaseToText(uint8_t phase) {
  switch (phase) {
    case 0: return "idle";
    case 1: return "pad";
    case 2: return "boost";
    case 3: return "coast";
    case 4: return "descent";
    case 5: return "landed";
    default: return "unknown";
  }
}

}  // namespace companion_proto
