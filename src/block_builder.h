#pragma once
#include <Arduino.h>
#include "records.h"

class BlockBuilder {
public:
  BlockBuilder();

  void reset(uint32_t seq, uint32_t t_start_us);
  uint8_t* payload_ptr();
  uint32_t payload_free() const;
  uint32_t payload_len() const;

  bool append_bytes(const uint8_t* data, uint32_t n);

  // Finalizes header (crc) into out_hdr and returns total bytes to write = sizeof(BlockHdr)+payload_len
  uint32_t finalize(BlockHdr& out_hdr);

private:
  uint32_t seq_;
  uint32_t t_start_us_;
  uint32_t used_;
  static constexpr uint32_t MAX_PAYLOAD = LOG_BLOCK_BYTES - sizeof(BlockHdr);
  uint8_t payload_[MAX_PAYLOAD];
};
