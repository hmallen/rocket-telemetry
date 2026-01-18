#include "block_builder.h"
#include "crc32.h"
#include "cfg.h"

BlockBuilder::BlockBuilder() : seq_(0), t_start_us_(0), used_(0) {}

void BlockBuilder::reset(uint32_t seq, uint32_t t_start_us) {
  seq_ = seq;
  t_start_us_ = t_start_us;
  used_ = 0;
}

uint8_t* BlockBuilder::payload_ptr() { return payload_; }
uint32_t BlockBuilder::payload_free() const { return MAX_PAYLOAD - used_; }
uint32_t BlockBuilder::payload_len() const { return used_; }

bool BlockBuilder::append_bytes(const uint8_t* data, uint32_t n) {
  if (n > payload_free()) return false;
  memcpy(payload_ + used_, data, n);
  used_ += n;
  return true;
}

uint32_t BlockBuilder::finalize(BlockHdr& out_hdr) {
  out_hdr.magic = BLOCK_MAGIC;
  out_hdr.ver = 1;
  out_hdr.hdr_len = sizeof(BlockHdr);
  out_hdr.seq = seq_;
  out_hdr.t_start_us = t_start_us_;
  out_hdr.payload_len = used_;
  out_hdr.crc32 = crc32_compute(payload_, used_);
  return sizeof(BlockHdr) + used_;
}
