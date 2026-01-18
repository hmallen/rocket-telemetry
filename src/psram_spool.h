#pragma once
#include <Arduino.h>

class PsramSpool {
public:
  PsramSpool() : buf_(nullptr), cap_(0), head_(0), tail_(0), drops_(0) {}

  void init(uint8_t* backing, uint32_t cap) {
    buf_ = backing;
    cap_ = cap;
    head_ = tail_ = 0;
    drops_ = 0;
  }

  uint32_t drops() const { return drops_; }

  uint32_t available() const {
    uint32_t h = head_, t = tail_;
    return (h >= t) ? (h - t) : (cap_ - (t - h));
  }

  uint32_t free_space() const {
    return cap_ - available() - 1;
  }

  bool write(const void* src, uint32_t n) {
    if (n > free_space()) { drops_++; return false; }
    const uint8_t* p = (const uint8_t*)src;
    uint32_t h = head_;
    uint32_t first = min(n, cap_ - h);
    memcpy(buf_ + h, p, first);
    if (n > first) memcpy(buf_, p + first, n - first);
    head_ = (h + n) % cap_;
    return true;
  }

  uint32_t read(void* dst, uint32_t n) {
    uint32_t avail = available();
    if (n > avail) n = avail;
    uint8_t* p = (uint8_t*)dst;
    uint32_t t = tail_;
    uint32_t first = min(n, cap_ - t);
    memcpy(p, buf_ + t, first);
    if (n > first) memcpy(p + first, buf_, n - first);
    tail_ = (t + n) % cap_;
    return n;
  }

private:
  uint8_t*  buf_;
  uint32_t  cap_;
  volatile uint32_t head_;
  volatile uint32_t tail_;
  volatile uint32_t drops_;
};
