// 2025, orthopteroid@gmail.com, BSDv2 license

#ifndef _SKIPJACK_H_
#define _SKIPJACK_H_

#include <cstdint>
#include <atomic>

#define LINESTR1(file, line) file ":" #line
#define LINESTR(file, line) LINESTR1(file, line)
#define FILE_LINE LINESTR(__FILE__, __LINE__)

namespace Skipjack
{

struct BUSConfig { int NSS, RESET, BUSY, IRQ; };

// safe from clock wrapping
inline bool CheckTimerExpired(uint32_t& eta, const uint32_t clock, const uint32_t period)
{
  const uint32_t period_complement = 0xFFFFFFFF - period;
  if(clock < eta) return false; // too soon
  if(clock - eta > period_complement) return false; // wrapped and too soon
  eta = clock + period;
  return true;
}

template<uint16_t SizePowerOf2>
struct RingBuffer
{
  const static uint16_t Mask = SizePowerOf2 -1;
  uint8_t buf[SizePowerOf2];
  std::atomic<uint16_t> writer = 0; // only writer need to be atomic, as it is used by both by writer and reader
  uint16_t reader = 0;
  void reset() { reader = writer = 0; }
  inline uint16_t count()
  { 
    int c = (writer & Mask) - (reader & Mask);
    return (c >= 0) ? c : c + SizePowerOf2;
  }
  inline uint8_t read() { return buf[(++reader) & Mask]; }
  inline void write(uint8_t t) { buf[(1 + writer++) & Mask] = t; }
  inline void fill(uint8_t* pt, uint16_t len)
  {
    // todo: optimize with either one or two memcpy?
    int j = writer +1;
    for(int i=0; i<len; i++) 
      buf[(j + i) & Mask] = *(pt++);
    writer += len;
  }
};

template<uint16_t Size>
struct DoubleBuffer
{
  uint8_t buf[Size << 1];
  std::atomic<uint8_t> data_length = 0;
  std::atomic<bool> w0r1 = true;
  inline uint8_t get_data_length() { return data_length; }
  inline uint8_t* get_read_buffer() { data_length = 0; return w0r1 ? (buf+Size) : (buf); }
  inline uint8_t* get_write_buffer() { return w0r1 ? (buf) : (buf+Size); }
  inline void write_complete(uint8_t len) { data_length = len; w0r1 = !w0r1; }
};

}; // namespace

#endif // _SKIPJACK_H_