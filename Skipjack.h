// 2025, orthopteroid@gmail.com, BSDv2 license

#ifndef _SKIPJACK_H_
#define _SKIPJACK_H_

#include <cstdint>

#define LINESTR1(file, line) file ":" #line
#define LINESTR(file, line) LINESTR1(file, line)
#define FILE_LINE LINESTR(__FILE__, __LINE__)

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

template<typename BufType, typename IndexType, uint16_t MaskSize>
struct RingBuffer
{
  const static uint16_t Size = 1 << MaskSize;
  const static uint16_t Mask = Size -1;
  BufType buf[Size];
  IndexType reader = 0, writer = 0;
  void reset() { reader = writer = 0; }
  inline uint16_t count()
  { 
    int c = (writer & Mask) - (reader & Mask);
    return (c >= 0) ? c : c + Size;
  }
  inline BufType read() { return buf[(++reader) & Mask]; }
  inline void write(BufType t) { buf[(1 + writer++) & Mask] = t; }
  inline void fill(BufType* pt, uint16_t len)
  {
    // todo: optimize with either one or two memcpy
    int j = writer +1;
    for(int i=0; i<len; i++) 
      buf[(j + i) & Mask] = *(pt++);
    writer += len;
  }
};

#endif // _SKIPJACK_H_