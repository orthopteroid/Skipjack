// 2025, orthopteroid@gmail.com, BSDv2 license

#ifndef _SKIPJACKSX126X_H
#define _SKIPJACKSX126X_H

#include <functional>

#include "Skipjack.h"

namespace Skipjack
{

struct SX126xConfig
{
  float frequency_Mhz;
  uint8_t bw_idx, sf_value, cr_index;
  uint8_t header_type; // [S1] p41 p97, explicit == include LoRa header & variable length packet
  uint16_t preamble_syms, sync_word; // NOTE: sync_word low nibbles (0x_4_4) are masked out
  uint8_t crc_option;
  uint32_t rx_timeout_sym;
  int8_t tx_power_dbm;
  uint8_t tx_ramp_idx, iq_option;
  bool gain_boosted, cad_moresensitive, osc_tcxo;

  constexpr static uint32_t bandwidth_hz[11] = {
    7'800, 15'600, 31'250, 62'500, 125'000, 250'000, 500'000, 0, 10'400, 20'800, 41'700
  };

  // returns false if error
  bool ChangeConfig(const char* config); // ie "long-fast" or "short-turbo"

  bool LDORecomended() const;
  uint32_t msTimeOnAir(uint8_t len) const;
};

struct SX126xImpl;

struct SX126x
{
  SX126x(BUSConfig bus, std::function<void(char*)> printlnFn);

  void begin(SX126xConfig rc);

  DoubleBuffer<256>& get_packet_buf();
  RingBuffer<32>& get_status_ring();

  // Hide the actual implementation from the calling code.
  // This makes the members here the 'interface' for other possible
  // implementations/radios without having to use structural tricks that
  // have conceptual or call-time consequences.
  SX126xImpl* pImpl;
};

}; // namespace

#endif //_SKIPJACKSX126X_H
