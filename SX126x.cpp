// 2025, orthopteroid@gmail.com, BSDv2 license

#include <Arduino.h>
#include <pins_arduino.h>
#include <SPI.h>
#include <atomic>

#include "Skipjack.h"
#include "SX126x.h"
#include "SX126x_internal.h"

////////////////////////////////
// [S1] Semtech DS.SX1261-2.W.APP (r2.2) "60852689.DS_SX1261_2 V2-2.pdf"
// [S2] Semtech AN1200.48 "SX126X CAD performance evaluation V2.1-1.pdf"
// [S3] "Semtech DS_SX1276-7-8-9_W_APP_V7.pdf"

// main documentation links
// https://heltec.org/project/wifi-lora-32-v3/
// https://www.semtech.com/products/wireless-rf/lora-connect/sx1262

// ISR and application structure based upon (BSDv2 license)
// [A1] https://github.com/antirez/freakwan/blob/main/techo-port/radio.ino

// meshtastic config
// [M1] https://github.com/meshtastic
// [M2] https://meshtastic.org/docs/overview/
// [M3] https://deepwiki.com/meshtastic/meshtastic/1.2-mesh-network-architecture
// [M4] https://github.com/meshtastic/firmware/blob/master/src/mesh/RadioInterface.cpp

// prior art on programming the chip
// https://github.com/gereic/GXAirCom/blob/master/lib/FANETLORA/radio/LoRa.cpp
// https://github.com/Lora-net/SWL2001/blob/master/lbm_lib/smtc_modem_core/radio_drivers/sx126x_driver/src/sx126x.c
// https://github.com/ivan-molnar/drivers/blob/master/sx126x/sx126x.go
// https://github.com/meshtastic/firmware/blob/master/src/mesh/SX126xInterface.cpp
// https://github.com/jgromes/RadioLib/blob/master/src/modules/SX126x/SX126x.cpp
// https://os.mbed.com/teams/Semtech/code/SX126xLib//file/1e2345700991/sx126x.cpp/

#define STATUS_DEBUG() \
  do { \
    Status_Debug(const_cast<char*>(FILE_LINE)); \
  } while(false)

#define ASSERT(t) \
  do { \
    if(!(t)) { \
      if(debug_printlnFn) \
        debug_printlnFn(const_cast<char*>(FILE_LINE)); \
      delay(100); while(true) ; \
    } \
  } while(false)

#define ASSERT_STATUS_OK() \
  do { \
    if(!Status_OK() ) { \
      if(debug_printlnFn) \
        debug_printlnFn(const_cast<char*>(FILE_LINE)); \
      delay(100); while(true) ; \
    } \
  } while(false)

///////////////////////////////////

namespace Skipjack
{

const static char hexchar[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

struct CADConfig { uint8_t sym_idx, detmin, detpeak; };

// cad settings according to SF. CAD unavailable for SF < 7. [S2] p15, p41.
const static CADConfig cadHighPowerTable[] =
{
  // detection efficient configuration
  {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, 
  /*SF7*/  {SX126X_CAD_ON_2_SYMB,10,22}, 
  /*SF8*/  {SX126X_CAD_ON_2_SYMB,10,22},
  /*SF9*/  {SX126X_CAD_ON_4_SYMB,10,23},
  /*SF10*/ {SX126X_CAD_ON_4_SYMB,10,24},
  /*SF11*/ {SX126X_CAD_ON_4_SYMB,10,25},
  /*SF12*/ {SX126X_CAD_ON_4_SYMB,10,28}
};
const static CADConfig cadLowPowerTable[] = 
{
  // power efficient configuration
  {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, 
  /*SF7*/  {SX126X_CAD_ON_2_SYMB,10,22},
  /*SF8*/  {SX126X_CAD_ON_2_SYMB,10,22},
  /*SF9*/  {SX126X_CAD_ON_2_SYMB,10,24},
  /*SF10*/ {SX126X_CAD_ON_2_SYMB,10,25},
  /*SF11*/ {SX126X_CAD_ON_2_SYMB,10,26},
  /*SF12*/ {SX126X_CAD_ON_2_SYMB,10,30}
};

// ms per symbol in 64ths, by bw_idx X sf_value.
// ie int(64*1000*(1<<SF)/HZ)
const static uint16_t msPerSymTable_64ths[] =
{
  /*SF0*/  8,4,2,1,1,0,0,0,6,3,2,
  /*SF1*/  16,8,4,2,1,1,0,0,12,6,3,
  /*SF2*/  33,16,8,4,2,1,1,0,25,12,6,
  /*SF3*/  66,33,16,8,4,2,1,0,49,25,12,
  /*SF4*/  131,66,33,16,8,4,2,0,98,49,25,
  /*SF5*/  263,131,66,33,16,8,4,0,197,98,49,
  /*SF6*/  525,263,131,66,33,16,8,0,394,197,98,
  /*SF7*/  1050,525,262,131,66,33,16,0,788,394,196,
  /*SF8*/  2101,1050,524,262,131,66,33,0,1575,788,393,
  /*SF9*/  4201,2101,1049,524,262,131,66,0,3151,1575,786,
  /*SF10*/ 8402,4201,2097,1049,524,262,131,0,6302,3151,1572,
  /*SF11*/ 16804,8402,4194,2097,1049,524,262,0,12603,6302,3143,
  /*SF12*/ 33608,16804,8389,4194,2097,1049,524,0,25206,12603,6286
};

// symbols per bit in 4096ths, by spreading factor.
// ie. int(4096/SF)
const static uint16_t symPerBitTable_4096ths[] = {
  /*SF0  SF1   SF2  SF3   SF4   SF5  SF6  SF7  SF8  SF9 SF10 SF11 SF12*/
     0, 4096, 2048, 1365, 1024, 819, 682, 585, 512, 455, 410, 372, 341
};

////////////////////////////////////

struct SX126xImpl
{
  // clock can run up to 16Mhz. [S1] p53
  const SPISettings spiSettings = {12'000'000, MSBFIRST, SPI_MODE0};

  const uint16_t RadioIRQMask = 
    SX126X_IRQ_PREAMBLE_DETECTED | // 1st stage of packet detection. manually start a timer and wait
    SX126X_IRQ_HEADER_VALID |      // 2nd stage of packet detection. wait a bit longer
    SX126X_IRQ_HEADER_ERR |        // 2nd stage (alternate). give up on current packet.
    SX126X_IRQ_RX_DONE |           // rx complete. need to extract
    SX126X_IRQ_CRC_ERR |           // packet bad. extract it but toss it
    SX126X_IRQ_TX_DONE;            // tx complete. now back to rx mode

  // quirky status layout. note shofting required to compare with #defines. [S1] p100
  union {
    struct {
      uint8_t bit0: 1;
      uint8_t cmd: 3; // SX126X_STATUS_<others> require >> 1
      uint8_t chip: 3; // SX126X_STATUS_MODE_... require >> 4
      uint8_t bit7: 1;
    } status;
    uint8_t status_byte = 0;
  };
  uint16_t status_irq = 0;
  uint16_t status_err = 0;
  uint8_t status_rssi = 0;

  BUSConfig bus;
  SX126xConfig rc;
  CADConfig cad; // runtime config dependant upon radio settings

  /* Variables used to detect "busy channel" condition for Listen Before Talk. */
  volatile std::atomic<uint32_t> isr_ms_preamble_start = 0;
  volatile std::atomic<bool> isr_header_valid = false;

  // fixed length buffers, long enough for...
  RingBuffer<32> isr_status_ring; // ... a reasonable number of status messages
  DoubleBuffer<256> isr_packet_buf;

  enum RadioStates {
      RadioStateStandby,
      RadioStateRx,
      RadioStateTx,
      RadioStateSleep
  };
  RadioStates isr_state = RadioStateStandby;

  std::function<void(char*)> debug_printlnFn;
  char debug_printf_buf[80];

  /////////////////

  SX126xImpl(BUSConfig bus, std::function<void(char*)> debug_printlnFn_);
  void begin(SX126xConfig rc);

  void Status_Debug(char* context);
  inline bool Status_OK() { return ((status_err == 0) && (status.cmd != (SX126X_STATUS_CMD_FAILED >> 1))); }

  void SPIBegin();
  void SPIEnd();
  void Errata_15_1_2();
  void Errata_15_3_2();
  void Errata_15_4_2();

  void SX126xISR();
};

static SX126xImpl *pInstance = 0; // singleton

static void isr_trampoline()
{
  pInstance->SX126xISR();
}

/////////////

SX126xImpl::SX126xImpl(BUSConfig bus_, std::function<void(char*)> debug_printlnFn_)
{
  bus = bus_;
  debug_printlnFn = debug_printlnFn_;
  pInstance = this;

  // to use CAD with fixed length packets air-time calculation is required
  // [S1] p98, p118
  cad = rc.cad_moresensitive ? cadHighPowerTable[rc.sf_value] : cadLowPowerTable[rc.sf_value];

  digitalWrite(bus.NSS, 1);   pinMode(bus.NSS, OUTPUT);
  digitalWrite(bus.RESET, 1); pinMode(bus.RESET, OUTPUT);
  pinMode(bus.BUSY, INPUT);
  pinMode(bus.IRQ, INPUT);
}

void SX126xImpl::begin(SX126xConfig rc_)
{
  rc = rc_;

  ////////////////////////////////
  // SX126x initialization
  // All this seems to be very order sensitive. And while some orderings are specified in [S1]
  // some are not: for instance doc says use SX126X_CMD_CLEAR_DEVICE_ERRORS to clear
  // SX126X_XOSC_START_ERR but doc doen't this causes a SX126X_STATUS_CMD_FAILED. However, we
  // can prevent a startup SX126X_STATUS_CMD_FAILED by starting with SX126X_CMD_SET_DIO3_AS_TCXO_CTRL
  // and SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL and then clearing the SX126X_XOSC_START_ERR with
  // the SX126X_CMD_CLEAR_DEVICE_ERRORS.

  digitalWrite(bus.RESET, 0); delay(100); digitalWrite(bus.RESET, 1); delay(100);
  digitalWrite(bus.RESET, 0); delay(100); digitalWrite(bus.RESET, 1); delay(100);

  while(digitalRead(bus.BUSY)) delay(1); // startup. [S1] p60

  // needed if TCXO is fitted. [S1] p61
  uint32_t ms64_tcxo = RADIO_TCXO_SETUP_TIME << 6;
  SPIBegin();
    SPI.write(SX126X_CMD_SET_DIO3_AS_TCXO_CTRL);
    SPI.write(SX126X_DIO3_OUTPUT_1_8);
    SPI.write((uint8_t)(ms64_tcxo >> 16));
    SPI.write((uint8_t)(ms64_tcxo >> 8));
    SPI.write((uint8_t)ms64_tcxo);
  SPIEnd();
  delay(RADIO_TCXO_SETUP_TIME << 1); // delay cpu as well so we don't probe before it's started
  STATUS_DEBUG();
  ASSERT(status.cmd != (SX126X_STATUS_CMD_FAILED >> 1)); // ignore status_err & SX126X_XOSC_START_ERR

  SPIBegin();
    SPI.write(SX126X_CMD_SET_DIO2_AS_RF_SWITCH_CTRL);
    SPI.write(SX126X_DIO2_AS_RF_SWITCH);
  SPIEnd();
  STATUS_DEBUG();
  ASSERT(status.cmd != (SX126X_STATUS_CMD_FAILED >> 1)); // ignore status_err & SX126X_XOSC_START_ERR

  // [S1] p87, normal error/warning that TCXO won't start
  if(status_err & SX126X_XOSC_START_ERR)
  {
    SPIBegin();
      SPI.transfer(SX126X_CMD_CLEAR_DEVICE_ERRORS);
      SPI.transfer(SX126X_CMD_NOP); // ignore return code for status commands
      SPI.transfer(SX126X_CMD_NOP);
    SPIEnd();
    STATUS_DEBUG();
  }
  ASSERT_STATUS_OK();

  ////////////////////////////////
  // power related
  // order important: PAConfig will change the OCP.

  // power amplifier. table specific to SX1262 only?
  struct PAConfig { uint8_t dutycycle; uint8_t hpmax; } pac = {0,0};
  if      (rc.tx_power_dbm <= -22) pac = { 0x04, 0x07 /*SX126X_PA_CONFIG_HP_MAX*/ };
  else if (rc.tx_power_dbm <= -20) pac = { 0x03, 0x05 };
  else if (rc.tx_power_dbm <= -17) pac = { 0x02, 0x03 };
  else                                pac = { 0x02, 0x02 };
  SPIBegin();
    SPI.write(SX126X_CMD_SET_PA_CONFIG); // [S1] p81-82
    SPI.write(pac.dutycycle);
    SPI.write(pac.hpmax);
    SPI.write(1); // 0=SX1262, 1=SX1261
    SPI.write(SX126X_PA_CONFIG_PA_LUT);
  SPIEnd();
  STATUS_DEBUG();
  ASSERT_STATUS_OK();

  struct TXLimit { int8_t mn; int8_t mx; } txLimitTable[] = {{-9,+22}, {-17,+14}};
  TXLimit txLimit = txLimitTable[0]; // 0=SX1262, 1=SX1261
  SPIBegin();
    SPI.write(SX126X_CMD_SET_TX_PARAMS); // [S1] p89
    SPI.write(max(txLimit.mn, min(txLimit.mx,rc.tx_power_dbm)));
    SPI.write(rc.tx_ramp_idx);
  SPIEnd();
  STATUS_DEBUG();
  ASSERT_STATUS_OK();

  // Over Current Protection (OCP) register
  // POR default is 60mA but OCP limits are SX1262=140mA=0x38 and SX1261=60mA=0x18
  // SX126X_CMD_SET_PA_CONFIG resets the OCP so this must be set if something other than default is wanted
  // "...  the default value is re-configured automatically each time the function SetPaConfig(...) is
  // called. To adjust the OCP value, it is necessary to change the register as a second step after
  // calling the function SetPaConfig(...)."
  SPIBegin();
    SPI.transfer(SX126X_CMD_WRITE_REGISTER);
    /*status_byte*/SPI.transfer16(SX126X_REG_OCP_CONFIGURATION); // [S1] p34
    status_byte = SPI.transfer(0x18);
  SPIEnd();
  STATUS_DEBUG();
  ASSERT_STATUS_OK();

  ////////////////////////////////
  // oscillator related

  ASSERT(
    (status.chip == 0) || // WAKE mode?
    (status.chip == (SX126X_STATUS_MODE_STDBY_RC >> 4)) // RC mode
  );

  if(rc.osc_tcxo)
  {
    SPIBegin();
      SPI.write(SX126X_CMD_SET_REGULATOR_MODE); // [S1] p34, p80
      SPI.write(rc.osc_tcxo ? SX126X_REGULATOR_DC_DC : SX126X_REGULATOR_LDO);
    SPIEnd();
    STATUS_DEBUG();
    ASSERT_STATUS_OK();

    // seems to be required before SX126X_CMD_SET_STANDBY/SX126X_STANDBY_XOSC
    // otherwise chip drops back into SX126X_STANDBY_RC
    SPIBegin();
      SPI.write(SX126X_CMD_CALIBRATE); // [S1] p40, p90
      SPI.write(0x7F); // calibrate all blocks
    SPIEnd();
    STATUS_DEBUG();
    ASSERT_STATUS_OK();
  }

  // POR defaults to calibration on 900-928mhz
  if(rc.frequency_Mhz < 900.0f)
  {
    uint8_t f1 = 0, f2 = 0;
    if(      rc.frequency_Mhz > 900.0f ) { f1 = 0xE1; f2 = 0xE9; }
    else if( rc.frequency_Mhz > 850.0f ) { f1 = 0xD7; f2 = 0xD8; }
    else if( rc.frequency_Mhz > 770.0f ) { f1 = 0xC1; f2 = 0xC5; }
    else if( rc.frequency_Mhz > 460.0f ) { f1 = 0x75; f2 = 0x81; }
    else if( rc.frequency_Mhz > 425.0f ) { f1 = 0x6B; f2 = 0x6F; }
    SPIBegin();
      SPI.write(SX126X_CMD_CALIBRATE_IMAGE); // [S1] p61
      SPI.write(f1);
      SPI.write(f2);
    SPIEnd();
    STATUS_DEBUG();
    ASSERT_STATUS_OK();
  }

  SPIBegin();
    SPI.write(SX126X_CMD_SET_STANDBY); // [S1] p73
    SPI.write(rc.osc_tcxo ? SX126X_STANDBY_XOSC : SX126X_STANDBY_RC);
  SPIEnd();
  STATUS_DEBUG();
  ASSERT_STATUS_OK();

  // seems to read reliability after osc is configured
  uint8_t version[17] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
  SPIBegin();
    SPI.transfer(SX126X_CMD_READ_REGISTER);
    /*status_byte =*/SPI.transfer16(SX126X_REG_VERSION_STRING);
    status_byte = SPI.transfer(SX126X_CMD_NOP);
    /*void*/SPI.transferBytes(0, version, 16); // optimized transfer
  SPIEnd();
  STATUS_DEBUG();
  ASSERT_STATUS_OK();
  snprintf(debug_printf_buf,80,"chip \"%s\"\n",version);
  Serial.printf(debug_printf_buf);

  //////////////////////////////
  // ideally located after any XOSC reconfig

  // additional steps would be required when using SetRxDutyCycle?
  if(rc.gain_boosted)
  {
    SPIBegin();
      SPI.transfer(SX126X_CMD_WRITE_REGISTER);
      /*status_byte*/SPI.transfer16(SX126X_REG_RX_GAIN); // [S1] p48, p62
      status_byte = SPI.transfer(rc.gain_boosted ? 0x96 : 0x94);
    SPIEnd();
    STATUS_DEBUG();
    ASSERT_STATUS_OK();
  }

  //////////////////////////
  // LORA. [S1] p106

  SPIBegin();
    SPI.write(SX126X_CMD_SET_PACKET_TYPE);
    SPI.write(SX126X_PACKET_TYPE_LORA);
  SPIEnd();
  SPIBegin();
    SPI.write(SX126X_CMD_SET_RF_FREQUENCY);
    SPI.write32(rc.frequency_Mhz * ((float)(1 << 25) / 32.0f)); // FREQ_DIV_EXP and XTAL_FREQ are SX126x specific
  SPIEnd();
  if(rc.iq_option == SX126X_LORA_IQ_INVERTED)
    Errata_15_4_2();
  SPIBegin();
    SPI.write(SX126X_CMD_SET_MODULATION_PARAMS); // [S1] p90
    SPI.write(rc.sf_value);
    SPI.write(rc.bw_idx);
    SPI.write(rc.cr_index);
    SPI.write(rc.LDORecomended() ? 0x01 : 0x00); // [S1] p40
  SPIEnd();
  if(rc.bw_idx == SX126X_LORA_BW_500_0)
    Errata_15_1_2();
  // SX126X_CMD_SET_PACKET_PARAMS must be set below prior to rx/tx

  /////////////////////
  // commands that have doubious explanation

  // likely unnecessary as we're not changing modes from what we set with SX126X_CMD_SET_STANDBY
  // SPIBegin();
  //   SPI.write(SX126X_CMD_SET_RX_TX_FALLBACK_MODE); // [S1] p82
  //   SPI.write(rc.osc_tcxo ? SX126X_RX_TX_FALLBACK_MODE_STDBY_XOSC : SX126X_RX_TX_FALLBACK_MODE_STDBY_RC);
  // SPIEnd();

  // likely firmware bug. set to 0 or don't use
  // https://community.st.com/t5/stm32-mcus-wireless/lora-rx-timeout-triggers-immediately-in-non-continuous-receive/td-p/666360
  // SPIBegin();
  //   SPI.write(SX126X_CMD_SET_LORA_SYMB_NUM_TIMEOUT); // [S1] p99
  //   SPI.write(0);
  // SPIEnd();

  /////////////////////

  SPIBegin();
    SPI.transfer(SX126X_CMD_WRITE_REGISTER);
    /*status_byte*/SPI.transfer16(SX126X_REG_LORA_SYNC_WORD_MSB); // MSB at lower address
    /*status_byte*/SPI.transfer16(rc.sync_word); // [S1] p70: public=0x3444, private=0x1424, limited to 0x?4?4
  SPIEnd();
  SPIBegin();
    SPI.transfer(SX126X_CMD_READ_REGISTER);
    /*status_byte =*/SPI.transfer16(SX126X_REG_LORA_SYNC_WORD_MSB);
    status_byte = SPI.transfer(SX126X_CMD_NOP);
    uint16_t rSyncWord = SPI.transfer16(SX126X_CMD_NOP);
  SPIEnd();
  STATUS_DEBUG();
  ASSERT(rSyncWord == rc.sync_word);
  ASSERT_STATUS_OK();
  snprintf(debug_printf_buf,80,"syncword %04X\n",rSyncWord);
  Serial.printf(debug_printf_buf);

  Serial.printf("setup complete\n");

  // begin antirez application pattern [A1]

  ASSERT( (status.chip == (SX126X_STATUS_MODE_STDBY_RC >> 4)) || (status.chip == (SX126X_STATUS_MODE_STDBY_XOSC >> 4)) );

  // for this application design we're not using the rx timeout, so leav this as defulat
  SPIBegin();
    SPI.write(SX126X_CMD_STOP_TIMER_ON_PREAMBLE); // [S1] p76
    SPI.write(0); // 0=stop on Sync Word or Header (default [S1] p75), 1=stop on Preamble
  SPIEnd();
  SPIBegin();
    SPI.write(SX126X_CMD_SET_BUFFER_BASE_ADDRESS);
    SPI.write(0); // txBaseAddress
    SPI.write(0); // rxBaseAddress
  SPIEnd();
  // packet size/length can be wrong if using SX126X_CMD_SET_BUFFER_BASE_ADDRESS when not in STANDBY mode
  // so, having a clean buffer can help debug problems like that. possibly others.
  // SPIBegin();
  //   /*ruf*/SPI.transfer(SX126X_CMD_WRITE_BUFFER);
  //   /*status_byte =*/ SPI.transfer(0);
  //   for(uint16_t i = 0; i < 256 >> 3; i++)
  //     SPI.write32(0);
  // SPIEnd();
  STATUS_DEBUG();
  ASSERT_STATUS_OK();

#if 1

  isr_state = RadioStateRx;
  SPIBegin();
    SPI.write(SX126X_CMD_SET_PACKET_PARAMS); // [S1] p97
    SPI.write16(rc.preamble_syms);
    SPI.write(rc.header_type);
    SPI.write(0xFF); // max packet size in RX mode.
    SPI.write(rc.crc_option);
    SPI.write(rc.iq_option);
  SPIEnd();
  SPIBegin();
    SPI.write(SX126X_CMD_SET_RX); // [S1] p75
    SPI.write((uint8_t)(rc.rx_timeout_sym >> 16));
    SPI.write((uint8_t)(rc.rx_timeout_sym >> 8));
    SPI.write((uint8_t)rc.rx_timeout_sym); // nb: values not SX126X_RX_TIMEOUT_INF should be << 6
  SPIEnd();
  STATUS_DEBUG();
  ASSERT_STATUS_OK();

#else

  // for future testing
  isr_state = RadioStateTx;
  SPIBegin();
    SPI.write(SX126X_CMD_SET_PACKET_PARAMS); // [S1] p97
    SPI.write16(rc.preamble_syms);
    SPI.write(rc.header_type);
    SPI.write(10);
    SPI.write(rc.crc_option);
    SPI.write(rc.iq_option);
  SPIEnd();
  SPIBegin();
    SPI.transfer(SX126X_CMD_WRITE_BUFFER);
    SPI.transfer(0); // device buffer addres to tx from
    for(uint16_t i = 0; i < 10; i++)
      /*status_byte =*/ SPI.transfer(0xFF); // 0xFF is placeholder
  SPIEnd();
  uint32_t ms64_tx = rc.msTimeOnAir(10) << 6;
  SPIBegin();
    SPI.write(SX126X_CMD_SET_TX); // [S1] p74
    SPI.write((uint8_t)(ms64_tx >> 16));
    SPI.write((uint8_t)(ms64_tx >> 8));
    SPI.write((uint8_t)ms64_tx);
  SPIEnd();
  STATUS_DEBUG();
  ASSERT_STATUS_OK();

#endif

  gpio_install_isr_service((int)ESP_INTR_FLAG_IRAM);
  gpio_set_intr_type((gpio_num_t)bus.IRQ, GPIO_INTR_POSEDGE); // rising edge
  gpio_isr_handler_add((gpio_num_t)bus.IRQ, (void (*)(void*))isr_trampoline, NULL);

  SPIBegin();
    SPI.write(SX126X_CMD_SET_DIO_IRQ_PARAMS); // [S1] p84
    SPI.write16(RadioIRQMask); // irq
    SPI.write16(RadioIRQMask); // dio1
    SPI.write16(SX126X_IRQ_NONE);
    SPI.write16(SX126X_IRQ_NONE);
  SPIEnd();
}

void SX126xImpl::Status_Debug(char* context)
{
  if(!debug_printlnFn) return;
  SPIBegin();
    SPI.transfer(SX126X_CMD_GET_DEVICE_ERRORS);
    status_byte = SPI.transfer(SX126X_CMD_NOP);
    status_err = SPI.transfer16(SX126X_CMD_NOP);
  SPIEnd();
  snprintf(debug_printf_buf,80,"status%X%X err%03X %s",status.chip,status.cmd,status_err,context);
  debug_printlnFn(debug_printf_buf);
}

void SX126xImpl::SPIBegin()
{
  while(digitalRead(bus.BUSY));
  digitalWrite(bus.NSS, LOW);
  SPI.beginTransaction(spiSettings);
}
void SX126xImpl::SPIEnd()
{
  SPI.endTransaction();
  digitalWrite(bus.NSS, HIGH);
  delayMicroseconds(1); /*settle time actually 600ns. [S1] p55 */
  while(digitalRead(bus.BUSY));
}

void SX126xImpl::Errata_15_1_2()
{
  // [S1] p83. when LORA BW is 500khz call this to improve TX modulation quality 
  SPIBegin();
    SPI.transfer(SX126X_CMD_READ_REGISTER);
    /*status_byte =*/SPI.transfer16(SX126X_REG_SENSITIVITY_CONFIG);
    /*status_byte =*/SPI.transfer(SX126X_CMD_NOP);
    uint8_t r0889 = SPI.transfer(SX126X_CMD_NOP);
  SPIEnd();
  SPIBegin();
    SPI.transfer(SX126X_CMD_WRITE_REGISTER);
    /*status_byte*/SPI.transfer16(SX126X_REG_SENSITIVITY_CONFIG);
    /*status_byte*/SPI.transfer(r0889 & 0xFB);
  SPIEnd();
}

void SX126xImpl::Errata_15_3_2()
{
  // [S1] p111. when using implicit headers and after receiving rx_done, RX timeout needs to be manually stopped
  // stop count_debuger
  SPIBegin();
    SPI.transfer(SX126X_CMD_WRITE_REGISTER);
    /*status_byte*/SPI.transfer16(SX126X_REG_DIO3_OUT_VOLTAGE_CTRL);
    /*status_byte*/SPI.transfer(0x00);
  SPIEnd();
  // clear event
  SPIBegin();
    SPI.transfer(SX126X_CMD_READ_REGISTER);
    /*status_byte =*/SPI.transfer16(SX126X_REG_EVENT_MASK);
    /*status_byte =*/SPI.transfer(SX126X_CMD_NOP);
    uint8_t r0944 = SPI.transfer(SX126X_CMD_NOP);
  SPIEnd();
  SPIBegin();
    SPI.transfer(SX126X_CMD_WRITE_REGISTER);
    /*status_byte*/SPI.transfer16(SX126X_REG_EVENT_MASK);
    /*status_byte*/SPI.transfer(r0944 | 0x02);
  SPIEnd();
}

void SX126xImpl::Errata_15_4_2()
{
  // [S1] p111. When exchanging LoRa packets with inverted IQ polarity, some packet losses
  // may be observed for longer packets.
  SPIBegin();
    SPI.transfer(SX126X_CMD_READ_REGISTER);
    /*status_byte =*/SPI.transfer16(SX126X_REG_IQ_CONFIG);
    /*status_byte =*/SPI.transfer(SX126X_CMD_NOP);
    uint8_t r0736 = SPI.transfer(SX126X_CMD_NOP);
  SPIEnd();
  if(rc.iq_option == SX126X_LORA_IQ_INVERTED) r0736 |= 0x04; else r0736 &= 0xFB;
  SPIBegin();
    SPI.transfer(SX126X_CMD_WRITE_REGISTER);
    /*status_byte*/SPI.transfer16(SX126X_REG_IQ_CONFIG);
    /*status_byte*/SPI.transfer(r0736);
  SPIEnd();
}

void SX126xImpl::SX126xISR()
{
  // interrupt service routine. per. antirez freakwan [A1] (BSDv2). C-style comments by antirez.
  SPIBegin();
    SPI.transfer(SX126X_CMD_GET_IRQ_STATUS);
    status_byte = SPI.transfer(SX126X_CMD_NOP);
    status_irq = SPI.transfer16(SX126X_CMD_NOP);
  SPIEnd();
  // isr_status_ring.write('0' + status.chip);
  // isr_status_ring.write('0' + status.cmd);
  if(status_irq & SX126X_IRQ_TX_DONE)            isr_status_ring.write('T');
  if(status_irq & SX126X_IRQ_RX_DONE)            isr_status_ring.write('R');
  if(status_irq & SX126X_IRQ_PREAMBLE_DETECTED)  isr_status_ring.write('P');
  if(status_irq & SX126X_IRQ_HEADER_VALID)       isr_status_ring.write('V');
  if(status_irq & SX126X_IRQ_CRC_ERR)            isr_status_ring.write('C'); // packet crc err [1] p58
  if(status_irq & SX126X_IRQ_HEADER_ERR)         isr_status_ring.write('E'); // header crc error [1] p58

  if (status_irq & SX126X_IRQ_RX_DONE)
  {
    /* Reset the SX1262 state to receive the next packet: note that
      * it will stay in RX mode, since we initialized the chip with
      * "infinite" timeout. */
    isr_ms_preamble_start = 0;
    isr_header_valid = false;
    //
    // if SX126X_CMD_SET_BUFFER_BASE_ADDRESS were used we'd have to switch into STANDBY and back
    // into RX modes. This mode-switching time costs: as much as 126us+62us [S1] p56. At high
    // bandwidths this can be a few LoRa symbols. Instead, we can always extract bad payloads,
    // we just won't queue them.
    SPIBegin();
      SPI.transfer(SX126X_CMD_GET_RX_BUFFER_STATUS); // [S1] p101
      /*status_byte =*/ SPI.transfer(SX126X_CMD_NOP);
      uint8_t packet_length = SPI.transfer(SX126X_CMD_NOP);
      uint8_t packet_start = SPI.transfer(SX126X_CMD_NOP);
    SPIEnd();
    // isr_status_ring.write('|');
    // isr_status_ring.write(hexchar[ packet_start >> 4 ]);
    // isr_status_ring.write(hexchar[ packet_start & 0x0F ]);
    SPIBegin();
      /*ruf*/SPI.transfer(SX126X_CMD_READ_BUFFER);
      /*status_byte =*/ SPI.transfer(packet_start);
      SPI.transfer(SX126X_CMD_NOP); // noop required
      SPI.transferBytes(0, isr_packet_buf.get_write_buffer(), packet_length);
    SPIEnd();
    if(!(status_irq & SX126X_IRQ_CRC_ERR))
    {
      isr_status_ring.write('|');
      isr_status_ring.write(hexchar[ packet_length >> 4 ]);
      isr_status_ring.write(hexchar[ packet_length & 0x0F ]);
      isr_packet_buf.write_complete(packet_length);
    }
  }
  else if (status_irq & SX126X_IRQ_TX_DONE)
  {
    isr_state = RadioStateRx;
    isr_ms_preamble_start = 0;
    isr_header_valid = false;
    //
    if(rc.bw_idx == SX126X_LORA_BW_500_0)
      Errata_15_1_2();
    SPIBegin();
      SPI.write(SX126X_CMD_SET_PACKET_PARAMS); // [S1] p97
      SPI.write16(rc.preamble_syms);
      SPI.write(rc.header_type);
      SPI.write(0xFF); // max packet size in RX mode.
      SPI.write(rc.crc_option);
      SPI.write(rc.iq_option);
    SPIEnd();
    SPIBegin();
      SPI.write(SX126X_CMD_SET_RX); // [S1] p75
      SPI.write((uint8_t)(rc.rx_timeout_sym >> 16));
      SPI.write((uint8_t)(rc.rx_timeout_sym >> 8));
      SPI.write((uint8_t)rc.rx_timeout_sym); // nb: values not SX126X_RX_TIMEOUT_INF should be << 6
    SPIEnd();
  }
  else if (status_irq & SX126X_IRQ_PREAMBLE_DETECTED)
  {
    /* In order to know if the radio is busy receiving, and avoid starting
      * a transmission while a packet is on the air, we remember if we
      * are receiving some packet right now, and at which time the
      * preamble started. */
    isr_ms_preamble_start = millis();
    isr_header_valid = false;
  }
  else if (status_irq & SX126X_IRQ_HEADER_VALID)
  {
    /* After the preamble, the LoRa radio may also detect that the
      * packet has a good looking header. We set this state, since, 
      * in this case, we are willing to wait for a larger timeout to
      * clear the radio busy condition: we hope we will receive the
      * RX DONE event, and set isr_ms_preamble_start to zero again. */
    isr_header_valid = true;
  }
  else if (status_irq & SX126X_IRQ_HEADER_ERR)
  {
    /* Header error event. Clear the packet on air state. */
    isr_ms_preamble_start = 0;
    isr_header_valid = false;
  }
  else
  {
    isr_status_ring.write('?'); // unanticipated irq
  }
  SPIBegin();
    SPI.write(SX126X_CMD_CLEAR_IRQ_STATUS);
    SPI.write16(status_irq);
  SPIEnd();
}

//////////////////////////

SX126x::SX126x(BUSConfig bus, std::function<void(char*)> debug_printlnFn)
  { pImpl = new SX126xImpl(bus, debug_printlnFn); }
void SX126x::begin(SX126xConfig rc) { pImpl->begin(rc); }

DoubleBuffer<256>& SX126x::get_packet_buf() { return pImpl->isr_packet_buf; }
RingBuffer<32>& SX126x::get_status_ring() { return pImpl->isr_status_ring; }

//////////////////////////

// for complete information see [M4]
static const SX126xConfig long_fast_20 =
{
  frequency_Mhz: 906.875f, bw_idx:SX126X_LORA_BW_250_0, sf_value:11, cr_index:SX126X_LORA_CR_4_5,

  header_type:SX126X_LORA_HEADER_EXPLICIT,
  preamble_syms:16, // meshtastic wiki say 16 [M2], code says other things [M1]
  sync_word:0x24b4,
  crc_option:SX126X_LORA_CRC_ON,

  // per antirez application code, we will use infinite timeout
  rx_timeout_sym:SX126X_RX_TIMEOUT_INF, // 0 or -1 or 100 (lora standard per sz127x series docs?)

  tx_power_dbm: -22, // no TX, this is presently just a sniffer
  
  tx_ramp_idx:SX126X_PA_RAMP_200U, iq_option:SX126X_LORA_IQ_STANDARD,
  gain_boosted:true, cad_moresensitive:true, osc_tcxo:true
};

bool SX126xConfig::ChangeConfig(const char* config)
{
  if(strcmp(config,"meshtastic/short-turbo")==0)
  {
    *this = long_fast_20;
    // meshtastic short-turbo settings in slot50
    this->frequency_Mhz = 926.75f;
    this->bw_idx = SX126X_LORA_BW_500_0;
    this->sf_value = 7;
    return true;
  }
  else if(strcmp(config,"meshtastic/long-fast")==0)
  {
    // meshtastic long-fast settings in slot20
    *this = long_fast_20;
    return true;
  }
  
  return false;
}

bool SX126xConfig::LDORecomended() const
{
  // enable LDO when symbol time is >= 16.38ms
  // [S1] p40, p90
  uint32_t msPerSym_64ths = msPerSymTable_64ths[11 * sf_value + bw_idx];
  return (msPerSym_64ths >> 6) > 16 ? true : false;
}

uint32_t SX126xConfig::msTimeOnAir(uint8_t len) const
{
  // this might slightly underestimate the time, but not by more than 10% it seems
  // fixed point version
  uint32_t codingRate = cr_index + 4;
  uint32_t bitsInHeader = header_type == SX126X_LORA_HEADER_EXPLICIT ? 20 : 0;
  uint32_t bitsInCRC = (crc_option == SX126X_LORA_CRC_ON) ? 16 : 0;
  uint32_t bitsPerSym = (LDORecomended() ? sf_value - 2 : sf_value);

  uint32_t symPerBit_4096ths = symPerBitTable_4096ths[ bitsPerSym ];
  uint32_t msPerSym_64ths = msPerSymTable_64ths[11 * sf_value + bw_idx];

  uint32_t bitsOfPayload = 8 + (len << 3) + bitsInCRC + bitsInHeader;
  uint32_t bitsPayloadCompression = sf_value << 2;
  if (bitsPayloadCompression > bitsOfPayload) bitsOfPayload=0; else bitsOfPayload -= bitsPayloadCompression;

  uint32_t symInPayload_4 = (bitsOfPayload * codingRate * symPerBit_4096ths) >> 12;

  uint32_t msOnAir_64ths = 8 /*symOverhead*/ * msPerSym_64ths;
  msOnAir_64ths += (preamble_syms + 4) * msPerSym_64ths + (msPerSym_64ths >> 2); // preamble
  msOnAir_64ths += ((symInPayload_4 * msPerSym_64ths) >> 2) + (msPerSym_64ths << 3/*+8 but was 4*/); // payload

  return msOnAir_64ths >> 6;
}

}; // namespace
