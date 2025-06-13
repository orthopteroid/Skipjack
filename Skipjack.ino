// 2025, orthopteroid@gmail.com, BSDv2 license
// radiolib-free dependency meshtastic sniffer
// 1st target platform: Heltec wifi lora v3 (SX126x)

#include <pins_arduino.h>
#include <SPI.h>
#include <SSD1306Wire.h>
#include <OLEDDisplayUi.h>

#include "Skipjack.h"
#include "SkipjackSX126x.h"

SSD1306Wire Display(0x3c, SDA_OLED, SCL_OLED, GEOMETRY_128_64);

Skipjack::BUSConfig bus = { NSS:SS, RESET:RST_LoRa, BUSY:BUSY_LoRa, IRQ:DIO0 };

Skipjack::SX126x SX126x(bus, [] (char* sz) { Serial.println(sz); });

char printf_buf[600]; // 512 or so is hex dump of longest packet

uint32_t ms_quantum = 0;

void setup() 
{
  // configure display
  pinMode(RST_OLED, OUTPUT);
  digitalWrite(RST_OLED, HIGH); delay(1);
  digitalWrite(RST_OLED, LOW); delay(20);
  digitalWrite(RST_OLED, HIGH);
  Display.init();
  Display.setContrast(255);
  Display.flipScreenVertically();

  SPI.begin();
  Serial.begin(115200);
  delay(100);

  Skipjack::SX126xConfig rc;
  auto ok = rc.ChangeConfig("meshtastic/long-fast");
  assert(ok);

  Serial.printf("msTimeOnAir tests (work in progress)\n");
  Serial.printf("TOA25  %dms\n", rc.msTimeOnAir(25));
  Serial.printf("TOA50  %dms\n", rc.msTimeOnAir(50));
  Serial.printf("TOA99  %dms\n", rc.msTimeOnAir(99));
  Serial.printf("TOA150 %dms\n", rc.msTimeOnAir(150));
  Serial.printf("TOA200 %dms\n", rc.msTimeOnAir(200));

  ms_quantum = rc.msTimeOnAir(16);
  Serial.printf("Polling quantum %dms\n", ms_quantum);

  Display.printf("F %.3f B %.1f\n", rc.frequency_Mhz, (float)Skipjack::SX126xConfig::bandwidth_hz[rc.bw_idx] / 1e3f);
  Display.printf("SF %i TXP %i dBm\n", rc.sf_value, rc.tx_power_dbm);

  SX126x.begin(rc); // should be last inside setup()
}

uint32_t ms_update = 0;
uint8_t display_col = 0;
const char hexchar[] = {'0','1','2','3','4','5','6','7','8','9','A','B','C','D','E','F'};

void loop()
{
  delay( ms_quantum );
  auto ms_now = millis();
  auto scount = SX126x.count_status_bytes();
  auto pcount = SX126x.count_packet_bytes();

  if( (scount==0) && (pcount==0) && !Skipjack::CheckTimerExpired(ms_update, ms_now, 1000) ) return;
  char* p = 0;

  auto putchar = [&] (char ch) { *(p++) = ch; if(((++display_col)&0x0F)==0x0F) *(p++) = '\n'; };
  auto puthex = [&] (uint8_t u8) { *(p++) = hexchar[u8 >> 4]; *(p++) = hexchar[u8 & 15]; };

  p = printf_buf;
  if(scount == 0) putchar('.'); else while(scount-- > 0) putchar(SX126x.read_status_byte());
  putchar(0);
  Display.printf(printf_buf);

  p = printf_buf;
  if(pcount > 0)
  {
    putchar('*');
    for(int i=0; i<pcount; i++) puthex(SX126x.read_packet_byte());
    putchar('\n'); putchar(0);
    Serial.printf(printf_buf);
  }
}
