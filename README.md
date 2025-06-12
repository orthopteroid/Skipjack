# Skipjack
Arduino SX126x sketch for sniffing Meshtastic (no radiolib dependency)

This firmware uses continious-receive mode on the Heltec wifi lora v3 esp32 board to sniff Meshtastic packets without the use of RadioLib to initialize the chip. I'm indebted to many very generous folks that have shared their code and knowledge, many of which have shout outs in the code. Perhaps this code can be ported to other chips as well, or made into a full RX/TX Meshtastic node.

On boot, the firmware prints messages to the serial port. It starts with checks for the time-on-air for various packet lengths. This calculation uses fixed-point math and is a bit of a work in progress but is reasonably accurate (less than 10%).

```
msTimeOnAir tests (work in progress)
TOA25  481ms
TOA50  667ms
TOA99  1031ms
TOA150 1410ms
TOA200 1782ms
```

The code then determines the time-on-air for a small 16 byte packet to use as the polling quantum in the cpu main loop. This time is dependant upon the radio configuration, so it needs to be determined at runtime. Longer polling intervals might miss very short packets so we want to keep loop() as quick as possible.

```
Polling quantum 415ms
```

Now the intitialization of the SX126x begins. Each line printed here is the chip status <chip mode> <command status> (consult SX126x docs for details) the error code and the line for the message. There are asserts inthe code to ensure that the chip does not get in a bad state. When an assert fails the line will be printed here.

You can see the chip mode proceeding from 2x (standby in 13 mhz oscillator mode) to 3x (standby in TCXO 33 mhz oscillator mode) to 51 (receiving mode).

```
status21 err020 X:\proj\Skipjack\SkipjackSX126x.cpp:241
status21 err020 X:\proj\Skipjack\SkipjackSX126x.cpp:248
status21 err000 X:\proj\Skipjack\SkipjackSX126x.cpp:259
status21 err000 X:\proj\Skipjack\SkipjackSX126x.cpp:280
status21 err000 X:\proj\Skipjack\SkipjackSX126x.cpp:290
status21 err000 X:\proj\Skipjack\SkipjackSX126x.cpp:304
status21 err000 X:\proj\Skipjack\SkipjackSX126x.cpp:321
status21 err000 X:\proj\Skipjack\SkipjackSX126x.cpp:330
status31 err000 X:\proj\Skipjack\SkipjackSX126x.cpp:356
status31 err000 X:\proj\Skipjack\SkipjackSX126x.cpp:367
chip "SX1261 V2D 2D02"
status31 err000 X:\proj\Skipjack\SkipjackSX126x.cpp:383
status31 err000 X:\proj\Skipjack\SkipjackSX126x.cpp:440
syncword 24B4
setup complete
status31 err000 X:\proj\Skipjack\SkipjackSX126x.cpp:470
status51 err000 X:\proj\Skipjack\SkipjackSX126x.cpp:490
```

Once things ar going the display on the board shows some of the radio settings currently configured as well as results from a status stream coming from the interrupt service routine. The contents of this stream are a bit cryptic without reading the code but the contents basically say that preambles ad packets have been received. In the case below it detected a Preamble, a Valid header and Received a packet of length 0x27, twice (Meshtestic does perform multiple TX on packets so this is normal):

![image](https://github.com/user-attachments/assets/7181cad3-b240-4d4e-827e-3673c4d531e5)

At the same time the interrupt service routine is sending packet contents to the serial port as those packets arrive. Each packet starts with a * and ends with a newline.

```
*FFFFFFFF44B3F693ADDEF3C8610800FC3716DA715E27212365A18A157A865DFD1F71A405A76904
*FFFFFFFF577B2FF972A3A9D1820800FCCCE974A6D0D6BD7051171C347ACB11F4446506E34F2DE2
*FFFFFFFF571D110E32776736610800DA0DBE730D5C50B5A0E654C1F73D91B7532BF3166FC09EF3946C9AB4F41B77B431DCAC
*FFFFFFFF95BF71235E12C46C610800DA4F5128808F6768679AF64902F7B43D46CD8EEF3B80C97F2D53129CDEB4A14E397678F4
*95BF71238C5F0907E128566C000800DAA1243C14ABE64B3026F75BA7C2586989D697D71D8E122C3CC0758402FD48B9256E8C76BF11D1FCFD942A9300A6AFDBA4A96AAC7B5FB8ACDBD77F
*95BF7123145E56DA680A5474A10800DAAEDD91DC363F01BF25D86155ABB49433BC39E2253A07EA4A55DDA605F9D3F01710539A84C8BE15D4CD76DB819A203E364683
*FFFFFFFFD475491C37CA613A610A00DA084312180DD73EC7671211086415583984401DB1E4B14125F390673E
```

The contents of these streams are specific to the SX126x implementation and what is dont with them is specific to the target board - I'm fortunate that with the Heltec I can use the display and the serial port to show different things. Or to take the board mobile without a serial port attached and test my packet capture.
