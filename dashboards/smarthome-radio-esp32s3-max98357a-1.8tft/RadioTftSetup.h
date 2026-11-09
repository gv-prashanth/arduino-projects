#ifndef GTS_RADIO_TFT_SETUP_H
#define GTS_RADIO_TFT_SETUP_H

// Forced into every C++ translation unit by build_opt.h (Arduino IDE) and the
// PlatformIO build flags. TFT_eSPI.cpp must see the same display selection as
// the sketch. C and assembly files require no TFT definitions.
#ifdef __cplusplus
#include "DisplayProfile.h"

#define USER_SETUP_LOADED 1
#define LOAD_GLCD

#if RADIO_DISPLAY == RADIO_DISPLAY_TFT_SHIELD_24
#define USER_SETUP_ID 0x475401UL
#define USER_SETUP_INFO "GTS RADIO parallel shield"
#define ILI9341_DRIVER
#define TFT_PARALLEL_8_BIT
#define TFT_CS 16
#define TFT_DC 15
#define TFT_RST 17
#define TFT_WR 7
#define TFT_RD 21
#define TFT_D0 8
#define TFT_D1 18
#define TFT_D2 14
#define TFT_D3 13
#define TFT_D4 12
#define TFT_D5 11
#define TFT_D6 10
#define TFT_D7 9
#define RADIO_SD_CS 38
#define RADIO_SD_MOSI 39
#define RADIO_SD_MISO 40
#define RADIO_SD_SCLK 41
#else
#define USER_SETUP_ID 0x475402UL
#define USER_SETUP_INFO "GTS RADIO ST7735"
#define ST7735_DRIVER
#define ST7735_BLACKTAB
#define TFT_RGB_ORDER 1 // Black-tab Adafruit setup uses RGB, not BGR.
#define TFT_WIDTH 128
#define TFT_HEIGHT 160
#define TFT_MOSI 11
#define TFT_MISO -1
#define TFT_SCLK 12
#define TFT_CS 10
#define TFT_DC 13
#define TFT_RST 14
#define SPI_FREQUENCY 32000000 // Matches the former Adafruit_ST77xx default.
#endif
#endif // __cplusplus

#endif // GTS_RADIO_TFT_SETUP_H
