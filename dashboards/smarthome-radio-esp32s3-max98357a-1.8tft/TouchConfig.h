#ifndef GTS_RADIO_TOUCH_CONFIG_H
#define GTS_RADIO_TOUCH_CONFIG_H

#include "RadioTftSetup.h"

// Verified by the user's working MCUFRIEND/TouchScreen Uno keypad sketch:
// XP=D8 (LCD_D0), XM=A2 (LCD_RS), YP=A3 (LCD_CS), YM=D9 (LCD_D1).
// This is a directly connected four-wire resistive panel, sharing LCD pins.
// XPT2046 is a different, explicitly selected SPI controller variant.
#define RADIO_TOUCH_RESISTIVE 1
#define RADIO_TOUCH_XPT2046 2
#ifndef RADIO_TOUCH_BACKEND
#define RADIO_TOUCH_BACKEND RADIO_TOUCH_RESISTIVE
#endif
#ifndef RADIO_TOUCH_ENABLED
#define RADIO_TOUCH_ENABLED (RADIO_DISPLAY == RADIO_DISPLAY_TFT_SHIELD_24)
#endif

#if RADIO_TOUCH_ENABLED && RADIO_DISPLAY != RADIO_DISPLAY_TFT_SHIELD_24
#error "Touch is supported only on RADIO_DISPLAY_TFT_SHIELD_24"
#endif

#if RADIO_TOUCH_ENABLED
#if RADIO_TOUCH_BACKEND != RADIO_TOUCH_RESISTIVE && RADIO_TOUCH_BACKEND != RADIO_TOUCH_XPT2046
#error "Select RADIO_TOUCH_RESISTIVE or RADIO_TOUCH_XPT2046"
#endif
#if RADIO_TOUCH_BACKEND == RADIO_TOUCH_RESISTIVE
// These are ESP32 GPIOs obtained from the existing LCD wiring, not Uno numbers.
// No extra touch wires or pin-detection scan is needed for this shield.
#ifndef RADIO_TOUCH_XP
#define RADIO_TOUCH_XP TFT_D0
#endif
#ifndef RADIO_TOUCH_XM
#define RADIO_TOUCH_XM TFT_DC
#endif
#ifndef RADIO_TOUCH_YP
#define RADIO_TOUCH_YP TFT_CS
#endif
#ifndef RADIO_TOUCH_YM
#define RADIO_TOUCH_YM TFT_D1
#endif
#else
// Set these only for a physically identified XPT2046 and verified SPI routing.
// LCD CS and SD CS do not identify touch CS. Unknown required pins stay -1.
#ifndef RADIO_TOUCH_XPT2046_CS
#define RADIO_TOUCH_XPT2046_CS -1
#endif
#ifndef RADIO_TOUCH_SPI_MOSI
#define RADIO_TOUCH_SPI_MOSI -1
#endif
#ifndef RADIO_TOUCH_SPI_MISO
#define RADIO_TOUCH_SPI_MISO -1
#endif
#ifndef RADIO_TOUCH_SPI_SCLK
#define RADIO_TOUCH_SPI_SCLK -1
#endif
// IRQ is optional: pressure polling works without an interrupt connection.
#ifndef RADIO_TOUCH_XPT2046_IRQ
#define RADIO_TOUCH_XPT2046_IRQ -1
#endif
#ifndef RADIO_TOUCH_SPI_FREQUENCY
#define RADIO_TOUCH_SPI_FREQUENCY 1000000
#endif
#ifndef RADIO_TOUCH_PRESSURE_MIN
#define RADIO_TOUCH_PRESSURE_MIN 300
#endif
#endif
#endif

#endif // GTS_RADIO_TOUCH_CONFIG_H
