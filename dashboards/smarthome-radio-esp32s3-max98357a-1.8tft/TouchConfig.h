#ifndef GTS_RADIO_TOUCH_CONFIG_H
#define GTS_RADIO_TOUCH_CONFIG_H

#include "RadioTftSetup.h"

// XPT2046 is a separate SPI touch controller. TFT_eSPI continues to drive the
// parallel ILI9341 display; no ESP32 ADC or LCD-data-pin probing is used.
// These four GPIOs must match the XPT2046's actual shield routing. The existing
// LCD CS and SD CS do not identify touch CS. Leave unknown pins at -1: the
// radio/knob keep working and calibration does not open on an unconfigured bus.
#ifndef RADIO_TOUCH_ENABLED
#define RADIO_TOUCH_ENABLED (RADIO_DISPLAY == RADIO_DISPLAY_TFT_SHIELD_24)
#endif

#if RADIO_TOUCH_ENABLED && RADIO_DISPLAY != RADIO_DISPLAY_TFT_SHIELD_24
#error "XPT2046 touch is supported only on RADIO_DISPLAY_TFT_SHIELD_24"
#endif

#if RADIO_TOUCH_ENABLED
#if defined(RADIO_TOUCH_XP) || defined(RADIO_TOUCH_XM) || defined(RADIO_TOUCH_YP) || defined(RADIO_TOUCH_YM)
#error "Raw touch electrodes are obsolete; configure the XPT2046 SPI pins below"
#endif
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

#endif // GTS_RADIO_TOUCH_CONFIG_H
