#ifndef GTS_RADIO_TOUCH_CONFIG_H
#define GTS_RADIO_TOUCH_CONFIG_H

#include "RadioTftSetup.h"

// Raw four-wire resistive touch on the UNO-format parallel shield. Its film
// shares LCD pins, so no additional X/Y header connections are required.
// SPFD5408/MCUFRIEND shield example: XP=D6, XM=A2/RS, YP=A1/WR, YM=D7.
// https://github.com/JoaoLopesF/SPFD5408/blob/master/examples/spfd5408_tftpaint/spfd5408_tftpaint.ino
// Keep TFT_eSPI as the display driver. Touch is sampled separately between
// display transfers. A different PCB revision can override these four macros.
#ifndef RADIO_TOUCH_ENABLED
#define RADIO_TOUCH_ENABLED (RADIO_DISPLAY == RADIO_DISPLAY_TFT_SHIELD_24)
#endif

#if RADIO_TOUCH_ENABLED && RADIO_DISPLAY != RADIO_DISPLAY_TFT_SHIELD_24
#error "Resistive touch is supported only on RADIO_DISPLAY_TFT_SHIELD_24"
#endif

#if RADIO_TOUCH_ENABLED
#ifndef RADIO_TOUCH_XP
#define RADIO_TOUCH_XP TFT_D6
#endif
#ifndef RADIO_TOUCH_XM
#define RADIO_TOUCH_XM TFT_DC
#endif
#ifndef RADIO_TOUCH_YP
#define RADIO_TOUCH_YP TFT_WR
#endif
#ifndef RADIO_TOUCH_YM
#define RADIO_TOUCH_YM TFT_D7
#endif
#endif

#endif // GTS_RADIO_TOUCH_CONFIG_H
