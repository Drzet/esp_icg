#pragma once

#include "driver/spi_master.h"

#define ICG_LCD_HOST       SPI2_HOST
#define ICG_LCD_PIN_CS     28
#define ICG_LCD_PIN_RST    17
#define ICG_LCD_PIN_DC     16
#define ICG_LCD_PIN_MOSI   29
#define ICG_LCD_PIN_SCLK   30
#define ICG_LCD_PIN_MISO   31
#define ICG_LCD_WIDTH      480
#define ICG_LCD_HEIGHT     320
#define ICG_PREVIEW_HEIGHT ICG_LCD_HEIGHT

/* XPT2046 on its own SPI bus. */
#define ICG_TOUCH_HOST       SPI3_HOST
#define ICG_TOUCH_PIN_CS     18
#define ICG_TOUCH_PIN_IRQ    19
#define ICG_TOUCH_PIN_SCLK   20
#define ICG_TOUCH_PIN_MOSI   21
#define ICG_TOUCH_PIN_MISO   22
#define ICG_TOUCH_CLOCK_HZ   (2 * 1000 * 1000)

/*
 * Initial XPT2046 calibration. The controller is mounted in the display's
 * portrait coordinate system while the LCD is used in landscape, hence the
 * axis swap. These generous limits keep edge touches usable; raw/mapped values
 * are logged when a control changes so the four constants are easy to tighten
 * on the actual panel if necessary.
 */
#define ICG_TOUCH_RAW_X_MIN  200
#define ICG_TOUCH_RAW_X_MAX  3900
#define ICG_TOUCH_RAW_Y_MIN  200
#define ICG_TOUCH_RAW_Y_MAX  3900
#define ICG_TOUCH_SWAP_XY    1
#define ICG_TOUCH_INVERT_X   0
#define ICG_TOUCH_INVERT_Y   1
