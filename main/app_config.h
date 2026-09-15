#pragma once

/* ILI9488 SPI display: verified wiring plan */
#define ICG_LCD_HOST       SPI2_HOST
#define ICG_LCD_PIN_CS     28
#define ICG_LCD_PIN_RST    17
#define ICG_LCD_PIN_DC     16
#define ICG_LCD_PIN_MOSI   29
#define ICG_LCD_PIN_SCLK   30
#define ICG_LCD_PIN_MISO   31
#define ICG_LCD_WIDTH      480
#define ICG_LCD_HEIGHT     320
#define ICG_PREVIEW_HEIGHT 280

/* XPT2046-compatible resistive touch, intentionally bit-banged so SPI3 stays free for SD. */
#define ICG_TOUCH_PIN_CS    18
#define ICG_TOUCH_PIN_IRQ   19
#define ICG_TOUCH_PIN_SCLK  20
#define ICG_TOUCH_PIN_MOSI  21
#define ICG_TOUCH_PIN_MISO  22

/* Initial calibration only; tune after first touch test. */
#define ICG_TOUCH_X_MIN 200
#define ICG_TOUCH_X_MAX 3900
#define ICG_TOUCH_Y_MIN 200
#define ICG_TOUCH_Y_MAX 3900
#define ICG_TOUCH_SWAP_XY 1
#define ICG_TOUCH_INVERT_X 0
#define ICG_TOUCH_INVERT_Y 1

/* SD SPI GPIOs are deliberately configured through Kconfig because the four
 * SD pads on the user's ILI9488 adapter have not yet been mapped to P4 GPIOs. */
#define ICG_SD_HOST SPI3_HOST
