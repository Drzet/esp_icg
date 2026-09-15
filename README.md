# esp_icg

ESP32-P4 prototype for an ICG fluorescence visualization/recording camera.

Current bring-up hardware:

- Wireless-Tag WT9932P4-TINY-1V2 / WT0132P4-A1-N16R32
- Raspberry Pi Camera v1.3 / OV5647 (temporary bring-up sensor)
- 3.5-inch 480x320 SPI IPS display / ILI9488
- XPT2046-compatible 4-wire resistive touch controller interface
- microSD socket on the ILI9488 adapter board
- ESP-IDF 6.0.x

The intended final sensor is Sony IMX708. The OV5647 code is deliberately isolated in the camera module so the sensor/control layer can be replaced later without restructuring the UI, display, storage, or recorder.

## Current prototype

Pipeline:

```
OV5647 RAW8 800x640
        |
     MIPI CSI
        |
 esp_video + ISP -> RGB565
        |              \
        |               -> P4 JPEG encoder -> microSD JPEG sequence
        v
 PPA crop/scale
        |
 480x280 preview
        |
 ILI9488 SPI
```

The lower 40 pixels of the display contain three touch zones:

- left: START camera
- middle: STOP camera
- right: REC toggle

Recording currently writes independent `ICG_000000.jpg`, `ICG_000001.jpg`, ... frames. This is intentional for initial validation; a container/video format can be added after capture and SD throughput are measured.

## Wiring

### ILI9488 display

| Display | ESP32-P4 GPIO |
|---|---:|
| CS | 28 |
| RST | 17 |
| D/C | 16 |
| SDI / MOSI | 29 |
| SCK | 30 |
| SDO / MISO | 31 |
| VDD | 3.3 V |
| GND | GND |
| BL | 3.3 V |

### Resistive touch

Touch uses separate GPIOs because the prototype wiring cannot physically fan out the LCD SPI pins.

| Touch | ESP32-P4 GPIO |
|---|---:|
| TCS | 18 |
| PEN / IRQ | 19 |
| TCK | 20 |
| TDI / MOSI | 21 |
| TDO / MISO | 22 |

The touch driver is currently a small XPT2046-compatible bit-banged implementation. Calibration constants in `main/app_config.h` are placeholders and must be calibrated on the physical panel.

### OV5647 / board CSI

The WT9932P4-TINY routes camera SCCB to GPIO7 (SDA) and GPIO8 (SCL). GPIO0 controls camera power/reset circuitry and is driven high before camera initialization. The camera itself uses the board MIPI-CSI connector.

### microSD on display adapter

The user confirmed four SD interface pads/wires are available. Their exact mapping is not yet identified, so no GPIO assignment is guessed.

Configure these after the four signals are identified:

- `ICG_SD_SCLK_GPIO`
- `ICG_SD_MOSI_GPIO`
- `ICG_SD_MISO_GPIO`
- `ICG_SD_CS_GPIO`

They are under `menuconfig -> ICG prototype`. Until all four are assigned, SD initialization is intentionally disabled.

SD uses SPI3 independently from the LCD. Initial clock is limited to 10 MHz for conservative bring-up over jumper wiring.

## Software modules

- `app_main.c` - application state and START/STOP/REC touch actions
- `camera.c` - OV5647 MIPI CSI / V4L2 capture and PPA preview conversion
- `display.c` - ILI9488 SPI driver and simple UI
- `touch.c` - resistive touch sampling
- `storage.c` - FAT/SDSPI mount
- `recorder.c` - asynchronous hardware-JPEG encoding and SD writes
- `app_config.h` - prototype GPIO assignments and touch calibration

## Build state

This branch is a development prototype and has not yet been compiled or hardware-tested as an integrated application. The first hardware validation sequence should be:

1. build
2. ILI9488/UI only
3. touch coordinates/calibration
4. OV5647 detection and START/STOP preview
5. identify/configure the four SD signals and mount a card
6. REC JPEG sequence and measure dropped frames / SD throughput

Do not merge this branch to `main` until those stages pass.
