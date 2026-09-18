# esp_icg — fixed focus and SD recording

Based on the working `investigate/6.0.3-af-enabled` branch at `3efbb04`.
Build with **ESP-IDF v6.0.3**, target **esp32p4**. The existing rev1 IPA archive
substitution, exposure/gain controls, white-balance behavior and 60 MHz LCD bus
are retained.

## Controls

There are no drawn buttons or overlays. Coordinates use the existing landscape
XPT2046 calibration:

| Region | Action |
|---|---|
| y=0–101 | Exposure slider |
| y=109–210 | Gain slider |
| y=218–319, x=0–239 | Start recording |
| y=218–319, x=240–479 | Stop and finalize recording |

The existing 7-pixel gaps stay inactive. Recording buttons fire once per press;
dragging into the region or across its center does not trigger another action.
Release for 60 ms before the next press. A repeated RECORD while recording or
finalizing does nothing. Serial logs report RECORD, STOP and errors.

Focus is set once after STREAMON to DW9807 code **509**, the previously adopted
standard ~75-degree lens map's nominal **50 cm** setting. This is a nominal map,
not a measured optical calibration of this individual NoIR lens under 780 nm.
`CONFIG_ESP_VIDEO_ISP_PIPELINE_CONTROL_CAMERA_MOTOR=n` remains mandatory; the
motor controller remains enabled. No per-frame motor writes or focus slider.

## SD wiring — external SPI socket/module

| SD signal | GPIO | Connection |
|---|---:|---|
| SCK / CLK | 20 | Shared with XPT2046 SCLK |
| MOSI / DI / CMD | 21 | Shared with XPT2046 MOSI |
| MISO / DO / DAT0 | 22 | Shared with XPT2046 MISO |
| CS / DAT3 | 23 | Dedicated SD CS, left header pin 8 |
| GND | — | Common ground |
| VCC | — | 3.3 V for a bare socket or 3.3 V module |

GPIO23 is an exposed general-purpose 3.3 V pin according to the
[WT9932P4-TINY manufacturer pin table](https://wiki.wireless-tag.com/docs/en/WT9932P4-TINY/board_features.html).
These are actual defaults in `main/app_config.h`. Module supply requirements must
be checked against its own circuit: this project does not identify the external
SD module or assume a 5 V regulator/level-shifter design. All bus signals are
3.3 V. Use a module that releases MISO when CS is high. Fit the SD pull-ups required
by the socket/module (including CS and data lines); a bare socket needs external
pull-ups. See [Espressif SD pull-up requirements](https://docs.espressif.com/projects/esp-idf/en/v6.0.3/esp32p4/api-reference/peripherals/sd_pullup_requirements.html).

Existing GPIO allocations audited from the working source:

| Function | GPIOs |
|---|---|
| Camera power / SCCB SDA / SCCB SCL | 0 / 7 / 8 |
| LCD DC / reset / CS / MOSI / SCLK / MISO | 16 / 17 / 28 / 29 / 30 / 31 |
| Touch CS / IRQ / SCLK / MOSI / MISO | 18 / 19 / 20 / 21 / 22 |
| UART0 console (reserved) | 37 / 38 |
| New SD CS | 23 |

Both general SPI controllers were already occupied. SD shares **SPI3 with touch**
at up to 20 MHz; touch stays at 2 MHz. LCD remains alone on SPI2. The SPI driver
arbitrates transfers; a separate mutex excludes touch during SD mount/unmount.
At boot a present card enters SPI mode before touch polling begins. Insert the
card before power-on; stop and wait for the `STOP finalized` log before removal.
Hot removal while recording is unsupported.

## Recording

- FAT32 card mounted at `/sdcard`; failed mounts never format the card.
- Full **1920×1080 RGB565 camera output → hardware JPEG q90, YUV422 → MJPEG AVI**.
  No audio. Full sensor view is retained with the preview's vertical orientation;
  the LCD alone uses its existing 1536×1024 center crop.
- Up to **10 fps** requested. One private PSRAM input buffer keeps encoder/SD work
  off the capture path; busy frames are dropped instead of blocking preview on SD.
  Actual throughput must be measured on the card and wiring. Frame copying still
  consumes memory bandwidth. Additional PSRAM is approximately 8.5 MB.
- `ICG00001.AVI`, `ICG00002.AVI`, etc. Exclusive creation prevents overwrites.
- STOP drains an accepted frame, writes the AVI index and timing, flushes, closes
  and unmounts the card. Mount, encode and write errors are reported over serial.
- AVI uses the measured average capture interval. Overall cadence is corrected
  when frames drop, but individual irregular gaps are not represented exactly.
- Recording stops at 6,000 frames or approximately 1 GiB, whichever comes first;
  press RECORD again for a new file. This bounds RAM index and FAT/RIFF sizes.
- Headers are checkpointed and synchronized about every 10 written frames.
  Always STOP before power-off; checkpoints do not guarantee recovery after power
  loss or card removal.

## Build and validation

```sh
. "$IDF_PATH/export.sh"
idf.py set-target esp32p4
idf.py build
```

GitHub Actions uses `espressif/idf:v6.0.3`. Host-side `tests/avi_smoke.c` exercises
AVI padding, index capacity, checkpoint/finalization and timestamp-derived rate;
its output can be decoded and seek-checked with FFmpeg. Hardware checks still
required: nominal focus at 50 cm, recorded color/orientation, SD/touch electrical
sharing and sustained recording with exposure/gain changes.
