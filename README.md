# ESP ICG — USB phone preview prototype

Branch: `feature/usb-phone-preview`, based on `78db029` from
`feature/fixed-focus-sd-recording`. Build with ESP-IDF **v6.0.3**, ESP32-P4.

## First milestone

Full 1920×1080 RGB565 capture → existing hardware JPEG q80 / YUV422 →
USB high-speed bulk UVC MJPEG → phone UVC viewer.

The selected IMX708 mode is the existing 1920×1080 binned 28 fps mode. The
14 fps `VIDIOC_S_PARM` frame-skipping request is removed. UVC advertises this
single native 28 fps mode; there is no application frame-rate timer. This is
an advertised mode, not a claim of achieved throughput. Serial reports encoded
fps and USB handoff fps separately. The phone must establish received fps.

The encoder reserves a free compressed buffer before doing any work. While
USB transmits its private copy, the encoder can prepare the next JPEG. When
that slot is occupied, incoming raw frames are returned without JPEG encoding.
No backlog of stale JPEGs accumulates. The raw camera buffer is returned only
after `jpeg_encoder_process()` completes.

## Phone connection and first test

Use the board's **HUSB / USB OTG high-speed** port for video; FUSB is the
Serial/JTAG programming port. The phone must act as USB host. Use a data cable
and a UVC viewer that supports **MJPEG over bulk transfers**. Accept its USB
permission prompt and select **1920×1080**. Native Android camera-app support
is not assumed. No particular phone/app combination has yet been validated.

The board manufacturer identifies separate FUSB and HUSB connectors:
https://wiki.wireless-tag.com/docs/zh/WT9932P4-TINY/board_features.html

1. Flash the CI artifact using its generated `flash_args` and binaries, or
   build and flash with `idf.py set-target esp32p4`, `idf.py build`, `idf.py flash`.
2. Open the UVC viewer. Serial should log `host opened`, then `camera streaming`.
3. Check full-frame orientation and live movement; note phone fps and the
   `encode` / `USB_handoff` / `skipped_before_encode` serial statistics.
4. Close/reopen the viewer and unplug/reconnect USB. The camera should stop
   within approximately one second of absent USB demand, then restart.
5. Phone recording is a later app-level acceptance test; firmware does not
   write an AVI or SD file. Do not interpret USB handoff as verified recording.

## Current controls and hardware use

Focus is fixed at the previous motor code **552**, nominal **30 cm** (not IR
calibrated). Exposure/gain remain automatic. Existing ISP tuning, AWB behaviour,
gamma, diagnostics and the rev1 IPA archive substitution are preserved.
Phone exposure/gain/focus controls are not implemented in this milestone.

LCD, PPA, touch, GPIO34 record button and SD/AVI are not initialized or linked.
Their old source files remain in the branch for reference; the original branch
remains the functional standalone firmware.

Three raw camera buffers plus one encoder output buffer and one USB transfer
buffer are allocated. Each JPEG buffer has the former encoder's full RGB565
byte capacity (4,147,200 bytes), not an assumed typical compressed-frame size.
Encoder allocation uses the JPEG driver's allocator; USB storage uses explicit
PSRAM allocation. Allocation or geometry failure prevents streaming.

## Lifecycle and validation

Only the camera task changes camera streaming state. UVC callbacks signal
intent and exchange compressed-buffer ownership; they do not perform blocking
camera setup. The stream additionally checks USB mount/suspend/stream state and
a one-second request timeout because bulk app closure does not reliably invoke
`stop_cb`. Restart invalidates stale encoded work; buffers still owned by JPEG
or UVC are not reused until returned.

The pinned dependency is `espressif/usb_device_uvc == 1.3.1`. Its configured
frame interval is part of UVC negotiation; application capture has no 14 fps
skip and no extra encoder pacing. ISP diagnostics remain read-only.

CI compilation validates APIs and linkage. Hardware acceptance requires the
phone tests above: enumeration, first picture, sustained rate, latency,
close/reopen, disconnect during transfer and reconnect.
