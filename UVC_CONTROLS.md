# Phone controls

Branch: `feature/usb-phone-preview`. The video path remains native 1920×1080
MJPEG q80, with no frame-rate gate and no rotations. USB VideoControl requests
now reach the camera/ISP; the phone app decides which controls appear in its UI.

| UVC control | Firmware behavior |
| --- | --- |
| Exposure mode | Auto (boot default) or manual. Auto jointly controls exposure and sensor gain. Select manual before setting either. |
| Exposure absolute | Standard 100 µs units, converted using `VIDIOC_G_SENSOR_FMT` line timing. Bounds come from the sensor driver; quantized to sensor lines. |
| Gain | Sensor driver's gain-table index, with driver-reported bounds. |
| Focus absolute | Lens codes 509–605: nominal 50–20 cm, default 552 (30 cm). These distances are not IR calibrated. |
| Brightness | Signed ISP offset; driver-reported bounds. |
| Contrast | ISP contrast; driver-reported bounds. |
| Hue | Signed −180…180 degrees, mapped to the ISP's wrapped angle. |
| Saturation | ISP saturation; driver-reported bounds. Zero removes colour. |
| Sharpness | 0–200%, relative to boot tuning; 100 preserves it, 0 disables sharpening. |
| Gamma | 50–300, relative adjustment to the tuned per-channel curves. 100 preserves them; higher values brighten midtones. |
| White balance component auto | On at boot. Off freezes the current red/blue gains. |
| White balance components | Blue and red gain ×1000, each 1–3999. Disable component auto first. |

Controls are queried before USB initialization and unsupported driver controls
are omitted from the advertised bitmaps. No initial SET requests are synthesized:
existing ISP tuning, exposure policy and focus remain unchanged until the host
changes a setting. Settings last until reboot, including across USB reconnects.
Autofocus, temperature in kelvin, zoom, pan/tilt, iris, and power-line frequency
are not advertised: this branch has no active, verified implementation for them.
White-balance components are the native red/blue controls; they are not a made-up
kelvin-to-gain conversion. Some apps may show only temperature and hide components.

## Implementation

- `main/uvc_controls.c` adds camera-terminal capability bits and a processing unit
  to the existing descriptor. It handles standard entity GET/SET requests, ranges,
  lengths, information flags and errors. TinyUSB still handles stream negotiation.
- `main/camera_controls.c` translates controls to the existing V4L2 sensor/ISP
  interfaces. SET failures stall EP0 and are available through REQUEST_ERROR_CODE.
  Successful/failed hardware writes log `uvc_controls: SET entity=… selector=…`.
- `cmake/uvc_controls.cmake` builds a generated copy of the pinned esp_video 2.4.1
  pipeline with a narrow configuration hook. It leaves managed files untouched.
  The hook and host writes share a lock, and manual settings mask the corresponding
  IPA metadata, so automatic updates cannot overwrite them. Returning to AE auto
  invalidates the pipeline's previous-write cache. Auto algorithms otherwise remain
  running; unaffected ISP stages retain their original behavior.
- TinyUSB 0.19's class and descriptor entry points are linked through `--wrap`;
  the component has no public entity-control callback. CMake fails if the expected
  pipeline function changes, so dependency upgrades need an explicit review.

## Check on nExt Camera

Reconnect the camera after flashing, then open its camera controls. Turn automatic
exposure off before testing exposure/gain. Test focus at both slider ends. If the
app offers component white balance, turn its auto mode off before changing R/B.
A visible slider alone is not proof: confirm a corresponding `SET … OK` log and
image change. GET requests alone do not change hardware. Recheck frame-rate and
error counters while streaming. Phone compatibility and control effects require
this hardware test; a successful compile does not establish them.

Host-side checks: `python tests/uvc_controls_test.py` and
`python tests/uvc_ownership_test.py`, plus `python tests/camera_controls_test.py`.
CI also checks the linked ELF for the descriptor/control wrappers and ISP hook.
