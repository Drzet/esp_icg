# IMX708 author snapshot baseline

This branch is an isolated hardware test based on the driver's `imx708_snapshot` example at upstream commit `6b326b87b5c5099054a6b775eb6a78ba12e7c973` and ESP-IDF 5.4.0. The WT9932P4-TINY/WT0132P4-A1 board adaptation is camera power on GPIO0; camera SCCB remains SDA GPIO7 / SCL GPIO8. Flash size is 16 MB.

The firmware sends captured images as binary payloads on the console UART at 2,000,000 baud. Do not use `idf.py monitor` for the capture: it holds the port and does not preserve the binary image stream. Use the author's receiver included in `tools/`.

From an activated ESP-IDF 5.4.0 shell at the repository root, with no serial monitor running:

```bash
python tools/capture.py --flash --project . --port /dev/ttyACM0 --out imx708-baseline
```

To capture after the firmware is already flashed:

```bash
python tools/capture.py --project . --port /dev/ttyACM0 --out imx708-baseline
```

The script defaults to 2,000,000 baud, resets the board, extracts the framed binary payload, verifies its CRC, saves the image under `captures/imx708-baseline/`, and writes the text console output to `log.txt`. It requires `pyserial`; the ESP-IDF Python environment normally provides it.

This baseline pins `esp_video` 2.4.1, `esp_ipa` 2.3.0, and `esp_cam_sensor_imx` 0.3.0 so subsequent registry updates cannot silently change the tested stack.
