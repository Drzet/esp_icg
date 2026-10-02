idf_component_register(
    SRCS "imx708_test.c" "uvc_stream.c" "isp_diag.c"
    INCLUDE_DIRS "."
    REQUIRES esp_video esp_cam_sensor_imx esp_driver_gpio esp_driver_jpeg
             esp_timer usb_device_uvc tinyusb
)
