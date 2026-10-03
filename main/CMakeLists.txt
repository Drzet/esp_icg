idf_component_register(
    SRCS "imx708_test.c" "uvc_stream.c" "uvc_controls.c" "camera_controls.c" "isp_diag.c"
    INCLUDE_DIRS "."
    REQUIRES esp_video esp_cam_sensor_imx esp_driver_gpio esp_driver_jpeg
             esp_timer usb_device_uvc tinyusb esp_ipa
)

# TinyUSB 0.19 has no entity-control callback. Keep VS negotiation unchanged.
target_link_libraries(${COMPONENT_LIB} INTERFACE
    "-Wl,-u,__wrap_videod_control_xfer_cb"
    "-Wl,-u,__wrap_tud_descriptor_configuration_cb"
    "-Wl,-u,__wrap_tud_descriptor_device_cb"
    "-Wl,--wrap=videod_control_xfer_cb"
    "-Wl,--wrap=tud_descriptor_configuration_cb"
    "-Wl,--wrap=tud_descriptor_device_cb")
