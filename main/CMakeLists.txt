idf_component_register(
    SRCS
        "ov9281_app.c"
        "display.c"
        "touch.c"
        "shared_spi.c"
        "recorder.c"
        "avi_writer.c"
    INCLUDE_DIRS "."
    REQUIRES
        espressif__esp_video
        espressif__esp_cam_sensor
        esp_driver_i2c
        esp_driver_gpio
        esp_driver_spi
        esp_driver_jpeg
        fatfs
        sdmmc
        esp_timer
        esp_lcd
)

# esp_cam_sensor 2.0.1 uses link-time sensor registration.
target_link_libraries(${COMPONENT_LIB} INTERFACE "-u ov9281_detect")
