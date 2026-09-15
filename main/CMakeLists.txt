idf_component_register(
    SRCS
        "imx708_test.c"
        "display.c"
    INCLUDE_DIRS "."
    REQUIRES
        claw_video
        esp_cam_sensor_imx
        esp_driver_gpio
        esp_driver_ppa
        esp_driver_spi
        esp_lcd
)
