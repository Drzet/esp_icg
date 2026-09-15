idf_component_register(
    SRCS "imx708_test.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_video
        esp_cam_sensor_imx
        esp_driver_gpio
)
