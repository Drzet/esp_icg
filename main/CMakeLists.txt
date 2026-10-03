# Keep the proven SD/AVI recorder implementation intact, but compile an
# OV9281-specific generated copy with only its input contract changed:
# 1920x1080 RGB565/YUV422 -> 1280x720 native GRAY/GRAY.
file(READ "${CMAKE_CURRENT_LIST_DIR}/recorder.c" OV_RECORDER_SOURCE)
string(REPLACE
    ".src_type = JPEG_ENCODE_IN_FORMAT_RGB565,"
    ".src_type = JPEG_ENCODE_IN_FORMAT_GRAY,"
    OV_RECORDER_SOURCE "${OV_RECORDER_SOURCE}")
string(REPLACE
    ".sub_sample = JPEG_DOWN_SAMPLING_YUV422,"
    ".sub_sample = JPEG_DOWN_SAMPLING_GRAY,"
    OV_RECORDER_SOURCE "${OV_RECORDER_SOURCE}")
string(REPLACE
    "if (width != 1920 || height != 1080) return ESP_ERR_NOT_SUPPORTED;"
    "if (width != 1280 || height != 720) return ESP_ERR_NOT_SUPPORTED;"
    OV_RECORDER_SOURCE "${OV_RECORDER_SOURCE}")
string(REPLACE
    "size_t row_bytes = (size_t)s_width * 2;"
    "size_t row_bytes = (size_t)s_width;"
    OV_RECORDER_SOURCE "${OV_RECORDER_SOURCE}")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/recorder_ov9281.c" "${OV_RECORDER_SOURCE}")

idf_component_register(
    SRCS
        "ov9281_app.c"
        "display.c"
        "touch.c"
        "shared_spi.c"
        "${CMAKE_CURRENT_BINARY_DIR}/recorder_ov9281.c"
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
