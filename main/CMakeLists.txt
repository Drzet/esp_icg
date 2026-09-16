# Pull the driver's own snapshot source from the exact upstream commit used for
# this baseline. The only source edits below are the WT0132P4 board's GPIO0
# camera-power handling; SCCB is already the same GPIO7/GPIO8 mapping upstream.
set(AUTHOR_SNAPSHOT_COMMIT "6b326b87b5c5099054a6b775eb6a78ba12e7c973")
set(AUTHOR_SNAPSHOT_URL
    "https://raw.githubusercontent.com/mushBrainDave/esp32-p4-imx-camera/${AUTHOR_SNAPSHOT_COMMIT}/components/esp_cam_sensor_imx/examples/imx708_snapshot/main/imx708_snapshot_main.c")
set(AUTHOR_SNAPSHOT_SRC "${CMAKE_CURRENT_BINARY_DIR}/imx708_snapshot_main.c")

file(DOWNLOAD "${AUTHOR_SNAPSHOT_URL}" "${AUTHOR_SNAPSHOT_SRC}"
     STATUS download_status TLS_VERIFY ON)
list(GET download_status 0 download_code)
if(NOT download_code EQUAL 0)
    message(FATAL_ERROR "Could not download author's IMX708 snapshot source: ${download_status}")
endif()

file(READ "${AUTHOR_SNAPSHOT_SRC}" snapshot_source)
string(REPLACE
    "#include \"esp_log.h\""
    "#include \"esp_log.h\"\n#include \"driver/gpio.h\""
    snapshot_source "${snapshot_source}")
string(REPLACE
    "#define CAM_PWDN_PIN        (-1)"
    "#define CAM_PWDN_PIN        (-1)\n#define CAM_BOARD_POWER_PIN  0"
    snapshot_source "${snapshot_source}")
string(REPLACE
    "    ESP_LOGI(TAG, \"IMX708 snapshot to SD\");"
    "    ESP_LOGI(TAG, \"IMX708 snapshot to SD\");\n\n    /* Wireless-Tag WT0132P4 carrier: GPIO0 enables camera power. */\n    gpio_config_t cam_power = {\n        .pin_bit_mask = 1ULL << CAM_BOARD_POWER_PIN,\n        .mode = GPIO_MODE_OUTPUT,\n    };\n    ESP_ERROR_CHECK(gpio_config(&cam_power));\n    ESP_ERROR_CHECK(gpio_set_level(CAM_BOARD_POWER_PIN, 1));\n    vTaskDelay(pdMS_TO_TICKS(100));"
    snapshot_source "${snapshot_source}")
file(WRITE "${AUTHOR_SNAPSHOT_SRC}" "${snapshot_source}")

idf_component_register(
    SRCS "${AUTHOR_SNAPSHOT_SRC}"
    INCLUDE_DIRS "."
    REQUIRES esp_video esp_cam_sensor_imx imx_serial_img fatfs esp_driver_sdmmc esp_driver_jpeg esp_driver_uart esp_driver_gpio
)

# Kept from the author's example: fail rather than silently run without tuning.
idf_build_get_property(imx708_ipa_json IMX708_IPA_JSON_RESOLVED)
idf_build_get_property(imx708_project_dir PROJECT_DIR)
get_filename_component(imx708_ipa_abs "${imx708_ipa_json}"
    ABSOLUTE BASE_DIR "${imx708_project_dir}")
if(NOT EXISTS "${imx708_ipa_abs}")
    message(FATAL_ERROR
        "IMX708 ISP tuning config not found at:\n  ${imx708_ipa_abs}\n"
        "esp_ipa would silently skip it and the camera would run with no "
        "auto-exposure and no white balance.")
endif()
