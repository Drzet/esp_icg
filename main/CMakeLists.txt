idf_component_register(
    SRCS "imx708_test.c" "display.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_video
        esp_cam_sensor_imx
        esp_driver_gpio
        esp_driver_ppa
        esp_driver_spi
        esp_lcd
)

# Keep the same guard as the snapshot baseline: never silently run without the
# IMX708 tuning file that feeds the ISP/IPA pipeline.
idf_build_get_property(imx708_ipa_json IMX708_IPA_JSON_RESOLVED)
idf_build_get_property(imx708_project_dir PROJECT_DIR)
get_filename_component(imx708_ipa_abs "${imx708_ipa_json}"
    ABSOLUTE BASE_DIR "${imx708_project_dir}")
if(NOT EXISTS "${imx708_ipa_abs}")
    message(FATAL_ERROR
        "IMX708 ISP tuning config not found at:\n  ${imx708_ipa_abs}\n"
        "esp_ipa would silently skip it and the camera would run without the intended tuning.")
endif()
