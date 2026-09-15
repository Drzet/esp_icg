cmake_minimum_required(VERSION 3.16)

set(IDF_TARGET "esp32p4")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Fixed-focus IMX708 ISP tuning. This is based on the sensor component's
# imx708_default.json with the autofocus block removed, so AE/AGC/AWB and the
# image-processing algorithms can resolve while the AF/motor stack stays off.
set(imx708_ipa_json "config/imx708_fixed_focus.json")
idf_build_set_property(ESP_IPA_JSON_CONFIG_FILE_PATH "${imx708_ipa_json}" APPEND)
idf_build_set_property(IMX708_IPA_JSON_RESOLVED "${imx708_ipa_json}")

project(esp_icg)
