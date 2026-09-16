# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.16)

# This test branch intentionally mirrors the driver's imx708_snapshot example.
# The driver is resolved from the component registry, so the tuning JSON lands
# under managed_components/ on a clean build.
set(imx708_ipa_json
    "managed_components/mushbraindave__esp_cam_sensor_imx/sensors/imx708/cfg/imx708_default.json")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

idf_build_set_property(ESP_IPA_JSON_CONFIG_FILE_PATH "${imx708_ipa_json}" APPEND)
idf_build_set_property(IMX708_IPA_JSON_RESOLVED "${imx708_ipa_json}")

# Kept exactly from the author's example for ESP-IDF 5.4.0-5.4.3. It makes
# esp_ipa_stats_t match the prebuilt esp_ipa ABI used by the original example.
if(IDF_VERSION_MAJOR EQUAL 5 AND IDF_VERSION_MINOR EQUAL 4 AND IDF_VERSION_PATCH LESS 4)
    idf_build_set_property(COMPILE_DEFINITIONS "ISP_AWB_WINDOW_X_NUM=5" APPEND)
    idf_build_set_property(COMPILE_DEFINITIONS "ISP_AWB_WINDOW_Y_NUM=5" APPEND)
endif()

project(imx708_snapshot)
