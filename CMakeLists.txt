cmake_minimum_required(VERSION 3.16)

set(IDF_TARGET "esp32p4")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# IMX708 ISP tuning. The component manager populates this path during project()
# configuration; esp_ipa consumes the property while its component is configured.
set(imx708_ipa_json
    "managed_components/mushbraindave__esp_cam_sensor_imx/sensors/imx708/cfg/imx708_default.json")
idf_build_set_property(ESP_IPA_JSON_CONFIG_FILE_PATH "${imx708_ipa_json}" APPEND)
idf_build_set_property(IMX708_IPA_JSON_RESOLVED "${imx708_ipa_json}")

project(esp_icg)

# ESP32-P4 rev1 cannot execute the Zba instructions in esp_ipa 2.3.0's
# IDF 6.0+ prebuilt archive. For this experiment only, keep esp_ipa 2.3.0
# sources/configuration but substitute its IDF 5.4 prebuilt archive.
set(esp_ipa_lib_dir
    "${CMAKE_SOURCE_DIR}/managed_components/espressif__esp_ipa/lib/esp32p4")
set(esp_ipa_v54_lib "${esp_ipa_lib_dir}/v5.4-/libesp_ipa.a")
set(esp_ipa_v60_lib "${esp_ipa_lib_dir}/v6.0+/libesp_ipa.a")

if(NOT EXISTS "${esp_ipa_v54_lib}")
    message(FATAL_ERROR "esp_ipa v5.4- archive not found: ${esp_ipa_v54_lib}")
endif()

file(SHA256 "${esp_ipa_v54_lib}" esp_ipa_v54_sha)
execute_process(
    COMMAND "${CMAKE_COMMAND}" -E copy_if_different
            "${esp_ipa_v54_lib}" "${esp_ipa_v60_lib}"
    RESULT_VARIABLE esp_ipa_copy_result
)
if(NOT esp_ipa_copy_result EQUAL 0)
    message(FATAL_ERROR "Failed to substitute esp_ipa v5.4- archive")
endif()
file(SHA256 "${esp_ipa_v60_lib}" esp_ipa_v60_sha)
if(NOT esp_ipa_v54_sha STREQUAL esp_ipa_v60_sha)
    message(FATAL_ERROR "esp_ipa archive substitution verification failed")
endif()
message(STATUS "TEST: esp_ipa 2.3.0 using v5.4- archive SHA256=${esp_ipa_v54_sha}")
