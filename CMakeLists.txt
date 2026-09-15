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

# esp_ipa 2.3.0 is built with RISC-V Zba instructions (sh1add/sh2add/sh3add),
# which are not supported by this ESP32-P4 rev1. esp_video 2.4.1 is ABI-
# compatible with the 2.2.0 esp_ipa archive, which contains no Zba opcodes.
# Keep the current component sources/configuration, but replace only the
# prebuilt archive with the known-good 2.2.0 binary before linking.
set(esp_ipa_lib "${CMAKE_SOURCE_DIR}/managed_components/espressif__esp_ipa/lib/esp32p4/v6.0+/libesp_ipa.a")
set(esp_ipa_22_url "https://raw.githubusercontent.com/espressif/esp-video-components/4a707c985706f49f9ef17a8d0d38d058846dae40/esp_ipa/lib/esp32p4/v6.0+/libesp_ipa.a")
file(DOWNLOAD
    "${esp_ipa_22_url}"
    "${esp_ipa_lib}"
    EXPECTED_HASH SHA256=c4fdfa21724d4b95006eb0496047c1fdd9b95115de0e9d3a9e37f11a23c625b3
    TLS_VERIFY ON
    STATUS esp_ipa_download_status
)
list(GET esp_ipa_download_status 0 esp_ipa_download_code)
if(NOT esp_ipa_download_code EQUAL 0)
    list(GET esp_ipa_download_status 1 esp_ipa_download_message)
    message(FATAL_ERROR "Failed to install rev1-safe esp_ipa 2.2 archive: ${esp_ipa_download_message}")
endif()
message(STATUS "Using rev1-safe esp_ipa 2.2 prebuilt archive")
