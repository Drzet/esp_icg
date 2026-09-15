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

# claw_video 2.4.0~1 is published with an esp_ipa 2.2.* dependency, but its
# archive includes two optional AGC-status wrappers that reference the later
# esp_ipa 2.3 ioctl API. The application does not use those wrappers; force a
# tiny compatibility shim only for claw_video so the rest of the 2.2 ABI stays
# untouched.
idf_component_get_property(claw_video_lib claw_video COMPONENT_LIB)
target_compile_options(${claw_video_lib} PRIVATE
    "-include${CMAKE_SOURCE_DIR}/config/esp_ipa_22_compat.h"
)
