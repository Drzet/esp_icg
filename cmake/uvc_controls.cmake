# esp_video 2.4.1 has no manual-ISP ownership callback. Build a generated copy
# with a narrow hook around its existing configuration function. Managed source
# files remain untouched. Fail configuration if the expected function changes.
idf_component_get_property(video_dir espressif__esp_video COMPONENT_DIR)
idf_component_get_property(video_lib espressif__esp_video COMPONENT_LIB)
set(pipeline_source "${video_dir}/src/esp_video_isp_pipeline.c")
file(READ "${pipeline_source}" pipeline_text)
set(hook_begin "static void config_isp_and_camera(esp_video_isp_t *isp, esp_ipa_metadata_t *metadata)\n{\n")
set(hook_end "    config_motor_position(isp, metadata);\n#endif\n}")
string(FIND "${pipeline_text}" "${hook_begin}" begin_at)
string(FIND "${pipeline_text}" "${hook_end}" end_at)
if(begin_at LESS 0 OR end_at LESS 0)
    message(FATAL_ERROR "esp_video pipeline changed: review UVC manual-control hooks")
endif()
string(REPLACE "${hook_begin}"
"extern uint32_t icg_ipa_controls_begin(void);\nextern bool icg_ipa_controls_reset_ae(void);\nextern void icg_ipa_controls_end(void);\n${hook_begin}    metadata->flags &= ~icg_ipa_controls_begin();\n    if (icg_ipa_controls_reset_ae()) {\n        isp->prev_gain_index = -1;\n        isp->prev_exposure_val = UINT32_MAX;\n    }\n"
    pipeline_text "${pipeline_text}")
string(REPLACE "${hook_end}"
    "    config_motor_position(isp, metadata);\n#endif\n    icg_ipa_controls_end();\n}"
    pipeline_text "${pipeline_text}")
set(generated_pipeline "${CMAKE_BINARY_DIR}/icg_isp_pipeline.c")
file(CONFIGURE OUTPUT "${generated_pipeline}" CONTENT "${pipeline_text}" @ONLY)
get_target_property(video_sources ${video_lib} SOURCES)
set(replaced FALSE)
set(new_sources "")
foreach(source IN LISTS video_sources)
    if(source MATCHES "(^|/)esp_video_isp_pipeline\\.c$")
        list(APPEND new_sources "${generated_pipeline}")
        set(replaced TRUE)
    else()
        list(APPEND new_sources "${source}")
    endif()
endforeach()
if(NOT replaced)
    message(FATAL_ERROR "esp_video ISP pipeline source not found on component target")
endif()
set_property(TARGET ${video_lib} PROPERTY SOURCES "${new_sources}")
