#pragma once

/*
 * claw_video 2.4.0~1 declares dependency compatibility with esp_ipa 2.2.*, but
 * its archive contains two optional AGC status wrappers added against the 2.3
 * ioctl API. They are not used by this application. Compile those wrappers as
 * unsupported so the intended 2.4.0~1 + 2.2.x stack remains usable without
 * pulling in the rev1-incompatible esp_ipa 2.3 binary.
 */
#ifndef ESP_IPA_AGC_S_STATUS
#define ESP_IPA_AGC_S_STATUS 0
#endif
#ifndef ESP_IPA_AGC_G_STATUS
#define ESP_IPA_AGC_G_STATUS 1
#endif
#ifndef esp_ipa_pipeline_ioctl
#define esp_ipa_pipeline_ioctl(handle, cmd, value) ESP_ERR_NOT_SUPPORTED
#endif
