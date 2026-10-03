#include "driver/isp_core.h"
#include "esp_log.h"

esp_err_t __real_esp_isp_new_processor(const esp_isp_processor_cfg_t *config,
                                     isp_proc_handle_t *processor);

esp_err_t __wrap_esp_isp_new_processor(const esp_isp_processor_cfg_t *config,
                                     isp_proc_handle_t *processor)
{
    if (config == NULL || config->input_data_source != ISP_INPUT_DATA_SOURCE_CSI ||
        config->input_data_color_type != ISP_COLOR_RAW8 ||
        config->output_data_color_type != ISP_COLOR_RAW8 ||
        config->h_res != 1280 || config->v_res != 720) {
        return __real_esp_isp_new_processor(config, processor);
    }

    /* esp_video 2.0.1 hardcodes 80 MHz. Test whether a faster ISP clock
     * resolves the HD mode's input FIFO overflow, without editing managed
     * components or changing the sensor's PLL, timing or pixel format. */
    esp_isp_processor_cfg_t hd_config = *config;
    hd_config.clk_src = ISP_CLK_SRC_PLL240;
    hd_config.clk_hz = 240000000;
    ESP_LOGI("p4d_isp", "HD RAW8 ISP clock test: %lu -> %lu Hz",
             (unsigned long)config->clk_hz, (unsigned long)hd_config.clk_hz);
    return __real_esp_isp_new_processor(&hd_config, processor);
}
