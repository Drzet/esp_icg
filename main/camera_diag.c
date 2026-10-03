#include "camera_diag.h"

#include <errno.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <sys/ioctl.h>

#include "esp_log.h"
#include "esp_video_ioctl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/mipi_csi_host_struct.h"
#include "soc/mipi_csi_bridge_struct.h"
#include "soc/isp_struct.h"

static const char *TAG = "csi_diag";

static int sensor_ioctl(int fd, uint32_t command, void *value, size_t size, bool read)
{
    struct v4l2_ext_control control = {
        .id = command,
        .size = size,
        .p_u8 = value,
    };
    struct v4l2_ext_controls controls = {
        .ctrl_class = V4L2_CTRL_CLASS_ESP_CAM_IOCTL,
        .count = 1,
        .controls = &control,
    };
    return ioctl(fd, read ? VIDIOC_G_EXT_CTRLS : VIDIOC_S_EXT_CTRLS, &controls);
}

static int sensor_read(int fd, uint16_t reg, uint32_t *value)
{
    esp_cam_sensor_reg_val_t data = {.regaddr = reg};
    if (sensor_ioctl(fd, ESP_CAM_SENSOR_IOC_G_REG, &data, sizeof(data), true) != 0) {
        ESP_LOGE(TAG, "sensor register 0x%04x read failed: errno=%d", reg, errno);
        return -1;
    }
    *value = data.value;
    return 0;
}

esp_err_t p4d_camera_sensor_standby(int fd)
{
    int stream = 0;
    if (sensor_ioctl(fd, ESP_CAM_SENSOR_IOC_S_STREAM, &stream, sizeof(stream), false) != 0) {
        ESP_LOGE(TAG, "sensor standby failed: errno=%d", errno);
        return ESP_FAIL;
    }
    /* Allow more than two nominal 20 ms frame periods before receiver setup. */
    vTaskDelay(pdMS_TO_TICKS(50));
    uint32_t mode;
    if (sensor_read(fd, 0x0100, &mode) != 0 || (mode & 1U)) {
        ESP_LOGE(TAG, "sensor did not confirm standby");
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "OV9281 standby confirmed (0x0100=0x%02lx) before CSI receiver setup",
             (unsigned long)mode);
    return ESP_OK;
}

void p4d_camera_diagnostics(int fd, const char *stage)
{
    ESP_LOGI(TAG, "snapshot: %s", stage);
    static const struct {
        uint16_t addr;
        const char *name;
    } regs[] = {
        {0x0100, "stream"}, {0x0302, "PLL1"}, {0x030d, "PLL2"},
        {0x030e, "PLL2 divider"}, {0x3808, "width high"}, {0x3809, "width low"},
        {0x380a, "height high"}, {0x380b, "height low"},
        {0x3662, "RAW format"}, {0x4800, "MIPI control"}, {0x4837, "MIPI timing"},
    };
    for (size_t i = 0; i < sizeof(regs) / sizeof(regs[0]); ++i) {
        uint32_t value;
        if (sensor_read(fd, regs[i].addr, &value) == 0) {
            ESP_LOGI(TAG, "OV9281 0x%04x %-13s = 0x%02lx", regs[i].addr,
                     regs[i].name, (unsigned long)value);
        }
    }

    /* Clock is gated in the selected sensor mode. Sample for several frames
     * instead of interpreting a single low clock reading as no activity. */
    uint32_t rx_seen = 0, stop_seen = 0, hs_samples = 0;
    for (unsigned i = 0; i < 64; ++i) {
        csi_host_phy_rx_reg_t rx = {.val = MIPI_CSI_HOST.phy_rx.val};
        rx_seen |= rx.val;
        stop_seen |= MIPI_CSI_HOST.phy_stopstate.val;
        hs_samples += rx.phy_rxclkactivehs;
        vTaskDelay(1);
    }
    ESP_LOGI(TAG, "PHY: rx_seen=0x%08lx stop_seen=0x%08lx HS-clock samples=%lu/64 lanes=%lu",
             (unsigned long)rx_seen, (unsigned long)stop_seen, (unsigned long)hs_samples,
             (unsigned long)(MIPI_CSI_HOST.n_lanes.n_lanes + 1));
    /* Some status registers clear on read: snapshot each once for this log. */
    ESP_LOGI(TAG, "HOST: main=%08lx phy_fatal=%08lx packet_fatal=%08lx phy=%08lx",
             (unsigned long)MIPI_CSI_HOST.int_st_main.val,
             (unsigned long)MIPI_CSI_HOST.int_st_phy_fatal.val,
             (unsigned long)MIPI_CSI_HOST.int_st_pkt_fatal.val,
             (unsigned long)MIPI_CSI_HOST.int_st_phy.val);
    ESP_LOGI(TAG, "HOST: boundary=%08lx sequence=%08lx frame_crc=%08lx payload_crc=%08lx data_id=%08lx ecc=%08lx",
             (unsigned long)MIPI_CSI_HOST.int_st_bndry_frame_fatal.val,
             (unsigned long)MIPI_CSI_HOST.int_st_seq_frame_fatal.val,
             (unsigned long)MIPI_CSI_HOST.int_st_crc_frame_fatal.val,
             (unsigned long)MIPI_CSI_HOST.int_st_pld_crc_fatal.val,
             (unsigned long)MIPI_CSI_HOST.int_st_data_id.val,
             (unsigned long)MIPI_CSI_HOST.int_st_ecc_corrected.val);
    ESP_LOGI(TAG, "BRIDGE: enable=%08lx frame=%08lx raw=%08lx type=%08lx dma=%08lx",
             (unsigned long)MIPI_CSI_BRIDGE.csi_en.val,
             (unsigned long)MIPI_CSI_BRIDGE.frame_cfg.val,
             (unsigned long)MIPI_CSI_BRIDGE.int_raw.val,
             (unsigned long)MIPI_CSI_BRIDGE.data_type_cfg.val,
             (unsigned long)MIPI_CSI_BRIDGE.dma_req_cfg.val);
    ESP_LOGI(TAG, "ISP: control=%08lx frame=%08lx hsync=%08lx raw=%08lx",
             (unsigned long)ISP.cntl.val, (unsigned long)ISP.frame_cfg.val,
             (unsigned long)ISP.hsync_cnt.val, (unsigned long)ISP.int_raw.val);
}
