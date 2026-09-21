#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/ppa.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_video_device.h"
#include "esp_video_init.h"
#include "esp_video_isp_pipeline.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "linux/videodev2.h"

#include "app_config.h"
#include "display.h"
#include "touch.h"
#include "recorder.h"
#include "shared_spi.h"

#if CONFIG_ESP_VIDEO_ISP_PIPELINE_CONTROL_CAMERA_MOTOR
#error "Manual focus requires CONFIG_ESP_VIDEO_ISP_PIPELINE_CONTROL_CAMERA_MOTOR=n; regenerate sdkconfig from sdkconfig.defaults (idf.py set-target esp32p4)."
#endif

#define CAMERA_POWER_GPIO      0
#define CAMERA_SCCB_I2C_PORT   0
#define CAMERA_SCCB_SCL        8
#define CAMERA_SCCB_SDA        7
#define CAMERA_SCCB_FREQ_HZ    100000
#define CAMERA_BUFFER_COUNT    3
#define PPA_CACHE_LINE_SIZE    128
#define PREVIEW_BUFFER_SIZE    (ICG_LCD_WIDTH * ICG_PREVIEW_HEIGHT * sizeof(uint16_t))
#define PREVIEW_BUFFER_COUNT   3
#define DISPLAY_TASK_STACK     4096
#define DISPLAY_TASK_PRIORITY  (tskIDLE_PRIORITY + 1)
#define TOUCH_TASK_STACK       3072
#define TOUCH_TASK_PRIORITY    (tskIDLE_PRIORITY + 1)

/* Three horizontal 102-pixel bands with two 7-pixel dead gaps: 102*3 + 7*2 = 320. */
#define TOUCH_ZONE_HEIGHT      102
#define TOUCH_ZONE_GAP         7

/* Last 10% of exposure/gain slider travel is a hard maximum plateau. */
#define SENSITIVITY_MAX_PLATEAU_X  ((ICG_LCD_WIDTH * 90) / 100)

/* Nominal 50 cm setting from the previously adopted standard-lens map. */
#define FOCUS_CODE_50CM        509

/*
 * PPA scale factors are quantized to 1/16. A centered 1536x1024 crop scales
 * exactly by 5/16 to 480x320, filling the landscape LCD without distortion.
 */
#define PREVIEW_CROP_WIDTH     1536
#define PREVIEW_CROP_HEIGHT    1024
#define PREVIEW_SCALE          (5.0f / 16.0f)

static const char *TAG = "imx708_preview";
static ppa_client_handle_t s_ppa;
static uint16_t *s_preview[PREVIEW_BUFFER_COUNT];
static TaskHandle_t s_display_task;
static portMUX_TYPE s_preview_lock = portMUX_INITIALIZER_UNLOCKED;
static int s_latest_preview = -1;
static int s_display_preview = -1;
static volatile uint32_t s_display_frames;
static volatile uint32_t s_dropped_previews;

/*
 * Fluorescence-oriented manual exposure range. Values below 512 lines are not
 * useful here, so the full slider travel is spent on 512..2624. The final
 * value is handled as a deliberately wide maximum plateau.
 */
static const uint16_t s_exposure_steps[] = {
    512, 576, 640, 704, 768, 832, 896, 960,
    1024, 1120, 1216, 1312, 1408, 1504, 1600, 1696,
    1792, 1888, 1984, 2080, 2176, 2272, 2368, 2464,
    2560, 2624,
};

/* Requests are produced by touch_task and consumed between captured frames. */
typedef struct {
    bool exposure_pending;
    int32_t exposure_lines;
    bool gain_pending;
    int32_t gain_index;
} control_requests_t;

static portMUX_TYPE s_control_lock = portMUX_INITIALIZER_UNLOCKED;
static control_requests_t s_control_requests;
static bool s_manual_ae;
static bool s_touch_available;

static const esp_video_init_csi_config_t s_csi_config[] = {
    {
        .sccb_config = {
            .init_sccb = true,
            .i2c_config = {
                .port = CAMERA_SCCB_I2C_PORT,
                .scl_pin = CAMERA_SCCB_SCL,
                .sda_pin = CAMERA_SCCB_SDA,
            },
            .freq = CAMERA_SCCB_FREQ_HZ,
        },
        .reset_pin = -1,
        .pwdn_pin = -1,
    },
};

static const esp_video_init_cam_motor_config_t s_motor_config[] = {
    {
        .sccb_config = {
            .init_sccb = true,
            .i2c_config = {
                .port = CAMERA_SCCB_I2C_PORT,
                .scl_pin = CAMERA_SCCB_SCL,
                .sda_pin = CAMERA_SCCB_SDA,
            },
            .freq = CAMERA_SCCB_FREQ_HZ,
        },
        .reset_pin = -1,
        .pwdn_pin = -1,
        .signal_pin = -1,
    },
};

static const esp_video_init_config_t s_video_config = {
    .csi = s_csi_config,
    .cam_motor = s_motor_config,
};

static esp_err_t camera_power_on(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << CAMERA_POWER_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    esp_err_t ret = gpio_config(&cfg);
    if (ret != ESP_OK) return ret;

    gpio_set_level(CAMERA_POWER_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    return ESP_OK;
}

static bool read_user_ctrl(int fd, uint32_t id, int32_t *out)
{
    struct v4l2_ext_control ctrl = { .id = id };
    struct v4l2_ext_controls ctrls = {
        .ctrl_class = V4L2_CTRL_CLASS_USER,
        .count = 1,
        .controls = &ctrl,
    };

    if (ioctl(fd, VIDIOC_G_EXT_CTRLS, &ctrls) != 0) return false;
    *out = ctrl.value;
    return true;
}

static bool write_user_ctrl(int fd, uint32_t id, int32_t value)
{
    struct v4l2_ext_control ctrl = { .id = id, .value = value };
    struct v4l2_ext_controls ctrls = {
        .ctrl_class = V4L2_CTRL_CLASS_USER,
        .count = 1,
        .controls = &ctrl,
    };
    return ioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrls) == 0;
}

static bool write_focus_ctrl(int fd, int32_t value)
{
    struct v4l2_ext_control ctrl = {
        .id = V4L2_CID_FOCUS_ABSOLUTE,
        .value = value,
    };
    struct v4l2_ext_controls ctrls = {
        .ctrl_class = V4L2_CID_CAMERA_CLASS,
        .count = 1,
        .controls = &ctrl,
    };
    return ioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrls) == 0;
}

static int touch_zone_from_y(int y)
{
    if (y >= 0 && y < TOUCH_ZONE_HEIGHT) return 0;

    int z1 = TOUCH_ZONE_HEIGHT + TOUCH_ZONE_GAP;
    if (y >= z1 && y < z1 + TOUCH_ZONE_HEIGHT) return 1;

    int z2 = z1 + TOUCH_ZONE_HEIGHT + TOUCH_ZONE_GAP;
    if (y >= z2 && y < z2 + TOUCH_ZONE_HEIGHT) return 2;

    return -1;
}

static int sensitivity_step_from_x(int x, int count)
{
    if (x < 0) x = 0;
    if (x >= ICG_LCD_WIDTH) x = ICG_LCD_WIDTH - 1;

    if (x >= SENSITIVITY_MAX_PLATEAU_X) {
        return count - 1;
    }

    /* Spread all non-maximum steps over the first 90% of slider travel. */
    const int non_max_steps = count - 1;
    return (x * (non_max_steps - 1) + (SENSITIVITY_MAX_PLATEAU_X - 1) / 2) /
           (SENSITIVITY_MAX_PLATEAU_X - 1);
}

static void queue_exposure(int32_t lines)
{
    portENTER_CRITICAL(&s_control_lock);
    s_control_requests.exposure_lines = lines;
    s_control_requests.exposure_pending = true;
    portEXIT_CRITICAL(&s_control_lock);
}

static void queue_gain(int32_t index)
{
    portENTER_CRITICAL(&s_control_lock);
    s_control_requests.gain_index = index;
    s_control_requests.gain_pending = true;
    portEXIT_CRITICAL(&s_control_lock);
}

static void touch_task(void *arg)
{
    (void)arg;
    int active_zone = -1;
    int active_step = -1;
    bool contact = false;
    unsigned release_samples = 0;

    while (true) {
        touch_point_t p;
        if (!touch_read(&p)) {
            if (!touch_is_pressed() && ++release_samples >= 3) {
                contact = false;
                active_zone = -1;
                active_step = -1;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        release_samples = 0;
        bool new_contact = !contact;
        contact = true;
        int zone = touch_zone_from_y(p.y);
        if (zone == 2) {
            /* One action per press; dragging in from another band does nothing. */
            if (new_contact) recorder_request(p.x < ICG_LCD_WIDTH / 2);
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (zone < 0) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        int step;
        if (zone == 0) {
            step = sensitivity_step_from_x(
                p.x, (int)(sizeof(s_exposure_steps) / sizeof(s_exposure_steps[0])));
        } else {
            step = sensitivity_step_from_x(p.x, 47);
        }

        if (zone == active_zone && step == active_step) {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        active_zone = zone;
        active_step = step;

        if (zone == 0) {
            int32_t lines = s_exposure_steps[step];
            queue_exposure(lines);
            ESP_LOGI(TAG, "touch EXP step=%d lines=%" PRId32 " raw=%u,%u xy=%d,%d",
                     step, lines, p.raw_x, p.raw_y, p.x, p.y);
        } else if (zone == 1) {
            queue_gain(step);
            ESP_LOGI(TAG, "touch GAIN idx=%d raw=%u,%u xy=%d,%d",
                     step, p.raw_x, p.raw_y, p.x, p.y);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

static void apply_control_requests(int fd)
{
    control_requests_t req;

    portENTER_CRITICAL(&s_control_lock);
    req = s_control_requests;
    memset(&s_control_requests, 0, sizeof(s_control_requests));
    portEXIT_CRITICAL(&s_control_lock);

    if (req.exposure_pending || req.gain_pending) {
        if (!s_manual_ae) {
            esp_err_t ret = esp_video_isp_pipeline_set_agc_status(
                ESP_VIDEO_ISP_PIPELINE_AGC_DISABLE);
            if (ret == ESP_OK) {
                s_manual_ae = true;
                ESP_LOGI(TAG, "manual exposure/gain takeover: AE/AGC disabled; AWB unchanged");
            } else {
                ESP_LOGE(TAG, "failed to disable AE/AGC: %s", esp_err_to_name(ret));
            }
        }

        if (req.exposure_pending &&
            !write_user_ctrl(fd, V4L2_CID_EXPOSURE, req.exposure_lines)) {
            ESP_LOGE(TAG, "setting exposure=%" PRId32 " failed errno=%d",
                     req.exposure_lines, errno);
        }
        if (req.gain_pending &&
            !write_user_ctrl(fd, V4L2_CID_GAIN, req.gain_index)) {
            ESP_LOGE(TAG, "setting gain_idx=%" PRId32 " failed errno=%d",
                     req.gain_index, errno);
        }
    }
}

static int preview_acquire_write_buffer(void)
{
    int index = -1;

    portENTER_CRITICAL(&s_preview_lock);
    for (int i = 0; i < PREVIEW_BUFFER_COUNT; ++i) {
        if (i != s_display_preview && i != s_latest_preview) {
            index = i;
            break;
        }
    }
    portEXIT_CRITICAL(&s_preview_lock);

    return index;
}

static void preview_publish(int index)
{
    portENTER_CRITICAL(&s_preview_lock);
    if (s_latest_preview >= 0) ++s_dropped_previews;
    s_latest_preview = index;
    portEXIT_CRITICAL(&s_preview_lock);

    xTaskNotifyGive(s_display_task);
}

static void display_task(void *arg)
{
    (void)arg;

    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        int index;
        portENTER_CRITICAL(&s_preview_lock);
        index = s_latest_preview;
        if (index >= 0) {
            s_latest_preview = -1;
            s_display_preview = index;
        }
        portEXIT_CRITICAL(&s_preview_lock);

        if (index < 0) continue;

        esp_err_t ret = display_draw_preview(s_preview[index]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "display failed: %s", esp_err_to_name(ret));
        } else {
            ++s_display_frames;
        }

        portENTER_CRITICAL(&s_preview_lock);
        s_display_preview = -1;
        portEXIT_CRITICAL(&s_preview_lock);
    }
}

static esp_err_t scale_preview_frame(const uint8_t *frame, uint32_t width,
                                     uint32_t height, size_t len, uint16_t *out)
{
    if (!frame || !out || len < (size_t)width * height * sizeof(uint16_t)) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (width < PREVIEW_CROP_WIDTH || height < PREVIEW_CROP_HEIGHT) {
        return ESP_ERR_INVALID_SIZE;
    }

    const uint32_t crop_x = (width - PREVIEW_CROP_WIDTH) / 2;
    const uint32_t crop_y = (height - PREVIEW_CROP_HEIGHT) / 2;

    ppa_srm_oper_config_t op = {
        .in = {
            .buffer = (void *)frame,
            .pic_w = width,
            .pic_h = height,
            .block_w = PREVIEW_CROP_WIDTH,
            .block_h = PREVIEW_CROP_HEIGHT,
            .block_offset_x = crop_x,
            .block_offset_y = crop_y,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .out = {
            .buffer = out,
            .buffer_size = PREVIEW_BUFFER_SIZE,
            .pic_w = ICG_LCD_WIDTH,
            .pic_h = ICG_PREVIEW_HEIGHT,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        /* Preserve camera orientation; PPA only crops and scales. */
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
        .scale_x = PREVIEW_SCALE,
        .scale_y = PREVIEW_SCALE,
        .mirror_x = false,
        .mirror_y = false,
        .mode = PPA_TRANS_MODE_BLOCKING,
    };

    return ppa_do_scale_rotate_mirror(s_ppa, &op);
}

static esp_err_t run_preview(int fd)
{
    uint8_t *buffers[CAMERA_BUFFER_COUNT] = {0};
    size_t lengths[CAMERA_BUFFER_COUNT] = {0};

    struct v4l2_format fmt = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    };
    if (ioctl(fd, VIDIOC_G_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "VIDIOC_G_FMT failed: errno=%d", errno);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "format: %" PRIu32 "x%" PRIu32 " fourcc=%c%c%c%c",
             fmt.fmt.pix.width, fmt.fmt.pix.height,
             (char)(fmt.fmt.pix.pixelformat & 0xff),
             (char)((fmt.fmt.pix.pixelformat >> 8) & 0xff),
             (char)((fmt.fmt.pix.pixelformat >> 16) & 0xff),
             (char)((fmt.fmt.pix.pixelformat >> 24) & 0xff));

    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_RGB565) {
        ESP_LOGE(TAG, "expected RGB565 from ISP, got 0x%08" PRIx32, fmt.fmt.pix.pixelformat);
        return ESP_ERR_NOT_SUPPORTED;
    }

    struct v4l2_streamparm parm = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    };
    parm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = 14;
    if (ioctl(fd, VIDIOC_S_PARM, &parm) != 0) {
        ESP_LOGW(TAG, "VIDIOC_S_PARM 14 fps failed: errno=%d", errno);
    } else {
        struct v4l2_streamparm actual = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        };
        if (ioctl(fd, VIDIOC_G_PARM, &actual) == 0 &&
            actual.parm.capture.timeperframe.numerator != 0) {
            ESP_LOGI(TAG, "capture interval: %u/%u s (%.2f fps)",
                     actual.parm.capture.timeperframe.numerator,
                     actual.parm.capture.timeperframe.denominator,
                     (double)actual.parm.capture.timeperframe.denominator /
                     actual.parm.capture.timeperframe.numerator);
        }
    }

    struct v4l2_requestbuffers req = {
        .count = CAMERA_BUFFER_COUNT,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(fd, VIDIOC_REQBUFS, &req) != 0 || req.count < 2) {
        ESP_LOGE(TAG, "VIDIOC_REQBUFS failed: errno=%d", errno);
        return ESP_FAIL;
    }

    uint32_t count = req.count > CAMERA_BUFFER_COUNT ? CAMERA_BUFFER_COUNT : req.count;
    for (uint32_t i = 0; i < count; ++i) {
        struct v4l2_buffer b = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index = i,
        };
        if (ioctl(fd, VIDIOC_QUERYBUF, &b) != 0) return ESP_FAIL;

        lengths[i] = b.length;
        buffers[i] = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, b.m.offset);
        if (buffers[i] == MAP_FAILED) {
            buffers[i] = NULL;
            return ESP_FAIL;
        }
        if (ioctl(fd, VIDIOC_QBUF, &b) != 0) return ESP_FAIL;
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) != 0) {
        ESP_LOGE(TAG, "VIDIOC_STREAMON failed: errno=%d", errno);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "live preview started: full-screen crop=%dx%d -> 480x320",
             PREVIEW_CROP_WIDTH, PREVIEW_CROP_HEIGHT);
    if (!write_focus_ctrl(fd, FOCUS_CODE_50CM)) {
        ESP_LOGE(TAG, "fixed focus failed errno=%d; stopping camera", errno);
        goto cleanup;
    }
    ESP_LOGI(TAG, "focus locked: nominal 50 cm, DW9807 code=%d", FOCUS_CODE_50CM);
    ESP_LOGI(TAG, "touch: top=exposure middle=gain bottom-left=record bottom-right=stop");
    esp_err_t rec_ret = recorder_init(fmt.fmt.pix.width, fmt.fmt.pix.height);
    if (rec_ret != ESP_OK) ESP_LOGE(TAG, "recorder unavailable: %s", esp_err_to_name(rec_ret));
    if (s_touch_available && xTaskCreate(touch_task, "touch", TOUCH_TASK_STACK, NULL,
                                        TOUCH_TASK_PRIORITY, NULL) != pdPASS) {
        ESP_LOGE(TAG, "touch task creation failed");
    }
    uint32_t frames = 0;

    while (true) {
        struct v4l2_buffer b = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        if (ioctl(fd, VIDIOC_DQBUF, &b) != 0) {
            ESP_LOGE(TAG, "VIDIOC_DQBUF failed: errno=%d", errno);
            break;
        }

        if (b.index >= count || !buffers[b.index]) {
            ESP_LOGE(TAG, "invalid camera buffer index");
            break;
        }

        int preview_index = preview_acquire_write_buffer();
        if (preview_index < 0) {
            ESP_LOGE(TAG, "no free preview buffer");
            break;
        }

        esp_err_t ret = scale_preview_frame(buffers[b.index], fmt.fmt.pix.width,
                                            fmt.fmt.pix.height, b.bytesused,
                                            s_preview[preview_index]);

        if (ret == ESP_OK) {
            recorder_submit(buffers[b.index], b.bytesused, fmt.fmt.pix.bytesperline);
        }

        /* recorder_submit() retains accepted buffers until JPEG has finished reading them. */
        if (ioctl(fd, VIDIOC_QBUF, &b) != 0) {
            ESP_LOGE(TAG, "VIDIOC_QBUF failed: errno=%d", errno);
            break;
        }

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "PPA preview failed: %s", esp_err_to_name(ret));
            break;
        }

        apply_control_requests(fd);
        preview_publish(preview_index);

        ++frames;
        if ((frames % 20) == 0) {
            int32_t exposure_lines = -1;
            int32_t gain_index = -1;
            bool exposure_ok = read_user_ctrl(fd, V4L2_CID_EXPOSURE, &exposure_lines);
            bool gain_ok = read_user_ctrl(fd, V4L2_CID_GAIN, &gain_index);

            if (exposure_ok && gain_ok) {
                ESP_LOGI(TAG, "sensor readback: exposure=%" PRId32 " gain_idx=%" PRId32
                         " mode=%s",
                         exposure_lines, gain_index, s_manual_ae ? "manual" : "auto");
            } else {
                ESP_LOGW(TAG, "sensor readback failed: exposure=%d gain=%d errno=%d",
                         exposure_ok, gain_ok, errno);
            }
        }

        if ((frames % 100) == 0) {
            ESP_LOGI(TAG, "capture frames=%" PRIu32 " display=%" PRIu32
                     " dropped_preview=%" PRIu32 " bytes=%" PRIu32,
                     frames, s_display_frames, s_dropped_previews, b.bytesused);
        }
    }

cleanup:
    recorder_request(false);
    ioctl(fd, VIDIOC_STREAMOFF, &type);
    for (uint32_t i = 0; i < count; ++i) {
        if (buffers[i]) munmap(buffers[i], lengths[i]);
    }
    return ESP_FAIL;
}

void app_main(void)
{
    ESP_LOGI(TAG, "IMX708 -> ISP/IPA -> PPA -> full-screen ILI9488 preview");

    ESP_ERROR_CHECK(display_init());
    ESP_ERROR_CHECK(camera_power_on());

    ESP_ERROR_CHECK(esp_video_init_with_flags(&s_video_config,
                                              ESP_VIDEO_INIT_FLAGS_MIPI_CSI |
                                              ESP_VIDEO_INIT_FLAGS_ISP |
                                              ESP_VIDEO_INIT_FLAGS_MOTOR));

    ppa_client_config_t ppa_cfg = {
        .oper_type = PPA_OPERATION_SRM,
        .max_pending_trans_num = 1,
    };
    ESP_ERROR_CHECK(ppa_register_client(&ppa_cfg, &s_ppa));

    for (int i = 0; i < PREVIEW_BUFFER_COUNT; ++i) {
        s_preview[i] = heap_caps_aligned_alloc(PPA_CACHE_LINE_SIZE, PREVIEW_BUFFER_SIZE,
                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (!s_preview[i]) {
            ESP_LOGE(TAG, "preview buffer %d allocation failed", i);
            return;
        }
    }

    if (xTaskCreate(display_task, "lcd_preview", DISPLAY_TASK_STACK, NULL,
                    DISPLAY_TASK_PRIORITY, &s_display_task) != pdPASS) {
        ESP_LOGE(TAG, "display task creation failed");
        return;
    }

    ESP_ERROR_CHECK(shared_spi_init());
    esp_err_t touch_ret = touch_init();
    s_touch_available = touch_ret == ESP_OK;
    if (!s_touch_available) {
        ESP_LOGE(TAG, "touch init failed: %s; preview will continue", esp_err_to_name(touch_ret));
    }

    int fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "open %s failed: errno=%d", ESP_VIDEO_MIPI_CSI_DEVICE_NAME, errno);
        return;
    }

    run_preview(fd);
    close(fd);
}
