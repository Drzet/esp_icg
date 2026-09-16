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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "linux/videodev2.h"

#include "app_config.h"
#include "display.h"

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

/*
 * ESP32-P4 PPA scale factors are quantized to 1/16 steps. 1920x1080 ->
 * 480x280 cannot therefore use the naive ~0.259 scale: it is truncated to
 * 0.25, producing a smaller block and leaving stale strips at the right and
 * bottom. A centered 1536x896 crop scales exactly by 5/16 to 480x280.
 */
#define PREVIEW_CROP_WIDTH     1536
#define PREVIEW_CROP_HEIGHT    896
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

/*
 * The DW9807 autofocus VCM is a separate SCCB device from the IMX708 sensor.
 * It must be registered independently so esp_video probes and exposes it to
 * the ISP/IPA autofocus pipeline.
 */
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

/*
 * Exposure and gain are USER-class controls on the camera video node. Reading
 * them back while streaming tells us whether IPA/AE is actually commanding the
 * sensor, independently of what the preview looks like.
 */
static bool read_user_ctrl(int fd, uint32_t id, int32_t *out)
{
    struct v4l2_ext_control ctrl = {
        .id = id,
    };
    struct v4l2_ext_controls ctrls = {
        .ctrl_class = V4L2_CTRL_CLASS_USER,
        .count = 1,
        .controls = &ctrl,
    };

    if (ioctl(fd, VIDIOC_G_EXT_CTRLS, &ctrls) != 0) {
        return false;
    }

    *out = ctrl.value;
    return true;
}

/*
 * Pick a preview buffer that is neither being transmitted to the LCD nor the
 * newest frame waiting for the LCD. With three buffers there is always one
 * available: display-busy, latest-pending, and capture-write can all be
 * different buffers.
 */
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
    if (s_latest_preview >= 0) {
        ++s_dropped_previews;
    }
    s_latest_preview = index;
    portEXIT_CRITICAL(&s_preview_lock);

    xTaskNotifyGive(s_display_task);
}

/*
 * LCD is deliberately downstream of camera capture. It consumes only the
 * newest completed preview frame. If capture/PPA outruns SPI, older pending
 * previews are dropped rather than back-pressuring CSI/ISP/IPA.
 */
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

        if (index < 0) {
            continue;
        }

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

/*
 * PPA may hold the camera buffer while it reads it, but only for the hardware
 * scale/rotate/mirror operation. The expensive SPI LCD transfer happens later
 * from a separate preview buffer in display_task().
 */
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
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_180,
        .scale_x = PREVIEW_SCALE,
        .scale_y = PREVIEW_SCALE,
        .mirror_x = true,
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

    ESP_LOGI(TAG, "live preview started: capture/3A decoupled from LCD; crop=%dx%d scale=5/16 rotation=180 mirror_x=1",
             PREVIEW_CROP_WIDTH, PREVIEW_CROP_HEIGHT);
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

        /* Camera buffer is no longer needed after PPA finishes reading it. */
        if (ioctl(fd, VIDIOC_QBUF, &b) != 0) {
            ESP_LOGE(TAG, "VIDIOC_QBUF failed: errno=%d", errno);
            break;
        }

        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "PPA preview failed: %s", esp_err_to_name(ret));
            break;
        }

        preview_publish(preview_index);

        ++frames;
        if ((frames % 20) == 0) {
            int32_t exposure_lines = -1;
            int32_t gain_index = -1;
            bool exposure_ok = read_user_ctrl(fd, V4L2_CID_EXPOSURE, &exposure_lines);
            bool gain_ok = read_user_ctrl(fd, V4L2_CID_GAIN, &gain_index);

            if (exposure_ok && gain_ok) {
                ESP_LOGI(TAG, "AE readback: exposure=%" PRId32 " gain_idx=%" PRId32,
                         exposure_lines, gain_index);
            } else {
                ESP_LOGW(TAG, "AE readback failed: exposure=%d gain=%d errno=%d",
                         exposure_ok, gain_ok, errno);
            }
        }

        if ((frames % 100) == 0) {
            ESP_LOGI(TAG, "capture frames=%" PRIu32 " display=%" PRIu32
                     " dropped_preview=%" PRIu32 " bytes=%" PRIu32,
                     frames, s_display_frames, s_dropped_previews, b.bytesused);
        }
    }

    ioctl(fd, VIDIOC_STREAMOFF, &type);
    for (uint32_t i = 0; i < count; ++i) {
        if (buffers[i]) munmap(buffers[i], lengths[i]);
    }
    return ESP_FAIL;
}

void app_main(void)
{
    ESP_LOGI(TAG, "IMX708 -> ISP/IPA/AF -> PPA -> decoupled ILI9488 preview");

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

    int fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "open %s failed: errno=%d", ESP_VIDEO_MIPI_CSI_DEVICE_NAME, errno);
        return;
    }

    run_preview(fd);
    close(fd);
}
