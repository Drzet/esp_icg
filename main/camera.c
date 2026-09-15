#include "camera.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/ppa.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_video_init.h"
#include "linux/videodev2.h"
#include "app_config.h"
#include "display.h"
#include "recorder.h"

#define CAMERA_DEV "/dev/video0"
#define CAMERA_WIDTH 800
#define CAMERA_HEIGHT 640
#define CAMERA_BUFFERS 3
#define CAMERA_SCCB_SCL 8
#define CAMERA_SCCB_SDA 7
#define CAMERA_POWER_GPIO 0
#define PPA_CACHE_LINE_SIZE 128
#define PREVIEW_BUFFER_SIZE (ICG_LCD_WIDTH * ICG_PREVIEW_HEIGHT * sizeof(uint16_t))

static const char *TAG = "camera";
static ppa_client_handle_t s_ppa;
static uint16_t *s_preview;
static volatile bool s_running;
static TaskHandle_t s_task;
static int s_fd = -1;
static void *s_buffers[CAMERA_BUFFERS];
static size_t s_buffer_lengths[CAMERA_BUFFERS];
static uint32_t s_buffer_count;
static uint32_t s_frame_width = CAMERA_WIDTH;
static uint32_t s_frame_height = CAMERA_HEIGHT;

static void cleanup_capture(void)
{
    for (uint32_t i = 0; i < s_buffer_count; ++i) {
        if (s_buffers[i] && s_buffers[i] != MAP_FAILED) {
            munmap(s_buffers[i], s_buffer_lengths[i]);
        }
        s_buffers[i] = NULL;
        s_buffer_lengths[i] = 0;
    }
    s_buffer_count = 0;
    if (s_fd >= 0) {
        close(s_fd);
        s_fd = -1;
    }
}

static void render_frame(uint8_t *buf, uint32_t width, uint32_t height, size_t len)
{
    recorder_submit_rgb565(buf, width, height, len);

    uint32_t crop_w = width;
    uint32_t crop_h = height;
    if ((uint64_t)width * ICG_PREVIEW_HEIGHT > (uint64_t)height * ICG_LCD_WIDTH) {
        crop_w = (uint32_t)((uint64_t)height * ICG_LCD_WIDTH / ICG_PREVIEW_HEIGHT);
    } else {
        crop_h = (uint32_t)((uint64_t)width * ICG_PREVIEW_HEIGHT / ICG_LCD_WIDTH);
    }

    ppa_srm_oper_config_t op = {
        .in = {
            .buffer = buf,
            .pic_w = width,
            .pic_h = height,
            .block_w = crop_w,
            .block_h = crop_h,
            .block_offset_x = (width - crop_w) / 2,
            .block_offset_y = (height - crop_h) / 2,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .out = {
            .buffer = s_preview,
            .buffer_size = PREVIEW_BUFFER_SIZE,
            .pic_w = ICG_LCD_WIDTH,
            .pic_h = ICG_PREVIEW_HEIGHT,
            .block_offset_x = 0,
            .block_offset_y = 0,
            .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
        },
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
        .scale_x = (float)ICG_LCD_WIDTH / (float)crop_w,
        .scale_y = (float)ICG_PREVIEW_HEIGHT / (float)crop_h,
        .mirror_x = true,
        .mode = PPA_TRANS_MODE_BLOCKING,
    };

    esp_err_t ret = ppa_do_scale_rotate_mirror(s_ppa, &op);
    if (ret == ESP_OK) ret = display_draw_preview(s_preview);
    if (ret != ESP_OK) ESP_LOGW(TAG, "preview frame failed: %s", esp_err_to_name(ret));
}

static void capture_task(void *arg)
{
    (void)arg;
    while (s_running) {
        struct v4l2_buffer b = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        if (ioctl(s_fd, VIDIOC_DQBUF, &b) != 0) {
            if (errno == EAGAIN) {
                vTaskDelay(pdMS_TO_TICKS(2));
                continue;
            }
            ESP_LOGW(TAG, "DQBUF failed: errno=%d", errno);
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if ((b.flags & V4L2_BUF_FLAG_DONE) && b.index < s_buffer_count) {
            render_frame((uint8_t *)s_buffers[b.index], s_frame_width, s_frame_height, b.bytesused);
        }

        if (ioctl(s_fd, VIDIOC_QBUF, &b) != 0) {
            ESP_LOGW(TAG, "QBUF failed: errno=%d", errno);
        }
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(s_fd, VIDIOC_STREAMOFF, &type);
    cleanup_capture();
    s_task = NULL;
    vTaskDelete(NULL);
}

esp_err_t camera_init(void)
{
    gpio_config_t pwr = {
        .pin_bit_mask = 1ULL << CAMERA_POWER_GPIO,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&pwr), TAG, "camera power gpio");
    gpio_set_level(CAMERA_POWER_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(100));

    esp_video_init_csi_config_t csi = {
        .sccb_config = {
            .init_sccb = true,
            .i2c_config = {
                .port = 0,
                .scl_pin = CAMERA_SCCB_SCL,
                .sda_pin = CAMERA_SCCB_SDA,
            },
            .freq = 400000,
        },
        .reset_pin = -1,
        .pwdn_pin = -1,
        .dont_init_ldo = false,
    };
    esp_video_init_config_t video_cfg = {
        .csi = &csi,
    };
    ESP_RETURN_ON_ERROR(esp_video_init_with_flags(&video_cfg,
                        ESP_VIDEO_INIT_FLAGS_MIPI_CSI | ESP_VIDEO_INIT_FLAGS_ISP), TAG, "esp_video init");

    ppa_client_config_t ppa_cfg = {
        .oper_type = PPA_OPERATION_SRM,
        .max_pending_trans_num = 1,
    };
    ESP_RETURN_ON_ERROR(ppa_register_client(&ppa_cfg, &s_ppa), TAG, "PPA client");

    s_preview = heap_caps_aligned_alloc(PPA_CACHE_LINE_SIZE, PREVIEW_BUFFER_SIZE,
                                        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_preview) return ESP_ERR_NO_MEM;
    ESP_LOGI(TAG, "PPA preview buffer @%p, size=%u, alignment=%u",
             s_preview, (unsigned)PREVIEW_BUFFER_SIZE, PPA_CACHE_LINE_SIZE);
    return ESP_OK;
}

esp_err_t camera_start(void)
{
    if (s_running) return ESP_OK;

    memset(s_buffers, 0, sizeof(s_buffers));
    memset(s_buffer_lengths, 0, sizeof(s_buffer_lengths));
    s_buffer_count = 0;

    s_fd = open(CAMERA_DEV, O_RDONLY | O_NONBLOCK);
    if (s_fd < 0) return ESP_FAIL;

    struct v4l2_format fmt = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .fmt.pix = {
            .width = CAMERA_WIDTH,
            .height = CAMERA_HEIGHT,
            .pixelformat = V4L2_PIX_FMT_RGB565,
        },
    };
    if (ioctl(s_fd, VIDIOC_S_FMT, &fmt) != 0) goto fail;
    if (fmt.fmt.pix.pixelformat != V4L2_PIX_FMT_RGB565) {
        ESP_LOGE(TAG, "RGB565 was not accepted, fourcc=0x%08lx", (unsigned long)fmt.fmt.pix.pixelformat);
        goto fail;
    }
    s_frame_width = fmt.fmt.pix.width;
    s_frame_height = fmt.fmt.pix.height;
    ESP_LOGI(TAG, "Camera format %lux%lu RGB565", (unsigned long)s_frame_width, (unsigned long)s_frame_height);

    struct v4l2_requestbuffers req = {
        .count = CAMERA_BUFFERS,
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(s_fd, VIDIOC_REQBUFS, &req) != 0 || req.count < 2) goto fail;
    s_buffer_count = req.count > CAMERA_BUFFERS ? CAMERA_BUFFERS : req.count;

    for (uint32_t i = 0; i < s_buffer_count; ++i) {
        struct v4l2_buffer b = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index = i,
        };
        if (ioctl(s_fd, VIDIOC_QUERYBUF, &b) != 0) goto fail;
        s_buffer_lengths[i] = b.length;
        s_buffers[i] = mmap(NULL, b.length, PROT_READ | PROT_WRITE, MAP_SHARED, s_fd, b.m.offset);
        if (s_buffers[i] == MAP_FAILED) {
            s_buffers[i] = NULL;
            goto fail;
        }
        if (ioctl(s_fd, VIDIOC_QBUF, &b) != 0) goto fail;
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(s_fd, VIDIOC_STREAMON, &type) != 0) goto fail;

    s_running = true;
    if (xTaskCreate(capture_task, "camera", 8192, NULL, 6, &s_task) != pdPASS) {
        s_running = false;
        ioctl(s_fd, VIDIOC_STREAMOFF, &type);
        goto fail;
    }
    return ESP_OK;

fail:
    cleanup_capture();
    return ESP_FAIL;
}

esp_err_t camera_stop(void)
{
    if (!s_running) return ESP_OK;
    recorder_set_active(false);
    s_running = false;
    for (int i = 0; i < 100 && s_task != NULL; ++i) vTaskDelay(pdMS_TO_TICKS(5));
    return s_task == NULL ? ESP_OK : ESP_ERR_TIMEOUT;
}

bool camera_running(void)
{
    return s_running;
}
