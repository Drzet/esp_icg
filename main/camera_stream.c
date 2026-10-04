#include "camera_stream.h"
#include "camera_diag.h"

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <unistd.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "esp_video_device.h"
#include "esp_video_init.h"
#include "esp_video_ioctl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "linux/videodev2.h"

static const char *TAG = "p4d_camera";

/*
 * CSI flow follows Wireless-Tag's WT9932P4-TINY BSP implementation:
 * components/wt_bsp/features/csi/wt_bsp_csi.c
 *
 * Only the camera parameters differ:
 * OV9281, 1280x720, RAW8, 3 MMAP buffers.
 */
#define CAMERA_WIDTH             P4D_CAMERA_WIDTH
#define CAMERA_HEIGHT            P4D_CAMERA_HEIGHT
#define CAMERA_BUFFER_COUNT      3
#define CAMERA_TASK_STACK_SIZE   8192
#define CAMERA_TASK_PRIORITY     (tskIDLE_PRIORITY + 1)
#define CAMERA_LANE_BIT_RATE_HZ   800000000

typedef struct {
    uint8_t *buffers[CAMERA_BUFFER_COUNT];
    size_t buffer_size;
    uint32_t buffer_count;
    /* The sensor driver retains this pointer after VIDIOC_S_SENSOR_FMT. */
    esp_cam_sensor_format_t sensor_format;
    int fd;
    bool initialized;
    bool streaming;
    TaskHandle_t task;
    p4d_camera_frame_cb_t frame_cb;
    void *user_ctx;
} p4d_camera_state_t;

static p4d_camera_state_t s_camera = {
    .fd = -1,
};

typedef struct {
    bool exposure_pending;
    int32_t exposure_value;
    bool gain_pending;
    int32_t gain_value;
} p4d_camera_control_request_t;

static portMUX_TYPE s_control_lock = portMUX_INITIALIZER_UNLOCKED;
static p4d_camera_control_request_t s_control_request;
static p4d_camera_control_info_t s_control_info;
static bool s_control_info_valid;

static esp_err_t query_controls(int fd)
{
    struct v4l2_query_ext_ctrl exposure = {
        .id = V4L2_CID_EXPOSURE_ABSOLUTE,
    };
    if (ioctl(fd, VIDIOC_QUERY_EXT_CTRL, &exposure) != 0) {
        ESP_LOGE(TAG, "VIDIOC_QUERY_EXT_CTRL exposure failed: %d", errno);
        return ESP_FAIL;
    }

    struct v4l2_query_ext_ctrl gain = {
        .id = V4L2_CID_GAIN,
    };
    if (ioctl(fd, VIDIOC_QUERY_EXT_CTRL, &gain) != 0) {
        ESP_LOGE(TAG, "VIDIOC_QUERY_EXT_CTRL gain failed: %d", errno);
        return ESP_FAIL;
    }

    s_control_info.exposure_min = exposure.minimum;
    s_control_info.exposure_max = exposure.maximum;
    s_control_info.exposure_step = exposure.step ? exposure.step : 1;
    s_control_info.exposure_default = exposure.default_value;
    s_control_info.gain_min = gain.minimum;
    s_control_info.gain_max = gain.maximum;
    s_control_info.gain_default = gain.default_value;
    s_control_info_valid = true;

    ESP_LOGI(TAG, "controls: exposure=%ld..%ld step=%ld default=%ld (100 us), gain=%ld..%ld default=%ld",
             (long)s_control_info.exposure_min, (long)s_control_info.exposure_max,
             (long)s_control_info.exposure_step, (long)s_control_info.exposure_default,
             (long)s_control_info.gain_min, (long)s_control_info.gain_max,
             (long)s_control_info.gain_default);
    return ESP_OK;
}

static void apply_control_requests(int fd)
{
    p4d_camera_control_request_t req;

    portENTER_CRITICAL(&s_control_lock);
    req = s_control_request;
    memset(&s_control_request, 0, sizeof(s_control_request));
    portEXIT_CRITICAL(&s_control_lock);

    if (req.exposure_pending) {
        struct v4l2_ext_control ctrl = {
            .id = V4L2_CID_EXPOSURE_ABSOLUTE,
            .value = req.exposure_value,
        };
        struct v4l2_ext_controls ctrls = {
            .ctrl_class = V4L2_CID_CAMERA_CLASS,
            .count = 1,
            .controls = &ctrl,
        };
        if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrls) != 0) {
            ESP_LOGE(TAG, "setting exposure=%ld failed: %d",
                     (long)req.exposure_value, errno);
        }
    }

    if (req.gain_pending) {
        struct v4l2_ext_control ctrl = {
            .id = V4L2_CID_GAIN,
            .value = req.gain_value,
        };
        struct v4l2_ext_controls ctrls = {
            .ctrl_class = V4L2_CTRL_CLASS_USER,
            .count = 1,
            .controls = &ctrl,
        };
        if (ioctl(fd, VIDIOC_S_EXT_CTRLS, &ctrls) != 0) {
            ESP_LOGE(TAG, "setting gain=%ld failed: %d",
                     (long)req.gain_value, errno);
        }
    }
}

esp_err_t p4d_camera_get_control_info(p4d_camera_control_info_t *info)
{
    if (!info || !s_control_info_valid) {
        return ESP_ERR_INVALID_STATE;
    }
    *info = s_control_info;
    return ESP_OK;
}

void p4d_camera_request_exposure(int32_t value)
{
    portENTER_CRITICAL(&s_control_lock);
    s_control_request.exposure_value = value;
    s_control_request.exposure_pending = true;
    portEXIT_CRITICAL(&s_control_lock);
}

void p4d_camera_request_gain(int32_t value)
{
    portENTER_CRITICAL(&s_control_lock);
    s_control_request.gain_value = value;
    s_control_request.gain_pending = true;
    portEXIT_CRITICAL(&s_control_lock);
}

static esp_err_t configure_sensor(int fd)
{
    esp_cam_sensor_format_t *sensor = &s_camera.sensor_format;
    if (ioctl(fd, VIDIOC_G_SENSOR_FMT, sensor) != 0) {
        ESP_LOGE(TAG, "VIDIOC_G_SENSOR_FMT failed: %d", errno);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "sensor: %s, %lux%lu, %u fps, %u lanes, metadata=%lu bit/s",
             sensor->name, (unsigned long)sensor->width, (unsigned long)sensor->height,
             (unsigned)sensor->fps, (unsigned)sensor->mipi_info.lane_num,
             (unsigned long)sensor->mipi_info.mipi_clk);

    if (sensor->width != CAMERA_WIDTH || sensor->height != CAMERA_HEIGHT ||
        sensor->format != ESP_CAM_SENSOR_PIXFORMAT_RAW8 ||
        sensor->mipi_info.lane_num != 2 ||
        strcmp(sensor->name, "MIPI_2lane_24Minput_RAW8_1280x720_50fps") != 0) {
        ESP_LOGE(TAG, "expected OV9281 1280x720 RAW8 mode; regenerate sdkconfig from the updated defaults");
        return ESP_ERR_INVALID_STATE;
    }

    /*
     * esp_cam_sensor 2.0.1's OV9281 table specifies 800 Mbps, but its
     * mipi_info contains the 400 MHz DDR clock. esp_video 2.0.1 divides this
     * field by 1e6 directly into lane_bit_rate_mbps (no DDR multiplication).
     * Override the metadata through the public sensor-format API; retain the
     * original register table and ISP information. No PLL register changes.
     */
    sensor->mipi_info.mipi_clk = CAMERA_LANE_BIT_RATE_HZ;
    if (ioctl(fd, VIDIOC_S_SENSOR_FMT, sensor) != 0) {
        ESP_LOGE(TAG, "VIDIOC_S_SENSOR_FMT failed: %d", errno);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "CSI receiver lane rate: %u Mbps", CAMERA_LANE_BIT_RATE_HZ / 1000000);
    /* The mode table ends in 0x0100=1. Stop it before STREAMON configures
     * the receiver; esp_video starts the sensor after CSI and ISP are ready. */
    return p4d_camera_sensor_standby(fd);
}

esp_err_t p4d_camera_init(i2c_master_bus_handle_t i2c_bus)
{
    if (i2c_bus == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(&s_camera, 0, sizeof(s_camera));
    s_camera.fd = -1;

    esp_video_init_csi_config_t csi_config = {0};
    csi_config.sccb_config.init_sccb = false;
    csi_config.sccb_config.i2c_handle = i2c_bus;
    csi_config.sccb_config.freq = 400000;
    csi_config.reset_pin = -1;
    csi_config.pwdn_pin = -1;

    esp_video_init_csi_config_t csi_config_arr[] = { csi_config };

    esp_video_init_config_t video_config = {
        .csi = csi_config_arr,
    };

    /* Same startup delays used by the WT9932P4-TINY BSP CSI implementation. */
    vTaskDelay(pdMS_TO_TICKS(50));

    esp_err_t ret = esp_video_init(&video_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_video_init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(200));

    s_camera.initialized = true;
    ESP_LOGI(TAG, "CSI initialized successfully");
    return ESP_OK;
}

static void camera_stream_task(void *arg)
{
    p4d_camera_state_t *camera = (p4d_camera_state_t *)arg;
    const int fd = camera->fd;
    uint32_t frames = 0;
    uint32_t waits = 0;
    int64_t last_idle_break = esp_timer_get_time();

    ESP_LOGI(TAG, "waiting for first CSI frame (2 s timeout)");

    while (camera->streaming) {
        struct v4l2_buffer buf = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };

        const int64_t wait_start = esp_timer_get_time();
        if (ioctl(fd, VIDIOC_DQBUF, &buf) == 0) {
            if (buf.index >= camera->buffer_count || buf.bytesused > camera->buffer_size) {
                ESP_LOGE(TAG, "invalid capture buffer: index=%u bytes=%u",
                         (unsigned)buf.index, (unsigned)buf.bytesused);
                break;
            }
            if (frames++ == 0) {
                ESP_LOGI(TAG, "first CSI frame: index=%u bytes=%u flags=0x%lx",
                         (unsigned)buf.index, (unsigned)buf.bytesused, (unsigned long)buf.flags);
            }
            if (!(buf.flags & V4L2_BUF_FLAG_ERROR) && camera->frame_cb != NULL) {
                camera->frame_cb(
                    camera->buffers[buf.index],
                    buf.bytesused,
                    CAMERA_WIDTH,
                    CAMERA_HEIGHT,
                    camera->user_ctx);
            }

            if (ioctl(fd, VIDIOC_QBUF, &buf) != 0) {
                ESP_LOGE(TAG, "VIDIOC_QBUF failed: %d", errno);
                break;
            }

            apply_control_requests(fd);

            /* A continuously populated capture queue need not block DQBUF.
             * Let the idle task run periodically, after returning the buffer.
             * taskYIELD alone would not allow a lower-priority task to run. */
            if (esp_timer_get_time() - last_idle_break >= 100000) {
                vTaskDelay(1);
                last_idle_break = esp_timer_get_time();
            }
        } else {
            const int saved_errno = errno;
            /* esp_video 2.0.1 maps an empty timed-out dequeue to EPERM,
             * not EAGAIN/ETIMEDOUT. Confirm that the timeout elapsed. */
            if (saved_errno == ETIMEDOUT || saved_errno == EAGAIN ||
                (saved_errno == EPERM && esp_timer_get_time() - wait_start >= 1900000)) {
                if (waits++ % 5U == 0) {
                    ESP_LOGW(TAG, "no completed CSI frame for 2 s (received=%lu, errno=%d); display test pattern is independent",
                             (unsigned long)frames, saved_errno);
                    if (waits == 1) {
                        p4d_camera_diagnostics(fd, "first capture timeout");
                    }
                }
                vTaskDelay(1);
                continue;
            }
            ESP_LOGE(TAG, "VIDIOC_DQBUF failed: %d", saved_errno);
            break;
        }
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd, VIDIOC_STREAMOFF, &type);

    camera->streaming = false;
    close(fd);
    camera->fd = -1;
    camera->task = NULL;
    vTaskDelete(NULL);
}

esp_err_t p4d_camera_start(p4d_camera_frame_cb_t cb, void *user_ctx)
{
    if (!s_camera.initialized || cb == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    if (s_camera.streaming) {
        return ESP_OK;
    }

    const int fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDONLY);
    if (fd < 0) {
        ESP_LOGE(TAG, "failed to open %s", ESP_VIDEO_MIPI_CSI_DEVICE_NAME);
        return ESP_FAIL;
    }
    s_camera.fd = fd;

    if (configure_sensor(fd) != ESP_OK) {
        goto err;
    }
    if (query_controls(fd) != ESP_OK) {
        goto err;
    }

    struct v4l2_format format = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .fmt.pix.width = CAMERA_WIDTH,
        .fmt.pix.height = CAMERA_HEIGHT,
        .fmt.pix.pixelformat = V4L2_PIX_FMT_SBGGR8,
    };

    if (ioctl(fd, VIDIOC_S_FMT, &format) != 0) {
        ESP_LOGE(TAG, "failed to set RAW8 1280x720 format");
        goto err;
    }

    struct v4l2_streamparm parm = {
        .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    };
    parm.parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
    parm.parm.capture.timeperframe.numerator = 1;
    parm.parm.capture.timeperframe.denominator = 20;
    if (ioctl(fd, VIDIOC_S_PARM, &parm) != 0) {
        ESP_LOGW(TAG, "VIDIOC_S_PARM 20 fps failed: %d", errno);
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

    if (ioctl(fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "failed to request buffers");
        goto err;
    }

    if (req.count == 0 || req.count > CAMERA_BUFFER_COUNT) {
        ESP_LOGE(TAG, "invalid buffer count: %u", (unsigned)req.count);
        goto err;
    }
    s_camera.buffer_count = req.count;

    for (uint32_t i = 0; i < req.count; ++i) {
        struct v4l2_buffer buf = {
            .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index = i,
        };

        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) != 0) {
            ESP_LOGE(TAG, "failed to query buffer %u", (unsigned)i);
            goto err;
        }

        s_camera.buffers[i] = mmap(
            NULL,
            buf.length,
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            fd,
            buf.m.offset);

        if (s_camera.buffers[i] == MAP_FAILED) {
            ESP_LOGE(TAG, "failed to mmap buffer %u", (unsigned)i);
            goto err;
        }

        s_camera.buffer_size = buf.length;
        if (buf.length < (size_t)CAMERA_WIDTH * CAMERA_HEIGHT) {
            ESP_LOGE(TAG, "capture buffer too small: %u", (unsigned)buf.length);
            goto err;
        }

        if (ioctl(fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(TAG, "failed to queue buffer %u", (unsigned)i);
            goto err;
        }
    }

    struct timeval timeout = {.tv_sec = 2, .tv_usec = 0};
    if (ioctl(fd, VIDIOC_S_DQBUF_TIMEOUT, &timeout) != 0) {
        ESP_LOGE(TAG, "failed to set capture timeout: %d", errno);
        goto err;
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(fd, VIDIOC_STREAMON, &type) != 0) {
        ESP_LOGE(TAG, "failed to start stream");
        goto err;
    }

    s_camera.frame_cb = cb;
    s_camera.user_ctx = user_ctx;
    s_camera.streaming = true;

    BaseType_t res = xTaskCreate(
        camera_stream_task,
        "csi_stream",
        CAMERA_TASK_STACK_SIZE,
        &s_camera,
        CAMERA_TASK_PRIORITY,
        &s_camera.task);

    if (res != pdPASS) {
        ESP_LOGE(TAG, "failed to create stream task");
        s_camera.streaming = false;
        ioctl(fd, VIDIOC_STREAMOFF, &type);
        goto err;
    }

    ESP_LOGI(TAG, "OV9281 stream requested: 1280x720 RAW8; waiting for actual frames");
    return ESP_OK;

err:
    close(fd);
    s_camera.fd = -1;
    return ESP_FAIL;
}
