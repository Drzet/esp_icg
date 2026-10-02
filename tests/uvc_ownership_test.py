from pathlib import Path
import re, subprocess, tempfile, os
root=Path(__file__).resolve().parents[1]
s=(root/'main/uvc_stream.c').read_text()
s=re.sub(r'^#include .*\n','',s,flags=re.M)
s=s[:s.index('esp_err_t uvc_stream_init(void)')]
pre=r'''
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <inttypes.h>
#include <assert.h>
#include <stdio.h>
#include <sys/time.h>
#define ICG_UVC_WIDTH 1920
#define ICG_UVC_HEIGHT 1080
#define ICG_UVC_QUALITY 80
#define CONFIG_UVC_CAM1_FRAMERATE 28
#define ESP_OK 0
#define ESP_ERR_NOT_SUPPORTED 1
#define ESP_ERR_INVALID_STATE 2
#define JPEG_ENCODE_IN_FORMAT_RGB565 1
#define JPEG_DOWN_SAMPLING_YUV422 1
#define UVC_FORMAT_JPEG 1
#define portMUX_INITIALIZER_UNLOCKED 0
#define portENTER_CRITICAL(x) ((void)(x))
#define portEXIT_CRITICAL(x) ((void)(x))
#define pdMS_TO_TICKS(x) (x)
#define pdTRUE 1
#define ESP_LOGI(tag,fmt,...) ((void)(tag))
#define ESP_LOGE(tag,fmt,...) ((void)(tag))
typedef int portMUX_TYPE,esp_err_t,uvc_format_t,SemaphoreHandle_t,jpeg_encoder_handle_t;
typedef struct {int width,height,src_type,sub_sample,image_quality;} jpeg_encode_cfg_t;
typedef struct {uint8_t *buf;size_t len,width,height;int format;struct timeval timestamp;} uvc_fb_t;
static int64_t now=100;
static bool mounted=true, suspended=false, streaming=true;
static bool signal_ready;
static int calls,encode_error;
static void (*during_encode)(void);
static int64_t esp_timer_get_time(void){return now;}
static bool tud_mounted(void){return mounted;}
static bool tud_suspended(void){return suspended;}
static bool tud_video_n_streaming(int a,int b){(void)a;(void)b;return streaming;}
static int xSemaphoreTake(int s,int ticks){(void)s;(void)ticks;bool v=signal_ready;signal_ready=false;return v;}
static void xSemaphoreGive(int s){(void)s;signal_ready=true;}
static int jpeg_encoder_process(int e,const jpeg_encode_cfg_t*c,const uint8_t*in,size_t len,uint8_t*out,size_t cap,uint32_t*size){
 (void)e;(void)c;(void)in;(void)len;(void)out;(void)cap;calls++;now+=29000;*size=120000;
 if(during_encode)during_encode();return encode_error;
}
'''
test=r'''
static void stop_during_encode(void){stop_cb(NULL);}
int main(void){
 uint8_t dummy=0;
 s_capacity=JPEG_CAPACITY;s_jpeg=&dummy;s_frame.buf=&dummy;
 assert(start_cb(UVC_FORMAT_JPEG,1920,1080,28,NULL)==ESP_OK);
 assert(start_cb(UVC_FORMAT_JPEG,640,480,28,NULL)==ESP_ERR_NOT_SUPPORTED);
 uvc_stream_submit(&dummy,JPEG_CAPACITY);assert(calls==1 && s_slot==SLOT_READY);
 uvc_stream_submit(&dummy,JPEG_CAPACITY);assert(calls==1); // no wasted encode when full
 uvc_fb_t *f=get_cb(NULL);assert(f==&s_frame && s_slot==SLOT_USB);
 uvc_stream_submit(&dummy,JPEG_CAPACITY);assert(calls==1); // consumer owns memory
 stop_cb(NULL);assert(s_slot==SLOT_USB);
 start_cb(UVC_FORMAT_JPEG,1920,1080,28,NULL);assert(s_slot==SLOT_USB);
 return_cb(f,NULL);assert(s_slot==SLOT_FREE);
 during_encode=stop_during_encode;
 uvc_stream_submit(&dummy,JPEG_CAPACITY);assert(calls==2 && s_slot==SLOT_FREE);
 assert(get_cb(NULL)==NULL); // stopped in-flight work cannot leak into new session
 during_encode=NULL;start_cb(UVC_FORMAT_JPEG,1920,1080,28,NULL);
 encode_error=1;uvc_stream_submit(&dummy,JPEG_CAPACITY);assert(s_slot==SLOT_FREE);
 encode_error=0;uvc_stream_submit(&dummy,JPEG_CAPACITY);assert(s_slot==SLOT_READY);
 uvc_stream_discard_ready();assert(s_slot==SLOT_FREE);assert(get_cb(NULL)==NULL);
 now+=HOST_IDLE_US;assert(!uvc_stream_active());
 get_cb(NULL);assert(uvc_stream_active()); // new demand resumes after idle
 mounted=false;assert(!uvc_stream_active());mounted=true;
 suspended=true;assert(!uvc_stream_active());suspended=false;
 streaming=false;assert(!uvc_stream_active());streaming=true;
 uvc_stream_disable();assert(start_cb(UVC_FORMAT_JPEG,1920,1080,28,NULL)==ESP_ERR_INVALID_STATE);
 (void)s_transfer;
 puts("PASS: backpressure, ownership, restart, stop during encode, error release, idle and disconnect");
}
'''
with tempfile.TemporaryDirectory(prefix='icg-uvc-test-') as temp:
    p=Path(temp)/'test.c'
    binary=Path(temp)/'test'
    p.write_text(pre+s+test)
    subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Wno-unused-variable',
                    '-Wno-misleading-indentation','-fsanitize=address,undefined',
                    str(p),'-o',str(binary)],check=True)
    subprocess.run([str(binary)],check=True,env={**os.environ,'ASAN_OPTIONS':'detect_leaks=0'})
