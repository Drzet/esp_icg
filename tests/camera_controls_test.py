#!/usr/bin/env python3
"""Exercise actual camera control translation/ownership with fake V4L2 hardware."""
from pathlib import Path
import os
import re
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[1]
source = (ROOT/'main/camera_controls.c').read_text()
with tempfile.TemporaryDirectory() as tmp:
    d = Path(tmp)
    for name in ['fcntl.h','sys/ioctl.h','sys/lock.h','esp_log.h','esp_cam_sensor.h',
                 'esp_ipa_types.h','esp_video_device.h','esp_video_ioctl.h',
                 'esp_video_isp_ioctl.h','linux/videodev2.h','esp_err.h']:
        p=d/name;p.parent.mkdir(parents=True,exist_ok=True);p.write_text('')
    definitions = '\n'.join(f'#define {name} {i+100}' for i,name in enumerate(sorted(set(re.findall(r'\b(?:V4L2_CID_\w+|VIDIOC_\w+)\b',source)))))
    definitions += '\n'+'\n'.join(f'#define {name} (1u<<{i})' for i,name in enumerate(sorted(set(re.findall(r'\bIPA_METADATA_FLAGS_\w+\b',source)))))
    shim = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_FAIL -1
#define ESP_LOGI(tag,...) ((void)tag)
#define ESP_VIDEO_ISP1_DEVICE_NAME "isp"
#define O_RDWR 2
#define ISP_GAMMA_CURVE_POINTS_NUM 16
#define ESP_VIDEO_ISP_GAMMA_EXT_FLAG_RED 1
#define ESP_VIDEO_ISP_GAMMA_EXT_FLAG_GREEN 2
#define ESP_VIDEO_ISP_GAMMA_EXT_FLAG_BLUE 4
typedef int _lock_t;
static void _lock_acquire(_lock_t*p){assert(!*p);*p=1;}
static void _lock_release(_lock_t*p){assert(*p);*p=0;}
struct v4l2_ext_control {uint32_t id;int32_t value;uint8_t*p_u8;uint32_t size;};
struct v4l2_ext_controls {uint32_t ctrl_class,count;struct v4l2_ext_control *controls;};
struct v4l2_query_ext_ctrl {uint32_t id;int64_t minimum,maximum;uint64_t step;};
typedef struct {bool enable;float red_gain,blue_gain;} esp_video_isp_wb_t;
typedef struct {bool enable;float h_coeff,m_coeff;uint8_t h_thresh,l_thresh,matrix[3][3];} esp_video_isp_sharpen_t;
typedef struct {uint8_t x,y;} esp_video_isp_gamma_point_t;
typedef struct {bool enable;uint32_t flags;esp_video_isp_gamma_point_t red_points[16],green_points[16],blue_points[16];} esp_video_isp_gamma_ext_t;
struct info {struct {uint32_t tline_ns;} isp_v1_info;};
typedef struct {struct info *isp_info;} esp_cam_sensor_format_t;
static int values[256], fail_write;
static esp_video_isp_wb_t wb={true,1.2236f,1.3645f};
static esp_video_isp_sharpen_t sharp={.enable=true,.h_coeff=.4f,.m_coeff=.6f};
static esp_video_isp_gamma_ext_t gamma_curve={.enable=true};
static int open(const char*p,int flags){(void)p;(void)flags;return 10;}
static int ioctl(int fd,unsigned long op,void*arg) {
 (void)fd;
 if(op==VIDIOC_G_SENSOR_FMT){static struct info info={{13505}};((esp_cam_sensor_format_t*)arg)->isp_info=&info;return 0;}
 if(op==VIDIOC_QUERY_EXT_CTRL){
  struct v4l2_query_ext_ctrl*q=arg;q->step=1;q->minimum=0;q->maximum=255;
  if(q->id==V4L2_CID_EXPOSURE){q->minimum=4;q->maximum=2624;}
  if(q->id==V4L2_CID_GAIN)q->maximum=46;
  if(q->id==V4L2_CID_FOCUS_ABSOLUTE)q->maximum=1023;
  if(q->id==V4L2_CID_BRIGHTNESS){q->minimum=-128;q->maximum=127;}
  if(q->id==V4L2_CID_HUE)q->maximum=360;
  return 0;
 }
 assert(op==VIDIOC_G_EXT_CTRLS || op==VIDIOC_S_EXT_CTRLS);
 struct v4l2_ext_control*c=((struct v4l2_ext_controls*)arg)->controls;
 bool write=op==VIDIOC_S_EXT_CTRLS;
 if(write && fail_write)return -1;
 void *data=NULL;size_t size=0;
 if(c->id==V4L2_CID_USER_ESP_ISP_WB){data=&wb;size=sizeof(wb);}
 if(c->id==V4L2_CID_USER_ESP_ISP_SHARPEN){data=&sharp;size=sizeof(sharp);}
 if(c->id==V4L2_CID_USER_ESP_ISP_GAMMA_EXT){data=&gamma_curve;size=sizeof(gamma_curve);}
 if(data){assert(c->size==size);if(write)memcpy(data,c->p_u8,size);else memcpy(c->p_u8,data,size);}
 else {assert(c->id<256);if(write)values[c->id]=c->value;else c->value=values[c->id];}
 return 0;
}
#include "camera_controls.c"
static const icg_control_t* control(uint8_t e,uint8_t s){const icg_control_t*c=camera_control_find(e,s);assert(c);return c;}
static uint32_t get(uint8_t e,uint8_t s){uint32_t v;assert(camera_control_get(control(e,s),&v)==0);return v;}
static int set(uint8_t e,uint8_t s,uint32_t v){return camera_control_set(control(e,s),v);}
static uint32_t mask(void){uint32_t m=icg_ipa_controls_begin();icg_ipa_controls_end();return m;}
int main(void){
 values[V4L2_CID_EXPOSURE]=2495;values[V4L2_CID_GAIN]=16;values[V4L2_CID_FOCUS_ABSOLUTE]=552;
 values[V4L2_CID_CONTRAST]=130;values[V4L2_CID_SATURATION]=128;
 for(int i=0;i<16;i++) gamma_curve.red_points[i]=gamma_curve.green_points[i]=gamma_curve.blue_points[i]=(esp_video_isp_gamma_point_t){i*16+15,i*16+15};
 assert(camera_controls_init(1)==0);assert(mask()==0);
 assert(camera_control_bitmap(ICG_CT)==0x2a && camera_control_bitmap(ICG_PU)==0x22bf);
 assert(get(ICG_CT,CT_AE_MODE)==2);
 assert(set(ICG_CT,CT_EXPOSURE,200)==CTRL_WRONG_STATE);
 assert(set(ICG_PU,PU_GAIN,20)==CTRL_WRONG_STATE);
 assert(set(ICG_CT,CT_AE_MODE,8)==CTRL_RANGE);
 assert(set(ICG_CT,CT_AE_MODE,1)==0 && mask()==AE_MASK);
 assert(set(ICG_CT,CT_EXPOSURE,200)==0);assert(values[V4L2_CID_EXPOSURE]==1481);
 assert(get(ICG_CT,CT_EXPOSURE)==200);
 assert(set(ICG_PU,PU_GAIN,46)==0 && values[V4L2_CID_GAIN]==46);
 assert(set(ICG_PU,PU_GAIN,47)==CTRL_RANGE);
 assert(set(ICG_CT,CT_EXPOSURE,0)==CTRL_RANGE);
 assert(set(ICG_CT,CT_AE_MODE,2)==0 && mask()==0);
 icg_ipa_controls_begin();assert(icg_ipa_controls_reset_ae());assert(!icg_ipa_controls_reset_ae());icg_ipa_controls_end();
 assert(set(ICG_CT,CT_FOCUS,508)==CTRL_RANGE);assert(set(ICG_CT,CT_FOCUS,605)==0);assert(get(ICG_CT,CT_FOCUS)==605);
 assert(set(ICG_PU,PU_WB_COMPONENT,0x03e807d0)==CTRL_WRONG_STATE);
 assert(set(ICG_PU,PU_WB_AUTO,0)==0 && mask()==WB_MASK);
 assert(set(ICG_PU,PU_WB_COMPONENT,(2000u<<16)|1000)==0);
 assert(wb.red_gain==2 && wb.blue_gain==1);assert(get(ICG_PU,PU_WB_COMPONENT)==((2000u<<16)|1000));
 assert(set(ICG_PU,PU_WB_COMPONENT,0)==CTRL_RANGE);
 assert(set(ICG_PU,PU_WB_AUTO,1)==0 && mask()==0);
 assert(set(ICG_PU,PU_BRIGHTNESS,(uint16_t)-20)==0);assert(values[V4L2_CID_BRIGHTNESS]==-20);
 assert(mask()==IPA_METADATA_FLAGS_BR);
 assert(set(ICG_PU,PU_HUE,(uint16_t)-30)==0);assert(values[V4L2_CID_HUE]==330);assert((int32_t)get(ICG_PU,PU_HUE)==-30);
 fail_write=1;uint32_t before=mask();assert(set(ICG_PU,PU_CONTRAST,100)==CTRL_NOT_READY);assert(mask()==before);fail_write=0;
 assert(set(ICG_PU,PU_SHARPNESS,0)==0 && !sharp.enable);
 assert(set(ICG_PU,PU_SHARPNESS,100)==0 && sharp.enable && sharp.h_coeff==.4f);
 assert(set(ICG_PU,PU_GAMMA,0)==CTRL_RANGE);assert(set(ICG_PU,PU_GAMMA,200)==0);assert(gamma_curve.red_points[0].y>15);
 assert(set(ICG_PU,PU_GAMMA,100)==0 && gamma_curve.red_points[0].y==15);
 puts("PASS: sensor units/ranges, AE and WB ownership, signed ISP controls, focus, failures, tuning restoration");
}
'''
    (d/'test.c').write_text(definitions+'\n'+shim)
    subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',
                    '-I'+str(d),'-I'+str(ROOT/'main'),str(d/'test.c'),'-lm','-o',str(d/'test')],check=True)
    subprocess.run([str(d/'test')],env={**os.environ,'ASAN_OPTIONS':'detect_leaks=0'},check=True)
