#!/usr/bin/env python3
"""Compile the actual EP0/descriptor implementation against a fake USB host."""
from pathlib import Path
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory() as tmp:
    d = Path(tmp)
    (d/'esp_err.h').write_text('typedef int esp_err_t;\n')
    (d/'tusb.h').write_text('''#pragma once
#include <stdbool.h>
#include <stdint.h>
#define TUSB_REQ_TYPE_CLASS 1
#define TUSB_REQ_RCPT_INTERFACE 1
#define CONTROL_STAGE_SETUP 0
#define CONTROL_STAGE_DATA 1
#define CONTROL_STAGE_ACK 2
typedef struct { union { uint8_t bmRequestType; struct {uint8_t recipient:5,type:2,direction:1;} bmRequestType_bit; }; uint8_t bRequest; uint16_t wValue,wIndex,wLength; } tusb_control_request_t;
bool tud_control_xfer(uint8_t,const tusb_control_request_t*,void*,uint16_t);
''')
    (d/'test.c').write_text(r'''
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "tusb.h"
#include "camera_controls.h"
static uint8_t reply[4];
static void *out_buffer;
static int forwarded, writes, set_error;
static uint32_t last_value;
static icg_control_t controls[] = {
 {ICG_CT,CT_EXPOSURE,3,4,false,true,1,350,1,100,0,0},
 {ICG_PU,PU_BRIGHTNESS,0,2,true,true,-128,127,1,0,0,0},
 {ICG_PU,PU_WB_COMPONENT,7,4,false,true,1,3999,1,0x04c70554,0,0},
};
const icg_control_t *camera_control_find(uint8_t e,uint8_t s) {
 for(unsigned i=0;i<3;i++) if(controls[i].entity==e && controls[i].selector==s) return controls+i;
 return NULL;
}
uint32_t camera_control_bitmap(uint8_t e) { return e==ICG_CT ? 0x2a : 0x22bf; }
uint8_t camera_control_info(const icg_control_t*c) { (void)c;return 3; }
int camera_control_get(const icg_control_t*c,uint32_t*v) { *v=c->def;return 0; }
int camera_control_set(const icg_control_t*c,uint32_t v) { (void)c;writes++;last_value=v;return set_error; }
bool tud_control_xfer(uint8_t p,const tusb_control_request_t*r,void*b,uint16_t n) {
 (void)p;assert(n<=4); if(r->bmRequestType_bit.direction) memcpy(reply,b,n);else out_buffer=b;return true;
}
bool __real_videod_control_xfer_cb(uint8_t p,uint8_t s,const tusb_control_request_t*r) {
 (void)p;(void)s;(void)r;forwarded++;return true;
}
static const uint8_t dev[18]={18,1};
uint8_t const *__real_tud_descriptor_device_cb(void){return dev;}
static const uint8_t cfg[]={
 9,2,74,0,2,1,0,128,250,
 8,11,0,2,14,3,0,0,
 9,4,0,0,0,14,1,1,0,
 13,0x24,1,0x50,1,40,0,0,0,0,0,1,1,
 18,0x24,2,1,1,2,0,0,0,0,0,0,0,0,3,0,0,0,
 9,0x24,3,2,1,1,0,1,0,
 8,0x24,99,0,0,0,0,0 /* deliberately invalid VC descriptor; fixed below */
};
uint8_t const *__real_tud_descriptor_configuration_cb(uint8_t i){(void)i;return cfg;}
#include "uvc_controls.c"
static tusb_control_request_t req(uint8_t e,uint8_t s,uint8_t op,uint16_t len) {
 tusb_control_request_t r={.bmRequestType=op==1?0x21:0xa1,.bRequest=op,.wValue=(uint16_t)s<<8,.wIndex=(uint16_t)e<<8,.wLength=len};return r;
}
int main(void) {
 uint8_t src[75], dst[100];
 /* Replace the final invalid descriptor with a real VS interface. */
 memcpy(src,cfg,66); uint8_t vs[]={9,4,1,0,1,14,2,1,0};memcpy(src+66,vs,9);src[2]=75;
 size_t n=icg_uvc_patch_descriptor(src,75,dst,sizeof(dst),0x2a,0x22bf);
 assert(n==88 && read16(dst+2)==88 && read16(dst+26+5)==53);
 assert(dst[39+15]==0x2a && dst[57]==13 && dst[57+3]==3 && dst[57+4]==1);
 assert(dst[57+8]==0xbf && dst[57+9]==0x22 && dst[70+7]==3);
 assert(!memcmp(dst+79,vs,9));
 assert(!icg_uvc_patch_descriptor(src,75,dst,80,0,0));
 src[39]=0;assert(!icg_uvc_patch_descriptor(src,75,dst,100,0,0));src[39]=18;
 assert(__wrap_tud_descriptor_device_cb()[12]==1);
 tusb_control_request_t r=req(ICG_PU,PU_BRIGHTNESS,0x82,2);
 assert(__wrap_videod_control_xfer_cb(0,0,&r));assert(reply[0]==128 && reply[1]==255);
 r=req(ICG_PU,PU_WB_COMPONENT,0x83,4);assert(__wrap_videod_control_xfer_cb(0,0,&r));
 assert(read16(reply)==3999 && read16(reply+2)==3999);
 r.bRequest=0x87;assert(__wrap_videod_control_xfer_cb(0,0,&r));assert(read16(reply)==0x554 && read16(reply+2)==0x4c7);
 r=req(ICG_CT,CT_EXPOSURE,1,4);assert(__wrap_videod_control_xfer_cb(0,0,&r));
 uint8_t val[]={0x23,1,0,0};memcpy(out_buffer,val,4);assert(writes==0);
 assert(__wrap_videod_control_xfer_cb(0,1,&r));assert(writes==1 && last_value==291);
 assert(__wrap_videod_control_xfer_cb(0,2,&r));assert(writes==1);
 r.wLength=2;assert(!__wrap_videod_control_xfer_cb(0,0,&r));assert(writes==1);
 r.wLength=4;assert(__wrap_videod_control_xfer_cb(0,0,&r));set_error=CTRL_RANGE;
 assert(!__wrap_videod_control_xfer_cb(0,1,&r));
 r=req(0,2,0x81,1);assert(__wrap_videod_control_xfer_cb(0,0,&r));assert(reply[0]==CTRL_RANGE);
 assert(__wrap_videod_control_xfer_cb(0,0,&r));assert(reply[0]==0);
 r=req(1,99,0x81,1);assert(!__wrap_videod_control_xfer_cb(0,0,&r));
 r=req(0,1,0x81,26);r.wIndex=1;assert(__wrap_videod_control_xfer_cb(0,0,&r));assert(forwarded==1);
 r.bmRequestType=1;assert(__wrap_videod_control_xfer_cb(0,0,&r));assert(forwarded==2);
 puts("PASS: UVC descriptor chain, GET/SET stages, signed and component values, lengths, errors, VS passthrough");
}
''')
    subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',
                    '-I'+str(d),'-I'+str(ROOT/'main'),str(d/'test.c'),'-o',str(d/'test')],check=True)
    import os
    subprocess.run([str(d/'test')],env={**os.environ,'ASAN_OPTIONS':'detect_leaks=0'},check=True)
