#include <stddef.h>
#include <string.h>
#include "tusb.h"
#include "camera_controls.h"

/* Wrap only VC entity requests. TinyUSB still owns VS PROBE/COMMIT and all
 * standard interface requests. Buffers outlive every EP0 transfer. */
extern bool __real_videod_control_xfer_cb(uint8_t, uint8_t, tusb_control_request_t const *);
extern uint8_t const *__real_tud_descriptor_configuration_cb(uint8_t);
extern uint8_t const *__real_tud_descriptor_device_cb(void);
static uint8_t s_payload[4], s_error;
static const icg_control_t *s_pending;
static uint8_t s_config[512];
static uint8_t s_device[18];

static uint16_t read16(const uint8_t *p) { return p[0] | (uint16_t)p[1] << 8; }
static void write16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }
static void pack(uint32_t v) { for (unsigned i = 0; i < 4; ++i) s_payload[i] = v >> (8*i); }
static bool fail(uint8_t error) { s_error = error; return false; }

/* Returns 0 on an unexpected descriptor layout, rather than enumerating a
 * camera whose descriptors and actual controls disagree. */
size_t icg_uvc_patch_descriptor(const uint8_t *src, size_t len, uint8_t *dst,
                              size_t cap, uint32_t ct_bits, uint32_t pu_bits)
{
    const size_t pu_len = 13;
    if (len < 9 || len + pu_len > cap || src[0] != 9 || src[1] != 2 || read16(src+2) != len) return 0;
    size_t in = 0, out = 0, vc_header = 0;
    unsigned cameras = 0, outputs = 0;
    bool vc = false;
    while (in < len) {
        unsigned n = src[in];
        if (n < 2 || in+n > len) return 0;
        const uint8_t *p = src+in;
        if (p[1] == 4) {
            if (n < 9) return 0;
            vc = p[2] == 0 && p[5] == 14 && p[6] == 1;
        }
        if (vc && p[1] == 0x24) {
            if (n < 3) return 0;
            if (p[2] == 1) {
                if (n < 13 || vc_header) return 0;
                vc_header = out;
            } else if (p[2] == 2) {
                if (n != 18 || p[3] != ICG_CT || read16(p+4) != 0x0201 || p[14] != 3) return 0;
                ++cameras;
            } else if (p[2] == 3) {
                if (n != 9 || p[3] != 2 || p[7] != ICG_CT || outputs) return 0;
                uint8_t pu[] = {13,0x24,5,ICG_PU,ICG_CT,0,0,3,
                               pu_bits,pu_bits>>8,pu_bits>>16,0,0};
                memcpy(dst+out, pu, sizeof(pu));
                out += sizeof(pu);
                ++outputs;
            } else return 0; /* This integration expects a single camera, no existing PU. */
        }
        memcpy(dst+out, p, n);
        if (vc && p[1] == 0x24 && p[2] == 2) {
            dst[out+15] = ct_bits; dst[out+16] = ct_bits>>8; dst[out+17] = ct_bits>>16;
        }
        if (vc && p[1] == 0x24 && p[2] == 3) dst[out+7] = ICG_PU;
        out += n; in += n;
    }
    if (cameras != 1 || outputs != 1 || !vc_header) return 0;
    write16(dst+2, out);
    write16(dst+vc_header+5, read16(dst+vc_header+5)+pu_len);
    return out;
}
uint8_t const *__wrap_tud_descriptor_configuration_cb(uint8_t index)
{
    const uint8_t *src = __real_tud_descriptor_configuration_cb(index);
    if (!src) return NULL;
    return icg_uvc_patch_descriptor(src, read16(src+2), s_config, sizeof(s_config),
            camera_control_bitmap(ICG_CT), camera_control_bitmap(ICG_PU)) ? s_config : NULL;
}
uint8_t const *__wrap_tud_descriptor_device_cb(void)
{
    memcpy(s_device, __real_tud_descriptor_device_cb(), sizeof(s_device));
    write16(s_device+12, 0x0101); /* Changed capabilities; help hosts invalidate cached descriptors. */
    return s_device;
}

bool __wrap_videod_control_xfer_cb(uint8_t rhport, uint8_t stage,
                                 tusb_control_request_t const *r)
{
    if (r->bmRequestType_bit.type != TUSB_REQ_TYPE_CLASS ||
        r->bmRequestType_bit.recipient != TUSB_REQ_RCPT_INTERFACE || (r->wIndex & 255) != 0)
        return __real_videod_control_xfer_cb(rhport, stage, r);
    uint8_t entity = r->wIndex >> 8, selector = r->wValue >> 8;
    if (!entity && selector != 2) return __real_videod_control_xfer_cb(rhport, stage, r);
    if (stage == CONTROL_STAGE_ACK) return true;
    if (stage == CONTROL_STAGE_DATA) {
        if (r->bRequest != 1) return true;
        const icg_control_t *c = s_pending;
        s_pending = NULL;
        if (!c || c->entity != entity || c->selector != selector) return fail(CTRL_INVALID_REQUEST);
        uint32_t value = 0;
        for (unsigned i = 0; i < c->size; ++i) value |= (uint32_t)s_payload[i] << (8*i);
        int error = camera_control_set(c, value);
        if (error) return fail(error);
        s_error = CTRL_OK;
        return true;
    }
    s_pending = NULL;
    if ((r->wValue & 255) || !r->wLength) return fail(CTRL_INVALID_REQUEST);
    bool input = r->bmRequestType_bit.direction != 0;
    if (!entity) { /* VideoControl REQUEST_ERROR_CODE_CONTROL */
        if (!input || r->wLength != 1) return fail(CTRL_INVALID_REQUEST);
        if (r->bRequest == 0x81) { pack(s_error); s_error = 0; }
        else if (r->bRequest == 0x86) pack(1);
        else return fail(CTRL_INVALID_REQUEST);
        return tud_control_xfer(rhport, r, s_payload, 1);
    }
    const icg_control_t *c = camera_control_find(entity, selector);
    if (!c) return fail(CTRL_INVALID_CONTROL);
    if (r->bRequest == 1) { /* SET_CUR */
        if (input || r->wLength != c->size) return fail(CTRL_INVALID_REQUEST);
        s_pending = c;
        memset(s_payload, 0, sizeof(s_payload));
        return tud_control_xfer(rhport, r, s_payload, c->size);
    }
    if (!input) return fail(CTRL_INVALID_REQUEST);
    uint32_t value;
    unsigned size = c->size;
    switch (r->bRequest) {
    case 0x81: {
        int error = camera_control_get(c, &value);
        if (error) return fail(error);
        break;
    }
    case 0x82: value = c->min; break;
    case 0x83: value = c->max; break;
    case 0x84: value = c->res; break;
    case 0x87: value = c->def; break;
    case 0x85: value = c->size; size = 2; break;
    case 0x86: value = camera_control_info(c); size = 1; break;
    default: return fail(CTRL_INVALID_REQUEST);
    }
    if (r->wLength != size) return fail(CTRL_INVALID_REQUEST);
    if (entity == ICG_PU && selector == PU_WB_COMPONENT &&
        (r->bRequest == 0x82 || r->bRequest == 0x83 || r->bRequest == 0x84))
        value |= value << 16; /* wBlue then wRed, each independently bounded. */
    pack(value);
    s_error = CTRL_OK;
    return tud_control_xfer(rhport, r, s_payload, size);
}
