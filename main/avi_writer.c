#include "avi_writer.h"
#include <string.h>

#define HEADER_SIZE 224u
#define MAX_FILE_SIZE (1024u * 1024u * 1024u)

static void put32(uint8_t *p, uint32_t v)
{
    p[0] = v; p[1] = v >> 8; p[2] = v >> 16; p[3] = v >> 24;
}
static void put16(uint8_t *p, uint16_t v) { p[0] = v; p[1] = v >> 8; }

static bool header(avi_writer_t *a, bool indexed)
{
    uint8_t h[HEADER_SIZE] = {0};
    uint32_t period = a->period_us;
    if (a->frames > 1) {
        int64_t mean = (a->last_us - a->first_us) / (a->frames - 1);
        if (mean > 0 && mean <= UINT32_MAX) period = (uint32_t)mean;
    }
    uint32_t file_end = a->end + (indexed ? 8 + a->frames * 16 : 0);
    memcpy(h, "RIFF", 4); put32(h + 4, file_end - 8); memcpy(h + 8, "AVI ", 4);
    memcpy(h + 12, "LIST", 4); put32(h + 16, 192); memcpy(h + 20, "hdrl", 4);
    memcpy(h + 24, "avih", 4); put32(h + 28, 56);
    put32(h + 32, period); put32(h + 44, indexed ? 0x10 : 0);
    put32(h + 48, a->frames); put32(h + 56, 1); put32(h + 60, a->max_frame);
    put32(h + 64, a->width); put32(h + 68, a->height);
    memcpy(h + 88, "LIST", 4); put32(h + 92, 116); memcpy(h + 96, "strl", 4);
    memcpy(h + 100, "strh", 4); put32(h + 104, 56);
    memcpy(h + 108, "vidsMJPG", 8);
    put32(h + 128, period); put32(h + 132, 1000000);
    put32(h + 140, a->frames); put32(h + 144, a->max_frame);
    put32(h + 148, UINT32_MAX);
    put16(h + 160, a->width); put16(h + 162, a->height);
    memcpy(h + 164, "strf", 4); put32(h + 168, 40); put32(h + 172, 40);
    put32(h + 176, a->width); put32(h + 180, a->height);
    put16(h + 184, 1); put16(h + 186, 24); memcpy(h + 188, "MJPG", 4);
    put32(h + 192, a->width * a->height * 3);
    memcpy(h + 212, "LIST", 4); put32(h + 216, a->end - 220);
    memcpy(h + 220, "movi", 4);
    return fseek(a->file, 0, SEEK_SET) == 0 && fwrite(h, 1, sizeof(h), a->file) == sizeof(h);
}

bool avi_begin(avi_writer_t *a, FILE *file, avi_index_t *index, uint32_t capacity,
               uint32_t width, uint32_t height, uint32_t period_us)
{
    if (!a || !file || !index || !capacity || !width || !height || !period_us) return false;
    *a = (avi_writer_t){ .file = file, .index = index, .capacity = capacity,
        .end = HEADER_SIZE, .width = width, .height = height, .period_us = period_us };
    return header(a, false);
}

bool avi_frame(avi_writer_t *a, const uint8_t *jpeg, uint32_t size, int64_t time_us)
{
    if (!jpeg || !size || a->frames >= a->capacity ||
        (uint64_t)a->end + 8 + size + (size & 1) + 8 + (a->frames + 1) * 16 > MAX_FILE_SIZE)
        return false;
    uint8_t chunk[8];
    memcpy(chunk, "00dc", 4); put32(chunk + 4, size);
    /* begin/previous frame/checkpoint already leaves the append position. */
    if (fwrite(chunk, 1, 8, a->file) != 8 || fwrite(jpeg, 1, size, a->file) != size ||
        ((size & 1) && fputc(0, a->file) == EOF)) return false;
    a->index[a->frames] = (avi_index_t){a->end - 220, size};
    a->end += 8 + size + (size & 1);
    if (a->frames == 0) a->first_us = time_us;
    a->last_us = time_us;
    ++a->frames;
    if (size > a->max_frame) a->max_frame = size;
    return true;
}

bool avi_checkpoint(avi_writer_t *a)
{
    return header(a, false) && fflush(a->file) == 0 &&
           fseek(a->file, a->end, SEEK_SET) == 0;
}

bool avi_finish(avi_writer_t *a)
{
    uint8_t entry[16];
    memcpy(entry, "idx1", 4); put32(entry + 4, a->frames * 16);
    if (fseek(a->file, a->end, SEEK_SET) != 0 || fwrite(entry, 1, 8, a->file) != 8) return false;
    for (uint32_t i = 0; i < a->frames; ++i) {
        memcpy(entry, "00dc", 4); put32(entry + 4, 0x10);
        put32(entry + 8, a->index[i].offset); put32(entry + 12, a->index[i].size);
        if (fwrite(entry, 1, sizeof(entry), a->file) != sizeof(entry)) return false;
    }
    return header(a, true) && fflush(a->file) == 0;
}
