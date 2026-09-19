#include "avi_writer.h"
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>

#define HEADER_SIZE 224u
#define MAX_FILE_SIZE (1024u * 1024u * 1024u)
#define SECTOR_SIZE 512u

/* Explicit buffering avoids libc-dependent fwrite behavior. Only this writer
 * owns the fd, and every VFS write starts at the same aligned DMA buffer.
 * FatFs may split these writes at cluster boundaries; SDSPI still sends the
 * protocol-required 512-byte data blocks within each multi-block command. */
static bool flush_buffer(avi_writer_t *a)
{
    if (a->io_failed) return false;
    size_t done = 0;
    while (done < a->buffered) {
        size_t bytes = a->buffered - done;
        if (bytes > a->io_buffer_size) bytes = a->io_buffer_size;
        memcpy(a->io_buffer, a->buffer + done, bytes);
        ssize_t written = write(a->fd, a->io_buffer, bytes);
        if (written < 0 && errno == EINTR) continue;
        if (written <= 0) {
            if (written == 0) errno = EIO;
            /* A partial flush cannot safely be replayed from its beginning. */
            a->io_failed = true;
            return false;
        }
        done += (size_t)written;
    }
    a->buffered = 0;
    return true;
}

static bool append(avi_writer_t *a, const void *data, size_t size)
{
    if (a->io_failed) return false;
    const uint8_t *src = data;
    while (size) {
        size_t bytes = a->buffer_size - a->buffered;
        if (bytes > size) bytes = size;
        memcpy(a->buffer + a->buffered, src, bytes);
        a->buffered += bytes;
        src += bytes;
        size -= bytes;
        if (a->buffered == a->buffer_size && !flush_buffer(a)) return false;
    }
    return true;
}

static bool seek_to(avi_writer_t *a, off_t offset)
{
    if (!flush_buffer(a)) return false;
    if (lseek(a->fd, offset, SEEK_SET) != offset) {
        a->io_failed = true;
        return false;
    }
    return true;
}

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
    return seek_to(a, 0) && append(a, h, sizeof(h));
}

bool avi_begin(avi_writer_t *a, int fd, avi_index_t *index, uint32_t capacity,
               uint32_t width, uint32_t height, uint32_t period_us,
               uint8_t *buffer, size_t buffer_size,
               uint8_t *io_buffer, size_t io_buffer_size)
{
    if (!a || fd < 0 || !index || !capacity || !width || !height || !period_us ||
        !buffer || !io_buffer || !buffer_size || !io_buffer_size ||
        buffer_size % SECTOR_SIZE || io_buffer_size % SECTOR_SIZE) return false;
    *a = (avi_writer_t){ .fd = fd, .buffer = buffer, .buffer_size = buffer_size,
        .io_buffer = io_buffer, .io_buffer_size = io_buffer_size,
        .index = index, .capacity = capacity,
        .end = HEADER_SIZE, .width = width, .height = height, .period_us = period_us };
    return header(a, false);
}

bool avi_frame(avi_writer_t *a, const uint8_t *jpeg, uint32_t size, int64_t time_us)
{
    if (a->io_failed || !jpeg || !size || a->frames >= a->capacity ||
        (uint64_t)a->end + 8 + size + (size & 1) + 8 + (a->frames + 1) * 16 > MAX_FILE_SIZE)
        return false;
    uint8_t chunk[8];
    memcpy(chunk, "00dc", 4); put32(chunk + 4, size);
    /* begin/previous frame/checkpoint already leaves the append position. */
    const uint8_t pad = 0;
    if (!append(a, chunk, sizeof(chunk)) || !append(a, jpeg, size) ||
        ((size & 1) && !append(a, &pad, 1))) return false;
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
    return header(a, false) && seek_to(a, a->end);
}

bool avi_finish(avi_writer_t *a)
{
    uint8_t entry[16];
    memcpy(entry, "idx1", 4); put32(entry + 4, a->frames * 16);
    if (!seek_to(a, a->end) || !append(a, entry, 8)) return false;
    for (uint32_t i = 0; i < a->frames; ++i) {
        memcpy(entry, "00dc", 4); put32(entry + 4, 0x10);
        put32(entry + 8, a->index[i].offset); put32(entry + 12, a->index[i].size);
        if (!append(a, entry, sizeof(entry))) return false;
    }
    return header(a, true) && flush_buffer(a);
}
