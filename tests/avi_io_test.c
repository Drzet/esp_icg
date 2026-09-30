/* Native regression tests for the buffered writer. Link with --wrap=write. */
#include "avi_writer.h"
#include <assert.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define BUFFER_BYTES (512u * 1024u)
#define IO_BYTES (16u * 1024u)

static const void *expected_buffer;
static unsigned calls;
static enum { NORMAL, SHORT_WRITE, ZERO_WRITE, ERROR_WRITE } behavior;

ssize_t __real_write(int fd, const void *data, size_t size);
ssize_t __wrap_write(int fd, const void *data, size_t size)
{
    assert(data == expected_buffer && size > 0 && size <= IO_BYTES);
    ++calls;
    if (behavior == ZERO_WRITE) return 0;
    if (behavior == ERROR_WRITE) { errno = ENOSPC; return -1; }
    if (behavior == SHORT_WRITE && calls == 1) { errno = EINTR; return -1; }
    if (behavior == SHORT_WRITE && calls == 2 && size > 7) size = 7;
    return __real_write(fd, data, size);
}

static uint32_t read32(const uint8_t *p)
{
    return p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static int temporary_file(void)
{
    char path[] = "/tmp/avi-io-XXXXXX";
    int fd = mkstemp(path);
    assert(fd >= 0 && unlink(path) == 0);
    return fd;
}

static void check_boundaries(uint8_t *buffer, uint8_t *io_buffer)
{
    const uint32_t sizes[] = {1, 511, 512, 513, 65535, BUFFER_BYTES + 1};
    uint8_t *frame = malloc(BUFFER_BYTES + 1);
    uint8_t *check = malloc(BUFFER_BYTES + 1);
    assert(frame && check);
    for (size_t i = 0; i < BUFFER_BYTES + 1; ++i) frame[i] = (uint8_t)(i * 37u);
    int fd = temporary_file();
    avi_writer_t avi;
    avi_index_t index[6];
    calls = 0;
    assert(avi_begin(&avi, fd, index, 6, 1920, 1080, 100000,
                     buffer, BUFFER_BYTES, io_buffer, IO_BYTES));
    assert(calls == 0); /* Initial header remains buffered. */
    for (unsigned i = 0; i < 6; ++i) {
        assert(avi_frame(&avi, frame, sizes[i], (int64_t)i * 200000));
        if (i < 5) assert(calls == 0); /* No per-frame flush/sync. */
    }
    assert(calls > 0 && avi_finish(&avi));
    uint8_t header[224];
    assert(pread(fd, header, sizeof(header), 0) == sizeof(header));
    assert(!memcmp(header, "RIFF", 4) && !memcmp(header + 8, "AVI ", 4));
    assert(read32(header + 32) == 200000 && read32(header + 48) == 6);
    assert(read32(header + 4) + 8 == avi.end + 8 + 6 * 16);
    off_t offset = 224;
    for (unsigned i = 0; i < 6; ++i) {
        uint8_t chunk[8];
        assert(pread(fd, chunk, 8, offset) == 8);
        assert(!memcmp(chunk, "00dc", 4) && read32(chunk + 4) == sizes[i]);
        assert(pread(fd, check, sizes[i], offset + 8) == sizes[i]);
        assert(!memcmp(check, frame, sizes[i]));
        if (sizes[i] & 1) {
            uint8_t pad = 1;
            assert(pread(fd, &pad, 1, offset + 8 + sizes[i]) == 1 && pad == 0);
        }
        uint8_t entry[16];
        assert(pread(fd, entry, 16, avi.end + 8 + i * 16) == 16);
        assert(!memcmp(entry, "00dc", 4));
        assert(read32(entry + 8) == (uint32_t)offset - 220);
        assert(read32(entry + 12) == sizes[i]);
        offset += 8 + sizes[i] + (sizes[i] & 1);
    }
    assert(offset == avi.end && close(fd) == 0);
    free(frame);
    free(check);
}

static void check_error(uint8_t *buffer, uint8_t *io_buffer)
{
    int fd = temporary_file();
    avi_writer_t avi;
    avi_index_t index[1];
    uint8_t *frame = calloc(1, BUFFER_BYTES);
    assert(frame);
    calls = 0;
    assert(avi_begin(&avi, fd, index, 1, 1920, 1080, 100000,
                     buffer, BUFFER_BYTES, io_buffer, IO_BYTES));
    assert(!avi_frame(&avi, frame, BUFFER_BYTES, 0));
    assert(avi.io_failed && calls == 1 && avi.frames == 0);
    assert(errno == (behavior == ZERO_WRITE ? EIO : ENOSPC));
    assert(!avi_finish(&avi) && calls == 1); /* Never replay a failed flush. */
    assert(close(fd) == 0);
    free(frame);
}

int main(void)
{
    uint8_t *buffer = malloc(BUFFER_BYTES);
    _Alignas(128) uint8_t io_buffer[IO_BYTES];
    assert(buffer);
    expected_buffer = io_buffer;
    behavior = NORMAL;
    check_boundaries(buffer, io_buffer);
    behavior = SHORT_WRITE;
    check_boundaries(buffer, io_buffer);
    behavior = ZERO_WRITE;
    check_error(buffer, io_buffer);
    behavior = ERROR_WRITE;
    check_error(buffer, io_buffer);
    free(buffer);
    return 0;
}
