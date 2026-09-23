#include "avi_writer.h"
#include <assert.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int main(int argc, char **argv)
{
    assert(argc == 3 || argc == 4);
    FILE *jpg = fopen(argv[1], "rb");
    assert(jpg);
    assert(fseek(jpg, 0, SEEK_END) == 0);
    long n = ftell(jpg);
    assert(n > 0 && fseek(jpg, 0, SEEK_SET) == 0);
    uint8_t *data = malloc(n + 1);
    assert(data && fread(data, 1, n, jpg) == (size_t)n);
    fclose(jpg);
    data[n] = 0; /* Legal trailing byte exercises both odd/even RIFF padding. */
    int out = open(argv[2], O_CREAT | O_TRUNC | O_RDWR, 0600);
    assert(out >= 0);
    assert(ftruncate(out, 64 * 1024 * 1024) == 0);
    uint8_t *buffer = malloc(512 * 1024);
    _Alignas(128) uint8_t io_buffer[16 * 1024];
    assert(buffer);
    avi_index_t index[10];
    avi_writer_t avi;
    assert(avi_begin(&avi, out, index, 10, 1920, 1080, 100000,
                     buffer, 512 * 1024, io_buffer, sizeof(io_buffer)));
    /* Irregular capture intervals must set average playback cadence, not 10fps. */
    for (int i = 0; i < 10; ++i) {
        assert(avi_frame(&avi, data, n + (i & 1), (int64_t)i * 200000));
        if (argc == 4 && (i == 3 || i == 7)) assert(avi_checkpoint(&avi));
    }
    assert(!avi_frame(&avi, data, n, 2000000));
    assert(avi_finish(&avi));
    assert(ftruncate(out, (off_t)avi.end + 8 + avi.frames * 16) == 0);
    assert(fsync(out) == 0);
    assert(close(out) == 0);
    free(buffer);
    free(data);
    return 0;
}
