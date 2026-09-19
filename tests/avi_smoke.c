#include "avi_writer.h"
#include <assert.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
    assert(argc == 3);
    FILE *jpg = fopen(argv[1], "rb");
    assert(jpg);
    assert(fseek(jpg, 0, SEEK_END) == 0);
    long n = ftell(jpg);
    assert(n > 0 && fseek(jpg, 0, SEEK_SET) == 0);
    uint8_t *data = malloc(n + 1);
    assert(data && fread(data, 1, n, jpg) == (size_t)n);
    fclose(jpg);
    data[n] = 0; /* Legal trailing byte exercises both odd/even RIFF padding. */
    FILE *out = fopen(argv[2], "wb+");
    assert(out);
    avi_index_t index[10];
    avi_writer_t avi;
    assert(avi_begin(&avi, out, index, 10, 1920, 1080, 100000));
    /* Irregular capture intervals must set average playback cadence, not 10fps. */
    for (int i = 0; i < 10; ++i) {
        assert(avi_frame(&avi, data, n + (i & 1), (int64_t)i * 200000));
        if (i == 3 || i == 7) assert(avi_checkpoint(&avi));
    }
    assert(!avi_frame(&avi, data, n, 2000000));
    assert(avi_finish(&avi));
    assert(fclose(out) == 0);
    free(data);
    return 0;
}
