# SD recording review (ESP-IDF v6.0.3)

The supplied run saved 37 frames, dropped 45 and finalized 1.85 seconds after
STOP. AVI metadata reported 3.19 fps. Copy/flip took about 70 ms and hardware JPEG
about 34 ms; AVI appends took 315–411 ms on average. These append times include
libc, FatFs, the SD driver and scheduling. They do not measure card programming
time in isolation.

## Verified findings

- The original 512 KiB stdio buffer and queued JPEGs were in PSRAM. In this IDF
  version, `sdspi_host_check_buffer_alignment()` checks four-byte alignment;
  aligned PSRAM is **not automatically reduced to single-block commands**.
  `SDSPI_HOST_DEFAULT()` sets the unaligned fallback chunk to 16 sectors.
  [SPI source](https://github.com/espressif/esp-idf/blob/v6.0.3/components/esp_driver_sdspi/src/sdspi_host.c),
  [defaults](https://github.com/espressif/esp-idf/blob/v6.0.3/components/esp_driver_sdspi/include/driver/sdspi_host.h).
- Within a write command, SDSPI copies each 512-byte data block from PSRAM into
  internal memory and submits multiple SPI transactions per sector, including
  card-busy polling. A large `setvbuf()` alone does not remove those costs.
  The 4096-byte SPI bus transfer limit is not the reason a multi-block SD command
  is split: SD data blocks are 512 bytes by protocol.
- Espressif recommends larger sector-aligned POSIX writes and internal FatFs
  buffers. Its storage benchmark uses internal, cache-aligned DMA allocations.
  [FatFs performance guidance](https://docs.espressif.com/projects/esp-idf/en/v6.0.3/esp32p4/api-reference/storage/fatfs.html#optimize-i-o-performance),
  [benchmark implementation](https://github.com/espressif/esp-idf/blob/v6.0.3/examples/storage/perf_benchmark/main/perf_benchmark_example_tests.c).
- The recorder copied and encoded frames even when its JPEG queue was full,
  then discarded them. The new admission check avoids this work when the queue
  is already known to be full. A slot becoming free immediately after the check
  may conservatively skip one frame; capture never waits for SD.
- The configured `.allocation_unit_size = 32 * 1024` applies when formatting.
  With formatting disabled it does not change an existing card's cluster size.
  FatFs can split application writes at the card's existing cluster boundaries.
- Espressif issue [#10940](https://github.com/espressif/esp-idf/issues/10940)
  documents extra waits when MISO does not go high before selection. That is a
  possible diagnostic path, not a demonstrated fault on this board. Disabling
  chip select or holding SD selected would conflict with the shared touch bus;
  this change keeps normal CS, busy polling and CRC checks.

## Changes

AVI buffering is explicit: 512 KiB in PSRAM, drained through one 16 KiB internal
DMA staging buffer with POSIX `write()`. Every write uses that aligned base
address; partial and interrupted writes preserve byte order. An I/O failure is
latched so finalization cannot replay a partially written buffer. No header or
JPEG content changes are intended. Stop-only synchronization and allocation
without zero-filling are retained.

Internal FatFs buffers avoid external-memory metadata traffic. The SD host's
unaligned fallback is explicitly 32 sectors. Normal recording payloads already
arrive aligned from the internal staging buffer and do not use that fallback.

The mount path retries at 20 MHz after an invalid response, CRC error or
unsupported initialization at a higher requested clock. It logs the actual
configured clock. This is a compatibility fallback, not a fix for the card's
40 MHz high-speed transition. No failed-write retry is added.

## Reading the next hardware run

Compare the same card, clock, exposure and scene for at least 30 seconds.
Capture boot/mount through `STOP finalized`.

| Log | Meaning |
|---|---|
| `SD clock: actual=...` | Clock calculated from the SPI configuration, not a logic-analyzer measurement. |
| `buffer=512 KiB DMA=16 KiB` | Identifies the explicit-buffer writer. |
| `record avg us: ... write=...` | Average append time over ten frames; includes buffer flushes in that interval. |
| `CMD24` / `CMD25` | Cumulative single-/multi-block write command counts. A few single-block metadata/tail writes are normal. |
| `driver_ms` / `KiB/s` | Time and successful bytes at the SD command boundary; includes scheduling and card busy time. Excludes application buffering/encoding. |
| `max_cmd_us` / `errors` | Longest write command and write-command failures. |

If appends improve but SD command throughput stays low, the remaining delay is
in driver execution, scheduling, bus arbitration or the card. If command time is
small compared with append time, investigate work above that boundary. These
metrics do not distinguish card busy time from every other driver wait.

Hardware throughput, fallback mounting, repeated recording and touch behavior
still require board testing. No specific frame-rate improvement is claimed.

## Native verification

The I/O regression test covers sector/buffer crossings, a frame larger than the
512 KiB buffer, odd/even RIFF padding, index offsets, EINTR/short writes and write
failures. The smoke test writes ten 1080p JPEG frames over multiple flushes and
removes unused 64 MiB preallocation. FFmpeg decodes both stop-only and checkpoint
variants identically at the expected timestamp-derived 5 fps.
