"""H.264 helper placeholder for the standalone IMX708 snapshot baseline.

The included capture.py imports the author's MP4 helper because it also supports
video examples. This branch contains only the snapshot example, so H.264 paths
are intentionally not used.
"""

def durations_from_timestamps(*args, **kwargs):
    raise RuntimeError("H.264/MP4 support is not part of the snapshot baseline")


def build_mp4(*args, **kwargs):
    raise RuntimeError("H.264/MP4 support is not part of the snapshot baseline")
