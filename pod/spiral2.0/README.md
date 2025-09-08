# Spiral2.0

## Author

ASif 
https://github.com/chipvista/spiral1

## Description



Device continuously records into the active buffer.

When that buffer fills (after 4s), it becomes the inactive snapshot and the system starts writing into the other buffer. The previously filled buffer is safe to read from for grains → no read/write collision.

Grain playback uses the most recently completed snapshot buffer — i.e., grains are playing from the last-finished 4-second chunk.

Until the first buffer has been completed, triggers will fall back to the single-buffer style behavior (so nothing crashes at startup).

Result: glitch-free grain playback from a clean snapshot while new audio records into the other buffer.