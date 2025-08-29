# Spiral1

## Author

ASif 
https://github.com/chipvista/spiral1


## Description

## Controls
4 s SDRAM circular buffer (recording pass-through). ✅

Single-grain engine (triggered by Button2) with:

fractional ring-read + linear interpolation. ✅

Hann envelope. ✅

size mapping (40 ms → 1000 ms), position mapping, encoder pitch mapping. ✅

Filter bank: 1× band-pass (Svf) with log frequency mapping 40–4000 Hz (knob in Edit mode). ✅

Delay effect: stereo, SDRAM-backed buffers (no SRAM overflow), feedback + mix, safe feedback clamp (0.85), smoothing. ✅

Hardware I/O mapping: knob1/knob2/encoder/button mapping; Edit mode to swap knob function. ✅

CLI/debug: integer-safe prints (no float printf), profiling with DWT cycles, CLI dump on encoder long-hold. ✅

Parameter smoothing to avoid clicks and sudden oscillation. ✅

Master volume removed (no global software scaling). ✅

Build fixes: Delay buffers placed in SDRAM; no .bss overflow. ✅







