# Spiral2.0

## Author

ASif 
https://github.com/chipvista/spiral1

## Description

Granular synthesis effect for Daisy Pod with automatic grain triggering, multiple envelope shapes, and buffer management modes.

## Controls

4s SDRAM circular buffer (recording pass-through). ✅

Granular engine with:
- Automatic triggering based on RATE control ✅
- Manual triggering (Button2) ✅
- Fractional ring-read + linear interpolation ✅
- Multiple envelope shapes (ramp up, Hann, ramp down) ✅
- SCATTER control for position randomization ✅
- Pitch randomization algorithm (-4 to +4 octaves) ✅
- Size mapping (40ms → 1000ms) ✅
- Position mapping ✅

Buffer management modes:
- Default: Constant recording ✅
- HOLD: Freeze buffer contents ✅  
- FOLLOW: Record only when sound detected ✅

Filter bank: 1× band-pass (Svf) with log frequency mapping 40–4000 Hz (knob in Edit mode). ✅

Hardware I/O mapping: 
- Knob1: RATE/Filter cutoff ✅
- Knob2: SCATTER/Level ✅  
- Encoder: PITCH range ✅
- Button1: Cycle buffer modes ✅
- Button2: Trigger grain/Cycle envelope shapes ✅
- Edit mode to swap knob function ✅

CLI/debug: integer-safe prints (no float printf), profiling with DWT cycles ✅

Parameter smoothing to avoid clicks and sudden oscillation. ✅

Master volume removed (no global software scaling). ✅

