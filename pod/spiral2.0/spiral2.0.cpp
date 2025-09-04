// spiral1.cpp
// Integrated Spiral-1 firmware for Daisy Pod (updated - delay removed, mode handling fixed)
// - 4s SDRAM circular buffer (recording pass-through)
// - Single-grain engine (manual trigger)
// - Hann window envelope
// - Filter bank (1x band-pass) driven by knob in Effect Edit mode
// - Level Edit mode for Wet/Dry
// - Parameter smoothing to avoid clicks/oscillation
// - CLI debug and periodic profiler prints use integer math (no %f floats)

#include "daisy_pod.h"
#include "daisysp.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

#include "FilterBank.h"

using namespace daisy;
using namespace daisysp;

// ----------------- project constants -----------------
static constexpr uint32_t SAMPLE_RATE_HZ = 48000u;
static constexpr uint32_t MAX_SECONDS    = 4u;
static constexpr uint32_t BUFFER_SIZE    = SAMPLE_RATE_HZ * MAX_SECONDS; // 192000

// --- Mix & UI state ---
static float g_level = 0.5f;        // 0=dry, 1=wet
static bool  g_level_edit = false;  // ON: knob2 edits wet/dry (LED blue)
static bool  editEffectsMode = false; // ON: knob1 edits filter cutoff (LED green)

// ----------------- hardware -----------------
DaisyPod pod;

// SDRAM buffers (stereo)
float DSY_SDRAM_BSS bufferL[BUFFER_SIZE];
float DSY_SDRAM_BSS bufferR[BUFFER_SIZE];

// ring heads
volatile uint32_t writeHead = 0;
volatile uint32_t readHead  = 0;

// profiling via DWT
#include "core_cm7.h"
volatile uint32_t audio_max_cycles = 0;
volatile uint32_t audio_min_cycles = UINT32_MAX;
volatile uint32_t audio_last_cycles = 0;
static inline void dwt_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}
static inline uint32_t dwt_cycles(void) { return DWT->CYCCNT; }

// ----------------- grain struct -----------------
struct Grain {
    bool    active = false;
    uint32_t length = 0;   // samples
    float   pos = 0.f;     // fractional read index
    float   step = 1.f;    // playback speed
    uint32_t phase = 0;    // current sample index within grain
    float   gain = 1.0f;
    float   pan = 0.5f;
    uint32_t startSample = 0;
} grain;

// ----------------- single-grain helpers -----------------
static inline void readStereoLinear(const float* bufL, const float* bufR,
                                    uint32_t bufSize, float pos,
                                    float &outL, float &outR)
{
    float p = pos;
    // wrap to [0,bufSize)
    if(p >= (float)bufSize) p -= (float)bufSize * floorf(p / (float)bufSize);
    else if(p < 0.f) p += (float)bufSize * (1 + (uint32_t)(-p / bufSize));

    uint32_t idx = (uint32_t)p;
    uint32_t idx1 = idx + 1;
    if(idx1 >= bufSize) idx1 -= bufSize;
    float frac = p - (float)idx;

    float s0L = bufL[idx];
    float s1L = bufL[idx1];
    float s0R = bufR[idx];
    float s1R = bufR[idx1];

    outL = s0L + frac * (s1L - s0L);
    outR = s0R + frac * (s1R - s0R);
}

// Hann window
static inline float hann_val(uint32_t n, uint32_t N)
{
    if(N <= 1) return 1.0f;
    float x = (2.0f * M_PI * (float)n) / (float)(N - 1);
    return 0.5f * (1.0f - cosf(x));
}

void triggerGrain(uint32_t startSample, uint32_t lengthSamples, float pitchRatio,
                  float gain = 1.0f, float pan = 0.5f)
{
    if(lengthSamples < 2) return;
    grain.active = true;
    grain.length = lengthSamples;
    grain.pos = (float)startSample;
    grain.step = pitchRatio;
    grain.phase = 0;
    grain.gain = gain;
    grain.pan = pan;
    grain.startSample = startSample;
}

// ----------------- global band-pass filter -----------------
// ----------------- DSP modules -----------------
FilterBank filterL;
FilterBank filterR;

// ----------------- UI / buffer length -----------------
uint8_t bufferLengthIndex = 2; // 0:900ms, 1:2000ms, 2:4000ms
const uint32_t bufferLengthMsTable[3] = {900, 2000, 4000};
uint32_t currentBufferLengthMs = 4000;

// Button1 long-press state (for Effect Edit toggle)
uint32_t btn1PressStartMs = 0;
bool     btn1LongFired    = false;
const uint32_t BTN1_HOLD_MS = 800; // 0.8s hold => effect-edit toggle

// ----------------- parameter smoothing -----------------
static float filtCut_sm    = 0.5f; // normalized knob value for filter
static const float SMOOTH_ALPHA = 0.06f; // 0..1, smaller = slower smoothing

// ----------------- audio callback -----------------
void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    uint32_t start_cycles = dwt_cycles();

    uint32_t w = writeHead;

    for(size_t i = 0; i < size; i++)
    {
        // record into ring
        bufferL[w] = in[0][i];
        bufferR[w] = in[1][i];
        if(++w >= BUFFER_SIZE) w = 0;

        float dryL = in[0][i];
        float dryR = in[1][i];

        float wetL = 0.0f;
        float wetR = 0.0f;

        if(grain.active)
        {
            float sL, sR;
            readStereoLinear(bufferL, bufferR, BUFFER_SIZE, grain.pos, sL, sR);

            float env = hann_val(grain.phase, grain.length);

            // equal-power pan
            float gL = sqrtf(1.0f - grain.pan);
            float gR = sqrtf(grain.pan);

            wetL += sL * env * grain.gain * gL;
            wetR += sR * env * grain.gain * gR;

            grain.pos += grain.step;
            grain.phase++;
            if(grain.phase >= grain.length)
            {
                grain.active = false;
            }
        }

        // Mix dry/wet
        float mixL = (1.0f - g_level) * dryL + g_level * wetL;
        float mixR = (1.0f - g_level) * dryR + g_level * wetR;

    // --- Global band-pass filter (both channels processed) ---
        filterL.Process(mixL);
        float filtL = filterL.Band();

        filterR.Process(mixR);
        float filtR = filterR.Band();


    // Write to output
    out[0][i] = filtL;
    out[1][i] = filtR;

    }

    writeHead = w;

    uint32_t end_cycles = dwt_cycles();
    uint32_t elapsed = end_cycles - start_cycles;
    audio_last_cycles = elapsed;
    if(elapsed > audio_max_cycles) audio_max_cycles = elapsed;
    if(elapsed < audio_min_cycles) audio_min_cycles = elapsed;
}

// ----------------- main -----------------
int main(void)
{
    pod.Init();
    pod.seed.StartLog(false);
    pod.seed.PrintLine("Spiral1: Granular (single-grain) + Filter (delay removed)");

    // SDRAM init
    pod.seed.sdram_handle.Init();

    // DWT profiling
    dwt_init();

    // DSP init
    
    filterL.Init((float)SAMPLE_RATE_HZ);
    filterR.Init((float)SAMPLE_RATE_HZ);


    // audio setup
    pod.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_48KHZ);
    pod.SetAudioBlockSize(48);
    pod.StartAdc();

    // clear buffers
    for(uint32_t i=0;i<BUFFER_SIZE;i++){ bufferL[i]=0.0f; bufferR[i]=0.0f; }
    writeHead = 0;
    readHead  = 0;

    // LED default
    pod.led1.Set(1.0f,0.0f,0.0f); // red = normal
    pod.UpdateLeds();

    // start audio
    pod.StartAudio(AudioCallback);
    pod.seed.PrintLine("Audio started. Button2=trigger grain. EncClick=LevelEdit. Btn1 hold=EffectEdit.");

    // defaults
    currentBufferLengthMs = bufferLengthMsTable[bufferLengthIndex];
    filtCut_sm = pod.knob1.Value();

    uint32_t lastTick = 0;
    static int32_t enc_acc = 0; // semitone accumulator for pitch

    while(1)
    {
        pod.ProcessAllControls();

        // === ENC CLICK: toggle Level Edit (mutually exclusive with Effect Edit) ===
        if(pod.encoder.RisingEdge())
        {
            g_level_edit = !g_level_edit;
            if(g_level_edit)
                editEffectsMode = false; // make modes exclusive
            pod.seed.PrintLine("LevelEdit=%d EffectEdit=%d", g_level_edit ? 1 : 0, editEffectsMode ? 1 : 0);
        }

        // While Level-Edit is ON, Knob2 sets wet/dry LEVEL
        if(g_level_edit)
        {
            g_level = pod.knob2.Value(); // 0..1
        }

        // === BTN1 LONG PRESS: toggle Effect Edit (mutually exclusive with Level Edit) ===
        // --- BTN1 LONG / SHORT handling (mutually exclusive) ---
        static bool btn1SuppressNextRise = false;

        if(pod.button1.Pressed())
    {
        if(btn1PressStartMs == 0)
        btn1PressStartMs = System::GetNow();

            // fire long once
        if(!btn1LongFired && (System::GetNow() - btn1PressStartMs >= BTN1_HOLD_MS))
     {
        editEffectsMode = !editEffectsMode;
        if(editEffectsMode) g_level_edit = false; // modes exclusive
        btn1LongFired = true;
        btn1SuppressNextRise = true;              // block upcoming short
        pod.seed.PrintLine("EffectEdit=%d LevelEdit=%d",
                           editEffectsMode ? 1 : 0, g_level_edit ? 1 : 0);
        }
}
else
    {
    // button released -> handle potential short press
    if(pod.button1.RisingEdge())
    {
        if(!btn1SuppressNextRise)
        {
            bufferLengthIndex = (bufferLengthIndex + 1) % 3;
            currentBufferLengthMs = bufferLengthMsTable[bufferLengthIndex];
            pod.seed.PrintLine("Buffer length set to %ums", currentBufferLengthMs);
        }
    }

    // reset all long/short tracking AFTER the rising-edge check
        btn1PressStartMs = 0;
        btn1LongFired = false;
        btn1SuppressNextRise = false;
    }

        // === BTN2: trigger single grain ===
        if(pod.button2.RisingEdge())
        {
            // size: knob1 -> 40ms..1000ms
            float vsize = pod.knob1.Value(); // 0..1
            float ms = 40.0f + vsize * (1000.0f - 40.0f);
            
            uint32_t lengthSamples = (uint32_t)(ms * (pod.AudioSampleRate() / 1000.0f));
            if(lengthSamples < 2) lengthSamples = 2;

            // pos: knob2 -> relative offset in current buffer window
            float vpos = pod.knob2.Value();
            uint32_t windowSamples = (uint32_t)((float)currentBufferLengthMs * (pod.AudioSampleRate() / 1000.0f));
            if(windowSamples < 2) windowSamples = 2;
            uint32_t offset = (uint32_t)(vpos * (float)windowSamples);
            int64_t start = (int64_t)writeHead - (int64_t)offset;
            while(start < 0) start += BUFFER_SIZE;
            uint32_t startIdx = (uint32_t)start % BUFFER_SIZE;

            // pitch: encoder detents adjust semitones ONLY when not in effect edit
            if(!editEffectsMode)
            {
                int32_t inc = pod.encoder.Increment();
                if(inc != 0) enc_acc += inc;
            }
            float pitchRatio = powf(2.0f, (float)enc_acc / 12.0f);

            triggerGrain(startIdx, lengthSamples, pitchRatio, 1.0f, 0.5f);

            pod.seed.PrintLine("Grain: start=%u len=%u semis=%d", startIdx, lengthSamples, enc_acc);
        }

        // === Effect Edit mapping (filter) ===
        float target_filt = filtCut_sm;
        if(editEffectsMode)
        {
            // knob1 -> filter cutoff when in Effect Edit
            target_filt = pod.knob1.Value();
        }
        // smooth and apply
        filtCut_sm += (target_filt - filtCut_sm) * SMOOTH_ALPHA;
        filterL.SetFreqFromKnob(filtCut_sm, 40.0f, 4000.0f);
        filterR.SetFreqFromKnob(filtCut_sm, 40.0f, 4000.0f);


        // === LED feedback (mutually exclusive colors) ===
        if(g_level_edit)
            pod.led1.Set(0.0f, 0.0f, 1.0f); // blue
        else if(editEffectsMode)
            pod.led1.Set(0.0f, 1.0f, 0.0f); // green
        else
            pod.led1.Set(1.0f, 0.0f, 0.0f); // red
        pod.UpdateLeds();

        // periodic profiler print (integer-safe)
        // ---- profiler print (CPU usage) ----
uint32_t t = System::GetNow();
if(t - lastTick >= 1000)
{
    lastTick = t;

    uint32_t maxc  = audio_max_cycles;
    uint32_t lastc = audio_last_cycles;
    uint32_t block = pod.AudioBlockSize();

    // cycles-per-sample ×100
    uint32_t cps_x100 = (maxc * 100U) / block;

    // CPU percent ×100
    const uint64_t CPU_FREQ = 480000000ULL; // Hz
    uint64_t cpu_pct_x100 =
        ((uint64_t)maxc * (uint64_t)pod.AudioSampleRate() * 10000ULL) /
        ((uint64_t)block * CPU_FREQ);

    uint32_t cpu_int  = (uint32_t)(cpu_pct_x100 / 100ULL);
    uint32_t cpu_frac = (uint32_t)(cpu_pct_x100 % 100ULL);

    pod.seed.PrintLine(
        "Audio cycles last=%lu max=%lu block=%lu cps_x100=%lu cpu=%lu.%02lu%%",
        (unsigned long)lastc,
        (unsigned long)maxc,
        (unsigned long)block,
        (unsigned long)cps_x100,
        (unsigned long)cpu_int,
        (unsigned long)cpu_frac);

    audio_max_cycles = 0;
    audio_min_cycles = UINT32_MAX;
}

        System::Delay(1);
    }

    return 0;
}
