// spiral1.cpp
// Spiral-1 -> Spiral-2: added Double Buffering (ping-pong snapshot).
// Other behavior preserved (single-grain, Hann, global band-pass filter, mode UI).

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
// thread-safe publish for filter knob value
volatile float filtCut_shared = 0.5f;   // knob value (0..1) pushed to audio thread
static float g_level_sm = 0.5f;
const float LEVEL_SMOOTH_ALPHA = 0.08f;

static bool  g_level_edit = false;  // ON: knob2 edits wet/dry (LED blue)
static bool  editEffectsMode = false; // ON: knob1 edits filter cutoff (LED green)

// ----------------- hardware -----------------
DaisyPod pod;

// ----------------- DOUBLE BUFFERS (stereo) -----------------
// Ping-pong buffers in SDRAM (two separate full-size buffers)
float DSY_SDRAM_BSS bufferA_L[BUFFER_SIZE];
float DSY_SDRAM_BSS bufferA_R[BUFFER_SIZE];
float DSY_SDRAM_BSS bufferB_L[BUFFER_SIZE];
float DSY_SDRAM_BSS bufferB_R[BUFFER_SIZE];

// which buffer is currently being written into (true = A, false = B)
volatile bool useBufferA = true;

// ring head inside the active buffer
volatile uint32_t writeHead = 0;

// flag indicating we have at least one completed (inactive) buffer ready for reading
volatile bool inactiveBufferReady = false;

// index of the last-sample (tail) in the inactive snapshot (always BUFFER_SIZE-1 after fill)
uint32_t inactiveTailIndex = 0;

// ----------------- profiling via DWT -----------------
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
    float   fixedStartPos = 0.f;  // Store fixed start position
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
    // store start relative to the inactive snapshot buffer indexing
    grain.fixedStartPos = (float)startSample;
    grain.pos = grain.fixedStartPos;
    grain.step = pitchRatio;
    grain.phase = 0;
    grain.gain = gain;
    grain.pan = pan;
    grain.startSample = startSample;
}

// ----------------- global band-pass filter (stereo) -----------------
FilterBank filterL;
FilterBank filterR;

// ----------------- UI / buffer length -----------------
uint8_t bufferLengthIndex = 2; // 0:900ms, 1:2000ms, 2:4000ms
const uint32_t bufferLengthMsTable[3] = {900, 2000, 4000};
uint32_t currentBufferLengthMs = 4000;

// Button long-press state
uint32_t btn1PressStartMs = 0;
bool     btn1LongFired    = false;
const uint32_t BTN1_HOLD_MS = 800; // ms

// ----------------- parameter smoothing -----------------
static float filtCut_sm    = 0.5f; // normalized knob value for filter
static const float SMOOTH_ALPHA = 0.06f; // 0..1, smaller = slower smoothing

// ----------------- audio callback -----------------
void AudioCallback(AudioHandle::InputBuffer  in,
                   AudioHandle::OutputBuffer out,
                   size_t                    size)
{
    uint32_t start_cycles = dwt_cycles();

    // Apply latest filter cutoff once per block (safe publish)
    static float last_applied_filt = -1.0f;
    float newf = filtCut_shared;
    if(fabsf(newf - last_applied_filt) > 1e-7f)
    {
        filterL.SetFreqFromKnob(newf, 40.0f, 4000.0f);
        filterR.SetFreqFromKnob(newf, 40.0f, 4000.0f);
        last_applied_filt = newf;
    }

    // Select active/inactive buffer pointers
    float* writeBufL  = useBufferA ? bufferA_L : bufferB_L;
    float* writeBufR  = useBufferA ? bufferA_R : bufferB_R;
    float* readBufL   = useBufferA ? bufferB_L : bufferA_L; // read from the other (inactive) buffer
    float* readBufR   = useBufferA ? bufferB_R : bufferA_R;

    uint32_t w = writeHead;

    for(size_t i = 0; i < size; i++)
    {
        // --- write input into active buffer ---
        writeBufL[w] = in[0][i];
        writeBufR[w] = in[1][i];

        // Advance write head
        if(++w >= BUFFER_SIZE)
        {
            // buffer full -> snapshot completed, swap buffers
            w = 0;
            useBufferA = !useBufferA; // swap active buffer

            // after swap, inactiveBufferReady = true (we now have a completed buffer to read)
            inactiveBufferReady = true;
            inactiveTailIndex = BUFFER_SIZE - 1;

            // update buffer pointers for subsequent samples (within same callback)
            writeBufL  = useBufferA ? bufferA_L : bufferB_L;
            writeBufR  = useBufferA ? bufferA_R : bufferB_R;
            readBufL   = useBufferA ? bufferB_L : bufferA_L;
            readBufR   = useBufferA ? bufferB_R : bufferA_R;
        }

        float dryL = in[0][i];
        float dryR = in[1][i];

        float wetL = 0.0f;
        float wetR = 0.0f;

        // --- grain from inactive snapshot buffer (only valid if we've captured at least one snapshot) ---
        if(grain.active && inactiveBufferReady)
        {
            float sL, sR;
            readStereoLinear(readBufL, readBufR, BUFFER_SIZE, grain.pos, sL, sR);

            float env = hann_val(grain.phase, grain.length);

            float gL = sqrtf(1.0f - grain.pan);
            float gR = sqrtf(grain.pan);

            wetL += sL * env * grain.gain * gL;
            wetR += sR * env * grain.gain * gR;

            // advance pos using fixedStartPos base (so grain reads are consistent)
            grain.pos = grain.fixedStartPos + (float)grain.phase * grain.step;
            if(grain.pos >= (float)BUFFER_SIZE)
                grain.pos -= (float)BUFFER_SIZE; // wrap

            grain.phase++;
            if(grain.phase >= grain.length)
            {
                grain.active = false;
            }
        }

        // Mix dry/wet
        float mixL = (1.0f - g_level) * dryL + g_level * wetL;
        float mixR = (1.0f - g_level) * dryR + g_level * wetR;

        // Global band-pass filter (stereo)
        filterL.Process(mixL);
        float filtL = filterL.Band();

        filterR.Process(mixR);
        float filtR = filterR.Band();

        // Output
        out[0][i] = filtL;
        out[1][i] = filtR;
    }

    // store back write head
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
    pod.seed.PrintLine("Spiral1: Granular (single-grain) + Filter (delay removed) + Double Buffer (ping-pong)");

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

    // clear both buffers
    for(uint32_t i=0;i<BUFFER_SIZE;i++){ bufferA_L[i]=0.0f; bufferA_R[i]=0.0f; bufferB_L[i]=0.0f; bufferB_R[i]=0.0f; }
    writeHead = 0;
    useBufferA = true;
    inactiveBufferReady = false;

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

        // While Level-Edit is ON, Knob2 sets wet/dry LEVEL (smoothed)
        if(g_level_edit)
        {
            float g_target = pod.knob2.Value();
            g_level_sm += (g_target - g_level_sm) * LEVEL_SMOOTH_ALPHA;
            g_level = g_level_sm;
        }

        // === Button1 long/short press handling ===
        static bool btn1_was_pressed = false;
        static uint32_t btn1_press_start = 0;
        static bool btn1_long_handled = false;

        if(pod.button1.Pressed())
        {
            if(!btn1_was_pressed)
            {
                btn1_was_pressed = true;
                btn1_press_start = System::GetNow();
                btn1_long_handled = false;
            }
            else
            {
                if(!btn1_long_handled && (System::GetNow() - btn1_press_start >= BTN1_HOLD_MS))
                {
                    editEffectsMode = !editEffectsMode;
                    if(editEffectsMode) g_level_edit = false;
                    btn1_long_handled = true;
                    pod.seed.PrintLine("EffectEdit=%d LevelEdit=%d", editEffectsMode ? 1 : 0, g_level_edit ? 1 : 0);
                }
            }
        }
        else
        {
            if(btn1_was_pressed)
            {
                if(!btn1_long_handled)
                {
                    bufferLengthIndex = (bufferLengthIndex + 1) % 3;
                    currentBufferLengthMs = bufferLengthMsTable[bufferLengthIndex];
                    pod.seed.PrintLine("*** BUFFER LENGTH: %u ms ***", currentBufferLengthMs);
                }
            }
            btn1_was_pressed = false;
            btn1_press_start = 0;
            btn1_long_handled = false;
        }

        // === BTN2: trigger single grain ===
        if(pod.button2.RisingEdge())
        {
            // size: knob1 -> 40ms..1000ms
            float vsize = pod.knob1.Value(); // 0..1
            float ms = 40.0f + vsize * (1000.0f - 40.0f);
            uint32_t lengthSamples = (uint32_t)(ms * (pod.AudioSampleRate() / 1000.0f));
            if(lengthSamples < 2) lengthSamples = 2;

            // pos: knob2 -> relative offset in current window, but computed against the last completed (inactive) buffer
            float vpos = pod.knob2.Value();
            uint32_t windowSamples = (uint32_t)((float)currentBufferLengthMs * (pod.AudioSampleRate() / 1000.0f));
            if(windowSamples < 2) windowSamples = 2;
            uint32_t offset = (uint32_t)(vpos * (float)windowSamples);

            uint32_t startIdx = 0;
            if(inactiveBufferReady)
            {
                // map offset so knob=0 -> 'most recent sample in snapshot'
                // most recent sample index in snapshot is inactiveTailIndex (BUFFER_SIZE-1 normally)
                // start = (inactiveTailIndex + 1 + offset) % BUFFER_SIZE  (this maps similar to previous mapping but based on snapshot)
                uint32_t base = (inactiveTailIndex + 1) % BUFFER_SIZE;
                startIdx = (base + offset) % BUFFER_SIZE;
            }
            else
            {
                // fallback to current writeHead if no snapshot yet (early boot)
                uint32_t offset2 = (uint32_t)(vpos * (float)BUFFER_SIZE);
                uint32_t s = writeHead + offset2;
                if(s >= BUFFER_SIZE) s %= BUFFER_SIZE;
                startIdx = s;
            }

            float pitchRatio = powf(2.0f, (float)enc_acc / 12.0f);

            triggerGrain(startIdx, lengthSamples, pitchRatio, 1.0f, 0.5f);

            pod.seed.PrintLine("Grain: start=%u len=%u semis=%d", startIdx, lengthSamples, enc_acc);
        }

        // === ENCODER HANDLING: adjust pitch semitones (only when not editing filter) ===
        if(!editEffectsMode)
        {
            int32_t inc = pod.encoder.Increment();
            if(inc != 0) {
                enc_acc += inc;
                if(enc_acc > 24) enc_acc = 24;
                if(enc_acc < -24) enc_acc = -24;
                pod.seed.PrintLine("Pitch: %d semitones", enc_acc);
            }
        }

        // === Effect Edit mapping (filter) ===
        float target_filt = filtCut_sm;
        if(editEffectsMode)
        {
            target_filt = pod.knob1.Value();
        }

        // smooth and publish to audio thread
        filtCut_sm += (target_filt - filtCut_sm) * SMOOTH_ALPHA;
        filtCut_shared = filtCut_sm;   // publish safely

        // === LED feedback ===
        if(g_level_edit)
            pod.led1.Set(0.0f, 0.0f, 1.0f); // blue
        else if(editEffectsMode)
            pod.led1.Set(0.0f, 1.0f, 0.0f); // green
        else
            pod.led1.Set(1.0f, 0.0f, 0.0f); // red
        pod.UpdateLeds();

        // periodic profiler print
        uint32_t t = System::GetNow();
        if(t - lastTick >= 1000)
        {
            lastTick = t;
            uint32_t maxc  = audio_max_cycles;
            uint32_t lastc = audio_last_cycles;
            uint32_t block = pod.AudioBlockSize();
            uint64_t cps_x100 = (uint64_t)maxc * 100ULL / (uint64_t)block;
            const uint64_t CPU_FREQ = 480000000ULL;
            uint64_t cpu_pct_x100 = ((uint64_t)maxc * (uint64_t)pod.AudioSampleRate() * 10000ULL)
                                    / ((uint64_t)block * CPU_FREQ);
            uint64_t cpu_int = cpu_pct_x100 / 100ULL;
            uint64_t cpu_frac = cpu_pct_x100 % 100ULL;

            pod.seed.PrintLine( "Audio cycles last=%lu max=%lu block=%lu cps_x100=%lu cpu=%lu.%02lu%%", 
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
