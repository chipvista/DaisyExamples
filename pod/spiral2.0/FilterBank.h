#pragma once
// FilterBank.h
// Simple wrapper around DaisySP Svf for a low-pass filter with log frequency mapping.

#include "daisysp.h"
#include <cmath>

class FilterBank
{
  public:
    FilterBank() : freq_(1000.0f) {}

    void Init(float sampleRate)
    {
        filt_.Init(sampleRate);
        filt_.SetRes(0.0f); // Very low for pure rolloff, no peak
        freq_ = 1000.0f;
    }

    // knobVal in 0..1 -> freq log scaled from minHz..maxHz
    void SetFreqFromKnob(float knobVal, float minHz = 40.0f, float maxHz = 4000.0f)
    {
        if(knobVal < 0.f) knobVal = 0.f;
        if(knobVal > 1.f) knobVal = 1.f;
        float logMin = log10f(minHz);
        float logMax = log10f(maxHz);
        float logF   = logMin + knobVal * (logMax - logMin);
        freq_ = powf(10.0f, logF);
        filt_.SetFreq(freq_);
    }

    // set resonance (Q style). Keep low for smooth rolloff
    void SetRes(float r) { 
        if(r < 0.0f) r = 0.0f;
        if(r > 0.3f) r = 0.3f;  // Cap at low value to prevent peaks
        filt_.SetRes(r); 
    }

    // process single sample (call Process before reading Low())
    void Process(float in)
    {
        filt_.Process(in);
    }

    // return low-pass output
    float Low() { return filt_.Low(); }

    // For compatibility, redirect Band() calls to Low()
    float Band() { return filt_.Low(); }

    float GetFreq() const { return freq_; }

  private:
    daisysp::Svf filt_;
    float freq_;
};