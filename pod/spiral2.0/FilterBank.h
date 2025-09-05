#pragma once
// FilterBank.h
// Simple wrapper around DaisySP Svf for a band-pass with log frequency mapping.

#include "daisysp.h"
#include <cmath>

class FilterBank
{
  public:
    FilterBank() : freq_(1000.0f) {}

    void Init(float sampleRate)
    {
        filt_.Init(sampleRate);
        filt_.SetRes(0.15f); //moderate resonance(changeable) essentially flat (no resonant peak)
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

    // set resonance (Q style). 0.1..2 reasonable
    void SetRes(float r) { filt_.SetRes(r); }

    // process single sample (call Process before reading Band())
    void Process(float in)
    {
        filt_.Process(in);
    }

    // return band-pass output
    float Band() { return filt_.Band(); }

    float GetFreq() const { return freq_; }

  private:
    daisysp::Svf filt_;
    float freq_;
};
