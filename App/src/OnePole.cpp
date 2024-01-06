
#include "OnePole.h"
#include <cmath>

OnePole::OnePole(float sampleRate, float cuttOffFreq,FilterType type)
{
   setup(sampleRate,cuttOffFreq,type);     
}

void OnePole::setup(float sampleRate, float cuttOffFreq,FilterType type)
{
   _sampleRate = sampleRate;
   _type = type;
   _z1 = .01;
   updateFc(cuttOffFreq);
}

float OnePole::process(float input)
{
    return _z1 = input * _a0 + _z1 * _b1;
}

void OnePole::updateFc(float cutoffFreq)
{
    switch (_type)
    {
    case FilterType::LOW_PASS:
        _b1 = expf(-2.0f * (float)(M_PI) * cutoffFreq/_sampleRate);
        _a0 = 1.0f - _b1;
        break;
    case FilterType::HIGH_PASS:
        _b1 = -expf(-2.0f * (float)(M_PI) * (0.5f - cutoffFreq/_sampleRate));
        _a0 = 1.0f + _b1;
        break;
    }   
}
