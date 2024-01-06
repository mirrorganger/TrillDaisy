#pragma once

class OnePole
{   
public:
    enum class FilterType{
        LOW_PASS,
        HIGH_PASS
    };
    OnePole() = default;
    OnePole(float sampleRate, float cuttOffFreq,FilterType type = FilterType::LOW_PASS);
    void setup(float sampleRate, float cuttOffFreq,FilterType type = FilterType::LOW_PASS);
    float process(float input);    
private:
    void updateFc(float cutoffFreq);
    FilterType _type;
    float _sampleRate;
    float _a0;
    float _b1;
    float _z1;
};



