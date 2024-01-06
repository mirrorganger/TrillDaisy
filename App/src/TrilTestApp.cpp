
#include "daisy_seed.h"
#include "daisysp.h"
#include "Trill.h"
#include "OnePole.h"

using namespace daisy;
using namespace daisysp;
using namespace daisy::seed;

static DaisySeed hw;
static TrillDaisy::Trill trill;
static Oscillator osc;
static bool trigger;
static Adsr env;
static OnePole sizeFilter;
static OnePole panningFilter;
static float ampL = 1.0;
static float ampR = 1.0;

void printer(const char * str)
{
    hw.PrintLine("%s",str);
}

void updateOsc(){
  if(trill.getNumDetectedVerticalTouches()>0){
    osc.SetFreq(fmap(trill.getAverageX(),120,600.0,daisysp::Mapping::EXP));
    osc.SetAmp(sizeFilter.process(trill.getAverageSize()*1.5));
    trigger = true;
  }else{
    trigger = false;
  }
  if(trill.getNumDetectedHorizontalTouches()>0){
     float panning = panningFilter.process(trill.getTouchHorizontalLocation(0));
     ampL = 1.0f - panning;
     ampR = panning;
  }
}


static void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                          AudioHandle::InterleavingOutputBuffer out,
                          size_t                                size)
{
    float sig;
    for(size_t i = 0; i < size; i += 2)
    {
        sig = osc.Process() * env.Process(trigger);
        // left out
        out[i] = sig * ampL;

        // right out
        out[i + 1] = sig * ampR;
    }
}


int main(void) {

  hw.Init();
  hw.Configure();
  hw.SetAudioBlockSize(16);
  float  sample_rate = hw.AudioSampleRate();
  osc.Init(sample_rate);
  osc.SetWaveform(osc.WAVE_SIN);
  osc.SetFreq(50);
  osc.SetAmp(0.9);
  env.Init(sample_rate);
  env.SetSustainLevel(1.0);
  sizeFilter.setup(sample_rate,1.0);
  panningFilter.setup(sample_rate,1000.0);
  hw.StartLog(false);

  printer("Starting app");

  trill.init(TrillDaisy::Trill::DeviceType::TRILL_SQUARE, &printer);

  hw.DelayMs(10);
  if(trill.prepareForDataRead()){
    hw.PrintLine("Ready for data reading");       
  }

  hw.StartAudio(AudioCallback);

  while(1) {
    hw.DelayMs(50);

    trill.scanSensor();
    updateOsc();
  }
}
