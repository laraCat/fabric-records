#include <Adafruit_TCS34725.h>

#define S30 D18  // SWITCH // VOLUME ON-OFF
#define S31 A0   // KNOB // PITCH
#define S32 A1   // KNOB // OSCILLATOR FREQUENCY
#define S33 A2   // KNOB // TIMBRE

///////////////////////////////////////////////////////////////
///////////////////// LIBRARIES & HARDWARE ////////////////////
#include "vox.h"
#include "flt.h"
#include <Wire.h>
#include <ColorSensorTCS34725.h>

// color sensor
#define COLOR_SDA_PIN 12
#define COLOR_SCL_PIN 11
ColorSensorTCS34725 colorSensor(COLOR_SDA_PIN, COLOR_SCL_PIN);
RGBC colorRef;
int lightLevel;
char color = '0';

///////////////////////////////////////////////////////////////
///////////////////// MODULES /////////////////////////////////

const int kVoxCount = 15;
const float kVoxVolumeKof = 3.0 / kVoxCount;
Vox voxs[kVoxCount];

Filter flt;

bool gate = false;

ReverbSc verb;

///////////////////////////////////////////////////////////////
///////////////////// AUDIO CALLBACK (PATCH) //////////////////
void AudioCallback(float** in, float** out, size_t size) {

  for (size_t i = 0; i < size; i++) {
    float output = 0;
    float RvOutput1;
    float RvOutput2;
    if (gate == 0) {
      verb.Process(in[0][i], in[1][i], &RvOutput1, &RvOutput2);
      for (auto& vox : voxs) output += vox.Process() + RvOutput1;
      output = flt.Process(output) * kVoxVolumeKof;
      
    }
    out[0][i] = out[1][i] = output;
  }
}

///////////////////////////////////////////////////////////////
///////////////////// SETUP ///////////////////////////////////

void setup() {
  Serial.begin(9600);
  while (!Serial);
  // DAISY SETUP
  DAISY.init(DAISY_SEED, AUDIO_SR_48K);
  auto sampleRate = DAISY.get_samplerate();

  // VOX SETUP
  float voxFreq = Vox::kOscLowestFreq;
  for (auto& vox : voxs) {
    vox.Init(sampleRate, voxFreq);
    voxFreq *= 1.5;
    if (voxFreq > 1500) voxFreq *= 0.5;
  
  }

  // Set the parameters for reverb
  verb.Init(sampleRate);
  verb.SetFeedback(0.85f);
  verb.SetLpFreq(18000.0f);

  //color sensor
  colorSensor.setWaitTime(0);
  colorSensor.setIntegrationTime(200);
  colorSensor.setGain(CS_GAIN_16);
  colorSensor.begin();
  colorRef = colorSensor.readRGBC();

  // FILTER SETUP
  flt.Init(sampleRate);

  // GATE SWITCH SETUP
  pinMode(S30, INPUT_PULLUP);

  // BEGIN CALLBACK
  DAISY.begin(AudioCallback);
}

///////////////////////////////////////////////////////////////
///////////////////// LOOP ////////////////////////////////////

void loop() {
  for (auto& vox : voxs) { vox.Read(S31, S32); 
    
    if (color == 'B'){
      vox.Wave(Oscillator::WAVE_SAW);
    }else {
      vox.Wave(Oscillator::WAVE_SIN);
    }
    
    }
  flt.SetTimbre(analogRead(S33) / 1023.0);
  gate = digitalRead(S30);
  delay(4);
  
  if (colorSensor.isReady()) {
    colorSensor.clearReady();
    RGBC v = colorSensor.readRGBCRelative(colorRef);
    RGBC lx = colorSensor.readRGBC();
    LightValues lv = colorSensor.calcLight(lx);
    lightLevel = lv.lux, 0;
    Serial.print("Lux: ");
    Serial.println(lightLevel);
    Serial.print("   Hue: ");
    int hue = colorSensor.calcHue(v);
   //Serial.print(hue);
    if (hue >= 330 || (hue >= 0 && hue < 20)) {
      color = 'R';
      Serial.println(color);  
    }
    if (hue >= 20 && hue < 50) {
      Serial.println(" orange");
    if (hue >= 50 && hue < 70) Serial.println(" yellow ");
    if (hue >= 70 && hue < 150) Serial.println(" green");
    if (hue >= 150 && hue < 210) Serial.println(" cyan");
    if (hue >= 210 && hue < 250) {
      color = 'B';
      Serial.println(color);
    }
    if (hue >= 250 && hue < 280) Serial.println(" purple");
    if (hue >= 280 && hue < 330) Serial.println(" magenta");
  }
delay(100);
}

