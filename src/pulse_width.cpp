#include <Arduino.h>

const int TRIG_PIN = 2;

volatile float fps = 60.0f;
const uint32_t EXPOSURE_US = 177;   // LevelHigh duration = exposure (trigger width mode)

IntervalTimer frameTimer;   // fires at frame period -> line HIGH (start exposure)
IntervalTimer pulseTimer;   // one-shot -> line LOW after 177us (end exposure)

void onExposureEnd() {
  digitalWriteFast(TRIG_PIN, LOW);   // falling edge -> camera closes shutter
  pulseTimer.end();                  // one-shot: disarm until next frame
}

void onFrameStart() {
  digitalWriteFast(TRIG_PIN, HIGH);          // line high -> camera starts exposing
  pulseTimer.begin(onExposureEnd, EXPOSURE_US);
}

void startTriggering(float newFps) {
  frameTimer.end();
  fps = newFps;
  uint32_t periodUs = (uint32_t)(1000000.0f / fps);
  frameTimer.begin(onFrameStart, periodUs);
  frameTimer.priority(0);   // highest priority so USB/serial can't delay edges
  pulseTimer.priority(0);
}

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  digitalWriteFast(TRIG_PIN, LOW);

  startTriggering(60.0f);   // 60 FPS, 177us exposure per frame
}

void loop() {
  // free — trigger + exposure gating run entirely in hardware timer ISRs
}