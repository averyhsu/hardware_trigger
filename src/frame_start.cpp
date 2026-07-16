#include <Arduino.h>

// ============================================================================
//  Camera trigger  —  TIMED mode
//  ----------------------------------------------------------------------------
//  The MCU emits ONE clean rising edge per frame. That edge means
//  "start exposing now." The CAMERA enforces the exposure length itself
//  (set ExposureTime on the camera — see the config notes at the bottom).
//
//  Because the camera owns the exposure, the pulse width below is irrelevant
//  to image brightness; it only has to be a clean, detectable pulse. There is
//  no exposure-ending one-shot timer here, and a stuck-high line cannot cause
//  a runaway exposure (the worst case is simply a missed next trigger).
//
//  Board: Teensy (uses IntervalTimer + digitalWriteFast). Will not compile on
//  a classic AVR Arduino or a Pico without changes.
// ============================================================================

const int TRIG_PIN = 2;

volatile float fps = 100.0f;              // frame rate (Hz)

// Trigger pulse width. Only needs to be a clean edge the camera can latch;
// 10 us sits well above any camera's minimum trigger-pulse spec. This does
// NOT set exposure — ExposureTime on the camera does.
const uint32_t TRIG_PULSE_US = 10;

IntervalTimer frameTimer;                // fires once per frame period

volatile uint32_t frameCount = 0;        // optional bring-up heartbeat

// ---- Per-trigger timestamp log ---------------------------------------------
//  Every trigger is stamped with micros() at the rising edge (= exposure start)
//  and its sequence number. The ISR pushes into a lock-free single-producer /
//  single-consumer ring; loop() drains it and prints "T <seq> <t_us>" over
//  serial. The HOST joins each camera frame to this by frame counter, so every
//  frame carries the Teensy exposure time — the system's master clock.
//  (32-bit aligned reads/writes are atomic on Teensy 4.x, so no locks needed
//   as long as the ISR only writes stampHead and loop() only writes stampTail.)
const uint32_t STAMP_BUF = 512;          // power of two; ~5 s of slack at 100 Hz
volatile uint32_t stampSeq[STAMP_BUF];
volatile uint32_t stampT[STAMP_BUF];
volatile uint32_t stampHead = 0;         // producer index (ISR only)
uint32_t stampTail = 0;                  // consumer index (loop only)

// ---- Frame trigger ISR -----------------------------------------------------
void onFrameStart() {
  digitalWriteFast(TRIG_PIN, HIGH);      // rising edge -> "start exposing now"
  uint32_t t = micros();                 // exposure-start stamp on the master clock
  delayMicroseconds(TRIG_PULSE_US);      // brief, clean pulse (~10 us)
  digitalWriteFast(TRIG_PIN, LOW);       // return low so the next edge is clean
  uint32_t h = stampHead;                // publish (seq, t_exposure) to the ring
  stampSeq[h] = frameCount;
  stampT[h]   = t;
  stampHead   = (h + 1) & (STAMP_BUF - 1);
  frameCount++;                          // camera closes its own shutter after ExposureTime
}

// ---- Start / change frame rate ---------------------------------------------
void startTriggering(float newFps) {
  frameTimer.end();
  fps = newFps;
  // Pass the period as a float: IntervalTimer accepts fractional microseconds,
  // and the Teensy 4.x PIT is clocked from the 24 MHz crystal (~41.7 ns/tick).
  // Truncating to whole microseconds here would add an accumulating drift
  // (e.g. 60 fps -> 16666 us -> 60.0024 Hz). The float keeps the period exact.
  float periodUs = 1000000.0f / fps;
  frameTimer.begin(onFrameStart, periodUs);
  frameTimer.priority(0);                // highest priority so USB/serial can't delay edges
}

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  digitalWriteFast(TRIG_PIN, LOW);       // idle low
  Serial.begin(115200);

  startTriggering(fps);                // uses `fps` above; exposure is enforced by the camera
}

void loop() {
  // Drain per-trigger stamps and emit "T <seq> <t_us>" for the host time-join.
  // Printing here (not in the ISR) keeps the trigger edges jitter-free.
  while (stampTail != stampHead) {
    uint32_t s = stampSeq[stampTail];
    uint32_t t = stampT[stampTail];
    stampTail = (stampTail + 1) & (STAMP_BUF - 1);
    Serial.printf("T %lu %lu\n", s, t);
  }

  // Optional heartbeat: confirms triggering is alive without needing a scope.
  // Remove this whole block if you want loop() empty.
  static uint32_t last = 0, lastCount = 0, t0 = 0;
  if (t0 == 0) t0 = micros();            // start of the averaging window
  if (millis() - last >= 1000) {
    uint32_t c = frameCount;             // 32-bit read is atomic on Teensy
    float avgHz = c * 1e6f / (float)(micros() - t0);   // exact long-run average
    Serial.printf("triggers/sec: %lu  (total %lu, avg %.4f Hz)\n", c - lastCount, c, avgHz);
    lastCount = c;
    last = millis();
  }
}