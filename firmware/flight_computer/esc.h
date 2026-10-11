// esc.h -- EDF throttle through the ESC's RC PWM input. docs/icd/edf_esc.md.
#pragma once
#include <Arduino.h>
#include <Servo.h>

class Esc {
 public:
  void begin(int pin);           // starts at ESC_MIN_US (motor stopped)
  void set(float throttle);      // 0..1, hard-clamped to THROTTLE_MAX
  void stop() { set(0); }
  float throttle() const { return throttle_; }
  int pulseUs() const { return us_; }

 private:
  Servo pwm_;                    // the Servo library makes the 50 Hz RC pulse
  float throttle_ = 0;
  int us_ = 0;
};
