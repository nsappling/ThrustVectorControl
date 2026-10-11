// esc.cpp -- see esc.h
#include "esc.h"
#include "config.h"

void Esc::begin(int pin) {
  pwm_.attach(pin, ESC_MIN_US, ESC_MAX_US);
  stop();   // ESCs only arm after seeing "zero throttle" at power-up
}

void Esc::set(float throttle) {
  if (!(throttle > 0)) throttle = 0;              // also catches NaN
  if (throttle > THROTTLE_MAX) throttle = THROTTLE_MAX;
  throttle_ = throttle;
  us_ = ESC_MIN_US + (int)(throttle * (ESC_MAX_US - ESC_MIN_US));
  pwm_.writeMicroseconds(us_);
}
