// imu.cpp -- see imu.h
#include "imu.h"
#include <Wire.h>
#include "config.h"

void Imu::write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool Imu::begin() {
  Wire.begin();
  Wire.setClock(400000);   // fast I2C: the 14-byte read takes ~0.4 ms
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) return false;
  write(0x6B, 0x01);   // PWR_MGMT_1: wake up, gyro X clock
  write(0x1A, 0x02);   // CONFIG: digital low-pass ~94 Hz
  write(0x1B, 0x08);   // GYRO_CONFIG: +/-500 deg/s (65.5 LSB per deg/s)
  write(0x1C, 0x08);   // ACCEL_CONFIG: +/-4 g (8192 LSB per g)
  delay(100);
  return true;
}

// Accelerometer tilt angle [rad] and gyro rate [rad/s] about the gimbal
// axis, with sign, "upright" offset and gyro bias applied.
bool Imu::readRaw(float &acc_angle, float &gyro_rate) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);   // ACCEL_XOUT_H: accel(6) + temp(2) + gyro(6)
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return false;

  int16_t raw[7];
  for (int i = 0; i < 7; i++) raw[i] = (Wire.read() << 8) | Wire.read();
  float ax = raw[0], ay = raw[1], az = raw[2];        // units cancel in atan2
  float gx = raw[4] / 65.5f, gy = raw[5] / 65.5f;     // deg/s

  float a_raw, g_raw;
  if (IMU_USE_Y_AXIS) { a_raw = atan2f(-ax, az); g_raw = radians(gy); }
  else                { a_raw = atan2f(ay, az);  g_raw = radians(gx); }

  // Wrap into -180..+180 deg so crossing atan2's seam isn't a 360 deg jump.
  float d = a_raw - angle_offset_;
  if (d > PI) d -= 2 * PI;
  if (d < -PI) d += 2 * PI;
  acc_angle = IMU_SIGN * d;
  gyro_rate = IMU_SIGN * (g_raw - gyro_bias_);
  return true;
}

bool Imu::update(float dt) {
  float acc_angle, gyro_rate;
  if (!readRaw(acc_angle, gyro_rate)) return false;
  // Complementary filter: the gyro is smooth but drifts, the accel is
  // noisy (EDF vibration) but doesn't drift. Blend: trust the gyro over
  // short times, the accel over long times (crossover ~COMP_TAU).
  const float k = COMP_TAU / (COMP_TAU + dt);
  theta_ = k * (theta_ + gyro_rate * dt) + (1.0f - k) * acc_angle;
  rate_ = gyro_rate;
  return true;
}

// Average while held still and upright: sets the gyro bias and defines
// "upright" as the current orientation (the IMU needn't be mounted
// perfectly straight).
void Imu::calibrate() {
  angle_offset_ = 0;
  gyro_bias_ = 0;
  const int N = 500;
  float a_sum = 0, g_sum = 0, a, g;
  int got = 0;
  for (int i = 0; i < N; i++) {
    if (readRaw(a, g)) { a_sum += a; g_sum += g; got++; }
    delay(2);
  }
  if (got == 0) return;
  // readRaw applied IMU_SIGN; undo it so the offsets are in raw units
  angle_offset_ = IMU_SIGN * a_sum / got;
  gyro_bias_ = IMU_SIGN * g_sum / got;
  theta_ = 0;
  rate_ = 0;
}
