// imu.h -- GY-521 (MPU-6050) over raw I2C + complementary filter.
// Ported from firmware/tvc_controller. See docs/icd/imu_i2c.md.
#pragma once
#include <Arduino.h>

class Imu {
 public:
  bool begin();                  // false = device not found at MPU_ADDR
  void calibrate();              // hold upright + still: sets gyro bias and "upright"
  bool update(float dt);         // read once and fuse; false = I2C read failed

  float theta() const { return theta_; }   // [rad] fused body angle
  float rate() const { return rate_; }     // [rad/s] gyro rate, bias-corrected

 private:
  bool readRaw(float &acc_angle, float &gyro_rate);
  void write(uint8_t reg, uint8_t val);

  float gyro_bias_ = 0, angle_offset_ = 0;
  float theta_ = 0, rate_ = 0;
};
