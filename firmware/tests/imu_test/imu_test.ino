// imu_test.ino -- bring-up STAGE 2: GY-521 (MPU-6050) on the Teensy 4.0.
// Board: Teensy 4.0 (USB Type: Serial).
// Wiring (docs/icd/imu_i2c.md, diagram docs/bringup/img/stage2_imu.svg):
//   GY-521 VCC -> Teensy 3.3V     GY-521 GND -> Teensy GND
//   GY-521 SCL -> Teensy 19       GY-521 SDA -> Teensy 18
// Procedure: docs/bringup/README.md, stage 2.
//
// On boot it scans the I2C bus, checks the chip's WHO_AM_I register, sets
// the SAME ranges as flight_computer (+/-500 deg/s, +/-4 g, ~94 Hz
// filter), then streams CSV at 20 Hz:
//   t_ms, ax_g, ay_g, az_g, gx_dps, gy_dps, gz_dps, angle_x_deg, angle_y_deg, temp_c
// angle_x = tilt about the IMU's X axis = atan2(ay, az) (flight_computer's
//           default, IMU_USE_Y_AXIS = false)
// angle_y = tilt about the IMU's Y axis = atan2(-ax, az)
// Tilt the board the way the rocket will tip: whichever angle changes is
// the axis to use.
//
// Commands (line ending "Newline"):
//   s   stream on/off
//   n   noise check: hold STILL; averages 2 s and prints mean + std dev
//   scan  re-scan the I2C bus
//
// Self-contained on purpose (no shared files), so it still works if
// flight_computer is broken.

#include <Wire.h>

const uint8_t MPU_ADDR = 0x68;   // AD0 open/low
bool streaming = true;

// One reading in physical units. Defined up here, before any function,
// because the Arduino IDE auto-generates function declarations at the
// top of the file and they need to know this type.
struct Sample { float ax, ay, az, gx, gy, gz, temp; };

// ---------------------------------------------------------------------
// I2C helpers
// ---------------------------------------------------------------------
void mpu_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

int mpu_read8(uint8_t reg) {   // -1 = no answer
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

// CSV header: at boot, and on "header" (for a logger that connected late)
void print_header() {
  Serial.println(F("t_ms,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,angle_x_deg,angle_y_deg,temp_c"));
}

void scan_bus() {
  Serial.println(F("# I2C scan:"));
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.print(F("#   device at 0x")); Serial.println(a, HEX);
      found++;
    }
  }
  if (!found) Serial.println(F("#   NOTHING found -- check VCC=3.3V, GND, SDA=18, SCL=19 (not swapped?)"));
}

// Raw sensor read. Returns false if the I2C transfer failed.

bool read_sample(Sample &s) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);   // ACCEL_XOUT_H: accel(6) + temp(2) + gyro(6)
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return false;
  int16_t r[7];
  for (int i = 0; i < 7; i++) r[i] = (Wire.read() << 8) | Wire.read();
  s.ax = r[0] / 8192.0f;  s.ay = r[1] / 8192.0f;  s.az = r[2] / 8192.0f;   // +/-4 g range
  s.temp = r[3] / 340.0f + 36.53f;                                          // datasheet formula
  s.gx = r[4] / 65.5f;    s.gy = r[5] / 65.5f;    s.gz = r[6] / 65.5f;     // +/-500 deg/s range
  return true;
}

// ---------------------------------------------------------------------
// Noise check: hold still for 2 s, report mean and std dev of each axis.
// At rest you expect: |accel| ~= 1 g total, gyro means within a few deg/s
// (that's the bias flight_computer calibrates out), gyro std dev < ~0.1
// deg/s, angle std dev < ~0.1 deg (higher once the EDF vibrates it).
// ---------------------------------------------------------------------
void noise_check() {
  Serial.println(F("# Noise check: hold STILL for 2 s..."));
  const int N = 400;
  double sum[6] = {0}, sq[6] = {0};
  int got = 0;
  for (int i = 0; i < N; i++) {
    Sample s;
    if (read_sample(s)) {
      float v[6] = { s.gx, s.gy, s.gz, degrees(atan2f(s.ay, s.az)), degrees(atan2f(-s.ax, s.az)),
                     sqrtf(s.ax*s.ax + s.ay*s.ay + s.az*s.az) };
      for (int k = 0; k < 6; k++) { sum[k] += v[k]; sq[k] += (double)v[k]*v[k]; }
      got++;
    }
    delay(5);
  }
  if (got < N / 2) { Serial.println(F("# Noise check FAILED: too many read errors")); return; }
  const char *names[6] = { "gyro_x dps", "gyro_y dps", "gyro_z dps", "angle_x deg", "angle_y deg", "|accel| g" };
  for (int k = 0; k < 6; k++) {
    double mean = sum[k] / got;
    double sd = sqrt(max(0.0, sq[k] / got - mean * mean));
    Serial.print(F("#   ")); Serial.print(names[k]);
    Serial.print(F(": mean ")); Serial.print(mean, 3);
    Serial.print(F("  std ")); Serial.println(sd, 3);
  }
  Serial.print(F("#   ")); Serial.print(got); Serial.print(F("/")); Serial.print(N);
  Serial.println(F(" samples read OK"));
}

// ---------------------------------------------------------------------
// SETUP
// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  Wire.begin();
  Wire.setClock(400000);   // same as flight_computer
  Serial.println(F("# imu_test: GY-521 on Teensy 4.0"));
  scan_bus();

  // WHO_AM_I (register 0x75) is 0x68 on a genuine MPU-6050. Some clone
  // boards answer 0x70, 0x72 or 0x98 but still work with the same registers.
  int who = mpu_read8(0x75);
  Serial.print(F("# WHO_AM_I = 0x")); Serial.println(who, HEX);
  if (who < 0) {
    Serial.println(F("# FAIL: no reply at 0x68. Fix wiring and reset the Teensy."));
  } else if (who != 0x68) {
    Serial.println(F("# Note: not 0x68 -- likely a clone chip. Continuing; check the readings make sense."));
  }

  // Same configuration as flight_computer/imu.cpp
  mpu_write(0x6B, 0x01);   // PWR_MGMT_1: wake up, gyro X clock
  mpu_write(0x1A, 0x02);   // CONFIG: digital low-pass ~94 Hz
  mpu_write(0x1B, 0x08);   // GYRO_CONFIG: +/-500 deg/s
  mpu_write(0x1C, 0x08);   // ACCEL_CONFIG: +/-4 g
  delay(100);

  print_header();
  Serial.println(F("# Commands: s (stream on/off), n (noise check), scan"));
}

// ---------------------------------------------------------------------
// LOOP
// ---------------------------------------------------------------------
void loop() {
  // --- commands ---
  static char buf[16];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (n == 0) continue;
      buf[n] = 0; n = 0;
      if (!strcmp(buf, "s")) { streaming = !streaming; Serial.println(streaming ? F("# stream on") : F("# stream off")); }
      else if (!strcmp(buf, "n")) noise_check();
      else if (!strcmp(buf, "scan")) scan_bus();
      else if (!strcmp(buf, "header")) print_header();
      else if (!strcmp(buf, "hb")) {}   // heartbeat from the logger: ignore
      else Serial.println(F("# Unknown. Commands: s, n, scan"));
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }

  // --- stream at 20 Hz ---
  static uint32_t last = 0;
  if (!streaming || millis() - last < 50) return;
  last = millis();
  Sample s;
  if (!read_sample(s)) { Serial.println(F("# read FAILED (loose wire?)")); return; }
  Serial.print(millis());                              Serial.print(',');
  Serial.print(s.ax, 3); Serial.print(',');  Serial.print(s.ay, 3); Serial.print(',');
  Serial.print(s.az, 3); Serial.print(',');
  Serial.print(s.gx, 2); Serial.print(',');  Serial.print(s.gy, 2); Serial.print(',');
  Serial.print(s.gz, 2); Serial.print(',');
  Serial.print(degrees(atan2f(s.ay, s.az)), 2);  Serial.print(',');
  Serial.print(degrees(atan2f(-s.ax, s.az)), 2); Serial.print(',');
  Serial.println(s.temp, 1);
}
