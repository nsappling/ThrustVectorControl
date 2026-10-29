// imu_servo_follow.ino -- the gimbal servo copies the IMU's tilt, one to one.
// Tilt the IMU 10 deg and the servo turns 10 deg the same way. No EDF, no
// control loop: this checks that the IMU angle, the axis choice, the sign,
// and the servo mapping are all right before closed-loop control.
//
// Based on Kailash's ServoHiwonderKeyboardControlBringupTest (same move
// packet, same Serial2 wiring).
//
// Board: Teensy 4.1
//
// Wiring:
//   GY-521 SDA  -> Teensy 18 (SDA)
//   GY-521 SCL  -> Teensy 19 (SCL)
//   GY-521 VCC  -> Teensy 3.3V  (NOT VIN/5 V: keeps SDA/SCL at a Teensy-safe 3.3 V)
//   GY-521 GND  -> Teensy GND
//   Teensy 8 (TX2) -> BusLinker RX
//   Teensy 7 (RX2) <- BusLinker TX
//   Teensy GND  -- BusLinker GND
//   BusLinker jumpers on "Servo + TTL", 12 V on the BusLinker power input.
//
// HOW TO USE
//   1. Hold the IMU (or the rocket) UPRIGHT and STILL, then power up / reset.
//      For ~1 s the sketch measures the gyro bias, finds which IMU axis is
//      "up" (the one gravity pulls on), and takes the current tilt as zero.
//   2. Open the Serial Monitor (115200, line ending "Newline").
//   3. Tilt the IMU slowly. The servo follows.
//
// Serial Monitor commands:
//   z   re-zero: hold it upright and still, then the current tilt becomes 0 deg
//   h   hold: servo goes to center and stops following
//   g   go: follow the IMU again
//   s   swap direction (servo turns the other way)
//
// SETTING THE CENTER (servo 500 is usually NOT "fan straight")
//   1. Type h. The servo goes to its center and holds.
//   2. Nudge it until the fan points straight down the rocket body:
//        +  /  -    move 1 count  (0.24 deg)
//        ]  /  [    move 10 counts (2.4 deg)
//   3. The current center is printed after every nudge. Copy that number
//      into SERVO_CENTER below (and config.h for flight_computer), upload
//      again, and type g to follow.
//   If you need more than ~50 counts (12 deg), re-seat the servo horn one
//   spline tooth closer to straight instead, so the full travel stays
//   usable on both sides.
//
// What gets printed 10x per second:
//   imu_deg  = filtered IMU tilt about the gimbal axis
//   servo_deg = what the servo was told (after clamping)
//   counts   = the servo position actually sent (center = fan straight)

#include <Arduino.h>
#include <Wire.h>

// ---------------------------------------------------------------------
// SETTINGS
// ---------------------------------------------------------------------
#define SERVO_ID 1

const uint8_t MPU_ADDR = 0x68;        // GY-521 with AD0 open / low

// Which IMU axis points in the direction the gimbal tilts (0 = X, 1 = Y, 2 = Z).
// The printed silkscreen arrows on the GY-521 show X and Y. Pick the axis that
// lies in the plane the fan swings in. If the servo responds to the WRONG
// tilt direction (e.g. tilting front/back moves it, side to side doesn't),
// change this to the other horizontal axis.
const int SIDE_AXIS = 0;

const float FOLLOW_GAIN = 1.0;        // servo deg per IMU deg (1.0 = one to one)
float servo_dir = 1.0;                // +1 / -1. Flip with "s" if it turns the wrong way.

const int   SERVO_CENTER = 500;       // counts where the FAN is straight. Find it with h, +/-, [/]
const int   SERVO_RANGE  = 150;       // allowed counts each side of center (~36 deg),
                                      // same as Kailash's 350..650 test
const float COUNTS_PER_DEG = 1000.0 / 240.0;   // HTD-30H: 0..1000 counts = 0..240 deg

const float COMP_TAU = 0.5;           // [s] complementary filter time constant.
                                      // Gyro for fast motion, accel to stop drift.

const uint32_t IMU_PERIOD_US   = 5000;   // read IMU at 200 Hz
const uint32_t SERVO_PERIOD_MS = 20;     // send a servo move at 50 Hz
const uint32_t PRINT_PERIOD_MS = 100;    // print at 10 Hz

// ---------------------------------------------------------------------
// STATE
// ---------------------------------------------------------------------
int   up_axis = 2;        // found at startup: the axis gravity pulls on
float up_sign = 1;        // +1 if that axis reads +1 g when upright, else -1
int   rot_axis = 1;       // the gyro axis the gimbal tilt rotates about
float gyro_sign = 1;      // makes the gyro rate agree with the accel angle
float gyro_bias = 0;      // [deg/s] measured at startup
float zero_deg = 0;       // tilt taken as "upright"

float angle_deg = 0;      // filtered tilt
float servo_deg = 0;
int   counts_sent = SERVO_CENTER;
bool  following = true;
int   center = SERVO_CENTER;   // live center, nudged with + - [ ]

// ---------------------------------------------------------------------
// SERVO (HiWonder 0x55 0x55 move command, as in Kailash's sketch)
// ---------------------------------------------------------------------
void moveServo(uint16_t position, uint16_t duration) {
  position = constrain(position, center - SERVO_RANGE, center + SERVO_RANGE);
  position = constrain(position, 0, 1000);   // servo's own limits

  uint8_t packet[10] = {
    0x55, 0x55,                 // Header
    SERVO_ID,                   // Servo ID
    0x07,                       // Length
    0x01,                       // Move command
    (uint8_t)(position & 0xFF),
    (uint8_t)(position >> 8),
    (uint8_t)(duration & 0xFF),
    (uint8_t)(duration >> 8),
    0x00                        // Checksum placeholder
  };
  uint16_t sum = 0;
  for (int i = 2; i < 9; i++) sum += packet[i];
  packet[9] = (uint8_t)(~sum);

  Serial2.write(packet, sizeof(packet));
  counts_sent = position;
}

// ---------------------------------------------------------------------
// IMU (MPU-6050 registers, no library needed)
// ---------------------------------------------------------------------
void imuWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Reads accel [g] and gyro [deg/s] for X, Y, Z. Returns false on an I2C error.
bool imuRead(float acc[3], float gyr[3]) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);                                   // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return false;
  int16_t raw[7];
  for (int i = 0; i < 7; i++) raw[i] = (Wire.read() << 8) | Wire.read();
  for (int i = 0; i < 3; i++) {
    acc[i] = raw[i] / 8192.0f;                        // +/-4 g range
    gyr[i] = raw[i + 4] / 65.5f;                      // +/-500 deg/s range
  }
  return true;
}

// Tilt from gravity alone [deg]: angle between "up" and the gravity vector,
// measured in the plane of the side axis.
float accelAngle(const float acc[3]) {
  return degrees(atan2f(acc[SIDE_AXIS], up_sign * acc[up_axis]));
}

// ---------------------------------------------------------------------
// STARTUP CALIBRATION: IMU must be upright and still
// ---------------------------------------------------------------------
void calibrate() {
  Serial.println("# Hold the IMU upright and still...");
  float acc_sum[3] = {0, 0, 0}, gyr_sum[3] = {0, 0, 0};
  int n = 0;
  for (int i = 0; i < 200; i++) {                     // ~1 s
    float a[3], g[3];
    if (imuRead(a, g)) {
      for (int k = 0; k < 3; k++) { acc_sum[k] += a[k]; gyr_sum[k] += g[k]; }
      n++;
    }
    delay(5);
  }
  if (n == 0) { Serial.println("# ERROR: no IMU data"); return; }

  // "Up" = whichever axis gravity pulls on the most (must not be SIDE_AXIS).
  float best = 0;
  for (int k = 0; k < 3; k++) {
    if (k == SIDE_AXIS) continue;
    float a = acc_sum[k] / n;
    if (fabsf(a) > fabsf(best)) { best = a; up_axis = k; }
  }
  up_sign = (best >= 0) ? 1 : -1;

  // The gimbal tilt rotates about the third axis. Right-hand rule:
  // if (side, up, rot) are in X->Y->Z order the gyro already agrees with
  // atan2(side, up); otherwise flip it. Flipping "up" also flips it.
  rot_axis = 3 - SIDE_AXIS - up_axis;
  bool cyclic = ((up_axis - SIDE_AXIS + 3) % 3) == 1;
  gyro_sign = (cyclic ? 1 : -1) * up_sign;
  gyro_bias = gyr_sum[rot_axis] / n;

  float a_avg[3] = { acc_sum[0] / n, acc_sum[1] / n, acc_sum[2] / n };
  zero_deg = accelAngle(a_avg);
  angle_deg = 0;

  const char *name = "XYZ";
  Serial.print("# up axis: ");      Serial.print(up_sign > 0 ? "+" : "-"); Serial.print(name[up_axis]);
  Serial.print("  side axis: ");    Serial.print(name[SIDE_AXIS]);
  Serial.print("  gyro axis: ");    Serial.print(name[rot_axis]);
  Serial.print("  gyro bias: ");    Serial.print(gyro_bias, 2);
  Serial.print(" deg/s  zero: ");   Serial.print(zero_deg, 2); Serial.println(" deg");
  if (fabsf(best) < 0.7) Serial.println("# WARNING: gravity not clearly on one axis. Was it upright?");
}

// ---------------------------------------------------------------------
// SETUP / LOOP
// ---------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  while (!Serial && millis() < 3000) {}

  Wire.begin();
  Wire.setClock(400000);
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("# ERROR: MPU-6050 not found at 0x68. Check SDA=18, SCL=19, VCC, GND.");
    while (true) {}
  }
  imuWrite(0x6B, 0x01);   // PWR_MGMT_1: wake up, gyro clock
  imuWrite(0x1A, 0x03);   // CONFIG: digital low-pass ~44 Hz (cuts vibration)
  imuWrite(0x1B, 0x08);   // GYRO_CONFIG: +/-500 deg/s
  imuWrite(0x1C, 0x08);   // ACCEL_CONFIG: +/-4 g
  delay(100);

  moveServo(center, 500);         // ease to center first
  delay(600);
  calibrate();
  Serial.println("# Following. Commands: z (re-zero) | h (hold) | g (go) | s (swap direction)");
  Serial.println("# Center: h, then + - (1 count) or ] [ (10 counts)");
  Serial.println("imu_deg,servo_deg,counts");
}

void loop() {
  static uint32_t last_imu_us = micros();
  static uint32_t last_servo_ms = 0, last_print_ms = 0;
  uint32_t now_us = micros();

  // --- IMU + complementary filter, 200 Hz ---
  if (now_us - last_imu_us >= IMU_PERIOD_US) {
    float dt = (now_us - last_imu_us) * 1e-6f;
    last_imu_us = now_us;
    float a[3], g[3];
    if (imuRead(a, g)) {
      float rate = gyro_sign * (g[rot_axis] - gyro_bias);   // deg/s
      float acc  = accelAngle(a) - zero_deg;                // deg
      float k = COMP_TAU / (COMP_TAU + dt);
      angle_deg = k * (angle_deg + rate * dt) + (1 - k) * acc;
    }
  }

  // --- Servo, 50 Hz ---
  uint32_t now_ms = millis();
  if (now_ms - last_servo_ms >= SERVO_PERIOD_MS) {
    last_servo_ms = now_ms;
    servo_deg = following ? servo_dir * FOLLOW_GAIN * angle_deg : 0;
    int counts = center + (int)lroundf(servo_deg * COUNTS_PER_DEG);
    moveServo(counts, SERVO_PERIOD_MS);   // reach it by the next update
  }

  // --- Print, 10 Hz ---
  if (now_ms - last_print_ms >= PRINT_PERIOD_MS) {
    last_print_ms = now_ms;
    Serial.print(angle_deg, 2); Serial.print(',');
    Serial.print((counts_sent - center) / COUNTS_PER_DEG, 2); Serial.print(',');
    Serial.println(counts_sent);
  }

  // --- Commands ---
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'z' || c == 'Z') { calibrate(); }
    else if (c == 'h' || c == 'H') { following = false; Serial.println("# hold at center"); }
    else if (c == 'g' || c == 'G') { following = true;  Serial.println("# following"); }
    else if (c == 's' || c == 'S') { servo_dir = -servo_dir; Serial.println("# direction swapped"); }
    else if (c == '+' || c == '=' || c == '-' || c == ']' || c == '[') {
      int step = (c == ']') ? 10 : (c == '[') ? -10 : (c == '-') ? -1 : 1;
      center = constrain(center + step, SERVO_RANGE, 1000 - SERVO_RANGE);
      Serial.print("# center = "); Serial.print(center);
      Serial.print("  (trim "); Serial.print(center - 500);
      Serial.println(" counts from servo 500)");
    }
  }

  // Throw away anything the BusLinker echoes back (move commands get no reply).
  while (Serial2.available()) Serial2.read();
}
