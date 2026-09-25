// hardware_test.ino
// Simple wiring check for the TVC hardware (SparkFun RedBoard):
//   - sweeps the servo slowly back and forth around center
//   - prints IMU readings and the servo position to the Serial Monitor
// No control/PID here -- that's tvc_controller.ino. Use this first to
// confirm the servo moves and the IMU reads sensibly.
//
// Wiring:
//   Servo signal -> D4, servo power -> separate 6V supply, grounds tied together
//   GY-521: VCC -> 5V, GND -> GND, SDA -> SDA, SCL -> SCL
//
// Serial Monitor: 115200 baud. Output is CSV (lines starting with # are
// status messages), so it can be saved with log_serial.m and opened in
// MATLAB/Excel.

#include <Wire.h>
#include <Servo.h>

const int SERVO_PIN = 4;
const uint8_t MPU_ADDR = 0x68;   // GY-521 default address

const int CENTER_DEG = 90;       // servo center (180 degree servo)
const int SWEEP_DEG = 35;        // sweep +/- this much around center -- same
                                 // as the controller's fan tilt limit, so the
                                 // gimbal never goes further than it will in use
const int STEP_DELAY_MS = 30;    // time per 1 degree step (bigger = slower sweep)

const bool SWEEP = true;
// Set to false to HOLD the servo still at center. If it still twitches
// while holding, the problem is power/ground/signal wiring -- not the
// IMU or the control code.

Servo servo;
int servo_deg = CENTER_DEG;
int direction = 1;
unsigned long imu_errors = 0;
bool imu_ok = false;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setWireTimeout(3000, true);   // don't freeze forever if the I2C bus glitches
  Serial.println(F("# BOOT (if you see this more than once, the board is resetting -- check servo power)"));

  servo.attach(SERVO_PIN, 500, 2500);   // 500-2500 us = 0-180 degrees
  servo.write(CENTER_DEG);

  // Wake up the MPU-6050 (it starts in sleep mode)
  Wire.beginTransmission(MPU_ADDR);
  imu_ok = (Wire.endTransmission() == 0);
  if (imu_ok) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);   // PWR_MGMT_1
    Wire.write(0x00);   // wake up
    Wire.endTransmission();
    Serial.println(F("# IMU found."));
  } else {
    Serial.println(F("# IMU NOT FOUND -- check SDA/SCL wiring and power. Servo will still sweep."));
  }

  delay(1000);   // sit at center for a second
  Serial.println(F("t_ms,servo_deg,angle_deg,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps"));
}

void loop() {
  // --- Move the servo one degree (or hold at center if SWEEP is false) ---
  if (SWEEP) {
    servo_deg += direction;
    if (servo_deg >= CENTER_DEG + SWEEP_DEG || servo_deg <= CENTER_DEG - SWEEP_DEG) {
      direction = -direction;   // turn around at the ends
    }
  }
  servo.write(servo_deg);

  // --- Read the IMU (default ranges: +/-2 g, +/-250 deg/s) ---
  float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
  if (imu_ok) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);   // start at ACCEL_XOUT_H
    bool ok = (Wire.endTransmission(false) == 0) &&
              (Wire.requestFrom(MPU_ADDR, (uint8_t)14) == 14);
    if (ok) {
      int16_t raw[7];
      for (int i = 0; i < 7; i++) raw[i] = (Wire.read() << 8) | Wire.read();
      ax = raw[0] / 16384.0;  ay = raw[1] / 16384.0;  az = raw[2] / 16384.0;   // g
      gx = raw[4] / 131.0;    gy = raw[5] / 131.0;    gz = raw[6] / 131.0;     // deg/s
    } else {
      imu_errors++;
      Serial.print(F("# IMU read failed, "));
      Serial.print(imu_errors);
      Serial.println(F(" total -- check SDA/SCL wires, grounds, servo power"));
    }
  }

  // Tilt angle from the accelerometer, about the IMU's X axis -- the same
  // axis tvc_controller.ino uses by default. Should read ~0 when the chip
  // is flat and face-up, and change as you tilt it.
  float angle_deg = degrees(atan2(ay, az));

  // --- Print (every 3rd step, about 10 rows per second) ---
  static int count = 0;
  if (++count % 3 == 0) {
    Serial.print(millis());   Serial.print(',');
    Serial.print(servo_deg);  Serial.print(',');
    Serial.print(angle_deg, 1); Serial.print(',');
    Serial.print(ax, 2); Serial.print(',');
    Serial.print(ay, 2); Serial.print(',');
    Serial.print(az, 2); Serial.print(',');
    Serial.print(gx, 1); Serial.print(',');
    Serial.print(gy, 1); Serial.print(',');
    Serial.println(gz, 1);
  }

  delay(STEP_DELAY_MS);
}
