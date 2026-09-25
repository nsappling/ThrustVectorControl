// tvc_controller.ino
// Arduino (SparkFun RedBoard / Uno-compatible) port of the AUTO-mode controller in rocketgimball.m /
// tvc_sim.m. Reads body angle from a GY-521 (MPU-6050) IMU, runs the
// same low-pass filter + PID, and drives the gimbal servo.
//
// BENCH TEST MODE: no fan needed. Tilt the IMU by hand to "fake" the
// rocket body angle and watch the servo respond. When the IMU is
// finally mounted on the body and the fan is running, this same code
// closes the real loop.
//
// Wiring (see README_hardware.md in this folder for details):
//   GY-521 VCC -> 5V           GY-521 GND -> GND
//   GY-521 SDA -> SDA pin      GY-521 SCL -> SCL pin  (same as A4/A5)
//   Servo signal (orange/yellow) -> D4
//   Servo power (red/brown) -> SEPARATE 6V supply, NOT the board's 5V pin.
//   Tie the supply ground to board GND.
//
// Serial (115200 baud, USB): streams CSV for logging, and accepts
// commands to retune without re-flashing:
//   kp 0.5    ki 0.1    kd 0.035    tau 0.02    zero    center    run    help
//
// No extra libraries needed: Wire and Servo ship with the Arduino IDE.

#include <Wire.h>
#include <Servo.h>

// =====================================================================
// CONTROL PARAMETERS -- same names/meaning as rocketgimball.m
// =====================================================================

float Kp = 0.42, Ki = 0.1, Kd = 0.015;
// PID gains, identical to rocketgimball.m. tune_pid.m suggests
// Kp = 0.5, Kd = 0.035 is more robust to the (still guessed) physical
// parameters -- try it with the "kp"/"kd" serial commands.

float tau_filter = 0.02;       // [s] low-pass filter time constant (same as MATLAB)
const float VIS_SCALE = 200.0; // u -> fan tilt [deg]; also sets Umax (same as MATLAB)
const float fan_tilt_max_deg = 35.0;
const float Umax_actuator = fan_tilt_max_deg / VIS_SCALE;
float ref = 0.0;               // [rad] target angle: upright

const float dt = 0.002;
// [s] control loop period (500 Hz). MATLAB used 0.0005 s, but that was
// a PHYSICS step size -- the PID math uses dt explicitly (integral*dt,
// derivative/dt, alpha from dt) so the gains carry over unchanged.
// 500 Hz leaves plenty of headroom for the I2C read on a 16 MHz ATmega328P.

const float alpha = dt / (tau_filter + dt);
// Discrete low-pass coefficient, same formula as MATLAB. (Recomputed
// in the loop if tau is changed over serial -- see alpha_now.)

const float theta_trip_deg = 45.0;
const float theta_resume_deg = 35.0;
// Safety: if the measured angle is past theta_trip_deg, center the servo
// and stop controlling (the real body should be resting on its stops by
// then). Control resumes AUTOMATICALLY once the angle comes back inside
// theta_resume_deg -- so tilting the IMU too far by hand during bench
// testing no longer leaves the servo stuck until you type 'run'.

// =====================================================================
// HARDWARE CONFIGURATION -- check these against your build
// =====================================================================

const int SERVO_PIN = 4;   // any digital pin works with the Servo library

const float SERVO_RANGE_DEG = 180.0;
// Our MIUZEI 20 kg servo is the 180 degree version (270 degree versions
// also exist). Sets how many microseconds equal one degree.

const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;
const int SERVO_CENTER_US = 1500;
int servo_trim_us = 0;
// Nudge this if the gimbal isn't straight when the servo is centered
// (use the "center" command, then adjust). Better yet, re-seat the horn.

const float SERVO_DIR = 1.0;
// +1 or -1. Flip if the fan swings the WRONG way in the direction check
// (README_hardware.md). This is the hardware equivalent of MATLAB's
// "b must be positive" -- wrong sign = positive feedback = falls over.

const float GIMBAL_RATIO = 1.0;
// Servo degrees per degree of fan tilt. 1.0 if the horn drives the
// gimbal directly (current design); change if a linkage/gear is added.

// --- IMU axis selection ---
// Mount the GY-521 with its Z axis along the rocket's long axis (chip
// face pointing up when the rocket is upright). Then tilting in the
// gimbal plane is rotation about the IMU's X axis:
//   angle from accel = atan2(ay, az),  rate from gyro = gx
// If your gimbal plane lines up with the IMU's Y axis instead, set
// IMU_USE_Y_AXIS to true (uses atan2(-ax, az) and gy).
const bool IMU_USE_Y_AXIS = false;
const float IMU_SIGN = 1.0;
// +1 or -1: make "nose tips RIGHT" read as POSITIVE theta, to match
// MATLAB's clockwise-positive convention. Check with the serial stream.

const float COMP_TAU = 0.5;
// [s] complementary-filter time constant for fusing gyro + accel into
// an angle. MATLAB didn't need this (it measured theta directly plus
// noise); on real hardware the accelerometer alone is noisy/jerky and
// the gyro alone drifts, so we blend them. Shorter = trusts accel more.

// =====================================================================
// STATE
// =====================================================================

Servo servo;
const uint8_t MPU_ADDR = 0x68;   // GY-521 default (AD0 pin low)

float gyro_bias = 0.0;     // [rad/s] measured at startup
float angle_offset = 0.0;  // [rad] angle reading when held upright at startup
float theta_est = 0.0;     // [rad] complementary-filter angle (the "measured_theta")
float theta_filt = 0.0;    // [rad] after the MATLAB low-pass filter
float integral_term = 0.0;
float prev_error = 0.0;
bool have_prev_error = false;   // MATLAB's prev_error = [] trick
float alpha_now = alpha;
bool running = true;       // false = servo held centered (tripped or "center" command)
bool tripped = false;      // true = paused by the angle limit (auto-resumes); false + !running = "center" command
float u = 0.0;
int servo_us = SERVO_CENTER_US;

unsigned long next_tick_us;
unsigned long tick = 0;
unsigned long imu_errors = 0;   // count of failed IMU reads (reported over serial)

// =====================================================================
// SETUP
// =====================================================================

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);   // fast I2C so the 14-byte read fits easily in 2 ms
  Wire.setWireTimeout(3000, true);
  // Without a timeout, one electrical glitch on SDA/SCL (servo current
  // spikes, a loose jumper) can freeze the Wire library forever -- the
  // servo then just holds its last position. 3 ms timeout + bus reset.

  servo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  servo.writeMicroseconds(SERVO_CENTER_US + servo_trim_us);

  if (!mpu_init()) {
    Serial.println(F("# ERROR: MPU-6050 not found. Check SDA/SCL wiring and power."));
    while (true) {}   // stay here with servo centered
  }

  Serial.println(F("# BOOT (if you see this more than once, the board is resetting -- check servo power)"));
  Serial.println(F("# Hold the IMU/rocket UPRIGHT and STILL -- calibrating..."));
  delay(500);
  calibrate();
  Serial.println(F("# Calibrated."));
  Serial.println(F("t_ms,theta_deg,theta_filt_deg,u,fan_tilt_deg,servo_us"));   // CSV header (log_serial.m uses it)
  Serial.println(F("# Type 'help' for commands."));

  theta_filt = theta_est;
  next_tick_us = micros();
}

// =====================================================================
// MAIN LOOP -- fixed-rate control at 1/dt
// =====================================================================

void loop() {
  handle_serial();

  // Wait for the next tick so dt is really dt (the PID math assumes it).
  if ((long)(micros() - next_tick_us) < 0) return;
  next_tick_us += (unsigned long)(dt * 1e6);

  float acc_angle, gyro_rate;
  if (!read_imu(acc_angle, gyro_rate)) {
    center_servo();   // I2C glitch: fail safe
    // Report glitches (at most once per second) so they show up in logs.
    static unsigned long last_report = 0;
    imu_errors++;
    if (millis() - last_report > 1000) {
      Serial.print(F("# IMU read failed, "));
      Serial.print(imu_errors);
      Serial.println(F(" total -- check SDA/SCL wires, grounds, servo power"));
      last_report = millis();
    }
    return;
  }

  // --- Angle estimate: complementary filter (hardware stand-in for ---
  // --- MATLAB's "measured_theta = theta + noise")                  ---
  const float k = COMP_TAU / (COMP_TAU + dt);
  theta_est = k * (theta_est + gyro_rate * dt) + (1.0 - k) * acc_angle;

  // --- Low-pass filter (same line as MATLAB) ---
  theta_filt = theta_filt + alpha_now * (theta_est - theta_filt);

  // --- Angle limit trip, with automatic resume ---
  if (running && fabs(theta_filt) > radians(theta_trip_deg)) {
    Serial.println(F("# TRIP: angle past limit, servo centered. Resumes when back inside 35 deg."));
    center_servo();
    running = false;
    tripped = true;
  } else if (tripped && fabs(theta_filt) < radians(theta_resume_deg)) {
    Serial.println(F("# Back in range, control resumed."));
    running = true;
    tripped = false;
    integral_term = 0;         // start fresh, don't carry old error history
    have_prev_error = false;
  }

  if (running) {
    // --- PID controller (same as MATLAB) ---
    float error = ref - theta_filt;
    if (!have_prev_error) {
      prev_error = error;      // avoid a derivative-kick on the very first sample
      have_prev_error = true;
    }
    float derivative = (error - prev_error) / dt;
    prev_error = error;

    // Anti-windup (NOT in the MATLAB sim): only let the integral grow
    // while the actuator is NOT already pinned at its limit in the same
    // direction. Otherwise, holding the IMU tilted on the bench (where
    // the servo can't move it back) fills the integral up, and the servo
    // stays stuck at full tilt for seconds after you tilt back.
    float u_try = Kp * error + Ki * (integral_term + error * dt) + Kd * derivative;
    bool pinned = (u_try > Umax_actuator && error > 0) || (u_try < -Umax_actuator && error < 0);
    if (!pinned) integral_term += error * dt;
    if (Ki > 0) integral_term = constrain(integral_term, -Umax_actuator / Ki, Umax_actuator / Ki);

    u = Kp * error + Ki * integral_term + Kd * derivative;

    // --- Actuator saturation (same as MATLAB) ---
    u = constrain(u, -Umax_actuator, Umax_actuator);

    // --- u -> fan tilt, same sign convention as rocketgimball.m's drawing ---
    float fan_tilt_deg = constrain(-u * VIS_SCALE, -fan_tilt_max_deg, fan_tilt_max_deg);
    write_servo(fan_tilt_deg);
  }

  // --- Log at 50 Hz (every 10th tick) so the serial link keeps up ---
  if (++tick % 10 == 0) {
    Serial.print(millis());                     Serial.print(',');
    Serial.print(degrees(theta_est), 2);        Serial.print(',');
    Serial.print(degrees(theta_filt), 2);       Serial.print(',');
    Serial.print(u, 4);                         Serial.print(',');
    Serial.print(-u * VIS_SCALE, 1);            Serial.print(',');
    Serial.println(servo_us);
  }
}

// =====================================================================
// SERVO
// =====================================================================

void write_servo(float fan_tilt_deg) {
  const float us_per_deg = (SERVO_MAX_US - SERVO_MIN_US) / SERVO_RANGE_DEG;
  float servo_deg = SERVO_DIR * GIMBAL_RATIO * fan_tilt_deg;
  servo_us = SERVO_CENTER_US + servo_trim_us + (int)(servo_deg * us_per_deg);
  servo_us = constrain(servo_us, SERVO_MIN_US, SERVO_MAX_US);
  servo.writeMicroseconds(servo_us);
}

void center_servo() {
  u = 0.0;
  servo_us = SERVO_CENTER_US + servo_trim_us;
  servo.writeMicroseconds(servo_us);
}

// =====================================================================
// IMU (MPU-6050 over raw I2C -- no library needed)
// =====================================================================

void mpu_write(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

bool mpu_init() {
  Wire.beginTransmission(MPU_ADDR);
  if (Wire.endTransmission() != 0) return false;
  mpu_write(0x6B, 0x01);   // PWR_MGMT_1: wake up, use gyro X clock
  mpu_write(0x1A, 0x02);   // CONFIG: digital low-pass ~94 Hz (~3 ms delay)
  mpu_write(0x1B, 0x08);   // GYRO_CONFIG: +/-500 deg/s  (65.5 LSB per deg/s)
  mpu_write(0x1C, 0x08);   // ACCEL_CONFIG: +/-4 g      (8192 LSB per g)
  delay(100);
  return true;
}

// Returns the accelerometer tilt angle [rad] and gyro rate [rad/s] for
// the gimbal axis, with sign/offset/bias corrections applied.
bool read_imu(float &acc_angle, float &gyro_rate) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);   // ACCEL_XOUT_H: read accel(6) + temp(2) + gyro(6)
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)14) != 14) return false;

  int16_t raw[7];
  for (int i = 0; i < 7; i++) raw[i] = (Wire.read() << 8) | Wire.read();
  float ax = raw[0], ay = raw[1], az = raw[2];   // units cancel in atan2
  float gx = raw[4] / 65.5, gy = raw[5] / 65.5;  // deg/s

  float a_raw, g_raw;
  if (IMU_USE_Y_AXIS) { a_raw = atan2(-ax, az); g_raw = radians(gy); }
  else                { a_raw = atan2(ay, az);  g_raw = radians(gx); }

  // Wrap the difference into -180..+180 deg. atan2 jumps from +180 to
  // -180 at one orientation; without this, crossing it would look like a
  // sudden 360 degree tilt (-> servo slam / trip).
  float d = a_raw - angle_offset;
  if (d > PI) d -= 2 * PI;
  if (d < -PI) d += 2 * PI;
  acc_angle = IMU_SIGN * d;
  gyro_rate = IMU_SIGN * (g_raw - gyro_bias);
  return true;
}

// Average readings while held still and upright: sets the gyro bias and
// defines "upright" as the current orientation (so the IMU doesn't need
// to be mounted perfectly straight).
void calibrate() {
  angle_offset = 0; gyro_bias = 0;
  const int N = 500;
  float a_sum = 0, g_sum = 0, a, g;
  int got = 0;
  for (int i = 0; i < N; i++) {
    if (read_imu(a, g)) { a_sum += a; g_sum += g; got++; }
    delay(2);
  }
  if (got == 0) return;
  // read_imu applied IMU_SIGN; undo it so the offsets are in raw units
  angle_offset = IMU_SIGN * a_sum / got;
  gyro_bias = IMU_SIGN * g_sum / got;
  theta_est = 0;
  integral_term = 0;
  have_prev_error = false;
}

// =====================================================================
// SERIAL COMMANDS (from Serial Monitor or the Pi)
// =====================================================================

void handle_serial() {
  static char buf[32];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (n > 0) { buf[n] = 0; run_command(buf); n = 0; }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
}

void run_command(char *cmd) {
  char *arg = strchr(cmd, ' ');
  float val = 0;
  if (arg) { *arg = 0; val = atof(arg + 1); }

  if      (!strcmp(cmd, "kp") && arg)  Kp = val;
  else if (!strcmp(cmd, "ki") && arg)  { Ki = val; integral_term = 0; }
  else if (!strcmp(cmd, "kd") && arg)  Kd = val;
  else if (!strcmp(cmd, "tau") && arg) { tau_filter = val; alpha_now = dt / (tau_filter + dt); }
  else if (!strcmp(cmd, "trim") && arg) servo_trim_us = (int)val;
  else if (!strcmp(cmd, "zero"))   { center_servo(); calibrate(); theta_filt = 0; Serial.println(F("# Re-zeroed.")); }
  else if (!strcmp(cmd, "center")) { running = false; tripped = false; center_servo(); Serial.println(F("# Servo centered, control paused. 'run' to resume.")); }
  else if (!strcmp(cmd, "run"))    { running = true; tripped = false; integral_term = 0; have_prev_error = false; Serial.println(F("# Running.")); }
  else if (!strcmp(cmd, "help")) {
    Serial.println(F("# kp/ki/kd/tau <val>, trim <us>, zero (hold upright+still), center, run"));
  } else {
    Serial.println(F("# Unknown command. Type 'help'."));
    return;
  }
  Serial.print(F("# Kp=")); Serial.print(Kp, 4);
  Serial.print(F(" Ki=")); Serial.print(Ki, 4);
  Serial.print(F(" Kd=")); Serial.print(Kd, 4);
  Serial.print(F(" tau=")); Serial.print(tau_filter, 4);
  Serial.print(F(" trim=")); Serial.println(servo_trim_us);
}
