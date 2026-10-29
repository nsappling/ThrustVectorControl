// flight_computer.ino -- Teensy 4.1 flight computer for the 1-DOF TVC demonstrator.
//
//   GY-521 IMU --I2C--> Teensy --UART--> BusLinker --bus--> HTD-30H gimbal servo
//                          |  \--PWM--> ESC --> EDF
//                          USB
//                     Raspberry Pi 5 (logging, tuning, later the joystick)
//
// Wiring: docs/pinout.md and docs/icd/. Settings: config.h.
// Serial commands and the CSV format: docs/icd/usb_link.md (or type "help").
//
// STATES
//   SAFE  (boot)  EDF off, servo centered and holding.
//   ARMED         Closed-loop gimbal control, EDF off. Use this for bench
//                 tests: tilt the IMU by hand and watch the gimbal respond.
//   RUN           Closed-loop control AND the EDF spins at the set throttle.
//   FAULT         EDF off, servo centered. Latched until "stop".
// The EDF can only spin in RUN, and RUN can only be entered from ARMED.

#include <Wire.h>
#include "config.h"
#include "imu.h"
#include "servo_bus.h"
#include "esc.h"

enum State : uint8_t { SAFE = 0, ARMED = 1, RUN = 2, FAULT = 3 };
const char *STATE_NAMES[] = { "SAFE", "ARMED", "RUN", "FAULT" };

Imu imu;
ServoBus servo;
Esc esc;

// --- controller state ---
State state = SAFE;
float Kp = KP_DEFAULT, Ki = KI_DEFAULT, Kd = KD_DEFAULT;
float ref_rad = 0;              // angle setpoint (later from the joystick)
float throttle_set = 0;         // EDF throttle used in RUN (0..1, capped in Esc)
float integral = 0;
float rate_filt = 0;            // low-passed gyro rate for the D term
float gimbal_cmd_deg = 0;       // commanded fan tilt
int   servo_trim = 0;           // counts added to SERVO_CENTER

// --- feedback from the servo (refreshed at SERVO_READ_HZ) ---
float gimbal_meas_deg = NAN;
float servo_v = NAN;
uint8_t servo_misses = 0;

// --- timing ---
const float DT = 1.0f / LOOP_HZ;
uint32_t next_tick_us;
uint32_t tick = 0;
uint32_t last_host_ms = 0;

// =====================================================================
// HELPERS
// =====================================================================

void enter(State s, const char *why) {
  state = s;
  if (s != RUN) esc.stop();
  if (s == SAFE || s == FAULT) gimbal_cmd_deg = 0;
  integral = 0;
  Serial.print(F("# STATE "));
  Serial.print(STATE_NAMES[s]);
  Serial.print(F(": "));
  Serial.println(why);
}

uint16_t tilt_to_counts(float fan_deg) {
  float c = SERVO_CENTER + servo_trim + SERVO_DIR * GIMBAL_RATIO * fan_deg * SERVO_COUNTS_PER_DEG;
  return (uint16_t)constrain(lroundf(c), 0, 1000);
}

float counts_to_tilt(int16_t counts) {
  return (counts - SERVO_CENTER - servo_trim) / (SERVO_DIR * GIMBAL_RATIO * SERVO_COUNTS_PER_DEG);
}

void update_led() {
  // SAFE slow blink, ARMED fast blink, RUN solid, FAULT very fast
  static const uint16_t period_ms[] = { 1000, 250, 0, 80 };
  uint16_t p = period_ms[state];
  digitalWrite(PIN_LED, p == 0 ? HIGH : ((millis() / p) & 1));
}

// =====================================================================
// SETUP
// =====================================================================

void setup() {
  pinMode(PIN_LED, OUTPUT);
  esc.begin(PIN_ESC);   // FIRST: hold the ESC at zero throttle from power-up
  Serial.begin(115200); // USB: the baud number is ignored on Teensy
  servo.begin(SERVO_BUS_SERIAL, SERVO_BUS_BAUD);

  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 2000) {}   // give the Pi/monitor a moment to connect

  Serial.println(F("# flight_computer boot"));
  if (!imu.begin()) {
    Serial.println(F("# ERROR: MPU-6050 not found at 0x68. Check SDA=18, SCL=19, VCC=3.3V."));
    state = FAULT;
  } else {
    Serial.println(F("# Hold the rocket UPRIGHT and STILL -- calibrating..."));
    delay(500);
    imu.calibrate();
    Serial.println(F("# Calibrated."));
  }

  servo.setTorque(SERVO_ID, true);
  servo.move(SERVO_ID, tilt_to_counts(0), 500);   // ease to center

  Serial.println(F("t_ms,state,theta_deg,rate_dps,gimbal_cmd_deg,gimbal_meas_deg,throttle,servo_v"));
  Serial.println(F("# Ready in SAFE. Type 'help'."));
  next_tick_us = micros();
  last_host_ms = millis();
}

// =====================================================================
// MAIN LOOP -- fixed rate at LOOP_HZ
// =====================================================================

void loop() {
  handle_serial();
  update_led();

  if ((int32_t)(micros() - next_tick_us) < 0) return;
  next_tick_us += 1000000UL / LOOP_HZ;
  tick++;

  // --- 1. Sense ---
  if (!imu.update(DT)) {
    if (state != FAULT) enter(FAULT, "IMU read failed (check I2C wiring/grounds)");
  }
  float theta = imu.theta();
  const float b = DT / (DT + 1.0f / (2 * PI * D_LPF_HZ));
  rate_filt += b * (imu.rate() - rate_filt);

  // --- 2. Safety checks ---
  if ((state == ARMED || state == RUN) && fabsf(theta) > radians(THETA_TRIP_DEG)) {
    enter(FAULT, "angle past THETA_TRIP_DEG");
  }
  if (state == RUN && HOST_TIMEOUT_MS > 0 && millis() - last_host_ms > HOST_TIMEOUT_MS) {
    enter(SAFE, "host heartbeat lost");
  }

  // --- 3. Control (same law as sim/tvc_sim/tvc_simulate.m) ---
  if (state == ARMED || state == RUN) {
    float e = ref_rad - theta;
    const float dmax = radians(GIMBAL_MAX_DEG);
    if (Ki > 0) integral = constrain(integral + e * DT, -dmax / Ki, dmax / Ki);   // anti-windup
    float delta = Kp * e - Kd * rate_filt + Ki * integral;
    gimbal_cmd_deg = constrain(degrees(delta), -GIMBAL_MAX_DEG, GIMBAL_MAX_DEG);
  }
  esc.set(state == RUN ? throttle_set : 0);

  // --- 4. Actuate the servo at SERVO_CMD_HZ ---
  if (tick % (LOOP_HZ / SERVO_CMD_HZ) == 0) {
    servo.move(SERVO_ID, tilt_to_counts(gimbal_cmd_deg), 0);
  }

  // --- 5. Servo feedback at SERVO_READ_HZ (alternate position / voltage) ---
  if (tick % (LOOP_HZ / SERVO_READ_HZ) == 0) {
    bool ok;
    if ((tick / (LOOP_HZ / SERVO_READ_HZ)) % 2 == 0) {
      int16_t pos;
      ok = servo.readPos(SERVO_ID, pos);
      if (ok) gimbal_meas_deg = counts_to_tilt(pos);
    } else {
      uint16_t mv;
      ok = servo.readVin(SERVO_ID, mv);
      if (ok) {
        servo_v = mv / 1000.0f;
        if ((state == ARMED || state == RUN) && (servo_v < SERVO_VMIN || servo_v > SERVO_VMAX)) {
          enter(FAULT, "servo voltage out of range");
        }
      }
    }
    servo_misses = ok ? 0 : min(servo_misses + 1, 255);
    if (servo_misses == SERVO_MAX_MISSES && (state == ARMED || state == RUN)) {
      enter(FAULT, "servo not responding (BusLinker wiring/jumpers/power?)");
    }
  }

  // --- 6. Telemetry at TELEM_HZ ---
  if (tick % (LOOP_HZ / TELEM_HZ) == 0) {
    Serial.print(millis());                 Serial.print(',');
    Serial.print(state);                    Serial.print(',');
    Serial.print(degrees(theta), 2);        Serial.print(',');
    Serial.print(degrees(rate_filt), 1);    Serial.print(',');
    Serial.print(gimbal_cmd_deg, 2);        Serial.print(',');
    Serial.print(gimbal_meas_deg, 2);       Serial.print(',');
    Serial.print(esc.throttle(), 3);        Serial.print(',');
    Serial.println(servo_v, 2);
  }
}

// =====================================================================
// SERIAL COMMANDS (from the Pi or the Serial Monitor)
// =====================================================================

void handle_serial() {
  static char buf[48];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (n > 0) { buf[n] = 0; last_host_ms = millis(); run_command(buf); n = 0; }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
}

void print_status() {
  Serial.print(F("# state=")); Serial.print(STATE_NAMES[state]);
  Serial.print(F(" Kp=")); Serial.print(Kp, 3);
  Serial.print(F(" Ki=")); Serial.print(Ki, 3);
  Serial.print(F(" Kd=")); Serial.print(Kd, 3);
  Serial.print(F(" ref=")); Serial.print(degrees(ref_rad), 1);
  Serial.print(F(" throttle=")); Serial.print(throttle_set, 3);
  Serial.print(F(" (cap ")); Serial.print(THROTTLE_MAX, 2);
  Serial.print(F(") trim=")); Serial.println(servo_trim);
}

void run_command(char *cmd) {
  char *arg = strchr(cmd, ' ');
  float val = 0;
  if (arg) { *arg = 0; val = atof(arg + 1); }

  if (!strcmp(cmd, "hb")) return;   // heartbeat: just refreshes last_host_ms
  else if (!strcmp(cmd, "stop"))  enter(SAFE, "stop command");
  else if (!strcmp(cmd, "arm")) {
    if (state == SAFE) enter(ARMED, "gimbal active, EDF off");
    else Serial.println(F("# arm: only from SAFE (send 'stop' first)"));
  }
  else if (!strcmp(cmd, "run")) {
    if (state == ARMED) enter(RUN, "EDF enabled");
    else Serial.println(F("# run: only from ARMED"));
  }
  else if (!strcmp(cmd, "throttle") && arg) throttle_set = constrain(val, 0.0f, THROTTLE_MAX);
  else if (!strcmp(cmd, "ref") && arg)  ref_rad = radians(constrain(val, -10.0f, 10.0f));
  else if (!strcmp(cmd, "kp") && arg)   Kp = val;
  else if (!strcmp(cmd, "ki") && arg)   { Ki = val; integral = 0; }
  else if (!strcmp(cmd, "kd") && arg)   Kd = val;
  else if (!strcmp(cmd, "trim") && arg) servo_trim = constrain((int)val, -100, 100);
  else if (!strcmp(cmd, "zero")) {
    if (state == RUN) { Serial.println(F("# zero: not while RUN")); return; }
    Serial.println(F("# Hold upright and still..."));
    imu.calibrate();
    Serial.println(F("# Re-zeroed."));
  }
  else if (!strcmp(cmd, "status")) { /* printed below */ }
  else if (!strcmp(cmd, "help")) {
    Serial.println(F("# stop | arm | run | throttle <0-1> | ref <deg> | kp/ki/kd <val>"));
    Serial.println(F("# trim <counts> | zero | status | hb"));
    return;
  }
  else { Serial.println(F("# Unknown command. Type 'help'.")); return; }
  print_status();
}
