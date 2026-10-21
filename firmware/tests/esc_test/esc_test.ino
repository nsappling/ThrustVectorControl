// esc_test.ino -- spin the EDF through the Skywalker 50A V2 ESC from the
// Serial Monitor: arming, throttle-range calibration, and capped throttle steps.
//
// Board: Teensy 4.1 or Teensy 4.0 (Tools > Board). The sketch is the same for
// both: pin 2 is a PWM-capable pin on either board.
//
// Wiring (docs/icd/edf_esc.md):
//   ESC signal (white/yellow) -> Teensy pin 2
//   ESC signal GND (black/brown) -> Teensy GND     (REQUIRED reference)
//   ESC red (BEC +5 V)        -> NOT CONNECTED      (pull the pin or insulate it;
//                                                    the Teensy runs from USB)
//   ESC power                 -> XT60 on the 15 A fused EDF branch
//
// !!! EDF restrained, guard on, nobody in line with the intake or exhaust,
// !!! inline watt meter on the XT60, hand on the AC switch / E-stop.
//
// HOW AN RC ESC IS "TALKED TO"
//   There is no data protocol. The ESC reads one pulse every 20 ms (50 Hz) and
//   sets motor power from the pulse WIDTH:
//     1000 us = stop, 2000 us = full throttle   [VERIFY after calibration]
//   The Teensy Servo library generates exactly this pulse, so "throttle 0..1"
//   just maps to 1000..2000 us.
//   The ESC will NOT start the motor until it has seen a steady "stop" pulse
//   for a moment after power-up (arming). That is why setup() sends 1000 us
//   before anything else, and why the sketch refuses throttle for ARM_HOLD_MS.
//
// Serial Monitor: 115200 baud, line ending "Newline".
//   t <0-1>   set target throttle (hard-capped at TEST_CAP, ramped at RAMP_PER_S)
//   s         stop NOW (no ramp). Any unrecognized line also stops.
//   ?         print status
//   cal       throttle-range calibration (only while stopped; follow prompts)
//
// Automatic stops:
//   - Serial Monitor closed / USB unplugged
//   - MAX_RUN_MS with no new "t" command (re-send "t" to keep it running)
//   - signal loss: the ESC itself stops the motor if the pulses stop
//
// Self-contained on purpose (no shared files).

#include <Servo.h>

// ---------------------------------------------------------------------
// SETTINGS
// ---------------------------------------------------------------------
const int   PIN_ESC     = 2;
const int   MIN_US      = 1000;   // [VERIFY] stop pulse, against the Skywalker manual
const int   MAX_US      = 2000;   // [VERIFY] full-throttle pulse
const float TEST_CAP    = 0.30;   // [DES] never exceed while the fuse is 15 A.
                                  // The EDF can pull ~50 A at full throttle.
                                  // Raise in small steps while watching the
                                  // watt meter; log each step in the ICD.
const float RAMP_PER_S  = 0.20;   // [DES] max throttle change per second going UP.
                                  // Avoids current spikes that pop the fuse.
const uint32_t ARM_HOLD_MS = 3000;  // hold "stop" this long after boot so the ESC
                                    // arms (listen for the arming beeps) [VERIFY]
const uint32_t MAX_RUN_MS  = 30000; // auto-stop if no "t" command for this long
const uint32_t STATUS_MS   = 1000;  // status print period while spinning

// ---------------------------------------------------------------------
// STATE
// ---------------------------------------------------------------------
Servo esc;
float target = 0;            // throttle the user asked for (0..TEST_CAP)
float current = 0;           // throttle actually being sent (ramps toward target)
int   pulse_us = MIN_US;     // pulse width actually being sent
uint32_t last_cmd_ms = 0;    // time of the last "t" command
uint32_t last_ramp_ms = 0;
uint32_t last_status_ms = 0;

enum CalStep { CAL_OFF, CAL_WAIT_UNPLUGGED, CAL_WAIT_MAX_BEEPS };
CalStep cal = CAL_OFF;

// ---------------------------------------------------------------------
// LOW LEVEL
// ---------------------------------------------------------------------
void write_us(int us) {
  pulse_us = us;
  esc.writeMicroseconds(us);
}

int throttle_to_us(float t) {
  return MIN_US + (int)(t * (MAX_US - MIN_US) + 0.5f);
}

// Instant stop: no ramp, clears everything.
void stop_now(const char *why) {
  target = 0;
  current = 0;
  cal = CAL_OFF;
  write_us(MIN_US);
  Serial.print("# STOP ("); Serial.print(why); Serial.println(")");
}

void print_status() {
  Serial.print("# target ");   Serial.print(target, 3);
  Serial.print("  sent ");     Serial.print(current, 3);
  Serial.print("  pulse ");    Serial.print(pulse_us);
  Serial.print(" us  cap ");   Serial.println(TEST_CAP, 2);
}

// Non-blocking line reader: returns true once a full line is in buf.
bool read_line(char *buf, size_t n) {
  static size_t len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len == 0) continue;          // ignore empty lines / CRLF leftovers
      buf[len] = '\0';
      len = 0;
      return true;
    }
    if (len < n - 1) buf[len++] = c;
  }
  return false;
}

// ---------------------------------------------------------------------
// COMMANDS
// ---------------------------------------------------------------------
void handle_calibration(const char *cmd) {
  // Calibration teaches the ESC what pulse means "full" and what means "stop".
  // Skywalker sequence [VERIFY in manual]: power the ESC up while it sees MAX,
  // it beeps to say "got max", then drop to MIN and it beeps to confirm.
  //
  // !!! MAX = FULL THROTTLE (~50 A). The ESC must be UNPOWERED (XT60 unplugged)
  // !!! when MAX starts, or the fan goes straight to full power.
  if (cal == CAL_WAIT_UNPLUGGED) {
    if (strcmp(cmd, "go") != 0) { stop_now("calibration aborted"); return; }
    write_us(MAX_US);
    cal = CAL_WAIT_MAX_BEEPS;
    Serial.println("# Sending MAX. Plug in the XT60 now.");
    Serial.println("# When the ESC beeps for max, type 'next'. Anything else aborts.");
  } else if (cal == CAL_WAIT_MAX_BEEPS) {
    if (strcmp(cmd, "next") != 0) { stop_now("calibration aborted"); return; }
    write_us(MIN_US);
    cal = CAL_OFF;
    Serial.println("# Sent MIN. Wait for the confirmation beeps. Calibration done.");
  }
}

void handle_command(char *cmd) {
  if (cal != CAL_OFF) { handle_calibration(cmd); return; }

  if (cmd[0] == 't' && cmd[1] == ' ') {
    if (millis() < ARM_HOLD_MS) { Serial.println("# still arming, wait"); return; }
    float t = atof(cmd + 2);
    if (!(t > 0)) t = 0;                        // also catches NaN
    if (t > TEST_CAP) {
      Serial.print("# capped at "); Serial.println(TEST_CAP, 2);
      t = TEST_CAP;
    }
    target = t;
    last_cmd_ms = millis();
    print_status();
  } else if (strcmp(cmd, "?") == 0) {
    print_status();
  } else if (strcmp(cmd, "cal") == 0) {
    stop_now("entering calibration");
    cal = CAL_WAIT_UNPLUGGED;
    Serial.println("# CALIBRATION: unplug the ESC's XT60 now, then type 'go'.");
  } else {
    stop_now(strcmp(cmd, "s") == 0 ? "command" : "unrecognized command");
  }
}

// ---------------------------------------------------------------------
// SETUP / LOOP
// ---------------------------------------------------------------------
void setup() {
  // FIRST thing: hold the ESC at "stop" so it arms and never sees a random pulse.
  esc.attach(PIN_ESC, MIN_US, MAX_US);
  write_us(MIN_US);

  Serial.begin(115200);                 // USB serial; baud is ignored on Teensy
  while (!Serial && millis() < 3000) {}
  Serial.println("# esc_test ready: t <0-1> | s | ? | cal");
  Serial.print("# holding stop for "); Serial.print(ARM_HOLD_MS);
  Serial.println(" ms so the ESC can arm (listen for the beeps)");
}

void loop() {
  static char line[32];
  if (read_line(line, sizeof line)) handle_command(line);

  uint32_t now = millis();

  // Safety: Serial Monitor closed or USB unplugged -> stop.
  if (!Serial && (target > 0 || cal != CAL_OFF)) stop_now("USB serial lost");

  // Safety: no fresh "t" command for MAX_RUN_MS -> stop.
  if (target > 0 && now - last_cmd_ms > MAX_RUN_MS) stop_now("run time limit");

  // During calibration the pulse is set directly; skip the ramp.
  if (cal != CAL_OFF) return;

  // Ramp: rise at RAMP_PER_S, drop immediately.
  float dt = (now - last_ramp_ms) * 1e-3f;
  last_ramp_ms = now;
  if (target > current) current = min(target, current + RAMP_PER_S * dt);
  else                  current = target;
  int us = throttle_to_us(current);
  if (us != pulse_us) write_us(us);

  // Status once a second while spinning (read the watt meter alongside it).
  if (current > 0 && now - last_status_ms >= STATUS_MS) {
    last_status_ms = now;
    print_status();
  }
}
