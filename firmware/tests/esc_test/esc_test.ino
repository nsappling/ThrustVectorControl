// esc_test.ino -- ESC throttle calibration and EDF current steps.
// Board: Teensy 4.0, ESC signal on pin 2, ESC signal GND to Teensy GND,
// ESC red (BEC) wire NOT connected. Wiring: docs/icd/edf_esc.md.
//
// !!! EDF restrained, guard on, nobody in line with the intake or exhaust,
// !!! inline watt meter on the XT60, hand on the AC switch / E-stop.
//
// Serial Monitor (line ending "Newline"):
//   cal       start throttle-range calibration (sends MAX, follow prompts)
//   t <0-1>   set throttle (hard-capped at TEST_CAP)
//   s         stop (throttle 0) -- also sent automatically on any typo
//
// Self-contained on purpose (no shared files).

#include <Servo.h>

const int PIN_ESC = 2;
const int MIN_US = 1000, MAX_US = 2000;   // [VERIFY] against the Skywalker manual
const float TEST_CAP = 0.30;              // never exceed while the fuse is 15 A
Servo esc;

void set_us(int us) {
  esc.writeMicroseconds(us);
  Serial.print("# pulse = "); Serial.print(us); Serial.println(" us");
}

String read_line() {
  String s;
  while (true) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') { if (s.length()) return s; }
      else s += c;
    }
  }
}

void setup() {
  esc.attach(PIN_ESC, MIN_US, MAX_US);
  esc.writeMicroseconds(MIN_US);           // zero throttle from power-up
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  Serial.println("# esc_test ready: cal | t <0-1> | s");
}

void loop() {
  String cmd = read_line();
  cmd.trim();
  if (cmd == "cal") {
    Serial.println("# CALIBRATION: unplug the ESC's XT60 now, then type 'go'.");
    if (read_line() != "go") { set_us(MIN_US); return; }
    set_us(MAX_US);
    Serial.println("# Sending MAX. Plug in the XT60. When the ESC beeps for max, type 'next'.");
    read_line();
    set_us(MIN_US);
    Serial.println("# Sent MIN. Wait for the confirmation beeps. Calibration done.");
  } else if (cmd.startsWith("t ")) {
    float t = constrain(cmd.substring(2).toFloat(), 0.0f, TEST_CAP);
    Serial.print("# throttle "); Serial.println(t, 3);
    set_us(MIN_US + (int)(t * (MAX_US - MIN_US)));
  } else {
    set_us(MIN_US);   // "s" or anything unrecognized = stop
  }
}
