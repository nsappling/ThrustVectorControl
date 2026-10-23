// usb_hello.ino -- bring-up STAGE 1: is the Teensy alive, and can the Pi
// flash it and talk to it?
// Board: Teensy 4.0 (USB Type: Serial). Wiring: USB cable to the Pi ONLY.
// Procedure: docs/bringup/README.md, stage 1.
//
// What it does:
//   - Blinks the onboard LED (pin 13) once per second.
//   - Prints "# alive" with the uptime once per second.
//   - Answers "ping" with "# pong", and echoes anything else it receives.
//
// PASS = the LED blinks, the Pi sees /dev/ttyACM0, and "ping" gets "# pong".
// That proves the whole Pi -> compile -> flash -> USB serial chain works
// before any other hardware is involved.
//
// Self-contained on purpose (no shared files).

const int PIN_LED = 13;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200);   // native USB: the number is ignored on Teensy
  while (!Serial && millis() < 3000) {}
  Serial.println(F("# usb_hello: Teensy 4.0 is alive. Type 'ping'."));
}

void loop() {
  // --- LED: on for the first half of every second ---
  digitalWrite(PIN_LED, (millis() % 1000) < 500);

  // --- Heartbeat line once per second ---
  static uint32_t last = 0;
  if (millis() - last >= 1000) {
    last += 1000;
    Serial.print(F("# alive, uptime "));
    Serial.print(millis() / 1000);
    Serial.println(F(" s"));
  }

  // --- Commands: read a line, answer it ---
  static char buf[48];
  static uint8_t n = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (n == 0) continue;
      buf[n] = 0;
      n = 0;
      if (!strcmp(buf, "ping")) Serial.println(F("# pong"));
      else if (!strcmp(buf, "hb")) {}   // heartbeat from the logger: ignore
      else { Serial.print(F("# echo: ")); Serial.println(buf); }
    } else if (n < sizeof(buf) - 1) {
      buf[n++] = c;
    }
  }
}
