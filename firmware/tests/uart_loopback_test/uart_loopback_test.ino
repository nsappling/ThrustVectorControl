// uart_loopback_test.ino -- bring-up STAGE 3: Teensy Serial1 through the
// BSS138 level shifter, BEFORE the BusLinker or servo is connected.
// Board: Teensy 4.0 (USB Type: Serial).
// Wiring (docs/icd/servo_bus.md S1, S4-S7; diagram docs/bringup/img/stage3_shifter.svg):
//   Teensy pin 1 (TX1) -> shifter LV2      Teensy pin 0 (RX1) <- shifter LV1
//   Teensy 3.3V        -> shifter LV       Teensy VIN (5 V)   -> shifter HV
//   Teensy GND         -> shifter GND
//   LOOPBACK JUMPER: shifter HV2 -> shifter HV1 (stands in for the BusLinker)
// Procedure: docs/bringup/README.md, stage 3.
//
// Every second it sends a 10-byte test packet (a real servo "move to
// center" command) out of TX1 at 115200 baud, through LV2 -> HV2 ->
// jumper -> HV1 -> LV1, back into RX1, and checks every byte came back.
//
// PASS = "# loopback OK" every second, 0 errors.
// FAIL = "nothing came back" (open wire, swapped LV1/LV2, no LV/HV power)
//     or "bytes wrong" (bad ground, wrong baud, noise).
// With the jumper REMOVED you should get "nothing came back" -- that
// proves the test isn't fooling itself.
//
// Self-contained on purpose (no shared files).

const uint32_t BAUD = 115200;   // same as the servo bus
// Hiwonder "move servo 1 to 500 in 0 ms" -- see docs/icd/servo_bus.md
const uint8_t PKT[10] = { 0x55, 0x55, 0x01, 0x07, 0x01, 0xF4, 0x01, 0x00, 0x00, 0x01 };

uint32_t ok_count = 0, fail_count = 0;

void setup() {
  Serial.begin(115200);
  Serial1.begin(BAUD);
  while (!Serial && millis() < 3000) {}
  Serial.println(F("# uart_loopback_test: jumper HV2 -> HV1 on the level shifter"));
}

void loop() {
  while (Serial.available()) Serial.read();   // ignore any commands (e.g. heartbeat)

  static uint32_t last = 0;
  if (millis() - last < 1000) return;
  last = millis();

  while (Serial1.available()) Serial1.read();  // clear stale bytes
  Serial1.write(PKT, sizeof(PKT));
  Serial1.flush();                             // wait until it's all sent

  uint8_t got[16];
  uint8_t n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 20 && n < sizeof(got)) {   // 10 bytes take ~0.9 ms
    if (Serial1.available()) got[n++] = Serial1.read();
  }

  bool match = (n == sizeof(PKT)) && memcmp(got, PKT, sizeof(PKT)) == 0;
  if (match) {
    ok_count++;
    Serial.print(F("# loopback OK"));
  } else {
    fail_count++;
    if (n == 0) Serial.print(F("# FAIL: nothing came back"));
    else {
      Serial.print(F("# FAIL: bytes wrong, got"));
      for (uint8_t i = 0; i < n; i++) { Serial.print(' '); Serial.print(got[i], HEX); }
    }
  }
  Serial.print(F("   (ok ")); Serial.print(ok_count);
  Serial.print(F(", errors ")); Serial.print(fail_count); Serial.println(')');
}
