// servo_bus_test.ino -- first power-up check of the HTD-30H through the BusLinker.
// Board: Teensy 4.0. Wiring: docs/icd/servo_bus.md (Serial1 pins 0/1 via
// the level shifter). BusLinker jumpers on "Servo + TTL". Only ONE servo
// on the bus (the ID read uses broadcast).
//
// Serial Monitor (any baud, line ending "Newline"), single-letter commands:
//   i  read servo ID          r  read position / voltage / temperature
//   c  center (500)           +  / -  step 10 deg either way
//   w  slow sweep +/-15 deg   l  limp (torque off)    t  torque on
//
// Self-contained on purpose (no shared files), so it still works if
// flight_computer is broken. The protocol is the same as
// flight_computer/servo_bus.cpp.

const uint32_t BAUD = 115200;
const float COUNTS_PER_DEG = 1000.0 / 240.0;
uint8_t id = 1;
int pos_cmd = 500;

void send(uint8_t sid, uint8_t cmd, const uint8_t *p, uint8_t n) {
  uint8_t len = n + 3, sum = sid + len + cmd;
  uint8_t hdr[5] = { 0x55, 0x55, sid, len, cmd };
  Serial1.write(hdr, 5);
  for (uint8_t i = 0; i < n; i++) { Serial1.write(p[i]); sum += p[i]; }
  Serial1.write((uint8_t)~sum);
}

// Send a read command; return the number of reply params (or -1). Prints
// every raw byte received, which helps debug wiring (e.g. only seeing our
// own echo means TX works but the servo isn't answering).
int request(uint8_t sid, uint8_t cmd, uint8_t *out) {
  while (Serial1.available()) Serial1.read();
  send(sid, cmd, nullptr, 0);
  Serial1.flush();
  uint8_t buf[32];
  int n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 20 && n < 32) if (Serial1.available()) buf[n++] = Serial1.read();
  Serial.print("  raw:");
  for (int i = 0; i < n; i++) { Serial.print(' '); Serial.print(buf[i], HEX); }
  Serial.println(n ? "" : " (nothing -- check TX/RX swap, jumpers, servo power)");
  // Find a reply with our CMD and at least one param (skips the echo)
  for (int i = 0; i + 5 < n; i++) {
    if (buf[i] == 0x55 && buf[i + 1] == 0x55 && buf[i + 4] == cmd && buf[i + 3] > 3) {
      int np = buf[i + 3] - 3;
      if (i + 5 + np >= n) break;
      for (int k = 0; k < np; k++) out[k] = buf[i + 5 + k];
      return np;
    }
  }
  return -1;
}

void move(int pos, uint16_t ms) {
  pos_cmd = constrain(pos, 0, 1000);
  uint8_t p[4] = { (uint8_t)(pos_cmd & 0xFF), (uint8_t)(pos_cmd >> 8), (uint8_t)(ms & 0xFF), (uint8_t)(ms >> 8) };
  send(id, 1, p, 4);
  Serial.print("# move to "); Serial.print(pos_cmd);
  Serial.print(" (");  Serial.print((pos_cmd - 500) / COUNTS_PER_DEG, 1); Serial.println(" deg from center)");
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(BAUD);
  while (!Serial && millis() < 3000) {}
  Serial.println("# servo_bus_test: i r c + - w l t");
}

void loop() {
  if (!Serial.available()) return;
  char c = Serial.read();
  uint8_t r[8];
  switch (c) {
    case 'i':
      if (request(254, 14, r) >= 1) { id = r[0]; Serial.print("# servo ID = "); Serial.println(id); }
      else Serial.println("# no ID reply");
      break;
    case 'r':
      if (request(id, 28, r) >= 2) { Serial.print("# position = "); Serial.println((int16_t)(r[0] | r[1] << 8)); }
      if (request(id, 27, r) >= 2) { Serial.print("# voltage  = "); Serial.print((r[0] | r[1] << 8) / 1000.0, 2); Serial.println(" V"); }
      if (request(id, 26, r) >= 1) { Serial.print("# temp     = "); Serial.print(r[0]); Serial.println(" C"); }
      break;
    case 'c': move(500, 500); break;
    case '+': move(pos_cmd + (int)(10 * COUNTS_PER_DEG), 300); break;
    case '-': move(pos_cmd - (int)(10 * COUNTS_PER_DEG), 300); break;
    case 'w':
      for (int k = 0; k < 2; k++) {
        move(500 + (int)(15 * COUNTS_PER_DEG), 1000); delay(1100);
        move(500 - (int)(15 * COUNTS_PER_DEG), 1000); delay(1100);
      }
      move(500, 500);
      break;
    case 'l': { uint8_t p = 0; send(id, 31, &p, 1); Serial.println("# limp"); break; }
    case 't': { uint8_t p = 1; send(id, 31, &p, 1); Serial.println("# torque on"); break; }
  }
}
