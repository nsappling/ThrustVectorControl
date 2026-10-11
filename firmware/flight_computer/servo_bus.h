// servo_bus.h -- Hiwonder serial bus servo protocol (HTD-30H), through
// the BusLinker. Packet format and commands: docs/icd/servo_bus.md.
//
//   packet = 0x55 0x55 | ID | LEN | CMD | params... | CHK
//   LEN = params + 3,  CHK = ~(ID + LEN + CMD + params) & 0xFF
#pragma once
#include <Arduino.h>

class ServoBus {
 public:
  void begin(HardwareSerial &port, uint32_t baud);

  // Move to pos (0..1000 = 0..240 deg) over time_ms (0 = as fast as possible).
  void move(uint8_t id, uint16_t pos, uint16_t time_ms = 0);
  void setTorque(uint8_t id, bool on);                      // false = limp
  void setAngleLimits(uint8_t id, uint16_t lo, uint16_t hi); // saved in the servo

  // Reads block until the reply arrives (~1.5-2 ms) or time out (false).
  bool readPos(uint8_t id, int16_t &pos);
  bool readVin(uint8_t id, uint16_t &millivolts);
  bool readTemp(uint8_t id, uint8_t &deg_c);
  bool readId(uint8_t &id);   // broadcast; only works with ONE servo on the bus

 private:
  void send(uint8_t id, uint8_t cmd, const uint8_t *params, uint8_t n);
  bool request(uint8_t id, uint8_t cmd, uint8_t *out, uint8_t n_out);
  HardwareSerial *port_ = nullptr;
};
