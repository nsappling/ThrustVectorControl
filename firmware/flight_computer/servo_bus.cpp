// servo_bus.cpp -- see servo_bus.h
#include "servo_bus.h"

namespace {
// Command values from the Hiwonder bus servo protocol
constexpr uint8_t CMD_MOVE_TIME_WRITE   = 1;
constexpr uint8_t CMD_ID_READ           = 14;
constexpr uint8_t CMD_ANGLE_LIMIT_WRITE = 20;
constexpr uint8_t CMD_TEMP_READ         = 26;
constexpr uint8_t CMD_VIN_READ          = 27;
constexpr uint8_t CMD_POS_READ          = 28;
constexpr uint8_t CMD_LOAD_WRITE        = 31;
constexpr uint8_t ID_BROADCAST          = 254;
constexpr uint32_t REPLY_TIMEOUT_US     = 6000;
}

void ServoBus::begin(HardwareSerial &port, uint32_t baud) {
  port_ = &port;
  port_->begin(baud);
}

void ServoBus::send(uint8_t id, uint8_t cmd, const uint8_t *params, uint8_t n) {
  uint8_t buf[16];
  uint8_t len = n + 3;
  uint8_t sum = id + len + cmd;
  buf[0] = 0x55; buf[1] = 0x55; buf[2] = id; buf[3] = len; buf[4] = cmd;
  for (uint8_t i = 0; i < n; i++) { buf[5 + i] = params[i]; sum += params[i]; }
  buf[5 + n] = ~sum;
  port_->write(buf, 6 + n);   // buffered: returns immediately
}

// Send a read command and wait for the matching reply. The BusLinker
// joins TX and RX onto one wire, so we may also hear our OWN request
// echoed back; it has LEN = 3 (no params), so it's skipped by the
// length check below.
bool ServoBus::request(uint8_t id, uint8_t cmd, uint8_t *out, uint8_t n_out) {
  while (port_->available()) port_->read();   // drop stale bytes
  send(id, cmd, nullptr, 0);
  port_->flush();                              // wait until it's on the wire

  uint8_t pkt[16];
  uint8_t idx = 0;
  uint32_t t0 = micros();
  while (micros() - t0 < REPLY_TIMEOUT_US) {
    if (!port_->available()) continue;
    uint8_t b = port_->read();
    // Byte-by-byte packet parser: header, then ID/LEN/CMD/params/CHK
    if (idx < 2) { idx = (b == 0x55) ? idx + 1 : 0; continue; }
    pkt[idx++] = b;
    if (idx >= 4 && (pkt[3] < 3 || pkt[3] > 10)) { idx = 0; continue; }  // bad LEN
    if (idx >= 4 && idx == pkt[3] + 3) {      // full packet received
      uint8_t len = pkt[3];
      uint8_t sum = 0;
      for (uint8_t i = 2; i < len + 2; i++) sum += pkt[i];
      bool ok = ((uint8_t)~sum == pkt[len + 2]) && pkt[4] == cmd && len == n_out + 3;
      if (ok) {
        for (uint8_t i = 0; i < n_out; i++) out[i] = pkt[5 + i];
        return true;
      }
      idx = 0;   // echo or someone else's packet: keep listening
    }
  }
  return false;
}

void ServoBus::move(uint8_t id, uint16_t pos, uint16_t time_ms) {
  if (pos > 1000) pos = 1000;
  uint8_t p[4] = { (uint8_t)(pos & 0xFF), (uint8_t)(pos >> 8),
                   (uint8_t)(time_ms & 0xFF), (uint8_t)(time_ms >> 8) };
  send(id, CMD_MOVE_TIME_WRITE, p, 4);
}

void ServoBus::setTorque(uint8_t id, bool on) {
  uint8_t p[1] = { (uint8_t)(on ? 1 : 0) };
  send(id, CMD_LOAD_WRITE, p, 1);
}

void ServoBus::setAngleLimits(uint8_t id, uint16_t lo, uint16_t hi) {
  uint8_t p[4] = { (uint8_t)(lo & 0xFF), (uint8_t)(lo >> 8),
                   (uint8_t)(hi & 0xFF), (uint8_t)(hi >> 8) };
  send(id, CMD_ANGLE_LIMIT_WRITE, p, 4);
}

bool ServoBus::readPos(uint8_t id, int16_t &pos) {
  uint8_t r[2];
  if (!request(id, CMD_POS_READ, r, 2)) return false;
  pos = (int16_t)(r[0] | (r[1] << 8));   // signed: can read slightly < 0
  return true;
}

bool ServoBus::readVin(uint8_t id, uint16_t &millivolts) {
  uint8_t r[2];
  if (!request(id, CMD_VIN_READ, r, 2)) return false;
  millivolts = r[0] | (r[1] << 8);
  return true;
}

bool ServoBus::readTemp(uint8_t id, uint8_t &deg_c) {
  return request(id, CMD_TEMP_READ, &deg_c, 1);
}

bool ServoBus::readId(uint8_t &id) {
  return request(ID_BROADCAST, CMD_ID_READ, &id, 1);
}
