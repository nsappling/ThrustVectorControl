
#include <Arduino.h>

#define SERVO_ID 1

/*
 * HTD-30H Servo Wiring — Teensy 4.1 + HiWonder BusLinker V3.0
 *
 * Teensy 4.1          BusLinker V3.0
 * -----------------------------------
 * Pin 8 (TX2)  -----> RX
 * Pin 7 (RX2)  <----- TX (3.3V compatible or level-shifted)
 * GND          ------ GND
 *
 * Power:
 * - External 12V DC supply -> BusLinker power input
 * - HTD-30H servo -> BusLinker servo port
 * - Teensy 4.1 -> Computer via USB
 * - Do NOT connect 12V directly to Teensy
 *
 * BusLinker Jumper Configuration:
 * - Set BOTH jumpers to Servo-TTL mode
 * - Each jumper bridges the Servo and TTL pins
 * - Do NOT use Servo-USB mode for Teensy control
 *
 * Communication:
 * - Teensy Serial2 (UART2)
 * - Baud rate: 115200
 * - Servo ID: 1
 * - Protocol: HiWonder 55 55 serial bus servo
 *
 * Serial Monitor Commands:
 * L = Left   (position 350)
 * C = Center (position 500)
 * R = Right  (position 650)
 *
 * Serial Monitor baud rate: 115200
 */

// HTD-30H movement command
void moveServo(uint16_t position, uint16_t duration = 1000) {

  // Limit movement to a small range around center
  position = constrain(position, 350, 650);

  uint8_t packet[10] = {
    0x55, 0x55,          // Header
    SERVO_ID,           // Servo ID
    0x07,               // Length
    0x01,               // Move command
    (uint8_t)(position & 0xFF),
    (uint8_t)(position >> 8),
    (uint8_t)(duration & 0xFF),
    (uint8_t)(duration >> 8),
    0x00                // Checksum placeholder
  };

  uint16_t sum = 0;

  for (int i = 2; i < 9; i++) {
    sum += packet[i];
  }

  packet[9] = (uint8_t)(~sum);

  // Send command through Teensy UART2
  Serial2.write(packet, sizeof(packet));
  Serial2.flush();

  Serial.print("Sent position: ");
  Serial.println(position);

  // Print transmitted bytes for debugging
  Serial.print("TX: ");
  for (int i = 0; i < 10; i++) {
    if (packet[i] < 16) Serial.print("0");
    Serial.print(packet[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  delay(1500);

  Serial.println("HTD-30H Servo Test");
  Serial.println("Teensy 4.1 using Serial2");
  Serial.println("----------------------");
  Serial.println("L = Left (450)");
  Serial.println("C = Center (500)");
  Serial.println("R = Right (550)");
}

void loop() {

  // Check for commands from computer
  if (Serial.available()) {

    char command = Serial.read();

    switch (command) {
      case 'L':
      case 'l':
        moveServo(350, 200);
        break;

      case 'C':
      case 'c':
        moveServo(500, 200);
        break;

      case 'R':
      case 'r':
        moveServo(650, 200);
        break;
    }
  }

  // Display any UART responses
  while (Serial2.available()) {

    uint8_t b = Serial2.read();

    Serial.print("RX: 0x");

    if (b < 16) Serial.print("0");
    Serial.println(b, HEX);
  }
}
