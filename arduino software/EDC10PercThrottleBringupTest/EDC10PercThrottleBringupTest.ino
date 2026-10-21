
#include <Servo.h>

/*
 * EDF + ESC + Teensy 4.1 Wiring
 *
 * Components:
 * - Teensy 4.1
 * - Hobbywing Skywalker 50A ESC
 * - 64 mm EDF motor
 * - 12 V DC power supply (600 W)
 *
 * ESC -> Teensy:
 * - Signal wire (white/yellow) -> Teensy pin 2
 * - Ground wire (black/brown)  -> Teensy GND
 * - Red wire (5V BEC)          -> DISCONNECTED and insulated
 *
 * ESC -> Power Supply:
 * - Thick red wire (+)        -> Power supply +12V (V+)
 * - Thick black wire (-)      -> Power supply GND (V-)
 *
 * ESC -> EDF Motor:
 * - Three ESC motor wires     -> Three EDF motor wires
 * - Wire order determines rotation direction.
 * - Verify correct rotation before operating.
 *
 * Teensy Power:
 * - Power Teensy separately through USB.
 * - Do NOT connect ESC's 5V BEC to Teensy 3.3V.
 *
 * Power Supply AC Input:
 * - L = AC Live
 * - N = AC Neutral
 * - PE = Protective Earth / Chassis Ground
 * - AC wiring must be installed and inspected by
 *   qualified personnel with proper insulation,
 *   strain relief, and an enclosed mains connection.
 *
 * Safety:
 * - Secure EDF behind a protective guard before testing.
 * - Keep hands and loose objects away from the fan.
 * - Keep a physical power disconnect accessible.
 * - 1000 us = minimum throttle command.
 * - 1100 us = 10% of nominal throttle signal range.
 * - STOP commands minimum throttle, not power isolation.
 */

Servo esc;

const int ESC_PIN = 2;

const int STOP_US = 1000;
const int TEST_US = 1100;  // 10% command
const unsigned long ARM_TIME = 5000;
const unsigned long RUN_TIME = 1000;

bool armed = false;

void stopEDF() {
  esc.writeMicroseconds(STOP_US);
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);

  esc.attach(ESC_PIN, 1000, 2000);
  stopEDF();

  delay(ARM_TIME);

  armed = true;

  Serial.println("ESC initialized.");
  Serial.println("Type RUN to start.");
  Serial.println("Type STOP for minimum throttle.");
}

void loop() {
  if (!Serial.available()) return;

  String command = Serial.readStringUntil('\n');
  command.trim();
  command.toUpperCase();

  if (command == "STOP") {
    stopEDF();
    Serial.println("Minimum throttle commanded.");
    return;
  }

  if (command == "RUN" && armed) {
    Serial.println("Starting 10% test.");

    esc.writeMicroseconds(TEST_US);
    delay(RUN_TIME);

    stopEDF();

    Serial.println("Test complete.");
  }
}
