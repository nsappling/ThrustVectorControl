
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
 *
 * Ramps (added): the throttle ramps up over RAMP_UP_MS and back down over
 * RAMP_DOWN_MS instead of jumping. Cutting throttle instantly makes the
 * spinning fan push energy back into the 12 V line; the Mean Well can't
 * absorb it, the voltage jumps, and its overvoltage protection shuts it
 * off until the AC is unplugged for a few minutes.
 */

Servo esc;

const int ESC_PIN = 2;

const int STOP_US = 1000;
const int TEST_US = 1100;  // 10% command
const unsigned long ARM_TIME = 5000;
const unsigned long RUN_TIME = 1000;
const unsigned long RAMP_UP_MS = 500;
const unsigned long RAMP_DOWN_MS = 1500;

int current_us = STOP_US;   // pulse being sent right now

// Move the throttle from current_us to target_us in a straight line over ms.
void rampTo(int target_us, unsigned long ms) {
  int start_us = current_us;
  unsigned long t0 = millis();
  while (millis() - t0 < ms) {
    float f = (float)(millis() - t0) / ms;
    current_us = start_us + (int)((target_us - start_us) * f);
    esc.writeMicroseconds(current_us);
    delay(10);
  }
  current_us = target_us;
  esc.writeMicroseconds(current_us);
}

bool armed = false;

void stopEDF() {
  rampTo(STOP_US, RAMP_DOWN_MS);
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);

  esc.attach(ESC_PIN, 1000, 2000);
  esc.writeMicroseconds(STOP_US);   // instant: the fan isn't spinning yet

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

    rampTo(TEST_US, RAMP_UP_MS);
    delay(RUN_TIME);

    stopEDF();

    Serial.println("Test complete.");
  }
}
