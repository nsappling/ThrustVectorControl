#include <Arduino.h>
#include <Servo.h>

/*
 * =====================================================
 * TEENSY 4.1 - EDF THROTTLE RAMP TEST
 * =====================================================
 *
 * Hardware:
 *   Teensy 4.1
 *   Hobbywing Skywalker 50A ESC
 *   64 mm EDF
 *
 * No IMU, BusLinker, or HiWonder servo required.
 *
 * Commands:
 *   arm          Enable throttle commands
 *   throttle 10  Ramp to 10%
 *   throttle 25  Ramp to 25%
 *   throttle 50  Ramp to 50% (maximum)
 *   throttle 0   Ramp down to zero
 *   stop         Ramp down to zero
 *   disarm       Immediately command minimum pulse
 *   estop        Immediately command minimum pulse
 *   status       Print current settings
 *   help         Show commands
 *
 * This ramps ESC pulse width, not measured RPM.
 *
 * =====================================================
 * WIRING
 * =====================================================
 *
 * ESC signal        -> Teensy pin 2
 * ESC signal ground -> Teensy GND
 *
 * ESC DC input      -> External DC supply
 * ESC motor wires   -> EDF motor
 * Teensy            -> Computer USB
 *
 * Keep ESC BEC power isolated from Teensy.
 */

// =====================================================
// ESC SETTINGS
// =====================================================

const int ESC_PIN = 2;

// Verify the ESC accepts this pulse-width range.
const int ESC_MIN_US = 1000;
const int ESC_MAX_US = 2000;

// Maximum permitted throttle command.
const int MAX_THROTTLE = 50;

Servo esc;

// =====================================================
// RAMP SETTINGS
// =====================================================

// Increase/decrease ESC command by 2 microseconds.
const int RAMP_STEP_US = 2;

// Make one adjustment every 50 milliseconds.
const unsigned long RAMP_INTERVAL_MS = 50;

// At these settings:
// 0% -> 10% takes about 2.5 seconds.
// 0% -> 25% takes about 6.25 seconds.
// 0% -> 50% takes about 12.5 seconds.

// =====================================================
// THROTTLE STATE
// =====================================================

// Pulse currently commanded to ESC.
int currentPulse = ESC_MIN_US;

// Pulse we are trying to reach.
int targetPulse = ESC_MIN_US;

// Current target percentage.
int targetThrottle = 0;

// Software arming state.
bool armed = false;

// =====================================================
// TIMING
// =====================================================

unsigned long lastRampUpdate = 0;
unsigned long lastStatusPrint = 0;

const unsigned long STATUS_INTERVAL_MS = 500;

// =====================================================
// THROTTLE CONVERSION
// =====================================================

int percentToPulse(int percent) {

  percent = constrain(percent, 0, MAX_THROTTLE);

  return map(
    percent,
    0,
    100,
    ESC_MIN_US,
    ESC_MAX_US
  );
}

// =====================================================
// SET TARGET THROTTLE
// =====================================================

void setThrottleTarget(int percent) {

  percent = constrain(percent, 0, MAX_THROTTLE);

  if (percent > 0 && !armed) {
    Serial.println("ERROR: Type 'arm' first.");
    return;
  }

  // Update the destination, not the current pulse.
  targetThrottle = percent;
  targetPulse = percentToPulse(percent);

  Serial.print("New target: ");
  Serial.print(targetThrottle);
  Serial.print("% (");
  Serial.print(targetPulse);
  Serial.println(" us)");

  Serial.println("Ramping toward target...");
}

// =====================================================
// UPDATE THROTTLE RAMP
// =====================================================

void updateThrottleRamp() {

  unsigned long now = millis();

  // Only update once per interval.
  if (now - lastRampUpdate < RAMP_INTERVAL_MS) {
    return;
  }

  lastRampUpdate = now;

  // RAMP UP
  if (currentPulse < targetPulse) {

    currentPulse += RAMP_STEP_US;

    // Never overshoot target.
    if (currentPulse > targetPulse) {
      currentPulse = targetPulse;
    }
  }

  // RAMP DOWN
  else if (currentPulse > targetPulse) {

    currentPulse -= RAMP_STEP_US;

    // Never undershoot target.
    if (currentPulse < targetPulse) {
      currentPulse = targetPulse;
    }
  }

  // Send current command to ESC.
  esc.writeMicroseconds(currentPulse);
}

// =====================================================
// NORMAL STOP: RAMP DOWN
// =====================================================

void stopEDF() {

  targetThrottle = 0;
  targetPulse = ESC_MIN_US;

  Serial.println("STOP: Ramping down to 0%.");
}

// =====================================================
// IMMEDIATE MINIMUM-THROTTLE COMMAND
// =====================================================

void emergencyStopEDF() {

  targetThrottle = 0;
  targetPulse = ESC_MIN_US;
  currentPulse = ESC_MIN_US;

  esc.writeMicroseconds(ESC_MIN_US);

  armed = false;

  Serial.println("Minimum throttle commanded.");
  Serial.println("EDF software control disarmed.");
}

// =====================================================
// PRINT STATUS
// =====================================================

void printStatus() {

  float currentPercent =
    100.0f * (currentPulse - ESC_MIN_US) /
    (ESC_MAX_US - ESC_MIN_US);

  Serial.print("Current: ");
  Serial.print(currentPercent, 1);
  Serial.print("%");

  Serial.print(" | Target: ");
  Serial.print(targetThrottle);
  Serial.print("%");

  Serial.print(" | Pulse: ");
  Serial.print(currentPulse);
  Serial.print(" us");

  Serial.print(" | Armed: ");
  Serial.print(armed ? "YES" : "NO");

  Serial.print(" | State: ");

  if (currentPulse < targetPulse) {
    Serial.println("RAMPING UP");
  }
  else if (currentPulse > targetPulse) {
    Serial.println("RAMPING DOWN");
  }
  else {
    Serial.println("HOLDING");
  }
}

// =====================================================
// HELP MENU
// =====================================================

void printHelp() {

  Serial.println();
  Serial.println("====== EDF RAMP TEST ======");

  Serial.println("arm          Unlock throttle");
  Serial.println("throttle 10  Ramp to 10%");
  Serial.println("throttle 25  Ramp to 25%");
  Serial.println("throttle 50  Ramp to 50%");
  Serial.println("throttle 0   Ramp down to 0%");
  Serial.println("stop         Ramp down to 0%");
  Serial.println("disarm       Immediate minimum");
  Serial.println("estop        Immediate minimum");
  Serial.println("status       Print status");
  Serial.println("help         Show commands");

  Serial.println();
  Serial.println("Maximum throttle command: 50%");
  Serial.println("===========================");
}

// =====================================================
// HANDLE SERIAL COMMANDS
// =====================================================

void handleCommand(String command) {

  command.trim();
  command.toLowerCase();

  if (command.length() == 0) {
    return;
  }

  if (command == "arm") {

    // Do not arm during an active ramp.
    if (currentPulse != ESC_MIN_US) {
      Serial.println("ERROR: Throttle must be zero.");
      return;
    }

    armed = true;

    Serial.println("EDF software throttle armed.");
  }

  else if (command.startsWith("throttle ")) {

    String value = command.substring(9);
    value.trim();

    // Require an integer from 0 to 50.
    bool valid = value.length() > 0;

    for (unsigned int i = 0; i < value.length(); i++) {
      if (!isDigit(value[i])) {
        valid = false;
      }
    }

    if (!valid) {
      Serial.println("ERROR: Use throttle 0 through 50.");
      return;
    }

    int requested = value.toInt();

    if (requested > MAX_THROTTLE) {
      Serial.println("ERROR: Maximum throttle is 50%.");
      return;
    }

    setThrottleTarget(requested);
  }

  else if (command == "stop") {

    stopEDF();
  }

  else if (command == "disarm" ||
           command == "estop") {

    emergencyStopEDF();
  }

  else if (command == "status") {

    printStatus();
  }

  else if (command == "help") {

    printHelp();
  }

  else {

    Serial.println("Unknown command. Type 'help'.");
  }
}

// =====================================================
// SETUP
// =====================================================

void setup() {

  Serial.begin(115200);

  // Begin with minimum throttle.
  currentPulse = ESC_MIN_US;
  targetPulse = ESC_MIN_US;
  targetThrottle = 0;
  armed = false;

  esc.attach(ESC_PIN, ESC_MIN_US, ESC_MAX_US);
  esc.writeMicroseconds(ESC_MIN_US);

  // Allow ESC to receive minimum throttle.
  delay(3000);

  Serial.println();
  Serial.println("EDF RAMP TEST READY");
  Serial.println("ESC minimum throttle commanded.");

  printHelp();
}

// =====================================================
// MAIN LOOP
// =====================================================

void loop() {

  // Read user commands.
  if (Serial.available() > 0) {

    String command = Serial.readStringUntil('\n');

    handleCommand(command);
  }

  // Ramp toward requested throttle.
  updateThrottleRamp();

  // Print progress twice per second.
  if (millis() - lastStatusPrint >= STATUS_INTERVAL_MS) {

    lastStatusPrint = millis();

    printStatus();
  }
}