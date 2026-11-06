#include <Arduino.h>
#include <Wire.h>
#include <Servo.h>
#include <math.h>

/*
 * ============================================================
 * TVC ROCKET - INTEGRATED SERVO, IMU, AND EDF CONTROL
 * ============================================================
 *
 * HARDWARE:
 *   Teensy 4.1
 *   MPU6050 / GY-521 IMU
 *   HiWonder HTD-30H serial bus servo
 *   HiWonder BusLinker
 *   Hobbywing Skywalker 50A ESC
 *   64 mm EDF
 *
 * FEATURES:
 *   - Manual servo positioning
 *   - Gyroscope-responsive servo positioning
 *   - Adjustable gyro gain
 *   - EDF throttle ramping UP and DOWN
 *   - Maximum ESC command of 50%
 *   - Live status output
 *   - Immediate minimum-throttle software stop
 *
 * IMPORTANT:
 *   Gyro control uses angular VELOCITY, not absolute tilt.
 *   This is not a complete stabilization controller.
 *
 *   The ESC ramp changes the COMMAND signal, not measured RPM.
 *
 *   This sketch does not have a communication watchdog.
 *   Use an independent physical power cutoff.
 *
 * ============================================================
 * WIRING
 * ============================================================
 *
 * MPU6050:
 *   VCC -> Teensy 3.3V
 *   GND -> Teensy GND
 *   SDA -> Teensy pin 18
 *   SCL -> Teensy pin 19
 *
 * BUSLINKER:
 *   RX  -> Teensy pin 8 (TX2)
 *   TX  -> Teensy pin 7 (RX2)
 *   GND -> Teensy GND
 *
 *   IMPORTANT:
 *   Verify BusLinker TX is safe for Teensy 3.3V inputs.
 *
 *   BusLinker jumpers:
 *   Both set to the previously tested Servo-TTL configuration.
 *
 * ESC:
 *   Signal -> Teensy pin 2
 *   GND    -> Teensy GND
 *
 *   ESC DC power -> External DC supply
 *   ESC motor wires -> EDF
 *
 *   Leave ESC BEC power wire disconnected and insulated
 *   when powering Teensy through USB.
 *
 *   Servo and ESC power must NOT come from Teensy pins.
 */

// ============================================================
// 1. HARDWARE CONFIGURATION
// ============================================================

// HiWonder servo ID
const uint8_t SERVO_ID = 1;

// MPU6050 I2C address
const uint8_t MPU_ADDR = 0x68;

// ESC signal output
const int ESC_PIN = 2;

// ESC output object
Servo esc;


// ============================================================
// 2. HIWONDER SERVO SETTINGS
// ============================================================

// HiWonder servo position units
const int SERVO_CENTER = 500;
const int SERVO_MIN = 400;
const int SERVO_MAX = 600;

// Requested movement duration
const uint16_t SERVO_MOVE_MS = 200;

// Last commanded servo position
int servoPosition = SERVO_CENTER;

// Whether gyro controls the servo
bool gyroControlEnabled = false;

// Servo response gain
float gyroGain = 0.15f;

// Reverse this to -1 if servo responds in wrong direction
const int CONTROL_DIRECTION = 1;

// Gyro control update interval
const unsigned long SERVO_UPDATE_MS = 20;

unsigned long lastServoUpdate = 0;


// ============================================================
// 3. ESC SETTINGS
// ============================================================

// ESC signal pulse range
const int ESC_MIN_US = 1000;
const int ESC_MAX_US = 2000;

// Maximum allowed throttle command
const int MAX_THROTTLE = 50;

// Current pulse sent to ESC
int currentPulse = ESC_MIN_US;

// Desired pulse
int targetPulse = ESC_MIN_US;

// Target throttle percentage
int targetThrottle = 0;

// Software throttle permission
bool armed = false;


// ============================================================
// 4. EDF THROTTLE RAMP SETTINGS
// ============================================================

// Change pulse width by 2 microseconds per update
const int RAMP_STEP_US = 2;

// Update every 50 milliseconds
const unsigned long RAMP_INTERVAL_MS = 50;

unsigned long lastRampUpdate = 0;

/*
 * Approximate full-range ramp times:
 *
 * 0% -> 10% :  2.5 seconds
 * 0% -> 20% :  5.0 seconds
 * 0% -> 30% :  7.5 seconds
 * 0% -> 50% : 12.5 seconds
 *
 * Ramp-down uses exactly the same rate.
 */


// ============================================================
// 5. IMU SETTINGS
// ============================================================

// Most recent Z-axis angular velocity
float gyroZ = 0.0f;

// Offset measured while stationary
float gyroZBias = 0.0f;

// Whether IMU initialized successfully
bool imuConnected = false;


// ============================================================
// 6. STATUS OUTPUT
// ============================================================

const unsigned long STATUS_INTERVAL_MS = 500;

unsigned long lastStatusPrint = 0;


// ============================================================
// 7. NONBLOCKING SERIAL INPUT
// ============================================================

/*
 * Read commands one character at a time.
 *
 * This avoids waiting inside readStringUntil(),
 * which could delay the throttle ramp and IMU updates.
 */

char commandBuffer[64];

size_t commandLength = 0;


// ============================================================
// 8. HIWONDER SERVO MOVEMENT
// ============================================================

void moveServo(int requestedPosition) {

  // Enforce safe software position limits
  int position = constrain(
    requestedPosition,
    SERVO_MIN,
    SERVO_MAX
  );

  // No need to resend the same position
  if (position == servoPosition) {
    return;
  }

  // HiWonder serial command packet
  uint8_t packet[10] = {
    0x55,
    0x55,
    SERVO_ID,
    0x07,
    0x01,
    (uint8_t)(position & 0xFF),
    (uint8_t)(position >> 8),
    (uint8_t)(SERVO_MOVE_MS & 0xFF),
    (uint8_t)(SERVO_MOVE_MS >> 8),
    0x00
  };

  // Calculate checksum
  uint16_t sum = 0;

  for (int i = 2; i < 9; i++) {
    sum += packet[i];
  }

  packet[9] = (uint8_t)(~sum);

  // Send command to BusLinker
  Serial2.write(packet, sizeof(packet));

  // Save last commanded position
  servoPosition = position;
}


// ============================================================
// 9. CONVERT THROTTLE PERCENTAGE TO ESC PULSE
// ============================================================

int percentToPulse(int percent) {

  percent = constrain(
    percent,
    0,
    MAX_THROTTLE
  );

  return map(
    percent,
    0,
    100,
    ESC_MIN_US,
    ESC_MAX_US
  );
}


// ============================================================
// 10. SET EDF THROTTLE TARGET
// ============================================================

/*
 * This function does NOT immediately change motor command.
 *
 * It only updates targetPulse.
 *
 * updateThrottleRamp() gradually moves currentPulse
 * toward targetPulse.
 */

void setThrottleTarget(int percent) {

  percent = constrain(
    percent,
    0,
    MAX_THROTTLE
  );

  if (percent > 0 && !armed) {
    Serial.println("ERROR: Type 'arm' first.");
    return;
  }

  targetThrottle = percent;

  targetPulse = percentToPulse(percent);

  Serial.print("New EDF target: ");
  Serial.print(targetThrottle);
  Serial.print("% (");
  Serial.print(targetPulse);
  Serial.println(" us)");

  Serial.println("Ramping toward target...");
}


// ============================================================
// 11. EDF THROTTLE RAMP
// ============================================================

/*
 * This is the same ramp logic from your working EDF test.
 *
 * It handles BOTH acceleration and deceleration.
 */

void updateThrottleRamp() {

  unsigned long now = millis();

  // Wait until next ramp interval
  if (now - lastRampUpdate < RAMP_INTERVAL_MS) {
    return;
  }

  lastRampUpdate = now;

  // ------------------------------
  // RAMP UP
  // ------------------------------

  if (currentPulse < targetPulse) {

    currentPulse += RAMP_STEP_US;

    if (currentPulse > targetPulse) {
      currentPulse = targetPulse;
    }
  }

  // ------------------------------
  // RAMP DOWN
  // ------------------------------

  else if (currentPulse > targetPulse) {

    currentPulse -= RAMP_STEP_US;

    if (currentPulse < targetPulse) {
      currentPulse = targetPulse;
    }
  }

  // Send current command to ESC
  esc.writeMicroseconds(currentPulse);
}


// ============================================================
// 12. NORMAL EDF STOP
// ============================================================

/*
 * Normal stop:
 * Set the TARGET to zero.
 *
 * The ESC command then ramps down gradually.
 */

void stopEDF() {

  targetThrottle = 0;
  targetPulse = ESC_MIN_US;

  Serial.println("STOP: EDF ramping down to 0%.");
}


// ============================================================
// 13. IMMEDIATE SOFTWARE STOP
// ============================================================

/*
 * Immediately request minimum throttle.
 *
 * This is NOT a physical emergency disconnect.
 * The EDF may continue spinning due to inertia.
 */

void emergencyStopEDF() {

  targetThrottle = 0;
  targetPulse = ESC_MIN_US;
  currentPulse = ESC_MIN_US;

  esc.writeMicroseconds(ESC_MIN_US);

  armed = false;

  Serial.println("Minimum ESC pulse commanded.");
  Serial.println("EDF software disarmed.");
}


// ============================================================
// 14. INITIALIZE MPU6050
// ============================================================

bool initializeIMU() {

  // Wake MPU6050
  Wire.beginTransmission(MPU_ADDR);

  Wire.write(0x6B);
  Wire.write(0x00);

  if (Wire.endTransmission() != 0) {
    return false;
  }

  // Set gyro to +/-250 degrees/second
  Wire.beginTransmission(MPU_ADDR);

  Wire.write(0x1B);
  Wire.write(0x00);

  return Wire.endTransmission() == 0;
}


// ============================================================
// 15. READ Z-AXIS GYROSCOPE
// ============================================================

bool readGyroZ(float &value) {

  // Point to Z-axis gyro registers
  Wire.beginTransmission(MPU_ADDR);

  Wire.write(0x47);

  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  // Read two bytes
  if (Wire.requestFrom(MPU_ADDR, (uint8_t)2, true) != 2) {
    return false;
  }

  // Combine high and low bytes
  int16_t raw =
    (Wire.read() << 8) | Wire.read();

  // Convert raw measurement to degrees/second
  value = raw / 131.0f;

  return true;
}


// ============================================================
// 16. CALIBRATE GYROSCOPE
// ============================================================

/*
 * Keep rocket stationary during this step.
 *
 * Collect 200 readings and calculate average offset.
 */

bool calibrateGyro() {

  Serial.println("Calibrating gyro...");
  Serial.println("Keep the rocket stationary.");

  float sum = 0.0f;
  int validSamples = 0;

  for (int i = 0; i < 200; i++) {

    float reading;

    if (readGyroZ(reading)) {

      sum += reading;

      validSamples++;
    }

    delay(5);
  }

  if (validSamples < 190) {

    Serial.println("ERROR: Gyro calibration failed.");

    return false;
  }

  gyroZBias = sum / validSamples;

  Serial.print("Gyro bias: ");
  Serial.println(gyroZBias, 4);

  return true;
}


// ============================================================
// 17. UPDATE GYRO AND SERVO
// ============================================================

/*
 * Read the rocket's angular velocity.
 *
 * If gyro control is enabled:
 *
 * correction = gain * angular velocity
 *
 * servo target = center - correction
 *
 * This is RATE feedback, not angle feedback.
 */

void updateGyroControl() {

  float rawGyroZ;

  // Read sensor
  if (!readGyroZ(rawGyroZ)) {

    imuConnected = false;

    gyroControlEnabled = false;

    // Fault response: immediately request minimum throttle
    emergencyStopEDF();

    Serial.println("ERROR: MPU6050 communication lost.");

    return;
  }

  // Remove stationary offset
  gyroZ = rawGyroZ - gyroZBias;

  // Manual servo mode: don't change position
  if (!gyroControlEnabled) {
    return;
  }

  float rate = gyroZ;

  // Ignore very small rotation rates
  if (fabsf(rate) < 1.0f) {
    rate = 0.0f;
  }

  // Proportional rate correction
  int correction = roundf(
    CONTROL_DIRECTION * gyroGain * rate
  );

  // Calculate servo position
  int targetPosition =
    SERVO_CENTER - correction;

  // Apply servo movement limits
  targetPosition = constrain(
    targetPosition,
    SERVO_MIN,
    SERVO_MAX
  );

  // Send servo position
  moveServo(targetPosition);
}


// ============================================================
// 18. PRINT SYSTEM STATUS
// ============================================================

void printStatus() {

  float currentThrottle =
    100.0f * (currentPulse - ESC_MIN_US) /
    (ESC_MAX_US - ESC_MIN_US);

  Serial.print("Gyro Z: ");
  Serial.print(gyroZ, 2);
  Serial.print(" deg/s");

  Serial.print(" | Servo: ");
  Serial.print(servoPosition);

  Serial.print(" | Gyro control: ");
  Serial.print(gyroControlEnabled ? "ON" : "OFF");

  Serial.print(" | Gain: ");
  Serial.print(gyroGain, 2);

  Serial.print(" | EDF: ");
  Serial.print(currentThrottle, 1);
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


// ============================================================
// 19. HELP MENU
// ============================================================

void printHelp() {

  Serial.println();
  Serial.println("========== TVC COMMANDS ==========");

  Serial.println();
  Serial.println("SERVO:");
  Serial.println("L             Servo position 400");
  Serial.println("C             Servo position 500");
  Serial.println("R             Servo position 600");

  Serial.println();
  Serial.println("GYROSCOPE:");
  Serial.println("gyro on       Enable gyro response");
  Serial.println("gyro off      Disable gyro response");
  Serial.println("gain 0.3      Set gyro gain to 0.3");
  Serial.println("gain 0.5      Set gyro gain to 0.5");

  Serial.println();
  Serial.println("EDF:");
  Serial.println("arm           Unlock throttle");
  Serial.println("throttle 10   Ramp to 10%");
  Serial.println("throttle 25   Ramp to 25%");
  Serial.println("throttle 50   Ramp to 50%");
  Serial.println("throttle 0    Ramp down to zero");
  Serial.println("stop          Ramp down to zero");
  Serial.println("disarm        Immediate minimum");
  Serial.println("estop         Immediate minimum");

  Serial.println();
  Serial.println("GENERAL:");
  Serial.println("status        Print current status");
  Serial.println("help          Show commands");

  Serial.println();
  Serial.println("Maximum EDF command: 50%");
  Serial.println("==================================");
}


// ============================================================
// 20. HANDLE SERIAL COMMANDS
// ============================================================

void handleCommand(String command) {

  command.trim();
  command.toLowerCase();

  if (command.length() == 0) {
    return;
  }

  // ----------------------------------------------------------
  // IMMEDIATE SOFTWARE STOP
  // ----------------------------------------------------------

  if (command == "estop" || command == "disarm") {

    emergencyStopEDF();

    gyroControlEnabled = false;

    moveServo(SERVO_CENTER);

    return;
  }

  // ----------------------------------------------------------
  // MANUAL SERVO CONTROL
  // ----------------------------------------------------------

  if (command == "l") {

    gyroControlEnabled = false;

    moveServo(SERVO_MIN);

    Serial.println("Servo commanded LEFT.");
  }

  else if (command == "c") {

    gyroControlEnabled = false;

    moveServo(SERVO_CENTER);

    Serial.println("Servo commanded CENTER.");
  }

  else if (command == "r") {

    gyroControlEnabled = false;

    moveServo(SERVO_MAX);

    Serial.println("Servo commanded RIGHT.");
  }

  // ----------------------------------------------------------
  // GYRO CONTROL
  // ----------------------------------------------------------

  else if (command == "gyro on") {

    if (!imuConnected) {

      Serial.println("ERROR: MPU6050 unavailable.");

      return;
    }

    gyroControlEnabled = true;

    Serial.println("Gyro servo response ENABLED.");
  }

  else if (command == "gyro off") {

    gyroControlEnabled = false;

    moveServo(SERVO_CENTER);

    Serial.println("Gyro servo response DISABLED.");
  }

  // ----------------------------------------------------------
  // GYRO GAIN
  // ----------------------------------------------------------

  else if (command.startsWith("gain ")) {

    String value = command.substring(5);
    value.trim();

    float requestedGain = value.toFloat();

    if (requestedGain < 0.0f || requestedGain > 1.0f) {

      Serial.println("ERROR: Gain must be 0.0 to 1.0.");

      return;
    }

    gyroGain = requestedGain;

    Serial.print("Gyro gain set to ");
    Serial.println(gyroGain, 3);
  }

  // ----------------------------------------------------------
  // ARM EDF
  // ----------------------------------------------------------

  else if (command == "arm") {

    if (currentPulse != ESC_MIN_US ||
        targetPulse != ESC_MIN_US) {

      Serial.println("ERROR: Throttle must be at zero.");

      return;
    }

    if (!imuConnected) {

      Serial.println("ERROR: IMU unavailable.");

      return;
    }

    armed = true;

    Serial.println("EDF software throttle ARMED.");
  }

  // ----------------------------------------------------------
  // SET EDF TARGET
  // ----------------------------------------------------------

  else if (command.startsWith("throttle ")) {

    String value = command.substring(9);
    value.trim();

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

  // ----------------------------------------------------------
  // NORMAL RAMPED STOP
  // ----------------------------------------------------------

  else if (command == "stop") {

    stopEDF();
  }

  // ----------------------------------------------------------
  // STATUS AND HELP
  // ----------------------------------------------------------

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


// ============================================================
// 21. NONBLOCKING SERIAL COMMAND READER
// ============================================================

void readSerialCommands() {

  while (Serial.available() > 0) {

    char incoming = Serial.read();

    // Ignore carriage return
    if (incoming == '\r') {
      continue;
    }

    // Newline means command is complete
    if (incoming == '\n') {

      commandBuffer[commandLength] = '\0';

      handleCommand(String(commandBuffer));

      commandLength = 0;

      continue;
    }

    // Store character if there is room
    if (commandLength < sizeof(commandBuffer) - 1) {

      commandBuffer[commandLength] = incoming;

      commandLength++;
    }

    else {

      commandLength = 0;

      Serial.println("ERROR: Command too long.");
    }
  }
}


// ============================================================
// 22. SETUP
// ============================================================

void setup() {

  // Computer USB Serial Monitor
  Serial.begin(115200);

  // BusLinker UART communication
  Serial2.begin(115200);

  // MPU6050 I2C communication
  Wire.begin();

  // Limit time spent waiting for an I2C response
  Wire.setWireTimeout(3000, true);

  // ----------------------------------------------------------
  // Initialize ESC
  // ----------------------------------------------------------

  currentPulse = ESC_MIN_US;
  targetPulse = ESC_MIN_US;
  targetThrottle = 0;

  armed = false;

  esc.attach(
    ESC_PIN,
    ESC_MIN_US,
    ESC_MAX_US
  );

  esc.writeMicroseconds(ESC_MIN_US);

  // Give ESC time to initialize at minimum command
  delay(3000);

  Serial.println();
  Serial.println("INTEGRATED TVC CONTROLLER STARTING");

  // ----------------------------------------------------------
  // Initialize MPU6050
  // ----------------------------------------------------------

  imuConnected = initializeIMU();

  if (imuConnected) {
    imuConnected = calibrateGyro();
  }

  if (imuConnected) {

    Serial.println("MPU6050 initialized.");
  }

  else {

    Serial.println("ERROR: MPU6050 initialization failed.");
  }

  // ----------------------------------------------------------
  // Center servo
  // ----------------------------------------------------------

  // Force the initial position command to be transmitted
  servoPosition = -1;

  moveServo(SERVO_CENTER);

  // ----------------------------------------------------------
  // Initial controller state
  // ----------------------------------------------------------

  gyroControlEnabled = false;

  armed = false;

  lastRampUpdate = millis();
  lastServoUpdate = millis();
  lastStatusPrint = millis();

  Serial.println("Controller initialized.");

  printHelp();
}


// ============================================================
// 23. MAIN LOOP
// ============================================================

void loop() {

  unsigned long now = millis();

  // ----------------------------------------------------------
  // TASK 1: Read Serial Monitor commands
  // ----------------------------------------------------------

  readSerialCommands();

  // ----------------------------------------------------------
  // TASK 2: Update EDF throttle ramp
  // ----------------------------------------------------------

  updateThrottleRamp();

  // ----------------------------------------------------------
  // TASK 3: Read gyro and update servo
  // ----------------------------------------------------------

  if (now - lastServoUpdate >= SERVO_UPDATE_MS) {

    lastServoUpdate = now;

    if (imuConnected) {

      updateGyroControl();
    }
  }

  // ----------------------------------------------------------
  // TASK 4: Print system status
  // ----------------------------------------------------------

  if (now - lastStatusPrint >= STATUS_INTERVAL_MS) {

    lastStatusPrint = now;

    printStatus();
  }

  // ----------------------------------------------------------
  // TASK 5: Clear unused BusLinker responses
  // ----------------------------------------------------------

  while (Serial2.available() > 0) {

    Serial2.read();
  }
}