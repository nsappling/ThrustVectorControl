
#include <Servo.h>

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
