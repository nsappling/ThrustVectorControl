#include <Wire.h>
#include <Servo.h>

Servo motor;
float angle = 90;
unsigned long lastUpdate = 0;

void setup() {
  Wire.begin();

  motor.attach(4);
  motor.write(90);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);                 // Wake up the GY-521
  Wire.endTransmission();

  lastUpdate = millis();
}

void loop() {
  if (millis() - lastUpdate < 10) return;
  lastUpdate += 10;              // Update every 10 ms

  Wire.beginTransmission(0x68);
  Wire.write(0x47);              // Z-axis gyro
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 2, true);

  int gyroZ = (Wire.read() << 8) | Wire.read();

  if (abs(gyroZ) > 300) {
    angle -= (gyroZ / 131.0) * 0.01 * 3.0;
    angle = constrain(angle, 0, 180);
    motor.write(angle);
  }
}