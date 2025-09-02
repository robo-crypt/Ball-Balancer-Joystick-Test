#include <Servo.h>
#include <math.h>

Servo servo1, servo2, servo3;

// Arm lengths in cm
const float L1 = 5.0;  // Lower arm
const float L2 = 7.0;  // Upper arm

// Joystick pins
const int joyXPin = A0;
const int joyYPin = A1;

// Servo neutral angles (based on your setup)
const float neutral1 = 160;
const float neutral2 = 170;
const float neutral3 = 160;

// Tilt limits in degrees
const float maxTilt = 20.0;

// Dead zone threshold
const int deadZone = 50;

// Convert degrees to radians
float degToRad(float deg) {
  return deg * PI / 180.0;
}

// Inverse kinematics calculation
float calculateZ(float x, float y, float alphaDeg, float betaDeg) {
  float alphaRad = degToRad(alphaDeg);
  float betaRad = degToRad(betaDeg);
  return x * tan(alphaRad) + y * tan(betaRad);
}

// Convert vertical displacement to servo angle
float zToServoAngle(float z) {
  float angleRad = asin(z / L1);
  float angleDeg = angleRad * 180.0 / PI;
  return angleDeg;
}

void setup() {
  servo1.attach(9);
  servo2.attach(10);
  servo3.attach(11);
  Serial.begin(9600);

  servo1.write(neutral1);
  servo2.write(neutral2);
  servo3.write(neutral3);
}

void loop() {
  int joyX = analogRead(joyXPin);
  int joyY = analogRead(joyYPin);

  float alpha = 0;
  float beta = 0;

  // Apply dead zone and map to tilt angles
  if (abs(joyX - 512) > deadZone) {
    alpha = map(joyX, 0, 1023, -maxTilt, maxTilt);
  }

  if (abs(joyY - 512) > deadZone) {
    beta = map(joyY, 0, 1023, maxTilt, -maxTilt);
  }

  // Servo platform positions (radius 9.5 cm, 120° apart)
    float x1 = 0,     y1 = 9.5;
    float x2 = -8.22, y2 = -4.75;
    float x3 =  8.22, y3 = -4.75;

  float z1 = calculateZ(x1, y1, alpha, beta);
  float z2 = calculateZ(x2, y2, alpha, beta);
  float z3 = calculateZ(x3, y3, alpha, beta);

  float a1 = neutral1 + zToServoAngle(z1);
  float a2 = neutral2 + zToServoAngle(z2);
  float a3 = neutral3 + zToServoAngle(z3);

  a1 = constrain(a1, 0, 180);
  a2 = constrain(a2, 0, 180);
  a3 = constrain(a3, 0, 180);

  servo1.write(a1);
  servo2.write(a2);
  servo3.write(a3);

  delay(20); // Small delay for smoother motion
}
