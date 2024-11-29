#include <Arduino.h>

// Define motor control pins
#define STEP_PIN PA0
#define DIR_PIN PA1
#define LED_PIN PC13 // Onboard LED

// Control parameters
#define STEP_DELAY 500 // Microseconds between steps
#define STEPS_PER_REV 1000 // Steps per revolution (adjust based on your motor)

void rotateMotor(bool clockwise, int steps);

void setup() {
  // Set motor pins as outputs
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  
  pinMode(LED_PIN, OUTPUT);

  // Initialize pins to low
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  // Rotate motor clockwise
  rotateMotor(true, STEPS_PER_REV);
  digitalWrite(PC13, LOW);
  delay(1000); // Pause for a second

  // Rotate motor counterclockwise
  rotateMotor(false, STEPS_PER_REV);
  digitalWrite(PC13, HIGH);
  delay(1000); // Pause for a second
}

void rotateMotor(bool clockwise, int steps) {
  // Set direction
  digitalWrite(DIR_PIN, clockwise ? HIGH : LOW);

  // Perform steps
  for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(STEP_DELAY); // Step pulse duration
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(STEP_DELAY); // Step pulse interval
  }
}
