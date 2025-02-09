#include <Arduino.h>

// Define motor control pins
#define STEP_PIN PA0
#define DIR_PIN PA1
#define LED_PIN PC13 // Onboard LED

// Control parameters
#define STEP_DELAY 100 // Microseconds between steps
#define STEPS_PER_REV 3000 // Steps per revolution (adjust based on your motor)

void rotateMotor(bool clockwise, int steps);

String inputString = "";  // A variable to hold the received string
bool inputComplete = false;  // A flag to indicate when the input is complete


void setup() {
  // Set motor pins as outputs
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  
  pinMode(LED_PIN, OUTPUT);

  // Initialize pins to low
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  while(!Serial);
  Serial.begin(115200);
  Serial.println("Stepper Motor Test");
  pinMode(PC13,OUTPUT);
}

void loop() {

  if (Serial.available() > 0) {
    // Read a line of incoming data (terminated by newline)
    String input = Serial.readStringUntil('\n');
    
    // Blink the LED to indicate data reception.
    digitalWrite(LED_BUILTIN, HIGH);
    digitalWrite(LED_BUILTIN, LOW);
    
    // Optionally echo the received data to Serial (useful when ROS2 node is not active).
    Serial.print("Received: ");
    Serial.println(input);
  }

  // // Rotate motor clockwise
  // rotateMotor(true, STEPS_PER_REV);
  // digitalWrite(PC13, LOW);
  // Serial.println("Up");

  // delay(100); // Pause for a second

  // // Rotate motor counterclockwise
  // rotateMotor(false, STEPS_PER_REV);
  // digitalWrite(PC13, HIGH);
  // Serial.println("Down");
  // delay(100); // Pause for a second
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
