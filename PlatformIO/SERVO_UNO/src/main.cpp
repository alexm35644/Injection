#include <Arduino.h>

#define LED 13

// Motor connections
int enA = 9;
int in1 = 8;
int in2 = 7;

void setup() {
  // Set motor pins as outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Start serial communication
  Serial.begin(115200);

  // Initialize motor to off
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  analogWrite(enA, 0);
}

void loop() {
  // Check if data is available on the serial port
  if (Serial.available() > 0) {
    // Read the input as a string
    String input = Serial.readStringUntil('\n');

    // Convert input to an integer
    int pwmValue = input.toInt();

    // Constrain the value between -255 and 255
    pwmValue = constrain(pwmValue, -255, 255);

    if (pwmValue > 0) {
      // Forward direction
      digitalWrite(in1, HIGH);
      digitalWrite(in2, LOW);
      analogWrite(enA, pwmValue);
    } else if (pwmValue < 0) {
      // Reverse direction
      digitalWrite(in1, LOW);
      digitalWrite(in2, HIGH);
      analogWrite(enA, abs(pwmValue));
    } else {
      // Stop the motor
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      analogWrite(enA, 0);
    }

    // Print the received value (for debugging purposes)
    Serial.print("PWM Value: ");
    Serial.println(pwmValue);
  }
}
