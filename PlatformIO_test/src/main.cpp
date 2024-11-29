#include <Arduino.h>

// put function declarations here:
int myFunction(int, int);

#define LED_PIN PC13  // Onboard LED is connected to PC13

void setup() {
  pinMode(LED_PIN, OUTPUT);  // Set PC13 as an output
}

void loop() {
  digitalWrite(LED_PIN, HIGH); // Turn the LED off (logic is inverted)
  delay(1000);                 // Wait for 1 second
  digitalWrite(LED_PIN, LOW);  // Turn the LED on
  delay(1000);                 // Wait for 1 second
}

// put function definitions here:
int myFunction(int x, int y) {
  return x + y;
}