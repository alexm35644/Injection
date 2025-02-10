#include <Arduino.h>

#define LED_PIN1 PD12
#define LED_PIN2 PD13
#define LED_PIN3 PD14
#define LED_PIN4 PD15

void setup() {
  // Configure the LED pins as outputs
  pinMode(LED_PIN1, OUTPUT);
  pinMode(LED_PIN2, OUTPUT);
  pinMode(LED_PIN3, OUTPUT);
  pinMode(LED_PIN4, OUTPUT);
}

void loop() {
  // Toggle the LED pins in sequence
  digitalWrite(LED_PIN1, !digitalRead(LED_PIN1));
  delay(100); // Wait for 100 milliseconds

  digitalWrite(LED_PIN2, !digitalRead(LED_PIN2));
  delay(100); // Wait for 100 milliseconds

  digitalWrite(LED_PIN3, !digitalRead(LED_PIN3));
  delay(100); // Wait for 100 milliseconds

  digitalWrite(LED_PIN4, !digitalRead(LED_PIN4));
  delay(100); // Wait for 100 milliseconds
}
