#include <Arduino.h>

#define LIMIT_SWITCH_PIN PC14  // Pin connected to the limit switch (Normally Closed)
#define LED_PIN PC13           // Pin for the onboard LED

void setup() {
  Serial.begin(115200);

  // Set up the limit switch pin with an external pull-up resistor (input without internal pull-up)
  pinMode(LIMIT_SWITCH_PIN, INPUT);

  // Set the LED pin as an output
  pinMode(LED_PIN, OUTPUT);

  // Ensure the LED is initially OFF
  digitalWrite(LED_PIN, LOW);

  Serial.println("Limit Switch Toggle LED Test Started");
}

void loop() {
  // Read the state of the limit switch
  int switchState = digitalRead(LIMIT_SWITCH_PIN);

  // If the limit switch is pressed (PC14 connected to 3.3V due to NC switch)
  if (switchState == HIGH) {
    // Toggle the LED
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));  // Toggle LED state

    // Print to Serial when toggling occurs
    Serial.println("Limit switch pressed! Toggling LED.");
  }

  delay(100);  // Delay to make loop less sensitive
}
