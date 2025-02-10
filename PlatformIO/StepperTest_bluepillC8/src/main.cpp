#include <Arduino.h>

#define STEP_PIN PA0
#define DIR_PIN PA1
#define LED_PIN PC13  // Onboard LED to indicate activity
#define LIMIT_SWITCH_PIN PC14  // Limit switch pin
#define FORWARD 1 // Away from limit switch
#define REVERSE 0 // Toward limit switch 
#define SYNC_BYTE 0xAA  // Sync byte to mark the beginning of each data packet

// Control parameters
const float targetDistance = 0.3; // Target distance (meters)
float currentDistance = 0.3;      // Current distance (received via UART)
float tolerance = 0.02;
const int maxSteps = 8000;         // Max step count limit (adjust as needed)
int stepCount = 0;                 // Tracks the number of steps taken

// Serial Inputs
char inputBuffer[4];  // Buffer to hold incoming data

// Stepper motor control parameters
const int stepDelay = 200;  // Delay between steps (microseconds)
const int stepsPerRev = 5;

void rotateMotor(bool clockwise, int steps);
void homingRoutine();
void stopMotor();
void readUARTData();

unsigned long previousMillis = 0;  // For non-blocking delay

void setup() {
  Serial.begin(115200);
  while (!Serial) { ; }  // Wait for Serial to be ready

  pinMode(STEP_PIN, OUTPUT);  // Step pin as output
  pinMode(DIR_PIN, OUTPUT);   // Direction pin as output
  pinMode(LED_PIN, OUTPUT);   // LED pin as output
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);  // Limit switch as input with internal pull-up

  Serial.println("Manual Stepper Control Started");

  // Perform homing routine
  homingRoutine();
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Read UART data every 100ms without blocking motor control
  if (currentMillis - previousMillis >= 100) {
    previousMillis = currentMillis;
    readUARTData();
  }

  // Decide direction based on currentDistance and limit checks
  if (currentDistance < targetDistance - tolerance && stepCount < maxSteps) {
    // Move stepper forward (increase distance)
    rotateMotor(REVERSE, stepsPerRev);
  } else if (currentDistance > targetDistance + tolerance) {
    // Move stepper backward (decrease distance)
    rotateMotor(FORWARD, stepsPerRev);
  }

  // Check the limit switch at each step
  if (digitalRead(LIMIT_SWITCH_PIN) == HIGH){
      stopMotor();
      return;
  }
}

void readUARTData() {
  if (Serial.available() >= 5) {  // 1 byte sync + 4 bytes data = 5 bytes
    byte syncByte = Serial.read();  // Read the sync byte

    // Debugging: print sync byte received
    Serial.print("Sync Byte: ");
    Serial.println(syncByte, HEX);

    // If sync byte matches, read 4 more bytes for the float data
    if (syncByte == SYNC_BYTE) {
      for (int i = 0; i < 4; i++) {
        inputBuffer[i] = Serial.read();
      }

      // Convert byte array to float
      memcpy(&currentDistance, inputBuffer, sizeof(float));

      // Debugging: print received distance value
      Serial.print("Received Distance: ");
      Serial.println(currentDistance, 4);

      // Toggle LED to indicate data reception
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    } else {
      // If the sync byte doesn't match, flush any leftover data
      Serial.flush();  // Discard any invalid bytes in the buffer
    }
  }
}

void rotateMotor(bool clockwise, int steps) {
  if(!clockwise){
    digitalWrite(DIR_PIN, FORWARD);
    for (int i = 0; i < steps; i++) {
      if(stepCount > maxSteps){
        stopMotor();
        return;
      }
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(stepDelay);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(stepDelay);
      stepCount++;  // Increment step count after each step
    }
  } else {
    digitalWrite(DIR_PIN, REVERSE);
    for (int i = 0; i < steps; i++) {
      if(digitalRead(LIMIT_SWITCH_PIN) == HIGH){
        stopMotor();
        return;
      }

      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(stepDelay);
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(stepDelay);
      stepCount--;  // Increment step count after each step
    }
  }
}

void homingRoutine() {
  Serial.println("Starting Homing Routine...");
  digitalWrite(DIR_PIN, REVERSE);

  while (digitalRead(LIMIT_SWITCH_PIN) == LOW) {  // While the limit switch is not triggered
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelay);
  }

  Serial.println("Homing Complete!");
}

void stopMotor() {
  digitalWrite(DIR_PIN, LOW);  // Set direction to zero or stop
  digitalWrite(STEP_PIN, LOW); // Ensure no more steps are sent
}
