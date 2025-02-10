#include <Arduino.h>

#define STEP_PIN PA0
#define DIR_PIN PA1
#define LED_PIN PC13  // Onboard LED to indicate activity
#define LIMIT_SWITCH_PIN PC14  // Limit switch pin
#define FORWARD 1 // Away from limit switch
#define REVERSE 0 // Toward limit switch 

// Control parameters
const float targetDistance = 0.3; // Target distance (meters)
float currentDistance = 0.3;      // Current distance (received via UART)
float tolerance = 0.02;
const int maxSteps = 8000;         // Max step count limit (adjust as needed)
int stepCount = 0;                 // Tracks the number of steps taken

// Serial Inputs
char inputBuffer[20];  // Buffer to hold incoming data
int bufferIndex = 0;   // Index for buffer

// Stepper motor control parameters
const int stepDelay = 1000;  // Delay between steps (microseconds)
const int stepsPerRev = 500;

void rotateMotor(bool clockwise, int steps);
void homingRoutine();
void stopMotor();
void readingUART();

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);
  while (!Serial) { ; }  // Wait for Serial to be ready

  pinMode(STEP_PIN, OUTPUT);  // Step pin as output
  pinMode(DIR_PIN, OUTPUT);   // Direction pin as output
  pinMode(LED_PIN, OUTPUT);   // LED pin as output
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);  // Limit switch as input with internal pull-up

  Serial.println("Manual Stepper Control Started");

  // Perform homing routine
  homingRoutine();

  // digitalWrite(DIR_PIN, FORWARD);
  // for (int i = 0; i < 1000; i++) {
  //   digitalWrite(STEP_PIN, HIGH);
  //   delayMicroseconds(stepDelay); // Step pulse duration
  //   digitalWrite(STEP_PIN, LOW);
  //   delayMicroseconds(stepDelay); // Step pulse interval
  //   stepCount++;  // Increment step count after each step
  // }
}

void loop() {
  if (Serial.available() > 0) {
    String input = Serial.readString();
    
    // Try to convert the input to a float
    currentDistance = input.toFloat();

    // Blink the LED to indicate data reception
    digitalWrite(LED_PIN, HIGH);
    digitalWrite(LED_PIN, LOW);
    
    // Print the received distance
    Serial.print("Received Distance: ");
    Serial.println(currentDistance);
  }

  // Decide direction based on currentDistance and limit checks
  if (currentDistance < targetDistance - tolerance && stepCount < maxSteps) {
    // Move stepper forward (increase distance)
    rotateMotor(REVERSE, stepsPerRev);
  } else if (currentDistance > targetDistance + tolerance) {
    // Move stepper backward (decrease distance)
    rotateMotor(FORWARD, stepsPerRev);
  }

  if(digitalRead(LIMIT_SWITCH_PIN) == HIGH){
      digitalWrite(DIR_PIN, LOW);  // Set direction to zero or stop
      digitalWrite(STEP_PIN, LOW); // Ensure no more steps are sent
      stepCount = 0;
      return;
    }

}

void rotateMotor(bool clockwise, int steps) {
  // Set direction
  if(!clockwise){
    // Perform steps
  digitalWrite(DIR_PIN, FORWARD);
  for (int i = 0; i < steps; i++) {
    if(stepCount>maxSteps){
      digitalWrite(DIR_PIN, LOW);  // Set direction to zero or stop
      digitalWrite(STEP_PIN, LOW); // Ensure no more steps are sent
      //Serial.println("max step count reached");
      return;
    }
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(stepDelay); // Step pulse duration
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelay); // Step pulse interval
    stepCount++;  // Increment step count after each step
  }
  } else{
  digitalWrite(DIR_PIN, REVERSE);
  
  for (int i = 0; i < steps; i++) {
    if(digitalRead(LIMIT_SWITCH_PIN) == HIGH){
      digitalWrite(DIR_PIN, LOW);  // Set direction to zero or stop
      digitalWrite(STEP_PIN, LOW); // Ensure no more steps are sent
      stepCount = 0;
      //Serial.println("lim switch triggered");
      return;
    } 

    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(stepDelay); // Step pulse duration
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelay); // Step pulse interval
    stepCount--;  // Increment step count after each step
  }
  }

  
}

void homingRoutine() {
  Serial.println("Starting Homing Routine...");

  // Set direction for homing (e.g., REVERSE)
  digitalWrite(DIR_PIN, REVERSE);

  while (digitalRead(LIMIT_SWITCH_PIN) == LOW) {  // While the limit switch is not triggered (NC logic)
    // Step the motor
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(stepDelay);
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(stepDelay);
  }

  Serial.println("Homing Complete!");
}

void stopMotor() {
  // Stop the motor (You can add logic to safely stop the motor, if needed)
  digitalWrite(DIR_PIN, LOW);  // Set direction to zero or stop
  digitalWrite(STEP_PIN, LOW); // Ensure no more steps are sent
}


