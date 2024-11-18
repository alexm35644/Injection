#include <Wire.h>

#define AS5600_ADDRESS 0x36       // I2C address for AS5600
#define RAW_ANGLE_HIGH 0x0C       // Register for high byte of raw angle
#define RAW_ANGLE_LOW  0x0D       // Register for low byte of raw angle

// TMC2209 Pins
const int DIR_PIN = 8;           // Direction pin
const int STEP_PIN = 9;          // Step pin
const int ENABLE_PIN = 10;       // Enable pin

// Control Parameters
const int TARGET_POSITION = 500;  // Target position (0-4095)
const int TOLERANCE = 20;          // Position tolerance

void setup() {
  Serial.begin(9600);             // Start serial communication
  Wire.begin();                   // Initialize I2C communication

  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);

  digitalWrite(ENABLE_PIN, LOW);  // Enable the TMC2209 driver
}

void loop() {
  int currentPosition = readAS5600Position();
  int error = TARGET_POSITION - currentPosition;

  Serial.print("Current Position: ");
  Serial.print(currentPosition);
  Serial.print(" | Target Position: ");
  Serial.print(TARGET_POSITION);
  Serial.print(" | Error: ");
  Serial.println(error);

  // Only move if the error is outside the tolerance range
  if (abs(error) > TOLERANCE) {
    // Set the direction based on the error
    if (error > 0) {
      digitalWrite(DIR_PIN, LOW);
    } else {
      digitalWrite(DIR_PIN, HIGH);
    }

    // Keep pulsing steps until within tolerance
    while (abs(error) > TOLERANCE) {
      digitalWrite(STEP_PIN, HIGH);
      delayMicroseconds(20);  // Reduced delay for faster speed
      digitalWrite(STEP_PIN, LOW);
      delayMicroseconds(20);

      // Update the current position and error after each step
      currentPosition = readAS5600Position();
      error = TARGET_POSITION - currentPosition;
    }
    Serial.println("Position within tolerance, motor stopped.");
  }
}

int readAS5600Position() {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(RAW_ANGLE_HIGH);     // Request high byte of raw angle
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDRESS, 2);

  if (Wire.available() >= 2) {
    int highByte = Wire.read();
    int lowByte = Wire.read();
    int position = (highByte << 8) | lowByte;
    return position;
  } else {
    return -1; // Return -1 if reading fails
  }
}
