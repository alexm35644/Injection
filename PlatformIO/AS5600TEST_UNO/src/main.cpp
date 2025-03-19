#include <Wire.h>
#include <Arduino.h>

#define AS5600_ADDRESS 0x36     // I2C address for AS5600
#define RAW_ANGLE_HIGH 0x0C     // Register for high byte of raw angle
#define RAW_ANGLE_LOW  0x0D     // Register for low byte of raw angle

// TMC2209 Pins
const int DIR_PIN = 8;           // Direction pin
const int STEP_PIN = 9;          // Step pin
const int ENABLE_PIN = 10;       // Enable pin

const int TARGET_POSITION = 2048; // Target position (0-4095)

// Function Prototypes
int readAS5600Position();
void moveToPosition(int error);

void setup() {
  Serial.begin(115200);           // Start serial communication at 9600 baud
  Wire.begin();                 // Initialize I2C communication
  
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(ENABLE_PIN, OUTPUT);


}
int error; 

void loop() {
  int position = readAS5600Position();
  Serial.print("Position: ");
  Serial.println(position);
  delay(100);                   // Small delay between readings
  error = TARGET_POSITION - position; 
  moveToPosition(error);

}

int readAS5600Position() {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(RAW_ANGLE_HIGH);   // Request high byte of raw angle
  Wire.endTransmission();
  Wire.requestFrom(AS5600_ADDRESS, 2); // Request 2 bytes

  if (Wire.available() >= 2) {
    int highByte = Wire.read();
    int lowByte = Wire.read();
    int position = (highByte << 8) | lowByte; // Combine bytes
    return position;
  } else {
    return -1;                  // Return -1 if no data is available
  }
}

void moveToPosition(int error){
  int steps = abs(error);


 if (error > 0) {
    digitalWrite(DIR_PIN, HIGH);  // Set direction towards the target
  } else if (error < 0) {
    digitalWrite(DIR_PIN, LOW);   // Set direction away from the target
  }

    for (int i = 0; i < steps; i++) {
    digitalWrite(STEP_PIN, HIGH);
    delayMicroseconds(500);       // Control the step speed; adjust as needed
    digitalWrite(STEP_PIN, LOW);
    delayMicroseconds(500);
  }

}
