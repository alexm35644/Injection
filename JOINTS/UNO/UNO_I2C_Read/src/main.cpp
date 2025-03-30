#include <Wire.h>
#include <Arduino.h>

// AS5600 I2C Address
#define AS5600_I2C_ADDR 0x36
#define RAW_ANGLE_HIGH  0x0C  // High byte of angle data
#define RAW_ANGLE_LOW   0x0D  // Low byte of angle data

int readAS5600Angle();

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Initialize I2C communication
  Wire.begin();
}

void loop() {
  int rawAngle = readAS5600Angle();
  
  if (rawAngle != -1) {
    Serial.print("Angle: ");
    Serial.println(rawAngle);
  } else {
    Serial.println("Error reading AS5600!");
  }

  delay(100);  // Delay for 100ms before reading again
}

int readAS5600Angle() {
  int rawAngle = -1;

  // Request data from AS5600
  Wire.beginTransmission(AS5600_I2C_ADDR);
  Wire.write(RAW_ANGLE_HIGH);  // Send the high byte register address
  if (Wire.endTransmission(false) != 0) {
    Serial.println("Error: I2C transmission failed");
    return rawAngle;
  }

  Wire.requestFrom(AS5600_I2C_ADDR, 2);  // Request 2 bytes (high and low byte of the angle)
  if (Wire.available() >= 2) {
    // Read the high and low byte
    int highByte = Wire.read();
    int lowByte = Wire.read();
    
    // Combine high and low bytes to form the 12-bit angle value
    rawAngle = (highByte << 8) | lowByte;
  } else {
    Serial.println("Error: I2C read failed");
  }

  return rawAngle;
}
