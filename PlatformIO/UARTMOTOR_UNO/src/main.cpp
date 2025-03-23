#include <Arduino.h>
#include <Wire.h>

#define LED 13

// Motor connections
int enA = 9;
int in1 = 8;
int in2 = 7;

// AS5600 I2C address
#define AS5600_I2C_ADDR 0x36
#define RAW_ANGLE_HIGH  0x0C
#define RAW_ANGLE_LOW   0x0D

// Timing variables
unsigned long previousMillis = 0;
const unsigned long interval = 100; // Interval in milliseconds

int readAS5600Angle();

void setup() {
  // Set motor pins as outputs
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Start serial communication
  Serial.begin(115200);

  // Initialize I2C communication
  Wire.begin();

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

  // Use millis() to control the reading and printing of the AS5600 angle
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Read and print the AS5600 angle
    int angle = readAS5600Angle();
    Serial.print("AS5600 Angle: ");
    Serial.println(angle);
  }
}

int readAS5600Angle() {
  int rawAngle = 0;

  // Request RAW_ANGLE register values
  Wire.beginTransmission(AS5600_I2C_ADDR);
  Wire.write(RAW_ANGLE_HIGH); // Starting with high byte
  Wire.endTransmission(false);
  Wire.requestFrom(AS5600_I2C_ADDR, 2); // Request 2 bytes

  if (Wire.available() >= 2) {
    rawAngle = Wire.read() << 8; // Read high byte
    rawAngle |= Wire.read();    // Read low byte
  }

  return rawAngle;
}
