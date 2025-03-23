#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>

#define LED 13
#define LEFT 0
#define RIGHT 1
#define PWM_LOWER_LIMIT -255
#define PWM_UPPER_LIMIT  255
#define TOLERANCE 5



// Motor connections
const int enA = 9;
const int in1 = 8;
const int in2 = 7;

// PID parameters
double Pk1 = 5;  // Speed it gets there
double Ik1 = 0;
double Dk1 = 0.05;
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, Pk1, Ik1, Dk1, DIRECT);
int error = 0;

// AS5600 I2C address
#define AS5600_I2C_ADDR 0x36
#define RAW_ANGLE_HIGH  0x0C
#define RAW_ANGLE_LOW   0x0D

// Timing variables
unsigned long previousMillis = 0;
const unsigned long interval = 100; // Interval in milliseconds

int encoderValue, inputValue, thetaValue;
int centerAngle = 2600; // 180 degrees
int angleDifference = 0;
int angleValue = 0;
int leftLimit = 3600;
int rightLimit = 850;
int pwmValue; 

int thetaTarget = 180; // Target angle (0–360 degrees)
int previousTarget = 180; 

// Quadrant stuff 
int numberOfTurns = 0;
float totalAngle = 0;

int readAS5600Angle();
void moveMotor(int pwmValue, bool direction);

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

  // PID Setup
  myPID.SetMode(AUTOMATIC);              
  myPID.SetOutputLimits(PWM_LOWER_LIMIT, PWM_UPPER_LIMIT);
  myPID.SetSampleTime(20);
}

void loop() {
  // Periodic AS5600 angle reading
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Read and print the AS5600 angle
    encoderValue = readAS5600Angle();
    Serial.print("AS5600 Angle: ");
    Serial.println(encoderValue);
  }

  // Read serial input for target angle
  if (Serial.available() > 0) {
    thetaTarget = Serial.parseInt(); // Parse integer directly
    thetaTarget = constrain(thetaTarget, 0, 360); // Constrain to valid range
    if (previousTarget != thetaTarget){
      Serial.print("Theta Target: ");
      Serial.println(thetaTarget);
    }
    previousTarget = thetaTarget; 
  }

  // Calculate limits based on center angle and offset

  // Adjust the angle value with wrap-around handling
  angleValue = (encoderValue + angleDifference) % 4096;
  if (angleValue < 0) {
    angleValue += 4096; // Handle negative wrap
  }

  // Map the target angle (0–360) to PID Setpoint range (-255 to 255)
  Setpoint = map(thetaTarget, 0, 360, PWM_LOWER_LIMIT, PWM_UPPER_LIMIT);

  // Map the current angle value to the PID Input range (-255 to 255)
  Input = map(angleValue, leftLimit, rightLimit, PWM_LOWER_LIMIT, PWM_UPPER_LIMIT);

  // Adjust PID parameters dynamically (optional)
  myPID.SetTunings(Pk1, Ik1, Dk1);

  // Run PID process to get Output value
  myPID.Compute();

  // Move the motor based on PID output
  if (Output > 1) { // Move right
    pwmValue = Output;
    pwmValue = constrain(pwmValue, 0, PWM_UPPER_LIMIT);
    moveMotor(pwmValue, RIGHT);
  } else if (Output < -1) { // Move left
    pwmValue = abs(Output);
    pwmValue = constrain(pwmValue, 0, PWM_UPPER_LIMIT);
    moveMotor(pwmValue, LEFT);
  } else { // Stop the motor
    moveMotor(0, LEFT);
  }
}

int readAS5600Angle() {
  int rawAngle = -1;

  // Request RAW_ANGLE register values
  Wire.beginTransmission(AS5600_I2C_ADDR);
  Wire.write(RAW_ANGLE_HIGH);
  if (Wire.endTransmission(false) != 0) {
    Serial.println("Error: I2C transmission failed");
    return rawAngle; // Return -1 on failure
  }

  Wire.requestFrom(AS5600_I2C_ADDR, 2);
  if (Wire.available() >= 2) {
    rawAngle = (Wire.read() << 8) | Wire.read();
  } else {
    Serial.println("Error: I2C read failed");
  }

  return rawAngle;
}

void moveMotor(int pwmValue, bool direction) {
  if (pwmValue == 0) {
    // Stop the motor
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enA, 0);
  } else if (direction == LEFT) {
    // Rotate left
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, pwmValue);
  } else if (direction == RIGHT) {
    // Rotate right
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, pwmValue);
  }
}
