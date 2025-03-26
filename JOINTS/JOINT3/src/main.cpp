#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>

#define LED 13
#define LEFT 1
#define RIGHT 0
#define PWM_LOWER_LIMIT -255
#define PWM_UPPER_LIMIT  255
#define TOLERANCE 5

// Motor connections
const int enA = 9;
const int in1 = 8;
const int in2 = 7;

// PID parameters
double Pk1 = 5; // Proportional gain
double Ik1 = 0; // Integral gain
double Dk1 = 0.05; // Derivative gain
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, Pk1, Ik1, Dk1, DIRECT);

// AS5600 I2C address
#define AS5600_I2C_ADDR 0x36
#define RAW_ANGLE_HIGH  0x0C

int encoderValue;
int leftLimit = 1400;  // Minimum angle
int rightLimit = 1580; // Maximum angle
int thetaTarget = 90;  // Initial target angle in degrees (0–180)
bool newData = false;
String inputString = "";

// Function prototypes
int readAS5600Angle();
void moveMotor(int pwmValue, bool direction);

void setup() {
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  Serial.begin(115200);
  Wire.begin();

  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(PWM_LOWER_LIMIT, PWM_UPPER_LIMIT);
  myPID.SetSampleTime(20);

  Serial.println("Motor Control Ready");
}

void loop() {
  // Read serial input for target angle
  // Read incoming serial data
  while (Serial.available() > 0) {
    char receivedChar = Serial.read(); // Read a single character
    if (receivedChar == '\n') {        // End of input (newline character)
      newData = true;                  // Mark data as complete
      break;                           // Exit the loop
    } else if (isDigit(receivedChar)) {
      inputString += receivedChar;     // Append the digit to the string
    }
  }

  // Process the completed input
  if (newData) {
    int inputValue = inputString.toInt(); // Convert string to integer
    if (inputValue >= 0 && inputValue <= 180) {
      thetaTarget = inputValue; // Update target angle
      Serial.print("Theta Target: ");
      Serial.println(thetaTarget);
    } else {
      Serial.println("Invalid input; please enter a value between 0 and 180.");
    }
    inputString = "";  // Clear the string for the next input
    newData = false;   // Reset the new data flag
  }

  // Map thetaTarget (0–180) to encoder range (1450–1550)
  int targetAngle = map(thetaTarget, 0, 180, leftLimit, rightLimit);

  // Read encoder value
  encoderValue = readAS5600Angle();

  // Set PID Input and Setpoint
  Input = encoderValue;
  Setpoint = targetAngle;

  // Compute PID
  myPID.Compute();

  double pwmValue = abs(Output);
  pwmValue = constrain(pwmValue, 80, PWM_UPPER_LIMIT);
  // Move motor based on PID output
  if (abs(Input - Setpoint) <= TOLERANCE) {
    // Within tolerance, stop the motor
    moveMotor(0, LEFT);
  } else if (Output > 0) {
    // Move right
    moveMotor(pwmValue, RIGHT);
  } else if (Output < 0) {
    // Move left
    moveMotor(abs(pwmValue), LEFT);
  }

  // Debugging
  Serial.print("Encoder: ");
  Serial.print(encoderValue);
  Serial.print(" Target: ");
  Serial.print(targetAngle);
  Serial.print(" Output: ");
  Serial.println(Output);
}

int readAS5600Angle() {
  int rawAngle = -1;

  Wire.beginTransmission(AS5600_I2C_ADDR);
  Wire.write(RAW_ANGLE_HIGH);
  if (Wire.endTransmission(false) != 0) {
    Serial.println("Error: I2C transmission failed");
    return rawAngle;
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
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(enA, 0);
  } else if (direction == LEFT) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(enA, pwmValue);
  } else if (direction == RIGHT) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(enA, pwmValue);
  }
}
