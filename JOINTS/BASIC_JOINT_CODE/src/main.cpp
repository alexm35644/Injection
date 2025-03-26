#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>

#define LED 13
#define LEFT 0
#define RIGHT 1
#define PWM_LOWER_LIMIT -127
#define PWM_UPPER_LIMIT  127
#define TOLERANCE 5


// String
String inputString = ""; // To store the serial input
bool inputComplete = false;

// Motor connections
const int enA = 9;
const int in1 = 8;
const int in2 = 7;

// PID parameters
double Pk1 = 2;  // Speed it gets there
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
const unsigned long interval = 50; // Interval in milliseconds

int encoderValue, inputValue, thetaValue;
int homeAngle = 2700; // 90 deg angle straight up
int angleDifference = 0;
int angleValue = 0;
int leftLimit = 3390;
int rightLimit = 2000;
int pwmValue; 

int thetaTarget = 90; // Target angle (0–360 degrees)
int previousTarget = 90; 


int readAS5600Angle();
void moveMotor(int pwmValue, bool direction);
void readSerial();

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

  readSerial();

  


  Setpoint = thetaTarget;
  // Map the current angle value to the PID Input range (-255 to 255)
  Input = map(encoderValue, rightLimit, leftLimit, 0, 180);
  // Run PID process to get Output value
  myPID.Compute();
  // Move the motor based on PID output
  Serial.print("target:");
  Serial.println(thetaTarget);
  Serial.print("PID Output: ");
  Serial.println(Output);
  if (Output > 1) { // Move right
    if(encoderValue > rightLimit-200){
      pwmValue = Output;
      pwmValue = constrain(pwmValue, 60, PWM_UPPER_LIMIT);
      moveMotor(pwmValue, RIGHT);
    }else{
      pwmValue = 0;
      moveMotor(pwmValue, RIGHT);
    }
  } else if (Output < -1) { // Move left
    if(encoderValue < leftLimit+200){
      pwmValue = abs(Output);
      pwmValue = constrain(pwmValue, 60, PWM_UPPER_LIMIT);
      moveMotor(pwmValue, LEFT);
    }else{
      pwmValue = 0;
      moveMotor(pwmValue, LEFT);
    }
  } else { // Stop the motor
      pwmValue = 0;
      moveMotor(pwmValue, RIGHT);
  }
  Serial.print("PWM Value: ");
  Serial.println(pwmValue);
  
  delay(50);
}

void readSerial() {
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    
    if (receivedChar == '\n') { // End of input
      inputComplete = true;
      break;
    } else if (isDigit(receivedChar) || receivedChar == '-') {
      inputString += receivedChar; // Append to the input string
    }
  }

  // Process the input if complete
  if (inputComplete) {
    int parsedValue = inputString.toInt(); // Convert string to integer
    thetaTarget = constrain(parsedValue, 0, 180); // Constrain to valid range
    if (previousTarget != thetaTarget) {
      Serial.print("Theta Target: ");
      Serial.println(thetaTarget);
    }
    previousTarget = thetaTarget;

    // Reset input handling
    inputString = "";
    inputComplete = false;
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
