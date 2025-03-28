#include <Arduino.h>
#include <Wire.h>

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


// AS5600 I2C address
#define AS5600_I2C_ADDR 0x36
#define RAW_ANGLE_HIGH  0x0C
#define RAW_ANGLE_LOW   0x0D

// Timing variables
unsigned long previousMillis = 0;
const unsigned long interval = 1; // Interval in milliseconds

int encoderValue, inputValue, thetaValue;
int angleDifference = 0;
int angleValue = 0;
int leftLimit = 1950;
int rightLimit = 1900;
int lowTheta = -255; 
int highTheta = 255;
int pwmValue; 
int sendFlag = 0; 

int home = 0;
int thetaTarget = home; // Target angle (0–360 degrees)
int previousTarget = home; 


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

}

void loop() {
  // Periodic AS5600 angle reading

  // unsigned long currentMillis = millis();
  // if (currentMillis - previousMillis >= interval) {
  //   previousMillis = currentMillis;

  //   // Read and print the AS5600 angle
  //   encoderValue = readAS5600Angle();
  //   Serial.print("AS5600 Angle: ");
  //   Serial.println(encoderValue);
  // }

  readSerial();
  
  
  if(thetaTarget > 1){
    pwmValue = thetaTarget;
    moveMotor(pwmValue, RIGHT);
  }else if (thetaTarget < -1){
    pwmValue = abs(thetaTarget);
    moveMotor(pwmValue, LEFT);
  }else{
    pwmValue = 0;
    moveMotor(pwmValue, RIGHT);
  }

  // if(sendFlag == 1){
  //   Serial.print("Input: ");
  //   Serial.print(thetaTarget);
  //   Serial.print("\n");
  //   Serial.print("PWM Value: ");
  //   Serial.print(pwmValue);
  //   Serial.print("\n");
  //   sendFlag = 0;
  // }
  
   delay(100); // Remove after testing
}

void readSerial() {
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    
    if (receivedChar == '\n') { // End of input
      inputComplete = true;
      break;
    } else if (isDigit(receivedChar) || receivedChar == '-') {
      inputString += receivedChar; // Append to the input string
    } else if (receivedChar == '\r') {
      // Do nothing (explicitly ignoring '\r')
    } else if (receivedChar == 'r'){
      inputString = "";
      inputString += 'r';
    }
    }
  

  // Process the input if complete
  if (inputComplete) {
    if(inputString == "r"){
      Serial.print(thetaTarget);
      Serial.print("\n");
    }else{
      int parsedValue = inputString.toInt(); // Convert string to integer
      thetaTarget = constrain(parsedValue, -255, 255); // Constrain to valid range
      if (previousTarget != thetaTarget) {
        Serial.print("Theta Target: ");
        Serial.print(thetaTarget);
        Serial.print("\r\n");
      }
      previousTarget = thetaTarget;
    }

    // Reset input handling
    inputString = "";
    inputComplete = false;
    sendFlag = 0; 
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
