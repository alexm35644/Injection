#include <Arduino.h>
#include <Wire.h>
#include <stm32f1xx_hal.h>

#define LED PC13
#define LEFT 0
#define RIGHT 1
#define PWM_LOWER_LIMIT -127
#define PWM_UPPER_LIMIT  127
#define TOLERANCE 5

// String
String inputString = ""; // To store the serial input
bool inputComplete = false;

// Motor connections
const int in1 = PB0;
const int in2 = PB1;

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

// Timer handle
TIM_HandleTypeDef htim3;

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3) {
    // Enable TIM3 clock
    __HAL_RCC_TIM3_CLK_ENABLE();
    
    // Enable GPIOB clock for PWM pin (PB0 for TIM3_CH1)
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Configure GPIO pin for PWM output
    GPIO_InitStruct.Pin = GPIO_PIN_0;  // PB0 (TIM3_CH1)
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;  // Alternate function push-pull
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}


void setup() {

  // Set motor pins as outputs
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);



  // Start serial communication
  Serial.begin(115200);

  // Initialize I2C communication
  Wire.begin();

  // Initialize the HAL library
  HAL_Init();

  // Configure TIM3 for PWM generation
  __HAL_RCC_TIM3_CLK_ENABLE();  // Enable TIM3 clock
  
  // Timer configuration
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72 - 1;  // Prescaler for 1 MHz timer clock (72 MHz / 72)
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000 - 1;  // Period for 1 kHz PWM frequency
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim3);
  
  // Configure PWM channel
  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 600;  // 50% duty cycle (half of 1000)
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);  // Channel 1 (PB0)

  // Start PWM signal
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

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
  } else if (direction == LEFT) {
    // Rotate left
    analogWrite(in1, pwmValue);
    analogWrite(in2, LOW);

  } else if (direction == RIGHT) {
    // Rotate right
    analogWrite(in1, LOW);
    analogWrite(in2, pwmValue);
  }
}
