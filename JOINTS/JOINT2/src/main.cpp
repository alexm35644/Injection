#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>
#include <stm32f1xx_hal.h>

// Uncomment the line below to enable debugging
// #define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_DELAY(x) delay(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_DELAY(x)
#endif

#define LED PC13
#define LEFT 0
#define RIGHT 1
#define PWM_LOWER_LIMIT -500
#define PWM_UPPER_LIMIT  500
#define TOLERANCE 1


// String
String inputString = ""; // To store the serial input
bool inputComplete = false;

// Motor connections
const int in1 = PB0;
const int in2 = PB1;

// PID parameters
double Pk1 = 2;  // Speed it gets there
double Ik1 = 0;
double Dk1 = 0.05;
double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, Pk1, Ik1, Dk1, DIRECT);
int error = 0;

// AS5600 I2C address
#define AS5600_I2C_ADDRESS (0x36 << 1) // AS5600 I2C address
#define AS5600_RAW_ANGLE_REGISTER 0x0C // Register for raw angle
#define RAW_ANGLE_HIGH  0x0C
#define RAW_ANGLE_LOW   0x0D

// Timing variables
unsigned long previousMillis = 0;
const unsigned long interval = 50; // Interval in milliseconds

int encoderValue, inputValue, thetaValue;
int angleDifference = 0;
int angleValue = 0;
int rightLimit = 3100; // 33 degrees
int leftLimit = 1767;  // 150 degrees
int pwmValue; 

int home = 45;
int thetaTarget = home; // Target angle (0–150 degrees)
int previousTarget = home; 


uint16_t AS5600_ReadRawAngle();
void moveMotor(int pwmValue, bool direction);
void readSerial();
void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim);
void I2C_Init();
bool I2C_CheckError();
void TIM3_INIT();

// Timer handle
TIM_HandleTypeDef htim3;

// I2C handle
I2C_HandleTypeDef hi2c1;



void setup() {
  // Set motor pins as outputs
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);

  // Start serial communication
  Serial.begin(115200);

  // Initialize motor to off
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);

  // Initialize the HAL library
  HAL_Init();

  // Initialize I2C communication
  I2C_Init(); // Initialize I2C
  // Serial.print("I2C Initialized");
  // Serial.print('\n');
  TIM3_INIT(); // Initialize TIM3 for PWM


  


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
    encoderValue = AS5600_ReadRawAngle();
    Serial.print("AS5600 Angle: ");
    Serial.println(encoderValue);
  }

  readSerial();

  Setpoint = thetaTarget;
  
  // Map the encoder value to a range of 0 to 150
  Input = map(encoderValue, rightLimit, leftLimit, 33, 150);
  // Run PID process to get Output value
  myPID.Compute();
  // Move the motor based on PID output
  DEBUG_PRINT("target:");
  DEBUG_PRINTLN(map(thetaTarget, 33, 150, rightLimit, leftLimit));
  DEBUG_PRINT("PID Output: ");
  DEBUG_PRINTLN(Output);
  if(abs(thetaTarget - Input) > TOLERANCE){
    if (Output > 1) { // Move right
      if(encoderValue < rightLimit+200){
          pwmValue = Output;
          pwmValue = constrain(pwmValue, 0, PWM_UPPER_LIMIT);
          moveMotor(pwmValue, RIGHT);       
      }else{
        pwmValue = 0;
        moveMotor(pwmValue, RIGHT);
      }
    } else if (Output < -1) { // Move left
      if(encoderValue > leftLimit-200){
          pwmValue = abs(Output);
          pwmValue = constrain(pwmValue, 0, PWM_UPPER_LIMIT);
          moveMotor(pwmValue, LEFT);
      }else{
        pwmValue = 0;
        moveMotor(pwmValue, LEFT);
      }
    } else { // Stop the motor
        pwmValue = 0;
        moveMotor(pwmValue, RIGHT);
    }
  } else{
    pwmValue = 0;
    moveMotor(pwmValue, RIGHT);
  }
  
  DEBUG_PRINT("PWM Value: ");
  DEBUG_PRINTLN(pwmValue);
  
  DEBUG_DELAY(50); // Remove after testing
}

void moveMotor(int pwmValue, bool direction) {
  if (pwmValue == 0) {
    // Stop the motor
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
  } else if (direction == LEFT) {
    // Rotate left using TIM3_CH4
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, pwmValue);
  } else if (direction == RIGHT) {
    // Rotate right using TIM3_CH3
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, pwmValue);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
  }
}


void readSerial() {
  while (Serial.available() > 0) {
    char receivedChar = Serial.read();
    if (receivedChar == '\n') {
      inputComplete = true;
      break;
    } else if (isDigit(receivedChar) || receivedChar == '-') {
      inputString += receivedChar;
    }
  }

  if (inputComplete) {
    int parsedValue = inputString.toInt();
    thetaTarget = constrain(parsedValue, 0, 150);
    inputString = "";
    inputComplete = false;
    DEBUG_PRINT("Parsed Value: ");
    DEBUG_PRINT(thetaTarget);
    DEBUG_PRINT("\n");
  }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM3) {
    // Enable TIM3 clock
    __HAL_RCC_TIM3_CLK_ENABLE();
    
    // Enable GPIOB clock for PWM pins (PB0 and PB1 for TIM3_CH3 and TIM3_CH4)
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    
    // Configure GPIO pin for TIM3_CH3 (PB0)
    GPIO_InitStruct.Pin = GPIO_PIN_0;  // PB0 (TIM3_CH3)
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;  // Alternate function push-pull
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    // Configure GPIO pin for TIM3_CH4 (PB1)
    GPIO_InitStruct.Pin = GPIO_PIN_1;  // PB1 (TIM3_CH4)
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
}

void TIM3_INIT() {
  // Configure TIM3 for PWM generation
  __HAL_RCC_TIM3_CLK_ENABLE();  // Enable TIM3 clock
  
  // Timer configuration
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 72 - 1;  // Prescaler for 1 MHz timer clock (72 MHz / 72)
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000 - 1;  // Period for 1 kHz PWM frequency
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_PWM_Init(&htim3);
  
  // Configure PWM channels
  TIM_OC_InitTypeDef sConfigOC = {0};
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;  // Start with 0% duty cycle
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  
  // TIM3_CH3 (PB0)
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3);
  // TIM3_CH4 (PB1)
  HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4);

  // Start PWM signals
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);

}

// Function to initialize I2C
void I2C_Init() {
  __HAL_RCC_I2C1_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_AFIO_REMAP_I2C1_ENABLE();
  
  // Configure PB8 and PB9 for I2C
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD; // Open-drain mode for I2C
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  // Configure I2C peripheral
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 50000; // Set I2C clock speed to 50kHz
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2; // Standard duty cycle
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  
  // Initialize I2C
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
      // Serial.println("I2C initialization failed!");
      while (1);
  }
}

// Function to check if I2C communication was successful
bool I2C_CheckError() {
  if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_AF)) {
      Serial.println("I2C Acknowledge Failure");
      return true;
  }
  if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BERR)) {
      Serial.println("I2C Bus Error");
      return true;
  }
  if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_ARLO)) {
      Serial.println("I2C Arbitration Lost");
      return true;
  }
  return false;
}

uint16_t AS5600_ReadRawAngle() {
  uint8_t buffer[2];
  uint16_t raw_angle = 0;

  // Print status before each read attempt
  // Serial.println("Attempting I2C read...");

  // Reset the I2C bus after each attempt
  HAL_I2C_DeInit(&hi2c1);
  HAL_I2C_Init(&hi2c1);

  // Try reading raw angle register
  if (HAL_I2C_Mem_Read(&hi2c1, AS5600_I2C_ADDRESS, AS5600_RAW_ANGLE_REGISTER, 
                      I2C_MEMADD_SIZE_8BIT, buffer, 2, HAL_MAX_DELAY) != HAL_OK) {
      // Check for I2C error
      if (I2C_CheckError()) {
          HAL_I2C_DeInit(&hi2c1);
          HAL_I2C_Init(&hi2c1);
      }
  } else {
      raw_angle = (buffer[0] << 8) | buffer[1]; // Combine MSB and LSB
      
  }

  return raw_angle;
}