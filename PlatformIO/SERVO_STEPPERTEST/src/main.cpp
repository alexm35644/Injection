#include <Arduino.h>
#include <stm32f1xx_hal.h>

#define DEBUG

#ifdef DEBUG
  #define DEBUG_PRINT(x) Serial.print(x)
  #define DEBUG_PRINTLN(x) Serial.println(x)
  #define DEBUG_DELAY(x) delay(x)
#else
  #define DEBUG_PRINT(x)
  #define DEBUG_PRINTLN(x)
  #define DEBUG_DELAY(x)
#endif

#define STEP_PIN PB1
#define DIR_PIN PB0
#define EN_PIN PA2

// AS5600 I2C address
#define AS5600_I2C_ADDRESS (0x36 << 1) // AS5600 I2C address
#define AS5600_RAW_ANGLE_REGISTER 0x0C // Register for raw angle
#define RAW_ANGLE_HIGH  0x0C
#define RAW_ANGLE_LOW   0x0D

int encoderValue, inputValue, thetaValue;

// Timing variables
unsigned long previousMillis = 0;
const unsigned long sensorInterval = 50; // Interval in milliseconds

// Stepper motor control variables
unsigned long previousStepTime = 0;
const unsigned long stepInterval = 5000; // 2000µs per step (500Hz)

bool stepState = LOW;
int stepCount = 0;
const int targetSteps = 400;

uint16_t AS5600_ReadRawAngle();
void I2C_Init();
bool I2C_CheckError();
void readSerial();

// I2C handle
I2C_HandleTypeDef hi2c1;

void setup() {
  // Set pins as output
  Serial.begin(115200);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(EN_PIN, OUTPUT);

  // Initialize the HAL library
  HAL_Init();

  // Initialize I2C communication
  I2C_Init(); // Initialize I2C

  // Enable the stepper driver
  digitalWrite(EN_PIN, LOW); // Active LOW to enable the driver

  // Set initial direction
  digitalWrite(DIR_PIN, HIGH); // HIGH = Clockwise, LOW = Counterclockwise
  Serial.println("Stepper motor is running...");
}

void loop() {

  //  unsigned long currentMillis = millis();
  //  unsigned long currentMicros = micros();

  //  // Periodic AS5600 angle reading
  //  if (currentMillis - previousMillis >= sensorInterval) {
  //    previousMillis = currentMillis;
 
  //    // Read and print the AS5600 angle
  //    encoderValue = AS5600_ReadRawAngle();
  //    DEBUG_PRINT("J1 A ");
  //    DEBUG_PRINTLN(encoderValue);
  //  }

  // // // Generate step pulses 
  // if (currentMicros - previousStepTime >= stepInterval) {
  //   previousStepTime = currentMicros;
  //   stepState = !stepState;
  //   digitalWrite(STEP_PIN, stepState);
    
  //   if (stepState) {
  //     stepCount++;
  //   }
  // }

  // // Change direction after completing a full cycle
  // if (stepCount >= targetSteps) {
  //   stepCount = 0; // Reset step count after completing a cycle
  //   digitalWrite(DIR_PIN, !digitalRead(DIR_PIN)); // Toggle direction
  //   DEBUG_PRINT("Direction Changed\n"); // Optional debug output
  // }
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
      while (1);
  }
}

// void readSerial() {
//   while (Serial.available() > 0) {
//     char receivedChar = Serial.read();
//     if (receivedChar == '\n') {
//       inputComplete = true;
//       break;
//     } else if (isDigit(receivedChar) || receivedChar == '-') {
//       inputString += receivedChar;
//     }
//   }

//   if (inputComplete) {
//     int parsedValue = inputString.toInt();
//     thetaTarget = constrain(parsedValue, 0, 180);
//     inputString = "";
//     inputComplete = false;
//   }
// }

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

  HAL_I2C_DeInit(&hi2c1);
  HAL_I2C_Init(&hi2c1);

  if (HAL_I2C_Mem_Read(&hi2c1, AS5600_I2C_ADDRESS, AS5600_RAW_ANGLE_REGISTER, 
                      I2C_MEMADD_SIZE_8BIT, buffer, 2, HAL_MAX_DELAY) != HAL_OK) {
      if (I2C_CheckError()) {
          HAL_I2C_DeInit(&hi2c1);
          HAL_I2C_Init(&hi2c1);
      }
  } else {
      raw_angle = (buffer[0] << 8) | buffer[1]; 
  }

  return raw_angle;
}
