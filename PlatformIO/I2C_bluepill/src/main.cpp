#include <Arduino.h>
#include "stm32f1xx_hal.h"

// Define I2C address and register for AS5600
#define AS5600_I2C_ADDRESS (0x36 << 1) // AS5600 I2C address
#define AS5600_RAW_ANGLE_REGISTER 0x0C // Register for raw angle
#define LED PC13

I2C_HandleTypeDef hi2c1;

// Function to initialize I2C
void I2C_Init() {
    __HAL_RCC_I2C1_CLK_ENABLE();
    
    // GPIOB clock enable for PB6 (SCL) and PB7 (SDA)
    __HAL_RCC_GPIOB_CLK_ENABLE();
    
    // Configure PB6 and PB7 for I2C
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
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
        Serial.println("I2C initialization failed!");
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
  Serial.println("Attempting I2C read...");

  // Reset the I2C bus after each attempt
  HAL_I2C_DeInit(&hi2c1);
  HAL_I2C_Init(&hi2c1);
  
  // Try reading raw angle register
  if (HAL_I2C_Mem_Read(&hi2c1, AS5600_I2C_ADDRESS, AS5600_RAW_ANGLE_REGISTER, 
                       I2C_MEMADD_SIZE_8BIT, buffer, 2, HAL_MAX_DELAY) != HAL_OK) {
      // Check for I2C error
      if (I2C_CheckError()) {
          Serial.println("I2C communication failed, resetting...");
          HAL_I2C_DeInit(&hi2c1);
          HAL_I2C_Init(&hi2c1);
      }
  } else {
      raw_angle = (buffer[0] << 8) | buffer[1]; // Combine MSB and LSB
      Serial.print("Successfully read angle: ");
      Serial.println(raw_angle);
  }
  
  return raw_angle;
}


void setup() {
    pinMode(PC13, OUTPUT);
    Serial.begin(115200);
    I2C_Init(); // Initialize I2C
    Serial.println("I2C Initialized");
}

void loop() {
  uint16_t raw_angle = AS5600_ReadRawAngle();
  
  Serial.print("Raw Angle: ");
  Serial.println(raw_angle);

  digitalWrite(PC13, !digitalRead(PC13));
  delay(50); // 200ms delay
}
