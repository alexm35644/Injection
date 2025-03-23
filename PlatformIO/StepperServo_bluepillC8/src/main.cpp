#include <Arduino.h>
#include "stm32f1xx_hal.h"
#include <PID_v1.h>

#define STEP_PIN PA0
#define DIR_PIN PA1
#define LED_PIN PC13

#define FORWARD 1  // Away from limit switch
#define MOTOR_REVERSE 0  // Toward limit switch
#define PWM_LOWER_LIMIT 100
#define PWM_UPPER_LIMIT 2000

// Define I2C address and register for AS5600
#define AS5600_I2C_ADDRESS (0x36 << 1) // AS5600 I2C address
#define AS5600_RAW_ANGLE_REGISTER 0x0C // Register for raw angle
I2C_HandleTypeDef hi2c1;

double targetAngle = 0;  // Target angle to move to (0-360 degrees)
String inputString = "";  // String to hold incoming data
bool inputComplete = false;
u_int16_t previousAngle = 0;

double input, output;
// double Kp = 2.0;  // Proportional constant
// double Ki = 5.0;  // Integral constant
// double Kd = 1.0;  // Derivative constant

PID myPID(&input, &output, &targetAngle, 4.0, 0, 0.5, DIRECT); // Empty constructor, we will set tunings later



uint16_t AS5600_ReadRawAngle();
bool I2C_CheckError();
void I2C_Init();
void serialEvent();

void setup() {
    I2C_Init();
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);

    Serial.begin(115200);
    Serial.println("Stepper motor ready. Send target angle (0-360).");

    // Initialize PID
    myPID.SetMode(AUTOMATIC);              
    // myPID.SetOutputLimits(PWM_LOWER_LIMIT, PWM_UPPER_LIMIT);
    // myPID.SetSampleTime(20);
}

void loop() {
    // Handle serial input for setting target angle
    if (inputComplete) {
        double inputVal = inputString.toDouble();  // Convert input to double
        if (inputVal >= 0 && inputVal <= 360) {
            targetAngle = inputVal;
            Serial.print("Target angle set to: ");
            Serial.println(targetAngle);
        } else {
            Serial.println("Invalid input. Please enter a value between 0 and 360.");
        }
        inputString = "";
        inputComplete = false;
    }

    // Get current angle from AS5600 encoder
    long currentAngle = AS5600_ReadRawAngle();
    currentAngle = map(currentAngle, 0, 4055, 0, 360);
    input = currentAngle;  // Set input to current angle

    // Use PID to compute the new output for motor speed
    myPID.Compute();
    //output = map(output, 0, 255, 100, 2000);

    // Calculate the direction to move the motor based on the current angle
    if (currentAngle < targetAngle) {
        digitalWrite(DIR_PIN, FORWARD);  // Move forward
    } else if (currentAngle > targetAngle) {
        digitalWrite(DIR_PIN, MOTOR_REVERSE);  // Move reverse
    }

    // Rotate the motor in the direction towards the target angle
    static unsigned long lastStepTime = 0;
    static bool stepState = LOW;

    // Step motor based on PID output (speed control)
    if (micros() - lastStepTime >= output) {  // Use PID output to control speed
        lastStepTime = micros();
        stepState = !stepState;
        digitalWrite(STEP_PIN, stepState);
    }

    // Print current angle for debugging
    static unsigned long lastReadTime = 0;
    if (millis() - lastReadTime >= 50) {  // Read every 200ms
        lastReadTime = millis();
        Serial.print("Current Angle: ");
        Serial.println(currentAngle);
        // Check if target angle is reached (with some tolerance)
        if (abs(currentAngle - targetAngle) < 5) {  // Adjust tolerance as needed
            Serial.print("Target angle reached: ");
            Serial.println(targetAngle);
        }
    }

    
}

void serialEvent() {
    while (Serial.available()) {
        char inChar = (char)Serial.read();
        if (inChar == '\n') {  // End of input
            inputComplete = true;
        } else {
            inputString += inChar;
        }
    }
}

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
    }
    
    return raw_angle;
}