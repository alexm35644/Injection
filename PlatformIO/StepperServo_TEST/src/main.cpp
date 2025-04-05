#include <Arduino.h>
#include "stm32f1xx_hal.h"

#define STEP_PIN PA0
#define DIR_PIN PA1
#define LED_PIN PC13
#define EN_PIN PA2

#define CW 1  
#define CCW 0  
#define MAX_STEP_SPEED 100
#define MIN_STEP_SPEED 2000
#define TOLERANCE 0
#define MAX_ANGLE 355
#define MIN_ANGLE 5

// Define I2C address and register for AS5600
#define AS5600_I2C_ADDRESS (0x36 << 1) // AS5600 I2C address
#define AS5600_RAW_ANGLE_REGISTER 0x0C // Register for raw angle
I2C_HandleTypeDef hi2c1;

double homeAngle = 150; // Home angle for the motor
double targetAngle = homeAngle;  // Target angle to move to (0-360 degrees)
String inputString = "";  // String to hold incoming data
bool inputComplete = false;
double previousAngle = homeAngle;
int limitFlag = 0; 

double input, output;
// double Kp = 2.0;  // Proportional constant
// double Ki = 5.0;  // Integral constant
// double Kd = 1.0;  // Derivative constant

// PID myPID(&input, &output, &targetAngle, 4.0, 0, 0.5, DIRECT); // Empty constructor, we will set tunings later



double AS5600_ReadRawAngle();
bool I2C_CheckError();
void I2C_Init();
void serialEvent();
void HomeMotor();

void setup() {
    I2C_Init();
    pinMode(STEP_PIN, OUTPUT);
    pinMode(DIR_PIN, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(EN_PIN, OUTPUT);
    digitalWrite(EN_PIN, LOW);  // Enable the driver
    digitalWrite(LED_PIN, HIGH);

    Serial.begin(115200);
    Serial.println("Stepper motor ready. Send target angle (0-360).");
    HomeMotor();

    // Initialize PID
    // myPID.SetMode(AUTOMATIC);              
    // myPID.SetOutputLimits(MAX_STEP_SPEED, MIN_STEP_SPEED);
    // myPID.SetSampleTime(20);
}

void loop() {
    // Handle serial input for setting target angle
    if (inputComplete) {
        double inputVal = inputString.toDouble();  // Convert input to double
        if (inputVal >= 5 && inputVal <= 355) {  // Check if the input is within the allowed range
            targetAngle = inputVal;
            if (previousAngle != targetAngle) {
                Serial.print("Target angle set to: ");
                Serial.println(targetAngle);
                previousAngle = targetAngle;
            }
        } 
        else {
            Serial.println("Invalid input. Please enter a value between 5 and 355.");
        }
        inputString = "";
        inputComplete = false;
    }

    // Get current angle from AS5600 encoder
    long currentAngle = AS5600_ReadRawAngle();


    // Calculate the direction to move the motor based on the current angle
    if (currentAngle < (targetAngle - TOLERANCE) && currentAngle > MIN_ANGLE) {
        digitalWrite(DIR_PIN, CCW);  // Move forward
        limitFlag = 0;
    } else if (currentAngle > (targetAngle + TOLERANCE) && currentAngle < MAX_ANGLE) {
        digitalWrite(DIR_PIN, CW);  // Move reverse
        limitFlag = 0;
    } else {
        // Stop the motor if the limit is reached
        limitFlag = 1;
    }

    // Rotate the motor in the direction towards the target angle
    static unsigned long lastStepTime = 0;
    static bool stepState = LOW;

    // Calculate the step delay based on the difference between current and target angles
    unsigned long stepDelay = map(abs(currentAngle - targetAngle), 0, 360, MIN_STEP_SPEED, MAX_STEP_SPEED);

    // Step motor based on calculated step delay
    if(limitFlag !=1){
    if (micros() - lastStepTime >= stepDelay) {
        lastStepTime = micros();
        stepState = !stepState;
        digitalWrite(STEP_PIN, stepState);
    }
    }

    // Print current angle for debugging
    static unsigned long lastReadTime = 0;
    if (millis() - lastReadTime >= 50) {  // Read every 50ms
        lastReadTime = millis();
        Serial.print("Current Angle: ");
        Serial.println(currentAngle);
        // Check if target angle is reached (with some tolerance)
        if (abs(currentAngle - targetAngle) < TOLERANCE) {
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

double AS5600_ReadRawAngle() {
    uint8_t buffer[2];
    uint16_t raw_angle = 0;
    double angle; 
  
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
    
    angle = (double)map(raw_angle, 0, 4055, 0, 360);
    return angle;
}

void HomeMotor(){
    // Move motor to home position
    printf("Homing motor...\n");
    digitalWrite(DIR_PIN, CCW);
    // if((int)AS5600_ReadRawAngle > (int)homeAngle){
    //     digitalWrite(DIR_PIN, CW);
    // }
    // else{
    //     digitalWrite(DIR_PIN, CCW);
    // }
    while (AS5600_ReadRawAngle() != homeAngle) {
        digitalWrite(STEP_PIN, HIGH);
        delayMicroseconds(500);
        digitalWrite(STEP_PIN, LOW);
        delayMicroseconds(500);
    }
    Serial.println("Motor homed.");
}