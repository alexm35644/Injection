/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "usb_otg.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define JOINT1_HOME 90
#define JOINT2_HOME 90
#define JOINT3_HOME 90
#define JOINT4_HOME 90
#define JOINT5_HOME 90
#define ACTUATOR_HOME 0

#define MAIN_UART &huart1
#define MAIN_UART_INSTANCE USART1
#define JOINT1_UART &huart6
#define JOINT1_UART_INSTANCE USART6
#define JOINT2_UART &huart2
#define JOINT2_UART_INSTANCE USART2
#define JOINT3_UART &huart3
#define JOINT3_UART_INSTANCE USART3
#define JOINT4_UART &huart4
#define JOINT4_UART_INSTANCE UART4
#define JOINT5_UART &huart5
#define JOINT5_UART_INSTANCE UART5


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define UART_BUFFER_SIZE 256
volatile uint8_t rx_byte;                         // Main UART
volatile uint8_t rx_byte1;                        // Joint 1
volatile uint8_t rx_byte2;                        // Joint 2
volatile uint8_t rx_byte3;                        // Joint 3
volatile uint8_t rx_byte4;                        // Joint 4
volatile uint8_t rx_byte5;                        // Joint 5

volatile uint8_t uart_buffer[UART_BUFFER_SIZE];  // Circular buffer
volatile uint16_t buffer_index = 0;     // Index for storing data

volatile uint8_t uart_received = 0; // Flag to indicate a new message is available
volatile char received_string[UART_BUFFER_SIZE]; // Store the received string
volatile char previous_string[UART_BUFFER_SIZE]; // Store the received string

int joint1 = JOINT1_HOME;
int joint2 = JOINT2_HOME;
int joint3 = JOINT3_HOME;
int joint4 = JOINT4_HOME; 
int joint5 = JOINT5_HOME; 
int actuator = ACTUATOR_HOME; 
int ledFlag = 0; 

int prev_joint1, prev_joint2, prev_joint3, prev_joint4, prev_joint5, prev_actuator; 


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

#define PUTCHAR_PROTOTYPE_USART2 int __io_putchar_usart2(int ch)
#define printf_joint2(...) do { \
    char buffer[256]; \
    int len = snprintf(buffer, sizeof(buffer), __VA_ARGS__); \
    for (int i = 0; i < len; i++) { \
        __io_putchar_usart2(buffer[i]); \
    } \
} while (0)

#define PUTCHAR_PROTOTYPE_USART3 int __io_putchar_usart3(int ch)
#define printf_joint3(...) do { \
    char buffer[256]; \
    int len = snprintf(buffer, sizeof(buffer), __VA_ARGS__); \
    for (int i = 0; i < len; i++) { \
        __io_putchar_usart3(buffer[i]); \
    } \
} while (0)

#define PUTCHAR_PROTOTYPE_UART4 int __io_putchar_uart4(int ch)
#define printf_joint4(...) do { \
    char buffer[256]; \
    int len = snprintf(buffer, sizeof(buffer), __VA_ARGS__); \
    for (int i = 0; i < len; i++) { \
        __io_putchar_uart4(buffer[i]); \
    } \
} while (0)

#define PUTCHAR_PROTOTYPE_UART5 int __io_putchar_uart5(int ch)
#define printf_joint5(...) do { \
    char buffer[256]; \
    int len = snprintf(buffer, sizeof(buffer), __VA_ARGS__); \
    for (int i = 0; i < len; i++) { \
        __io_putchar_uart5(buffer[i]); \
    } \
} while (0)

#define PUTCHAR_PROTOTYPE_USART6 int __io_putchar_usart6(int ch)
#define printf_joint1(...) do { \
    char buffer[256]; \
    int len = snprintf(buffer, sizeof(buffer), __VA_ARGS__); \
    for (int i = 0; i < len; i++) { \
        __io_putchar_usart6(buffer[i]); \
    } \
} while (0)

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  

  printf("System Initialized\r\n");

  HomeSet();

  HAL_UART_Receive_IT(MAIN_UART, &rx_byte, 1); // Start receiving single bytes in interrupt mode
  HAL_UART_Receive_IT(JOINT1_UART, &rx_byte1, 1); // Start receiving single bytes in interrupt mode
  HAL_UART_Receive_IT(JOINT2_UART, &rx_byte2, 1); // Start receiving single bytes in interrupt mode
  HAL_UART_Receive_IT(JOINT3_UART, &rx_byte3, 1); // Start receiving single bytes in interrupt mode
  HAL_UART_Receive_IT(JOINT4_UART, &rx_byte4, 1); // Start receiving single bytes in interrupt mode
  HAL_UART_Receive_IT(JOINT5_UART, &rx_byte5, 1); // Start receiving single bytes in interrupt mode


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */




  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 72;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 3;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/**
  * @brief  Retargets the C library printf function to the USART.
  * @retval None
  */
 PUTCHAR_PROTOTYPE
 {
   /* Place your implementation of fputc here */
   /* e.g. write a character to the USART1 and Loop until the end of transmission */
   HAL_UART_Transmit(MAIN_UART, (uint8_t *)&ch, 1, 0xFFFF);
 
   return ch;
 }

 PUTCHAR_PROTOTYPE_USART2{
    HAL_UART_Transmit(JOINT2_UART, (uint8_t *)&ch, 1, 0xFFFF);

      return ch;
 }

 PUTCHAR_PROTOTYPE_USART3
 {
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART3 and Loop until the end of transmission */
    HAL_UART_Transmit(JOINT3_UART, (uint8_t *)&ch, 1, 0xFFFF);

    return ch;
 }

 PUTCHAR_PROTOTYPE_UART4
 {
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART3 and Loop until the end of transmission */
    HAL_UART_Transmit(JOINT4_UART, (uint8_t *)&ch, 1, 0xFFFF);

    return ch;
 }

 PUTCHAR_PROTOTYPE_UART5
 {
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART3 and Loop until the end of transmission */
    HAL_UART_Transmit(JOINT5_UART, (uint8_t *)&ch, 1, 0xFFFF);

    return ch;
 }

 PUTCHAR_PROTOTYPE_USART6
 {
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART6 and Loop until the end of transmission */
    HAL_UART_Transmit(JOINT1_UART, (uint8_t *)&ch, 1, 0xFFFF);

    return ch;
 }

 void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    if (huart->Instance == MAIN_UART_INSTANCE) // Correct UART instance

    {
        // Store received byte in buffer
        if (buffer_index < UART_BUFFER_SIZE - 1) 
        {
            uart_buffer[buffer_index++] = rx_byte;

            // Check for newline or carriage return
            if (rx_byte == '\n') 
            {
                uart_buffer[buffer_index] = '\0'; // Null-terminate the string
                buffer_index = 0;                // Reset the index for the next string

                // Copy received string to the global buffer for main loop access
                strcpy((char *)received_string, (char *)uart_buffer);

                if(strcmp("print", (char* )received_string)){
                  ProcessReceivedString((char *)received_string);
                  strcpy((char *)previous_string, (char *)received_string);
                }
                else if(strcmp((char *)received_string, (char *)previous_string)){
                  ProcessReceivedString((char *)received_string);
                  strcpy((char *)previous_string, (char *)received_string);
                }

                
            }
        }
        else 
        {
            // Buffer overflow, reset index
            buffer_index = 0;
            printf("Buffer overflow\r\n");
        }

        // Restart UART reception for the next byte
        HAL_UART_Receive_IT(MAIN_UART, &rx_byte, 1);
    }

    else if(huart -> Instance == JOINT1_UART_INSTANCE){
        // Store received byte in buffer
        if (buffer_index < UART_BUFFER_SIZE - 1) 
        {
            uart_buffer[buffer_index++] = rx_byte1;

            // Check for newline or carriage return
            if (rx_byte1 == '\n') 
            {
                uart_buffer[buffer_index] = '\0'; // Null-terminate the string
                buffer_index = 0;                // Reset the index for the next string

                // Copy received string to the global buffer for main loop access
                strcpy((char *)received_string, (char *)uart_buffer);

                if(strcmp("print", (char* )received_string)){
                  ProcessReceivedString((char *)received_string);
                  strcpy((char *)previous_string, (char *)received_string);
                }
                else if(strcmp((char *)received_string, (char *)previous_string)){
                  ProcessReceivedString((char *)received_string);
                  strcpy((char *)previous_string, (char *)received_string);
                }

                
            }
        }
        else 
        {
            // Buffer overflow, reset index
            buffer_index = 0;
            printf("Buffer overflow\r\n");
        }

        // Restart UART reception for the next byte
        HAL_UART_Receive_IT(JOINT1_UART, &rx_byte1, 1);
    }

    else if(huart -> Instance == JOINT2_UART_INSTANCE){
        // Store received byte in buffer
        if (buffer_index < UART_BUFFER_SIZE - 1) 
        {
            uart_buffer[buffer_index++] = rx_byte2;

            // Check for newline or carriage return
            if (rx_byte2 == '\n') 
            {
                uart_buffer[buffer_index] = '\0'; // Null-terminate the string
                buffer_index = 0;                // Reset the index for the next string

                // Copy received string to the global buffer for main loop access
                strcpy((char *)received_string, (char *)uart_buffer);

                if(strcmp("print", (char* )received_string)){
                  ProcessReceivedString((char *)received_string);
                  strcpy((char *)previous_string, (char *)received_string);
                }
                else if(strcmp((char *)received_string, (char *)previous_string)){
                  ProcessReceivedString((char *)received_string);
                  strcpy((char *)previous_string, (char *)received_string);
                }

                
            }
        }
        else 
        {
            // Buffer overflow
            buffer_index = 0;
            printf("Buffer overflow\r\n");
        }
        HAL_UART_Receive_IT(JOINT2_UART, &rx_byte2, 1);

      }

      else if(huart -> Instance == JOINT3_UART_INSTANCE){
        // Store received byte in buffer
        if (buffer_index < UART_BUFFER_SIZE - 1) 
        {
            uart_buffer[buffer_index++] = rx_byte3;

            // Check for newline or carriage return
            if (rx_byte3 == '\n') 
            {
                uart_buffer[buffer_index] = '\0'; // Null-terminate the string
                buffer_index = 0;                // Reset the index for the next string

                // Copy received string to the global buffer for main loop access
                strcpy((char *)received_string, (char *)uart_buffer);

                if(strcmp("print", (char* )received_string)){
                  ProcessReceivedString((char *)received_string);
                  strcpy((char *)previous_string, (char *)received_string);
                }
                else if(strcmp((char *)received_string, (char *)previous_string)){
                  ProcessReceivedString((char *)received_string);
                  strcpy((char *)previous_string, (char *)received_string);
                }

                
            }
        }
        else 
        {
            // Buffer overflow
            buffer_index = 0;
            printf("Buffer overflow\r\n");
        }
        HAL_UART_Receive_IT(JOINT3_UART, &rx_byte3, 1);

      }

      else if(huart -> Instance == JOINT4_UART_INSTANCE){
        // Store received byte in buffer
        if (buffer_index < UART_BUFFER_SIZE - 1) 
        {
            uart_buffer[buffer_index++] = rx_byte4;

            // Check for newline or carriage return
            if (rx_byte4 == '\n') 
            {
                uart_buffer[buffer_index] = '\0'; // Null-terminate the string
                buffer_index = 0;                // Reset the index for the next string

                // Copy received string to the global buffer for main loop access
                strcpy((char *)received_string, (char *)uart_buffer);

                if(strcmp("print", (char* )received_string)){
                  ProcessReceivedString((char *)received_string);
                  strcpy((char *)previous_string, (char *)received_string);
                }
                else if(strcmp((char *)received_string, (char *)previous_string)){
                  ProcessReceivedString((char *)received_string);
                  strcpy((char *)previous_string, (char *)received_string);
                }

                
            }
        }
        else 
        {
            // Buffer overflow
            buffer_index = 0;
            printf("Buffer overflow\r\n");
        }
        HAL_UART_Receive_IT(JOINT4_UART, &rx_byte4, 1);

      }

      else if(huart -> Instance == JOINT5_UART_INSTANCE){
        // Store received byte in buffer
        if (buffer_index < UART_BUFFER_SIZE - 1) 
        {
            uart_buffer[buffer_index++] = rx_byte5;

            // Check for newline or carriage return
            if (rx_byte5 == '\n') 
            {
                uart_buffer[buffer_index] = '\0'; // Null-terminate the string
                buffer_index = 0;                // Reset the index for the next string

                // Copy received string to the global buffer for main loop access
                strcpy((char *)received_string, (char *)uart_buffer);

                if(strcmp("print", (char* )received_string)){
                  ProcessReceivedString((char *)received_string);
                  strcpy((char *)previous_string, (char *)received_string);
                }
                else if(strcmp((char *)received_string, (char *)previous_string)){
                  ProcessReceivedString((char *)received_string);
                  strcpy((char *)previous_string, (char *)received_string);
                }

                
            }
        }
        else 
        {
            // Buffer overflow
            buffer_index = 0;
            printf("Buffer overflow\r\n");
        }
        HAL_UART_Receive_IT(JOINT5_UART, &rx_byte5, 1);

      }

}


 


 

// Function to process the received string
void ProcessReceivedString(char *str)
{
    // Check if the string starts with "set"
    if (strncmp(str, "set", 3) == 0)
    {
        // Move the pointer to after "set" and the space
        str += 4;  // Skip "set " (4 characters)

        // Now we expect a format like "000-000-000-000-000-000"
        if (sscanf(str, "%3d-%3d-%3d-%3d-%3d-%3d", &joint1, &joint2, &joint3, &joint4, &joint5, &actuator) == 6)
        {
            // Successfully parsed the 6 numbers
            printf("Received numbers: %d %d %d %d %d %d\r\n", joint1, joint2, joint3, joint4, joint5, actuator);

            // Check for changes in each joint and send new values if changed
            if (joint1 != prev_joint1) {
                // send to joint
                prev_joint1 = joint1;
                Joint1Set(joint1);
            }
            if (joint2 != prev_joint2) {
                // send to joint
                prev_joint2 = joint2;
                Joint2Set(joint2);
            }
            if (joint3 != prev_joint3) {
                // send to joint
                prev_joint3 = joint3;
                Joint3Set(joint3);
            }
            if (joint4 != prev_joint4) {
                // send to joint
                prev_joint4 = joint4;
            }
            if (joint5 != prev_joint5) {
                // send to joint
                prev_joint5 = joint5;
            }
            if (actuator != prev_actuator) {
                // send to actuator
                prev_actuator = actuator;
            }
        }
        else
        {
            printf("Invalid format!\r\n");
        }
    }
    // Just setting joint1 
    else if (strncmp(str, "joint1", 6) == 0)
    {
        str += 7;

        if (sscanf(str, "%3d", &joint1) == 1)
        {
            // Successfully parsed joint1
            printf("Joint1 set: %d\r\n", joint1);

            if (joint1 != prev_joint1) {
                Joint1Set(joint1);
                prev_joint1 = joint1;
            }
        }
        else
        {
            printf("Invalid format!\r\n");
        }
    }

    // Just setting joint2
    else if (strncmp(str, "joint2", 6) == 0)
    {
        str += 7;

        if (sscanf(str, "%3d", &joint2) == 1)
        {
            // Successfully parsed joint2
            printf("Joint2 set: %d\r\n", joint2);

            if (joint2 != prev_joint2) {
                Joint2Set(joint2);
                prev_joint2 = joint2;
            }
        }
        else
        {
            printf("Invalid format!\r\n");
        }
    }

    // Just setting joint3
    else if (strncmp(str, "joint3", 6) == 0)
    {
        str += 7;

        if (sscanf(str, "%3d", &joint3) == 1)
        {
            // Successfully parsed joint3
            printf("Joint3 set: %d\r\n", joint3);

            if (joint3 != prev_joint3) {
                Joint3Set(joint3);
                prev_joint3 = joint3;
            }
        }
        else
        {
            printf("Invalid format!\r\n");
        }
    }

    // Just setting joint4
    else if (strncmp(str, "joint4", 6) == 0)
    {
        str += 7;

        if (sscanf(str, "%3d", &joint4) == 1)
        {
            // Successfully parsed joint4
            printf("Joint4 set: %d\r\n", joint4);

            if (joint4 != prev_joint4) {
                Joint4Set(joint4);
                prev_joint4 = joint4;
            }
        }
        else
        {
            printf("Invalid format!\r\n");
        }
    }

    // Just setting joint5
    else if (strncmp(str, "joint5", 6) == 0)
    {
        str += 7;

        if (sscanf(str, "%3d", &joint5) == 1)
        {
            // Successfully parsed joint5
            printf("Joint5 set: %d\r\n", joint5);

            if (joint5 != prev_joint5) {
                Joint5Set(joint5);
                prev_joint5 = joint5;
            }
        }
        else
        {
            printf("Invalid format!\r\n");
        }
    }

    // Just setting actuator
    else if (strncmp(str, "actuator", 8) == 0)
    {
        str += 9;

        if (sscanf(str, "%3d", &actuator) == 1)
        {
            // Successfully parsed joint5
            printf("Actuator set: %d\r\n", actuator);

            if (actuator != actuator) {
                ActuatorSet(actuator);
                prev_actuator = actuator;
            }
        }
        else
        {
            printf("Invalid format!\r\n");
        }
    }

    else if (strncmp(str, "j1read", 6) == 0)
    {
        printf_joint1("r\r\n");
    }

    else if (strncmp(str, "j2read", 6) == 0)
    {
        printf_joint2("r\r\n");
    }

    else if (strncmp(str, "j3read", 6) == 0)
    {
        printf_joint3("r\r\n");
    }
      
      else if (strncmp(str, "j4read", 6) == 0)
      {
          printf_joint4("r\r\n");
      }
  
      else if (strncmp(str, "j5read", 6) == 0)
      {
          printf_joint5("r\r\n");
      }
  
      else if (strncmp(str, "aread", 5) == 0)
      {
          // Print acutator position or status
      }


    // Set Home 
    else if (strncmp(str, "home", 4) == 0)
    {
        str += 5;
        
        HomeSet();
        
        printf("Set all joints home\r\n");

    }

    else if (strncmp(str, "flash", 5) == 0)
    {
        str += 6;
        ledFlag = !ledFlag;
        
        printf("LEDs Toggled\r\n");

    }

    // Prints all joints
    else if (strncmp(str, "print", 5) == 0)
    {
       printf("Joint1: %d\r\n", joint1);
       printf("Joint2: %d\r\n", joint2);
       printf("Joint3: %d\r\n", joint3);
       printf("Joint4: %d\r\n", joint4);
       printf("Joint5: %d\r\n", joint5);
       printf("Actuator: %d\r\n", actuator);
    }

    // Echoes string if no keywords are sent
    else
    {
        printf("Received: %s\r\n", str);
    }
}

void ProcessReceivedJoint(char *str){

}
void HomeSet(){
  //Set all joints to home
  joint1 = JOINT1_HOME;
  Joint1Set(joint1);
  joint2 = JOINT2_HOME;
  Joint2Set(joint2);
  joint3 = JOINT3_HOME;
  Joint3Set(joint3);
  joint4 = JOINT4_HOME;
  Joint4Set(joint4);
  joint5 = JOINT5_HOME;
  Joint5Set(joint5);
  actuator = ACTUATOR_HOME;
  ActuatorSet(actuator);
}

void Joint1Set(int theta){
  printf_joint1("%d\r\n", theta);

}

void Joint2Set(int theta){
  printf_joint2("%d\n", theta);

}

void Joint3Set(int theta){
  printf_joint3("%d\r\n", theta);

}

void Joint4Set(int theta){
  printf_joint4("%d\r\n", theta);

}

void Joint5Set(int theta){
  printf_joint5("%d\r\n", theta);

}

void ActuatorSet(int theta){
  // Set joint. This is connected to the actuator motor controller. 
  // Make an actuator status function

}

int ActuatorStatus(){
  // Get actuator status
  return 0;
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
