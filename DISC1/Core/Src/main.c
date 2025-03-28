/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "i2c.h"
#include "i2s.h"
#include "spi.h"
#include "usart.h"
#include "usb_host.h"
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

#define JOINT1_HOME 0
#define JOINT2_HOME 45
#define JOINT3_HOME 0
#define JOINT4_HOME 0 
#define JOINT5_HOME 0
#define ACTUATOR_HOME 0
#define MAIN_UART &huart2
#define MAIN_UART_INSTANCE USART2

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define UART_BUFFER_SIZE 256
volatile uint8_t rx_byte;                        // For receiving single byte
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
void MX_USB_HOST_Process(void);

/* USER CODE BEGIN PFP */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

#define PUTCHAR_PROTOTYPE_USART3 int __io_putchar_usart3(int ch)
#define printf_usart3(...) do { \
    char buffer[256]; \
    int len = snprintf(buffer, sizeof(buffer), __VA_ARGS__); \
    for (int i = 0; i < len; i++) { \
        __io_putchar_usart3(buffer[i]); \
    } \
} while (0)

#define PUTCHAR_PROTOTYPE_USART6 int __io_putchar_usart6(int ch)
#define printf_usart6(...) do { \
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
  MX_I2C1_Init();
  MX_I2S3_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_USB_HOST_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(MAIN_UART, &rx_byte, 1); // Start receiving single bytes in interrupt mode
  // HomeSet(); // Set all joints to home position
  ToggleLEDs(1);
  printf("System Initialized\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    MX_USB_HOST_Process();
    ToggleLEDs(ledFlag);
    

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
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

 PUTCHAR_PROTOTYPE_USART3
 {
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART3 and Loop until the end of transmission */
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);

    return ch;
 }

 PUTCHAR_PROTOTYPE_USART6
 {
    /* Place your implementation of fputc here */
    /* e.g. write a character to the USART6 and Loop until the end of transmission */
    HAL_UART_Transmit(&huart6, (uint8_t *)&ch, 1, 0xFFFF);

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


void ToggleLEDs(int state){
  /* Toggle LEDs sequentially */
  if(state){
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    HAL_Delay(100);
    HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
    HAL_Delay(100);
    HAL_GPIO_TogglePin(LED3_GPIO_Port, LED3_Pin);
    HAL_Delay(100);
    HAL_GPIO_TogglePin(LED4_GPIO_Port, LED4_Pin);
    HAL_Delay(100);
  }else{
    HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED3_GPIO_Port, LED3_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LED4_GPIO_Port, LED4_Pin, GPIO_PIN_RESET);
  }
  
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
  //Set joint

}

void Joint2Set(int theta){
  printf_usart3("%d\r\n", theta);

}

void Joint3Set(int theta){
  printf_usart6("%d\r\n", theta);

}

void Joint4Set(int theta){
  //Set joint

}

void Joint5Set(int theta){
  //Set joint

}

void ActuatorSet(int theta){
  //Set joint

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
