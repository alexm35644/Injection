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
#include "can.h"
#include "i2c.h"
#include "usart.h"
#include "usb.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h> 

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
// Define the I2C address for the AS5600 (shifted left for 7-bit addressing)
#define AS5600_I2C_ADDR     (0x36 << 1)

// Register addresses
#define AS5600_RAW_ANGLE    0x0C   // High byte of raw angle starts here
#define AS5600_RAW_ANGLE_L  0x0D   // Low byte of raw angle

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */



CAN_TxHeaderTypeDef   TxHeader;
CAN_RxHeaderTypeDef   RxHeader; //CAN Bus Transmit Header
uint8_t               TxData[8];
uint32_t              TxMailbox;
CAN_FilterTypeDef canfil; //CAN Bus Filter
uint32_t canMailbox; //CAN Bus Mail box variable

uint8_t canRX[8] = {0x0,0x0,0x0,0x0,0x0,0x0,0x0,0x0};  //CAN Bus Receive Buffer
uint8_t buffer[1];



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Function prototypes
HAL_StatusTypeDef AS5600_ReadRawAngle(I2C_HandleTypeDef *hi2c, uint16_t *angle);


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
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */
  // Initialize I2C (Assume hi2c1 is configured in CubeMX)

  uint16_t angle = 0;


  canfil.FilterBank = 0;
  canfil.FilterMode = CAN_FILTERMODE_IDMASK;
  canfil.FilterFIFOAssignment = CAN_RX_FIFO0;
  canfil.FilterIdHigh = 0;
  canfil.FilterIdLow = 0;
  canfil.FilterMaskIdHigh = 0;
  canfil.FilterMaskIdLow = 0;
  canfil.FilterScale = CAN_FILTERSCALE_32BIT;
  canfil.FilterActivation = ENABLE;
  canfil.SlaveStartFilterBank = 14;

  TxHeader.IDE = CAN_ID_STD; // Standard ID. CAN_ID_EXT is extended ID. 
  TxHeader.StdId = 0x111;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8;          // Data frame size. Max 8. 

  TxData[0] = 0x00;  
  TxData[1] = 0x00; 
  TxData[2] = 0x00; 
  TxData[3] = 0x00; 
  TxData[4] = 0x00; 
  TxData[5] = 0x00; 
  TxData[6] = 0x00; 
  TxData[7] = 0x00; 

  HAL_CAN_ConfigFilter(&hcan,&canfil);
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
	  Error_Handler();
  }

  int toggle = 0; 



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    
    HAL_Delay(100);
    HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
    // if (HAL_UART_Receive(&huart1, buffer, 1, 0xFFFF) == HAL_OK) {
    //         // If data is received, trigger action

    //         HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
    //         receive_transmitCAN(buffer[0]);
    //         //HAL_UART_Transmit(&huart1, buffer, 1, 0xFFFF);
            
    //         if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
    //           {
    //             //Error_Handler ();
    //           }
            
    //     }
      if (AS5600_ReadRawAngle(&hi2c1, &angle) == HAL_OK) {
            // Do something with the angle
            printf("Angle: %u\r\n", angle);
        } else {
            // Handle error
            //Error_Handler();
        }
      

    // toggle_transmitCAN(toggle);
    // toggle ^= 1; 
    
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART1 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;
}

void send_uart_message(char *message) {
    HAL_UART_Transmit(&huart1, (uint8_t *)message, strlen(message), HAL_MAX_DELAY);
}



void printRPM(uint8_t LSB, uint8_t MSB){
  int RPM; 
  RPM = MSB * 256 + LSB; 
  printf("RPM: %d", RPM); 
}

void receive_transmitCAN(char target){

  if(target == '0')
  {
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.StdId = 0x6B1;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    TxData[0] = 0x9;  
    TxData[1] = 0x10; 
    TxData[2] = 0x90; 
    TxData[3] = 0x01; 
    TxData[4] = 0x05; 
    TxData[5] = 0x06; 
    TxData[6] = 0x07; 
    TxData[7] = 0x11; 
  }
  else if (target == '1'){
    TxHeader.IDE = CAN_ID_EXT;
    TxHeader.ExtId = 0x0CF11E05;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 8;

    TxData[0] = 0x9;  
    TxData[1] = 0x10; 
    TxData[2] = 0x90; 
    TxData[3] = 0x01; 
    TxData[4] = 0x05; 
    TxData[5] = 0x06; 
    TxData[6] = 0x07; 
    TxData[7] = 0x11; 
  }

}
HAL_StatusTypeDef AS5600_ReadRawAngle(I2C_HandleTypeDef *hi2c, uint16_t *angle) {
    uint8_t buffer[2]; // To store the two bytes of the raw angle

    // Read the high byte and low byte of the raw angle
    if (HAL_I2C_Mem_Read(hi2c, AS5600_I2C_ADDR, AS5600_RAW_ANGLE, I2C_MEMADD_SIZE_8BIT, buffer, 2, HAL_MAX_DELAY) != HAL_OK) {
        return HAL_ERROR;
    }

    // Combine high and low bytes into a 16-bit angle value
    *angle = ((uint16_t)buffer[0] << 8) | buffer[1];

    return HAL_OK;
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
    //printf("bruh");
    HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
    HAL_Delay(200);
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