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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

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

const char *error_messages[16] = {
    "ERR0: Identification error.",
    "ERR1: Over voltage.",
    "ERR2: Low voltage.",
    "ERR3: Reserved.",
    "ERR4: Stall.",
    "ERR5: Internal volts fault.",
    "ERR6: Over temperature.",
    "ERR7: Throttle error at power-up.",
    "ERR8: Reserved.",
    "ERR9: Internal reset.",
    "ERR10: Hall throttle open or short-circuit.",
    "ERR11: Angle sensor error.",
    "ERR12: Reserved.",
    "ERR13: Reserved.",
    "ERR14: Motor over-temperature.",
    "ERR15: Hall galvanometer sensor error."
};



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)

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
  MX_CAN_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_USB_PCD_Init();
  /* USER CODE BEGIN 2 */

  CAN_FilterTypeDef canfilterconfig;


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

  TxHeader.IDE = CAN_ID_STD;
  TxHeader.StdId = 0x446;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 8;

  TxData[0] = 0x01;  
  TxData[1] = 0x02; 
  TxData[2] = 0x03; 
  TxData[3] = 0x04; 
  TxData[4] = 0x05; 
  TxData[5] = 0x06; 
  TxData[6] = 0x07; 
  TxData[7] = 0x08; 




/**
  * @brief Print CANBUS Error messages 
  * @author Dean
  * @retval None
  */

   


  HAL_CAN_ConfigFilter(&hcan,&canfil);
  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan,CAN_IT_RX_FIFO0_MSG_PENDING);

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
	  Error_Handler();
  }

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



/**
  * @brief Print CANBUS Error messages 
  * @author Dean
  * @retval None
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan1)
{
    if (HAL_CAN_GetRxMessage(hcan1, CAN_RX_FIFO0, &RxHeader, canRX) != HAL_OK)
    {
        printf("CAN Message Read Failed. HAL ERROR... \r\n");
        return;
    }
    else
    {
        HAL_GPIO_TogglePin(USER_LED_GPIO_Port, USER_LED_Pin);
        if (RxHeader.IDE == CAN_ID_STD)
        {
            printf("Message has standard ID type...\r\n");
            printf("Message ID:\t%#lx\r\n", RxHeader.StdId);

            if(RxHeader.StdId == 0x6B0){ // BMS
              uint16_t pack_current     = (canRX[1] << 8) | canRX[0];
              uint16_t pack_voltage     = (canRX[3] << 8) | canRX[2];
              uint8_t pack_soc          = canRX[4];
              uint8_t pack_relay_state  = canRX[5];

              printf("Pack Current: %d\r\n", pack_current);
              printf("Pack Voltage: %d\r\n", pack_voltage);
              printf("Pack SoC: %d\r\n", pack_soc);
              printf("Pack Relay Status: %d\r\n", pack_relay_state);

            }
            else if(RxHeader.StdId == 0x6B1){ // BMS
              uint32_t pack_DCL = (canRX[3]<<24) | (canRX[2]<<16) | (canRX[1]<<8) | canRX[0];
              uint8_t high_temp_derating = canRX[4];
              uint8_t low_temp_derating   = canRX[5];

              printf("Pack DCL: %d\r\n", pack_DCL);
              printf("High Temp Derating: %d\r\n", high_temp_derating);
              printf("low_temp_derating: %d\r\n", low_temp_derating);
            }
            else
            {
            printf("ERROR: Unknown IDE type\r\n");
            return;
            }
        }
        else if (RxHeader.IDE == CAN_ID_EXT)
        {
            printf("Message has extended ID type...\r\n");
            printf("Message ID:\t%#lx\r\n", RxHeader.ExtId);

            if (RxHeader.ExtId == 0x0CF11E05)
            {
                uint16_t RPM = (canRX[1] << 8) | canRX[0];
                uint16_t Current = (canRX[3] << 8) | canRX[2];
                uint16_t Voltage = (canRX[5] << 8) | canRX[4];

                printf("R  P  M = %u rpm\r\n", RPM);
                printf("Current = %u A\r\n", Current/ 10);
                printf("Voltage = %u V\r\n", Voltage / 10);

                for (uint8_t i = 0; i < 16; i++)
                {
                    if (((canRX[7] << 8) | canRX[6]) & (1 << i))
                    {
                        printf("%s\r\n", error_messages[i]);
                    }
                }
            }
          
            else if (RxHeader.ExtId == 0x0CF11F05)
            {
                
                uint8_t throttle_signal = canRX[0];
                int8_t controller_temp = canRX[1] - 40; 
                int8_t motor_temp = canRX[2] - 30; 
                printf("Throttle Signal: %d V\r\n", throttle_signal);
                printf("Controller Temperature: %d ℃\r\n", controller_temp);
                printf("Motor Temperature: %d ℃\r\n", motor_temp);

               
                if (canRX[4] & 0x01)
                    printf("Controller Command: Forward\r\n");
                else if (canRX[4] & 0x02)
                    printf("Controller Command: Backward\r\n");
                else if (canRX[4] & 0x03)
                    printf("Controller Command: Reserved\r\n");
                else printf("Controller Command: Neutral\r\n");

                if (((canRX[4] >> 2) & 0x03 )== 0x00)
                    printf("Feedback: Stationary\r\n");
                else if (((canRX[4] >> 2) & 0x03 ) == 0x01)
                    printf("Feedback: Forward\r\n");
                else if (((canRX[4] >> 2) & 0x03 ) == 0x02)
                    printf("Feedback: Backward\r\n");
                else
                    printf("Feedback: Reserved\r\n");

                printf("Switch Status:\r\n");
                printf("  Boost: %s\r\n", (canRX[5] & 0x80) ? "ON" : "OFF");
                printf("  Foot Switch: %s\r\n", (canRX[5] & 0x40) ? "ON" : "OFF");
                printf("  Forward Switch: %s\r\n", (canRX[5] & 0x20) ? "ON" : "OFF");
                printf("  Backward Switch: %s\r\n", (canRX[5] & 0x10) ? "ON" : "OFF");
                printf("  12V Brake Switch: %s\r\n", (canRX[5] & 0x08) ? "ON" : "OFF");
                printf("  Hall C: %s\r\n", (canRX[5] & 0x04) ? "ON" : "OFF");
                printf("  Hall B: %s\r\n", (canRX[5] & 0x02) ? "ON" : "OFF");
                printf("  Hall A: %s\r\n", (canRX[5] & 0x01) ? "ON" : "OFF");
            }
        }
        else
        {
            printf("ERROR: Unknown IDE type\r\n");
            return;
        }

    
        printf("Message length is %ld byte(s)\r\n", RxHeader.DLC);
        for (uint8_t i = 0; i < 8; i++)
        {
            printf("Byte %d: 0x%02X\r\n", i, canRX[i]);
        }
    }
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