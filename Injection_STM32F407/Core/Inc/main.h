/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define DC3_IN1_Pin GPIO_PIN_2
#define DC3_IN1_GPIO_Port GPIOE
#define DC3_IN2_Pin GPIO_PIN_3
#define DC3_IN2_GPIO_Port GPIOE
#define DC4_IN1_Pin GPIO_PIN_4
#define DC4_IN1_GPIO_Port GPIOE
#define DC4_IN2_Pin GPIO_PIN_5
#define DC4_IN2_GPIO_Port GPIOE
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define ADC1_IN0_ACT_Pin GPIO_PIN_0
#define ADC1_IN0_ACT_GPIO_Port GPIOA
#define ADC1_IN1_ROT1_Pin GPIO_PIN_1
#define ADC1_IN1_ROT1_GPIO_Port GPIOA
#define ADC1_IN2_ROT2_Pin GPIO_PIN_2
#define ADC1_IN2_ROT2_GPIO_Port GPIOA
#define DIR5_Pin GPIO_PIN_6
#define DIR5_GPIO_Port GPIOA
#define IN1_Pin GPIO_PIN_5
#define IN1_GPIO_Port GPIOC
#define USER1_Pin GPIO_PIN_0
#define USER1_GPIO_Port GPIOB
#define USER1_EXTI_IRQn EXTI0_IRQn
#define DIR3_Pin GPIO_PIN_8
#define DIR3_GPIO_Port GPIOE
#define DIR1_Pin GPIO_PIN_10
#define DIR1_GPIO_Port GPIOE
#define DIR2_Pin GPIO_PIN_12
#define DIR2_GPIO_Port GPIOE
#define DIR4_Pin GPIO_PIN_15
#define DIR4_GPIO_Port GPIOE
#define USER2_Pin GPIO_PIN_12
#define USER2_GPIO_Port GPIOB
#define DC1_IN2_Pin GPIO_PIN_13
#define DC1_IN2_GPIO_Port GPIOB
#define DC2_IN1_Pin GPIO_PIN_14
#define DC2_IN1_GPIO_Port GPIOB
#define DC2_IN2_Pin GPIO_PIN_15
#define DC2_IN2_GPIO_Port GPIOB
#define DIAG2_Pin GPIO_PIN_9
#define DIAG2_GPIO_Port GPIOC
#define DIAG1_Pin GPIO_PIN_8
#define DIAG1_GPIO_Port GPIOA
#define DIR6_Pin GPIO_PIN_7
#define DIR6_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
