/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

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
#define ONE_WIRE_Pin GPIO_PIN_13
#define ONE_WIRE_GPIO_Port GPIOC
#define ROW4_Pin GPIO_PIN_0
#define ROW4_GPIO_Port GPIOA
#define ROW3_Pin GPIO_PIN_1
#define ROW3_GPIO_Port GPIOA
#define ROW2_Pin GPIO_PIN_2
#define ROW2_GPIO_Port GPIOA
#define ROW1_Pin GPIO_PIN_3
#define ROW1_GPIO_Port GPIOA
#define ROW0_Pin GPIO_PIN_4
#define ROW0_GPIO_Port GPIOA
#define COL2_Pin GPIO_PIN_5
#define COL2_GPIO_Port GPIOA
#define COL2_EXTI_IRQn EXTI9_5_IRQn
#define COL1_Pin GPIO_PIN_6
#define COL1_GPIO_Port GPIOA
#define COL1_EXTI_IRQn EXTI9_5_IRQn
#define COL0_Pin GPIO_PIN_7
#define COL0_GPIO_Port GPIOA
#define COL0_EXTI_IRQn EXTI9_5_IRQn
#define BUZZER_Pin GPIO_PIN_11
#define BUZZER_GPIO_Port GPIOB
#define SPI2_SCK___LED_SCLK_Pin GPIO_PIN_13
#define SPI2_SCK___LED_SCLK_GPIO_Port GPIOB
#define SPI2_MOSI___LED_SDI_A_Pin GPIO_PIN_15
#define SPI2_MOSI___LED_SDI_A_GPIO_Port GPIOB
#define LED_ST_CP_Pin GPIO_PIN_8
#define LED_ST_CP_GPIO_Port GPIOA
#define RS485_DE_Pin GPIO_PIN_11
#define RS485_DE_GPIO_Port GPIOA
#define RS485_RE_Pin GPIO_PIN_12
#define RS485_RE_GPIO_Port GPIOA
#define RGB_BLUE_Pin GPIO_PIN_3
#define RGB_BLUE_GPIO_Port GPIOB
#define RGB_GREEN_Pin GPIO_PIN_4
#define RGB_GREEN_GPIO_Port GPIOB
#define RGB_RED_Pin GPIO_PIN_5
#define RGB_RED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
