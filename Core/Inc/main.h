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
#define RS485_2_TX_Pin GPIO_PIN_2
#define RS485_2_TX_GPIO_Port GPIOA
#define RS485_2_RX_Pin GPIO_PIN_3
#define RS485_2_RX_GPIO_Port GPIOA
#define RS485_2_DE_Pin GPIO_PIN_6
#define RS485_2_DE_GPIO_Port GPIOA
#define RS485_2_RE_Pin GPIO_PIN_7
#define RS485_2_RE_GPIO_Port GPIOA
#define RS485_1_DE_Pin GPIO_PIN_0
#define RS485_1_DE_GPIO_Port GPIOB
#define RS485_1_RE_Pin GPIO_PIN_1
#define RS485_1_RE_GPIO_Port GPIOB
#define RS485_1_TX_Pin GPIO_PIN_10
#define RS485_1_TX_GPIO_Port GPIOB
#define RS485_1_RX_Pin GPIO_PIN_11
#define RS485_1_RX_GPIO_Port GPIOB
#define SW_2_Pin GPIO_PIN_12
#define SW_2_GPIO_Port GPIOB
#define SW_3_Pin GPIO_PIN_13
#define SW_3_GPIO_Port GPIOB
#define LED_3_Pin GPIO_PIN_14
#define LED_3_GPIO_Port GPIOB
#define LED_2_Pin GPIO_PIN_15
#define LED_2_GPIO_Port GPIOB
#define RS232_TX_Pin GPIO_PIN_9
#define RS232_TX_GPIO_Port GPIOA
#define RS232_RX_Pin GPIO_PIN_10
#define RS232_RX_GPIO_Port GPIOA
#define PWM_OUT_Pin GPIO_PIN_6
#define PWM_OUT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
