/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define Relay_Pin GPIO_PIN_15
#define Relay_GPIO_Port GPIOC
#define VTso_NTC_Pin GPIO_PIN_0
#define VTso_NTC_GPIO_Port GPIOA
#define U_Current_Pin GPIO_PIN_1
#define U_Current_GPIO_Port GPIOA
#define V_Current_Pin GPIO_PIN_2
#define V_Current_GPIO_Port GPIOA
#define W_Current_Pin GPIO_PIN_3
#define W_Current_GPIO_Port GPIOA
#define U_Voltage_Pin GPIO_PIN_4
#define U_Voltage_GPIO_Port GPIOA
#define V_Voltage_Pin GPIO_PIN_5
#define V_Voltage_GPIO_Port GPIOA
#define W_Voltage_Pin GPIO_PIN_6
#define W_Voltage_GPIO_Port GPIOA
#define TRIP_Pin GPIO_PIN_12
#define TRIP_GPIO_Port GPIOB
#define FAN_12V_Pin GPIO_PIN_11
#define FAN_12V_GPIO_Port GPIOA
#define FAN_12V_EXTI_IRQn EXTI15_10_IRQn
#define BKIN_Pin GPIO_PIN_12
#define BKIN_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
