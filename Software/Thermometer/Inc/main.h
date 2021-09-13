/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#define NTC_Pin GPIO_PIN_1
#define NTC_GPIO_Port GPIOA
#define S9013_Pin GPIO_PIN_2
#define S9013_GPIO_Port GPIOA
#define KEY_HOLD_Pin GPIO_PIN_3
#define KEY_HOLD_GPIO_Port GPIOA
#define KEY_SWITCH_Pin GPIO_PIN_4
#define KEY_SWITCH_GPIO_Port GPIOA
#define LED_C_Pin GPIO_PIN_5
#define LED_C_GPIO_Port GPIOA
#define LED_F_Pin GPIO_PIN_6
#define LED_F_GPIO_Port GPIOA
#define LED_K_Pin GPIO_PIN_7
#define LED_K_GPIO_Port GPIOA
#define C_Pin GPIO_PIN_10
#define C_GPIO_Port GPIOB
#define D_Pin GPIO_PIN_11
#define D_GPIO_Port GPIOB
#define E_Pin GPIO_PIN_12
#define E_GPIO_Port GPIOB
#define F_Pin GPIO_PIN_13
#define F_GPIO_Port GPIOB
#define G_Pin GPIO_PIN_14
#define G_GPIO_Port GPIOB
#define DP_Pin GPIO_PIN_15
#define DP_GPIO_Port GPIOB
#define HOLD_Pin GPIO_PIN_8
#define HOLD_GPIO_Port GPIOA
#define CC1_Pin GPIO_PIN_4
#define CC1_GPIO_Port GPIOB
#define CC2_Pin GPIO_PIN_5
#define CC2_GPIO_Port GPIOB
#define CC3_Pin GPIO_PIN_6
#define CC3_GPIO_Port GPIOB
#define CC4_Pin GPIO_PIN_7
#define CC4_GPIO_Port GPIOB
#define A_Pin GPIO_PIN_8
#define A_GPIO_Port GPIOB
#define B_Pin GPIO_PIN_9
#define B_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
