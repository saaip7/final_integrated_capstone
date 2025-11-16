/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Echo_1_Pin GPIO_PIN_9
#define Echo_1_GPIO_Port GPIOE
#define Trig_1_Pin GPIO_PIN_10
#define Trig_1_GPIO_Port GPIOE
#define Echo_2_Pin GPIO_PIN_11
#define Echo_2_GPIO_Port GPIOE
#define Trig_2_Pin GPIO_PIN_12
#define Trig_2_GPIO_Port GPIOE
#define Echo_3_Pin GPIO_PIN_13
#define Echo_3_GPIO_Port GPIOE
#define Trig_3_Pin GPIO_PIN_14
#define Trig_3_GPIO_Port GPIOE
#define Trig_8_Pin GPIO_PIN_12
#define Trig_8_GPIO_Port GPIOD
#define Trig_7_Pin GPIO_PIN_13
#define Trig_7_GPIO_Port GPIOD
#define Trig_6_Pin GPIO_PIN_14
#define Trig_6_GPIO_Port GPIOD
#define Trig_5_Pin GPIO_PIN_15
#define Trig_5_GPIO_Port GPIOD
#define Echo_8_Pin GPIO_PIN_6
#define Echo_8_GPIO_Port GPIOC
#define Echo_7_Pin GPIO_PIN_7
#define Echo_7_GPIO_Port GPIOC
#define Echo_6_Pin GPIO_PIN_8
#define Echo_6_GPIO_Port GPIOC
#define Echo_5_Pin GPIO_PIN_9
#define Echo_5_GPIO_Port GPIOC
#define Echo_4_Pin GPIO_PIN_11
#define Echo_4_GPIO_Port GPIOA
#define Trig_4_Pin GPIO_PIN_12
#define Trig_4_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
