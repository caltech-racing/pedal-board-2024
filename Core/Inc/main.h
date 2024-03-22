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
#define HVIL_OUT_OBSERVE_Pin GPIO_PIN_2
#define HVIL_OUT_OBSERVE_GPIO_Port GPIOE
#define N_HVIL_FLOAT_Pin GPIO_PIN_3
#define N_HVIL_FLOAT_GPIO_Port GPIOE
#define HVIL_IN_OBSERVE_Pin GPIO_PIN_4
#define HVIL_IN_OBSERVE_GPIO_Port GPIOE
#define N_HVIL_RESET_Pin GPIO_PIN_5
#define N_HVIL_RESET_GPIO_Port GPIOE
#define HVIL_LOGIC_OBSERVE_Pin GPIO_PIN_6
#define HVIL_LOGIC_OBSERVE_GPIO_Port GPIOE
#define DEBUG_LED_2_Pin GPIO_PIN_0
#define DEBUG_LED_2_GPIO_Port GPIOA
#define BTN2_EXTI_Pin GPIO_PIN_1
#define BTN2_EXTI_GPIO_Port GPIOA
#define BTN1_EXTI_Pin GPIO_PIN_2
#define BTN1_EXTI_GPIO_Port GPIOA
#define DEBUG_LED_1_Pin GPIO_PIN_3
#define DEBUG_LED_1_GPIO_Port GPIOA
#define ShiftReg_CLK_Pin GPIO_PIN_5
#define ShiftReg_CLK_GPIO_Port GPIOA
#define ShiftReg_Latch_Pin GPIO_PIN_6
#define ShiftReg_Latch_GPIO_Port GPIOA
#define N_BRAKES_ON_Pin GPIO_PIN_14
#define N_BRAKES_ON_GPIO_Port GPIOE
#define RANGE_ANOMALY_Pin GPIO_PIN_15
#define RANGE_ANOMALY_GPIO_Port GPIOE
#define ShiftReg_MOSI_Pin GPIO_PIN_5
#define ShiftReg_MOSI_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
