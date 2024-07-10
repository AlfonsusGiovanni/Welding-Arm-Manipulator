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
#define PHASE_A1_Pin GPIO_PIN_13
#define PHASE_A1_GPIO_Port GPIOC
#define PHASE_B1_Pin GPIO_PIN_14
#define PHASE_B1_GPIO_Port GPIOC
#define PHASE_A2_Pin GPIO_PIN_15
#define PHASE_A2_GPIO_Port GPIOC
#define PHASE_B2_Pin GPIO_PIN_0
#define PHASE_B2_GPIO_Port GPIOA
#define PHASE_A3_Pin GPIO_PIN_1
#define PHASE_A3_GPIO_Port GPIOA
#define PHASE_B3_Pin GPIO_PIN_2
#define PHASE_B3_GPIO_Port GPIOA
#define PHASE_A4_Pin GPIO_PIN_3
#define PHASE_A4_GPIO_Port GPIOA
#define PHASE_B4_Pin GPIO_PIN_4
#define PHASE_B4_GPIO_Port GPIOA
#define PHASE_A5_Pin GPIO_PIN_5
#define PHASE_A5_GPIO_Port GPIOA
#define PHASE_B5_Pin GPIO_PIN_6
#define PHASE_B5_GPIO_Port GPIOA
#define PHASE_A6_Pin GPIO_PIN_7
#define PHASE_A6_GPIO_Port GPIOA
#define ENA_A_Pin GPIO_PIN_0
#define ENA_A_GPIO_Port GPIOB
#define ENA_B_Pin GPIO_PIN_1
#define ENA_B_GPIO_Port GPIOB
#define PHASE_B6_Pin GPIO_PIN_10
#define PHASE_B6_GPIO_Port GPIOB
#define ENABLE_Pin GPIO_PIN_12
#define ENABLE_GPIO_Port GPIOB
#define PULSE_1_Pin GPIO_PIN_13
#define PULSE_1_GPIO_Port GPIOB
#define DIR_1_Pin GPIO_PIN_14
#define DIR_1_GPIO_Port GPIOB
#define PULSE_2_Pin GPIO_PIN_15
#define PULSE_2_GPIO_Port GPIOB
#define DIR_2_Pin GPIO_PIN_8
#define DIR_2_GPIO_Port GPIOA
#define PULSE_3_Pin GPIO_PIN_9
#define PULSE_3_GPIO_Port GPIOA
#define DIR_3_Pin GPIO_PIN_10
#define DIR_3_GPIO_Port GPIOA
#define PULSE_4_Pin GPIO_PIN_11
#define PULSE_4_GPIO_Port GPIOA
#define DIR_4_Pin GPIO_PIN_12
#define DIR_4_GPIO_Port GPIOA
#define PULSE_5_Pin GPIO_PIN_15
#define PULSE_5_GPIO_Port GPIOA
#define DIR_5_Pin GPIO_PIN_3
#define DIR_5_GPIO_Port GPIOB
#define PULSE_6_Pin GPIO_PIN_4
#define PULSE_6_GPIO_Port GPIOB
#define DIR_6_Pin GPIO_PIN_5
#define DIR_6_GPIO_Port GPIOB
#define RS323_TXD_Pin GPIO_PIN_6
#define RS323_TXD_GPIO_Port GPIOB
#define RS232_RXD_Pin GPIO_PIN_7
#define RS232_RXD_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
