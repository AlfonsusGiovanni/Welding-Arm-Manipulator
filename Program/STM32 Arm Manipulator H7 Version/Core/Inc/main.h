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
#include "stm32h7xx_hal.h"

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
#define LIMIT1_Pin GPIO_PIN_2
#define LIMIT1_GPIO_Port GPIOE
#define LIMIT1_EXTI_IRQn EXTI2_IRQn
#define LIMIT2_Pin GPIO_PIN_3
#define LIMIT2_GPIO_Port GPIOE
#define LIMIT2_EXTI_IRQn EXTI3_IRQn
#define LIMIT3_Pin GPIO_PIN_4
#define LIMIT3_GPIO_Port GPIOE
#define LIMIT3_EXTI_IRQn EXTI4_IRQn
#define LIMIT4_Pin GPIO_PIN_5
#define LIMIT4_GPIO_Port GPIOE
#define LIMIT4_EXTI_IRQn EXTI9_5_IRQn
#define LIMIT5_Pin GPIO_PIN_6
#define LIMIT5_GPIO_Port GPIOE
#define LIMIT5_EXTI_IRQn EXTI9_5_IRQn
#define LIMIT6_Pin GPIO_PIN_13
#define LIMIT6_GPIO_Port GPIOC
#define LIMIT6_EXTI_IRQn EXTI15_10_IRQn
#define ENABLE1_Pin GPIO_PIN_0
#define ENABLE1_GPIO_Port GPIOC
#define ENABLE2_Pin GPIO_PIN_1
#define ENABLE2_GPIO_Port GPIOC
#define ENC6A_Pin GPIO_PIN_0
#define ENC6A_GPIO_Port GPIOA
#define ENC6B_Pin GPIO_PIN_1
#define ENC6B_GPIO_Port GPIOA
#define DIR6_Pin GPIO_PIN_2
#define DIR6_GPIO_Port GPIOA
#define PUL6_Pin GPIO_PIN_4
#define PUL6_GPIO_Port GPIOA
#define ENC3A_Pin GPIO_PIN_5
#define ENC3A_GPIO_Port GPIOA
#define ENC5A_Pin GPIO_PIN_6
#define ENC5A_GPIO_Port GPIOA
#define ENC5B_Pin GPIO_PIN_7
#define ENC5B_GPIO_Port GPIOA
#define DIR5_Pin GPIO_PIN_4
#define DIR5_GPIO_Port GPIOC
#define PUL5_Pin GPIO_PIN_0
#define PUL5_GPIO_Port GPIOB
#define DIR4_Pin GPIO_PIN_2
#define DIR4_GPIO_Port GPIOB
#define PUL4_Pin GPIO_PIN_8
#define PUL4_GPIO_Port GPIOE
#define ENC4A_Pin GPIO_PIN_9
#define ENC4A_GPIO_Port GPIOE
#define DIR3_Pin GPIO_PIN_10
#define DIR3_GPIO_Port GPIOE
#define ENC4B_Pin GPIO_PIN_11
#define ENC4B_GPIO_Port GPIOE
#define PUL3_Pin GPIO_PIN_12
#define PUL3_GPIO_Port GPIOE
#define DIR2_Pin GPIO_PIN_14
#define DIR2_GPIO_Port GPIOE
#define PUL2_Pin GPIO_PIN_10
#define PUL2_GPIO_Port GPIOB
#define DIR1_Pin GPIO_PIN_12
#define DIR1_GPIO_Port GPIOB
#define PUL1_Pin GPIO_PIN_14
#define PUL1_GPIO_Port GPIOB
#define MIG_SW_IN_Pin GPIO_PIN_8
#define MIG_SW_IN_GPIO_Port GPIOD
#define ENC1A_Pin GPIO_PIN_12
#define ENC1A_GPIO_Port GPIOD
#define ENC1B_Pin GPIO_PIN_13
#define ENC1B_GPIO_Port GPIOD
#define ENC2A_Pin GPIO_PIN_6
#define ENC2A_GPIO_Port GPIOC
#define ENC2B_Pin GPIO_PIN_7
#define ENC2B_GPIO_Port GPIOC
#define SDMMC1_BSP_Pin GPIO_PIN_0
#define SDMMC1_BSP_GPIO_Port GPIOD
#define ENC3B_Pin GPIO_PIN_3
#define ENC3B_GPIO_Port GPIOB
#define I2C_SCL_Pin GPIO_PIN_6
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_7
#define I2C_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
