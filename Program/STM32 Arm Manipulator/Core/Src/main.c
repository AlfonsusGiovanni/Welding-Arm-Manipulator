/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "TB6600_Driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/*--- DRIVER STEPPER TYPEDEF ---*/
//-------------------------------
Driver_t Stepper_Driver1;
Driver_t Stepper_Driver2;
Driver_t Stepper_Driver3;
Driver_t Stepper_Driver4;
Driver_t Stepper_Driver5;
Driver_t Stepper_Driver6;
//-------------------------------


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

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
	
	/*--- COFIGURATION FOR STEPPER DRIVER 1 ---*/
	//-------------------------------------------------
	Stepper_Driver1.DIR_PORT = DIR_1_GPIO_Port;
	Stepper_Driver1.DIR_PIN = DIR_1_Pin;
	Stepper_Driver1.PULSE_PORT = PULSE_1_GPIO_Port;
	Stepper_Driver1.PULSE_PIN = PULSE_1_Pin;
	Stepper_Driver1.ENA_PORT = ENABLE_GPIO_Port;
	Stepper_Driver1.ENA_PIN = ENABLE_Pin;
	Stepper_Driver1.driver_step = 200;
	Driver_set_power(&Stepper_Driver1, DRIVER_ENABLE);
	//-------------------------------------------------
	
	
	/*--- COFIGURATION FOR STEPPER DRIVER 2 ---*/
	//-------------------------------------------------
	Stepper_Driver2.DIR_PORT = DIR_2_GPIO_Port;
	Stepper_Driver2.DIR_PIN = DIR_2_Pin;
	Stepper_Driver2.PULSE_PORT = PULSE_2_GPIO_Port;
	Stepper_Driver2.PULSE_PIN = PULSE_2_Pin;
	Stepper_Driver2.ENA_PORT = ENABLE_GPIO_Port;
	Stepper_Driver2.ENA_PIN = ENABLE_Pin;
	Stepper_Driver2.driver_step = 200;
	Driver_set_power(&Stepper_Driver2, DRIVER_ENABLE);
	//-------------------------------------------------
	
	
	/*--- COFIGURATION FOR STEPPER DRIVER 3 ---*/
	//-------------------------------------------------
	Stepper_Driver3.DIR_PORT = DIR_3_GPIO_Port;
	Stepper_Driver3.DIR_PIN = DIR_3_Pin;
	Stepper_Driver3.PULSE_PORT = PULSE_3_GPIO_Port;
	Stepper_Driver3.PULSE_PIN = PULSE_3_Pin;
	Stepper_Driver3.ENA_PORT = ENABLE_GPIO_Port;
	Stepper_Driver3.ENA_PIN = ENABLE_Pin;
	Stepper_Driver3.driver_step = 200;
	Driver_set_power(&Stepper_Driver3, DRIVER_ENABLE);
	//-------------------------------------------------
	
	
	/*--- COFIGURATION FOR STEPPER DRIVER 4 ---*/
	//-------------------------------------------------
	Stepper_Driver4.DIR_PORT = DIR_4_GPIO_Port;
	Stepper_Driver4.DIR_PIN = DIR_4_Pin;
	Stepper_Driver4.PULSE_PORT = PULSE_4_GPIO_Port;
	Stepper_Driver4.PULSE_PIN = PULSE_4_Pin;
	Stepper_Driver4.ENA_PORT = ENABLE_GPIO_Port;
	Stepper_Driver4.ENA_PIN = ENABLE_Pin;
	Stepper_Driver4.driver_step = 200;
	Driver_set_power(&Stepper_Driver4, DRIVER_ENABLE);
	//-------------------------------------------------
	
	
	/*--- COFIGURATION FOR STEPPER DRIVER 5 ---*/
	//-------------------------------------------------
	Stepper_Driver5.DIR_PORT = DIR_5_GPIO_Port;
	Stepper_Driver5.DIR_PIN = DIR_5_Pin;
	Stepper_Driver5.PULSE_PORT = PULSE_5_GPIO_Port;
	Stepper_Driver5.PULSE_PIN = PULSE_5_Pin;
	Stepper_Driver5.ENA_PORT = ENABLE_GPIO_Port;
	Stepper_Driver5.ENA_PIN = ENABLE_Pin;
	Stepper_Driver5.driver_step = 200;
	Driver_set_power(&Stepper_Driver5, DRIVER_ENABLE);
	//-------------------------------------------------
	
	
	/*--- COFIGURATION FOR STEPPER DRIVER 6 ---*/
	//-------------------------------------------------
	Stepper_Driver6.DIR_PORT = DIR_6_GPIO_Port;
	Stepper_Driver6.DIR_PIN = DIR_6_Pin;
	Stepper_Driver6.PULSE_PORT = PULSE_6_GPIO_Port;
	Stepper_Driver6.PULSE_PIN = PULSE_6_Pin;
	Stepper_Driver6.ENA_PORT = ENABLE_GPIO_Port;
	Stepper_Driver6.ENA_PIN = ENABLE_Pin;
	Stepper_Driver6.driver_step = 200;
	Driver_set_power(&Stepper_Driver6, DRIVER_ENABLE);
	//-------------------------------------------------
	
	
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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL2;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, ENA_A_Pin|ENA_B_Pin|ENABLE_Pin|PULSE_1_Pin
                          |DIR_1_Pin|PULSE_2_Pin|PULSE_6_Pin|DIR_6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, DIR_2_Pin|PULSE_3_Pin|DIR_3_Pin|PULSE_4_Pin
                          |DIR_4_Pin|PULSE_5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PHASE_A1_Pin PHASE_B1_Pin PHASE_A2_Pin */
  GPIO_InitStruct.Pin = PHASE_A1_Pin|PHASE_B1_Pin|PHASE_A2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PHASE_B2_Pin PHASE_A3_Pin PHASE_B3_Pin PHASE_A4_Pin
                           PHASE_B4_Pin PHASE_A5_Pin PHASE_B5_Pin PHASE_A6_Pin */
  GPIO_InitStruct.Pin = PHASE_B2_Pin|PHASE_A3_Pin|PHASE_B3_Pin|PHASE_A4_Pin
                          |PHASE_B4_Pin|PHASE_A5_Pin|PHASE_B5_Pin|PHASE_A6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : ENA_A_Pin ENA_B_Pin ENABLE_Pin PULSE_1_Pin
                           DIR_1_Pin PULSE_2_Pin PULSE_6_Pin DIR_6_Pin */
  GPIO_InitStruct.Pin = ENA_A_Pin|ENA_B_Pin|ENABLE_Pin|PULSE_1_Pin
                          |DIR_1_Pin|PULSE_2_Pin|PULSE_6_Pin|DIR_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PHASE_B6_Pin */
  GPIO_InitStruct.Pin = PHASE_B6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PHASE_B6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR_2_Pin PULSE_3_Pin DIR_3_Pin PULSE_4_Pin
                           DIR_4_Pin PULSE_5_Pin */
  GPIO_InitStruct.Pin = DIR_2_Pin|PULSE_3_Pin|DIR_3_Pin|PULSE_4_Pin
                          |DIR_4_Pin|PULSE_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR_5_Pin */
  GPIO_InitStruct.Pin = DIR_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(DIR_5_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

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
