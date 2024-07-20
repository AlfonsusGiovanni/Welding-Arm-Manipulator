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
#include "EEPROM_lib.h"
#include "RS232_Driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//--- EEPROM TYPEDEF ---//
//////////////////////////
EEPROM_t eeprom;
//////////////////////////


//--- RS232 TYPEDEF ---//
//////////////////////////
Data_Get_t command;
//////////////////////////


/*--- DRIVER STEPPER TYPEDEF ---*/
//////////////////////////////////
Driver_t Stepper_Driver1;
Driver_t Stepper_Driver2;
Driver_t Stepper_Driver3;
Driver_t Stepper_Driver4;
Driver_t Stepper_Driver5;
Driver_t Stepper_Driver6;
//////////////////////////////////

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*SYSTEM CONFIGURATION*/
//----------------------
#define USE_EEPROM
#define USE_RS232
#define USE_STEPPER
//----------------------


/*EEPROM ADDRESS SET*/
//----------------------------------
#define EEPROM_ADDRESS					0xA0

#define PATTERN1_POS_PAGE_ADDR 	0x00
#define PATTERN2_POS_PAGE_ADDR 	0x03
#define PATTERN3_POS_PAGE_ADDR 	0x06

#define PATTERN1_ANG_PAGE_ADDR 	0x09
#define PATTERN2_ANG_PAGE_ADDR 	0x10
#define PATTERN3_ANG_PAGE_ADDR 	0x25

#define POINT1_BYTE_ADDR				0x00
#define POINT2_BYTE_ADDR				0x07
#define POINT3_BYTE_ADDR				0x0F
#define POINT4_BYTE_ADDR				0x17
#define POINT5_BYTE_ADDR				0x1F
#define POINT6_BYTE_ADDR				0x27
#define POINT7_BYTE_ADDR				0x2F
#define POINT8_BYTE_ADDR				0x37
//----------------------------------


/*ROBOT TEST SET*/
//-------------------
//#define EEPROM_TEST
#define RS232_TEST
//#define STEPPER_TEST
//-------------------

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/*EEPROM VARIABLE*/
////////////////////////////////////////
double 
array_pos[3],
array_angle[6];

uint8_t
saved_posX[8], read_saved_posX[8],
saved_posY[8], read_saved_posY[8],
saved_posZ[8], read_saved_posZ[8],

saved_angle1[8], read_saved_angle1[8],
saved_angle2[8], read_saved_angle2[8],
saved_angle3[8], read_saved_angle3[8],
saved_angle4[8], read_saved_angle4[8],
saved_angle5[8], read_saved_angle5[8],
saved_angle6[8], read_saved_angle6[8];
////////////////////////////////////////


/*MOVE VARIABLE*/
///////////////////
double
move_position[3],
prev_position[3],
delta_move_pos[3],
move_angle[6], 
delta_move_ang[6],
prev_angle[6];
///////////////////

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/*EEPROM CUSTOM FUNCTION*/
void Save_Pattern_Position(uint16_t page_select, uint8_t start_addr, double* pos_data, uint8_t size);
void Read_Pattern_Position(uint16_t page_select, uint8_t start_addr, double* stored_data, uint8_t size);
void Save_Pattern_Angle(uint16_t page_select, uint8_t start_addr, double* angle_data, uint8_t size);
void Read_Pattern_Angle(uint16_t page_select, uint8_t start_addr, double* stored_data, uint8_t size);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*RS232 CONFIGURATION*/
//------------------------------------------------------
uint32_t RS232_state, RS232_err_status;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	Get_command(&command);
	RS232_state = check_state();
	RS232_err_status = check_error();
}
//------------------------------------------------------

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	
	#ifdef USE_STEPPER
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
	#endif
	
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
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
	
	/*EEPROM CONFIGURATION*/
	//-----------------------------------------------------------
	#ifdef USE_EEPROM
	EEPROM_Init(&hi2c1, &eeprom, MEM_SIZE_256Kb, EEPROM_ADDRESS);
	#endif
	//-----------------------------------------------------------
	
	
	/*RS232 COM CONFIGURATION*/
	//-------------------------
	#ifdef USE_RS232
	RS232_Init(&huart1);
	Start_get_command();
	Get_command(&command);
	//-------------------------
	#endif
	
	
	/*EEPROM TEST*/
	//---------------------------------------------------------------------------------------------
	#ifdef EEPROM_TEST
	double 
	test_pos[3] = {40.96, 12.15, 37.42},
	test_angle[6] = {125.6, 12.56, 1.256, 0.1245, 0.234, 2.34};
	
	Save_Pattern_Position(PATTERN1_POS_PAGE_ADDR, POINT1_BYTE_ADDR, test_pos, sizeof(test_pos));
	Save_Pattern_Angle(PATTERN1_ANG_PAGE_ADDR, POINT1_BYTE_ADDR, test_angle, sizeof(test_angle));
	
	HAL_Delay(500);
	
	Read_Pattern_Position(PATTERN1_POS_PAGE_ADDR, POINT1_BYTE_ADDR, array_pos, sizeof(array_pos));
	Read_Pattern_Angle(PATTERN1_ANG_PAGE_ADDR, POINT1_BYTE_ADDR, array_angle, sizeof(array_angle));
  #endif
	//---------------------------------------------------------------------------------------------
	
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		#ifdef RS232_TEST
		if(command.type == AUTO_HOME){
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			Send_feedback(AUTO_HOME_DONE);
		}
		
		else HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		#endif
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
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  huart1.Init.BaudRate = 38400;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);

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
  HAL_GPIO_WritePin(GPIOB, ENA_A_Pin|ENA_B_Pin|LED_Pin|ENABLE_Pin
                          |PULSE_1_Pin|DIR_1_Pin|PULSE_2_Pin|PULSE_6_Pin
                          |DIR_6_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : ENA_A_Pin ENA_B_Pin LED_Pin ENABLE_Pin
                           PULSE_1_Pin DIR_1_Pin PULSE_2_Pin PULSE_6_Pin
                           DIR_6_Pin */
  GPIO_InitStruct.Pin = ENA_A_Pin|ENA_B_Pin|LED_Pin|ENABLE_Pin
                          |PULSE_1_Pin|DIR_1_Pin|PULSE_2_Pin|PULSE_6_Pin
                          |DIR_6_Pin;
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

/*--- SAVE PATTERN POSITION VALUE ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Save_Pattern_Position(uint16_t page_select, uint8_t start_addr, double* pos_data, uint8_t size){
	uint16_t
	Xpos_page_addr = page_select,
	Ypos_page_addr = page_select + 0x01,
	Zpos_page_adrr = page_select + 0x02;
	
	memcpy(saved_posX, &pos_data[0], sizeof(double));
	memcpy(saved_posY, &pos_data[1], sizeof(double));
	memcpy(saved_posZ, &pos_data[2], sizeof(double));
	
	EEPROM_PageWrite(&eeprom, Xpos_page_addr, start_addr, saved_posX, 8);
	HAL_Delay(5);
	EEPROM_PageWrite(&eeprom, Ypos_page_addr, start_addr, saved_posY, 8);
	HAL_Delay(5);
	EEPROM_PageWrite(&eeprom, Zpos_page_adrr, start_addr, saved_posZ, 8);
	HAL_Delay(5);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- READ PATTERN POSITION VALUE ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Read_Pattern_Position(uint16_t page_select, uint8_t start_addr, double* stored_data, uint8_t size){
	uint16_t
	Xpos_page_addr = page_select,
	Ypos_page_addr = page_select + 0x01,
	Zpos_page_adrr = page_select + 0x02;
	
	EEPROM_PageRead(&eeprom, Xpos_page_addr, start_addr, read_saved_posX, 8);
	EEPROM_PageRead(&eeprom, Ypos_page_addr, start_addr, read_saved_posY, 8);
	EEPROM_PageRead(&eeprom, Zpos_page_adrr, start_addr, read_saved_posZ, 8);
	
	memcpy(&stored_data[0], read_saved_posX, sizeof(double));
	memcpy(&stored_data[1], read_saved_posY, sizeof(double));
	memcpy(&stored_data[2], read_saved_posZ, sizeof(double));
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- SAVE PATTERN ANGLE VALUE ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Save_Pattern_Angle(uint16_t page_select, uint8_t start_addr, double* angle_data, uint8_t size){
	uint16_t
	Angle1_page_addr = page_select,
	Angle2_page_addr = page_select + 0x01,
	Angle3_page_addr = page_select + 0x02,
	Angle4_page_addr = page_select + 0x03,
	Angle5_page_addr = page_select + 0x04,
	Angle6_page_addr = page_select + 0x05;
	
	memcpy(saved_angle1, &angle_data[0], sizeof(double));
	memcpy(saved_angle2, &angle_data[1], sizeof(double));
	memcpy(saved_angle3, &angle_data[2], sizeof(double));
	memcpy(saved_angle4, &angle_data[3], sizeof(double));
	memcpy(saved_angle5, &angle_data[4], sizeof(double));
	memcpy(saved_angle6, &angle_data[5], sizeof(double));
	
	EEPROM_PageWrite(&eeprom, Angle1_page_addr, start_addr, saved_angle1, sizeof(saved_angle1));
	HAL_Delay(5);
	EEPROM_PageWrite(&eeprom, Angle2_page_addr, start_addr, saved_angle2, sizeof(saved_angle2));
	HAL_Delay(5);
	EEPROM_PageWrite(&eeprom, Angle3_page_addr, start_addr, saved_angle3, sizeof(saved_angle3));
	HAL_Delay(5);
	EEPROM_PageWrite(&eeprom, Angle4_page_addr, start_addr, saved_angle4, sizeof(saved_angle4));
	HAL_Delay(5);
	EEPROM_PageWrite(&eeprom, Angle5_page_addr, start_addr, saved_angle5, sizeof(saved_angle5));
	HAL_Delay(5);
	EEPROM_PageWrite(&eeprom, Angle6_page_addr, start_addr, saved_angle6, sizeof(saved_angle6));
	HAL_Delay(5);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- SAVE PATTERN ANGLE VALUE ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Read_Pattern_Angle(uint16_t page_select, uint8_t start_addr, double* stored_data, uint8_t size){
	uint16_t
	Angle1_page_addr = page_select,
	Angle2_page_addr = page_select + 0x01,
	Angle3_page_addr = page_select + 0x02,
	Angle4_page_addr = page_select + 0x03,
	Angle5_page_addr = page_select + 0x04,
	Angle6_page_addr = page_select + 0x05;
	
	EEPROM_PageRead(&eeprom, Angle1_page_addr, start_addr, read_saved_angle1, sizeof(read_saved_angle1));
	EEPROM_PageRead(&eeprom, Angle2_page_addr, start_addr, read_saved_angle2, sizeof(read_saved_angle2));
	EEPROM_PageRead(&eeprom, Angle3_page_addr, start_addr, read_saved_angle3, sizeof(read_saved_angle3));
	EEPROM_PageRead(&eeprom, Angle4_page_addr, start_addr, read_saved_angle4, sizeof(read_saved_angle4));
	EEPROM_PageRead(&eeprom, Angle5_page_addr, start_addr, read_saved_angle5, sizeof(read_saved_angle5));
	EEPROM_PageRead(&eeprom, Angle6_page_addr, start_addr, read_saved_angle6, sizeof(read_saved_angle6));
	
	memcpy(&stored_data[0], read_saved_angle1, sizeof(double));
	memcpy(&stored_data[1], read_saved_angle2, sizeof(double));
	memcpy(&stored_data[2], read_saved_angle3, sizeof(double));
	memcpy(&stored_data[3], read_saved_angle4, sizeof(double));
	memcpy(&stored_data[4], read_saved_angle5, sizeof(double));
	memcpy(&stored_data[5], read_saved_angle6, sizeof(double));
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


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
