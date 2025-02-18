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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "TB6600_Driver.h"
#include "EEPROM_lib.h"
#include "RS232_Driver.h"

#include "fatfs_sd.h"
#include "string.h"

#include "fonts.h"
#include "ssd1306.h"

#include "stdio.h"
#include "stdlib.h"

#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

// EEPROM TYPEDEF
EEPROM_t eeprom1;
EEPROM_t eeprom2;

// RS232 TYPEDEF
Data_Get_t command;

// DRIVER STEPPER TYPEDEF
Driver_t Stepper_Driver1;
Driver_t Stepper_Driver2;
Driver_t Stepper_Driver3;
Driver_t Stepper_Driver4;
Driver_t Stepper_Driver5;
Driver_t Stepper_Driver6;

// OLED MENU TYPEDEF
typedef enum{
	MENU1,	// RS-232 Communication Status
	MENU2,	// EEPROM Available Memory
}Oled_Menu_t;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*SYSTEM CONFIGURATION*/
//----------------------
//#define USE_EEPROM
//#define USE_RS232
//#define USE_OLED
//#define USE_STEPPER
//#define USE_SDCARD
//#define USE_ENCODER
//----------------------


/*EEPROM ADDRESS SET*/
//-----------------------------------
#define EEPROM1_ADDRESS					0xA0	// PRIMARY EEPROM ADDRESS
#define EEPROM2_ADDRESS					0xA1	// SECONDARY EEPROM ADDRESS

#define SP_POS_BYTE_ADDR				0x00	// START POINT POSITION ADDR
#define EP_POS_BYTE_ADDR				0x18	// END POINT POSITION ADDR
#define SP_ANG_BYTE_ADDR				0x30	// START POINT ANGLE ADDR
#define EP_ANG_BYTE_ADDR				0x48	// END POINT ANGLE ADDR
#define PATTERN_BYTE_ADDR				0x60	// PATTERN ADDR
//-----------------------------------


/*EEPROM PATTERN SET*/
//-----------------------------------
#define LINEAR_PATTERN					0x01
#define CIRCULAR_PATTERN				0x02
#define ZIGZAG_PATTERN					0x03
//-----------------------------------


/*ROBOT TEST SET*/
//-------------------
//#define EEPROM_TEST
#define RS232_TEST
//#define STEPPER_TEST
//#define SDCARD_TEST
//#define ENCODER_TEST
//#define OLED_TEST
//-------------------

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/*EEPROM VARIABLE*/
////////////////////////////////////////
double 
array_pos_start[3],
array_pos_end[3],
array_ang_start[6],
array_ang_end[6];

uint8_t
page_data_byte[128],

saved_pos[24], read_saved_pos[24],
read_saved_posX[8],
read_saved_posY[8],
read_saved_posZ[8],

saved_angle[48], read_saved_angle[48],
read_saved_angle1[8],
read_saved_angle2[8],
read_saved_angle3[8],
read_saved_angle4[8],
read_saved_angle5[8],
read_saved_angle6[8],

welding_pattern,
read_saved_pattern[1];
////////////////////////////////////////


/*MOVE VARIABLE*/
///////////////////
double
current_position[3],
move_position[3],
prev_position[3],
delta_move_pos[3],

current_angle[6],
move_angle[6], 
delta_move_ang[6],
prev_angle[6];

uint8_t
current_point,
current_speed;
///////////////////


/*RS232 VARIABLE*/
///////////////////
uint8_t
rx_savepoint,
rx_patterntype[200],
rx_pointtype,
rx_rotate_mode,
rx_rotate_value[8];
///////////////////

/*OLED VARIABLE*/
/////////////////////////
uint8_t selected_menu;

char 
title[] = "WELDING ARM V1";
/////////////////////////

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/*EEPROM CUSTOM FUNCTION*/
//----------------------------------------------------------------------------------------------------------
void Save_WeldingPoint_Position(uint16_t welding_point, uint8_t start_addr, double* pos_data, size_t size);
void Read_WeldingPoint_Position(uint16_t welding_point, uint8_t start_addr, double* stored_data, size_t size);

void Save_WeldingPoint_Angle(uint16_t welding_point, uint8_t start_addr, double* angle_data, size_t size);
void Read_WeldingPoint_Angle(uint16_t welding_point, uint8_t start_addr, double* stored_data, size_t size);

void Save_WeldingPoint_Pattern(uint16_t welding_point, uint8_t select_pattern);
void Read_WeldingPoint_Pattern(uint16_t welding_point, uint8_t store_pattern);

void Read_All_WeldingDataBuyte(uint16_t welding_point);

uint16_t Check_Free_WeldingPoint(void);

void Show_Menu(Oled_Menu_t sel_menu);
//----------------------------------------------------------------------------------------------------------

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*OLED MENU CONFIGURATION*/
//------------------------------------------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM3) Show_Menu(selected_menu);
}
//------------------------------------------------------------

/*RS232 CONFIGURATION*/
//------------------------------------------------------
uint32_t RS232_state, RS232_err_status;
uint8_t dummy_byte[100];
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
	Stepper_Driver1.driver_step = 800;
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
	Stepper_Driver2.driver_step = 800;
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
	Stepper_Driver3.driver_step = 800;
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
	Stepper_Driver4.driver_step = 800;
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
	Stepper_Driver5.driver_step = 800;
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
	Stepper_Driver6.driver_step = 800;
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
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	
	/*EEPROM CONFIGURATION*/
	#ifdef USE_EEPROM
	EEPROM_Init(&hi2c1, &eeprom1, MEM_SIZE_512Kb, EEPROM1_ADDRESS);
	EEPROM_Init(&hi2c1, &eeprom2, MEM_SIZE_512Kb, EEPROM2_ADDRESS);
	#endif
	
	
	/*RS232 COM CONFIGURATION*/
	#ifdef USE_RS232
	RS232_Init(&huart1);
	Start_get_command();
	#endif
	
	
	/*OLED CONFIGURATION*/
	#ifdef USE_OLED
	SSD1306_Init ();
	HAL_TIM_Base_Start_IT(&htim3);
	#endif
	
	
	/*ENCODER CONFIGURATION*/
	#ifdef USE_ENCODER
	
	#endif
	
	
	/*EEPROM TEST*/
	#ifdef EEPROM_TEST
//	for(int i=0; i<512; i++){
//		EEPROM_PageReset(&eeprom1, i);
//		HAL_Delay(50);
//	}
	
	double 
	test_pos_start[3] = {-40.96, 12.15, -37.42},
	test_pos_end[3] = {-20.96, 2.15, -37.42},
	test_angle_start[6] = {70.55, -45.5, 90.58, 150.45, 55.17, 178.77},
	test_angle_end[6] = {60, -20, 70.25, 115.62, 66.18, 122.76};
	
	Save_WeldingPoint_Position(0x00, SP_POS_BYTE_ADDR, test_pos_start, sizeof(test_pos_start));
	Save_WeldingPoint_Position(0x00, EP_POS_BYTE_ADDR, test_pos_end, sizeof(test_pos_end));
	Save_WeldingPoint_Angle(0x00, SP_ANG_BYTE_ADDR, test_angle_start, sizeof(test_angle_start));
	Save_WeldingPoint_Angle(0x00, EP_ANG_BYTE_ADDR, test_angle_end, sizeof(test_angle_end));
	Save_WeldingPoint_Pattern(0x00, LINEAR_PATTERN);
	HAL_Delay(500);
	Read_WeldingPoint_Position(0x00, SP_POS_BYTE_ADDR, array_pos_start, sizeof(array_pos_start));
	Read_WeldingPoint_Position(0x00, EP_POS_BYTE_ADDR, array_pos_end, sizeof(array_pos_end));
	Read_WeldingPoint_Angle(0x00, SP_ANG_BYTE_ADDR, array_ang_start, sizeof(array_ang_start));
	Read_WeldingPoint_Angle(0x00, EP_ANG_BYTE_ADDR, array_ang_end, sizeof(array_ang_end));
	Read_WeldingPoint_Pattern(0x00, welding_pattern);
	HAL_Delay(500);
	Read_All_WeldingDataBuyte(0x00);
  #endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		#ifdef RS232_TEST
//		for(int i=0; i<6; i++){
//			if(i<3) current_position[i] = test_pos_start[i] + move_position[i];
//			current_angle[i]  = test_angle_start[i] + move_angle[i];
//		}
		
		if(command.type == AUTO_HOME){
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		}
		
		else if(command.type == MAPPING){
			
		}
		
		else if(command.type == PREVIEW){
			
		}
		
		else if(command.type == MOVE){
			if(command.control_mode == CARTESIAN_CTRL){
				move_position[command.move_variable-1] = command.move_value;
			}
			
			else if(command.control_mode == JOINT_CTRL){
				move_angle[command.move_variable-4] = command.move_value;
			}
		}
		
		else if(command.type == RUN){
		}
		
		else if(command.type == REQ_DATA){
			HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			Send_requested_data(current_position, current_angle, current_point, NOT_SET, current_speed); 
		}
		
		else if(command.type == SEND_REQ){
		}
		
		else if(command.type == MOTOR_STATE){
		}
		
		else if(command.type == WELDER_STATE){
		}
		
		else if(command.type == FEEDBACK){
		}
		
		else if(command.type == NONE){
		}
		
		else HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		#endif
		
		#ifdef OLED_TEST
		selected_menu = MENU1;
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1151;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 62499;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SD_SS_Pin|DIR_2_Pin|PULSE_3_Pin|DIR_3_Pin
                          |PULSE_4_Pin|DIR_4_Pin|PULSE_5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, SD_CS_Pin|ENABLE_Pin|PULSE_1_Pin|DIR_1_Pin
                          |PULSE_2_Pin|PULSE_6_Pin|DIR_6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC2_A_Pin ENC2_B_Pin */
  GPIO_InitStruct.Pin = ENC2_A_Pin|ENC2_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_SS_Pin DIR_2_Pin PULSE_3_Pin DIR_3_Pin
                           PULSE_4_Pin DIR_4_Pin PULSE_5_Pin */
  GPIO_InitStruct.Pin = SD_SS_Pin|DIR_2_Pin|PULSE_3_Pin|DIR_3_Pin
                          |PULSE_4_Pin|DIR_4_Pin|PULSE_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_CS_Pin ENABLE_Pin PULSE_1_Pin DIR_1_Pin
                           PULSE_2_Pin PULSE_6_Pin DIR_6_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin|ENABLE_Pin|PULSE_1_Pin|DIR_1_Pin
                          |PULSE_2_Pin|PULSE_6_Pin|DIR_6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PHASE_B6_Pin */
  GPIO_InitStruct.Pin = PHASE_B6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PHASE_B6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIR_5_Pin */
  GPIO_InitStruct.Pin = DIR_5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(DIR_5_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*--- SAVE WELDING POINT POSITION VALUE ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Save_WeldingPoint_Position(uint16_t welding_point, uint8_t start_addr, double* pos_data, size_t size){
	for(int i=0; i<24; i++) saved_pos[i] = 0;
	for(size_t i=0; i<size; i++) memcpy(&saved_pos[i*8], &pos_data[i], sizeof(double));
	
	EEPROM_PageWrite(&eeprom1, welding_point, start_addr, saved_pos, sizeof(saved_pos));
	HAL_Delay(10);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- READ WELDING POINT POSITION VALUE ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Read_WeldingPoint_Position(uint16_t welding_point, uint8_t start_addr, double* stored_data, size_t size){
	for(int i=0; i<24; i++) read_saved_pos[i] = 0;
	for(size_t i=0; i<size; i++) stored_data[i] = 0;
	
	EEPROM_PageRead(&eeprom1, welding_point, start_addr, read_saved_pos, sizeof(read_saved_pos));
	
	for(int i=0; i<8; i++){
		read_saved_posX[i] = read_saved_pos[i];
		read_saved_posY[i] = read_saved_pos[i+8];
		read_saved_posZ[i] = read_saved_pos[i+16];
	}
	
	memcpy(&stored_data[0], read_saved_posX, sizeof(double));
	memcpy(&stored_data[1], read_saved_posY, sizeof(double));
	memcpy(&stored_data[2], read_saved_posZ, sizeof(double));
	HAL_Delay(10);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- SAVE WELDING POINT ANGLE VALUE ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Save_WeldingPoint_Angle(uint16_t welding_point, uint8_t start_addr, double* angle_data, size_t size){
	for(int i=0; i<48; i++) saved_angle[i] = 0;
	for(size_t i=0; i<size; i++) memcpy(&saved_angle[i*8], &angle_data[i], sizeof(double));
	
	EEPROM_PageWrite(&eeprom1, welding_point, start_addr, saved_angle, sizeof(saved_angle));
	HAL_Delay(10);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- SAVE WELDING POINT ANGLE VALUE ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Read_WeldingPoint_Angle(uint16_t welding_point, uint8_t start_addr, double* stored_data, size_t size){
	for(int i=0; i<48; i++) read_saved_angle[i] = 0;
	for(size_t i=0; i<size; i++) stored_data[i] = 0;
	
	EEPROM_PageRead(&eeprom1, welding_point, start_addr, read_saved_angle, sizeof(read_saved_angle));
	
	for(int i=0; i<8; i++){
		read_saved_angle1[i] = read_saved_angle[i];
		read_saved_angle2[i] = read_saved_angle[i+8];
		read_saved_angle3[i] = read_saved_angle[i+16];
		read_saved_angle4[i] = read_saved_angle[i+24];
		read_saved_angle5[i] = read_saved_angle[i+32];
		read_saved_angle6[i] = read_saved_angle[i+40];
	}
	
	memcpy(&stored_data[0], read_saved_angle1, sizeof(double));
	memcpy(&stored_data[1], read_saved_angle2, sizeof(double));
	memcpy(&stored_data[2], read_saved_angle3, sizeof(double));
	memcpy(&stored_data[3], read_saved_angle4, sizeof(double));
	memcpy(&stored_data[4], read_saved_angle5, sizeof(double));
	memcpy(&stored_data[5], read_saved_angle6, sizeof(double));
	HAL_Delay(10);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- SAVE WELDING POINT PATTERN ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Save_WeldingPoint_Pattern(uint16_t welding_point, uint8_t select_pattern){
	EEPROM_ByteWrite(&eeprom1, welding_point, PATTERN_BYTE_ADDR, select_pattern, sizeof(select_pattern));
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- READ WELDING POINT PATTERN ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Read_WeldingPoint_Pattern(uint16_t welding_point, uint8_t store_pattern){
	EEPROM_ByteRead(&eeprom1, welding_point, PATTERN_BYTE_ADDR, read_saved_pattern, sizeof(store_pattern));
	welding_pattern = read_saved_pattern[0]; 
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- READ ALL WELDING POINT DATA IN BYTE ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Read_All_WeldingDataBuyte(uint16_t welding_point){
	EEPROM_PageRead(&eeprom1, welding_point, 0x00, page_data_byte, sizeof(page_data_byte));
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- CHECK EEPROM FREE WLEDING POINT ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint16_t Check_Free_WeldingPoint(void){
	return 0;
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- SHOW OLED MENU FUNCTION*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Show_Menu(Oled_Menu_t sel_menu){
	if(sel_menu == MENU1){
		SSD1306_GotoXY(18,10);
		SSD1306_Puts (title, &Font_7x10, SSD1306_COLOR_WHITE);
		
		SSD1306_GotoXY(48,25);
		SSD1306_Puts("RS232", &Font_7x10, SSD1306_COLOR_WHITE);
		
		SSD1306_GotoXY(0,50);
		SSD1306_Puts("State:", &Font_7x10, SSD1306_COLOR_WHITE);
		
		SSD1306_GotoXY(45,50);
		if(RS232_state == 0x00) SSD1306_Puts("reset", &Font_7x10, SSD1306_COLOR_WHITE);
		else if(RS232_state == 0x20) SSD1306_Puts("ready", &Font_7x10, SSD1306_COLOR_WHITE);
		else if(RS232_state == 0x24) SSD1306_Puts("busy", &Font_7x10, SSD1306_COLOR_WHITE);
		else if(RS232_state == 0x21) SSD1306_Puts("busy tx", &Font_7x10, SSD1306_COLOR_WHITE);
		else if(RS232_state == 0x22) SSD1306_Puts("busy rx", &Font_7x10, SSD1306_COLOR_WHITE);
		else if(RS232_state == 0x23) SSD1306_Puts("busy all", &Font_7x10, SSD1306_COLOR_WHITE);
		else if(RS232_state == 0xA0) SSD1306_Puts("timeout", &Font_7x10, SSD1306_COLOR_WHITE);
		else if(RS232_state == 0xE0) SSD1306_Puts("error", &Font_7x10, SSD1306_COLOR_WHITE);
		
		SSD1306_UpdateScreen();
	}
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
