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
#include "LCD_I2C.h"
#include "Keypad_Driver.h"
#include "RS232_Driver.h"
#include "EEPROM_lib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//--- LCD MENU TYPEDEF ---//
////////////////////////////
typedef enum{
	BOOTING_MENU,
	AUTO_HOME_MENU,
	HOME_MENU,
	PREP_MENU,
	RUNNING_MENU_1,
	RUNNING_MENU_2,
	RUNNING_MENU_3,
	PAUSE_MENU,
	ERROR_MENU,
}LCD_Menu_t;
////////////////////////////


//--- KEYPAD TYPEDEF ---//
//////////////////////////
Keypad_t keypad;
//////////////////////////


//--- EEPROM TYPEDEF ---//
//////////////////////////
EEPROM_t eeprom;
//////////////////////////


//--- RS232 TYPEDEF ---//
//////////////////////////
Data_Get_t command;
//////////////////////////

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */

/*PENDANT GLOBAL VARIABLE*/
//////////////////////////////
bool 
booting_menu 	= false,
home_menu 		= false,
running 			= false,
stop 					= false;

float
angle_value[6], angle_limit,
pos_value[3], pos_limit,
A1, A2, A3, A4, A5, A6,
moveX, moveY, moveZ,
distance_val,
max_distance = 300;

char
string_distance[20] = "0";
//////////////////////////////


/*MENU MODE VARIABLE*/
///////////////////////////////////
float
increase_decrease_value = 2.5;

char
pos_ctrl[] 				= "POS",
angle_ctrl[] 			= "ANG",

cont_change[] 		= "CONT",
dist_change[] 		= "DIST",
step_change[] 		= "STEP",

pattern_mode[] 		= "PATTERN",
repeat_mode[] 		= "REPEAT",

low_speed[] 			= "LOW",
med_speed[] 			= "MED",
high_speed[] 			= "HIGH";

uint8_t 
ctrl_mode_counter,
change_value_counter,
move_var_counter,
run_mode_counter,
speed_mode_counter,
add_col;

char num_keys[] = {
'1', '2', '3', '4', '5',
'6', '7', '8', '9', '0',
'.'
};
///////////////////////////////////

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

void show_menu(LCD_Menu_t menu);
void ui_handler(void);
bool check_numkeys_pressed(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*LCD MENU CONFIGURATION*/
//------------------------------------------------------------
uint8_t 
prev_select_menu,
select_menu,
prev_sub_menu,
sub_menu;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2) show_menu(select_menu);
}
//------------------------------------------------------------


/*KEYPAD CONFIGURATION*/
//-------------------------------
unsigned long 
prev_tick,
debounce = 200;

const uint8_t
num_rows = 5, 
num_cols = 4;

char key[num_rows][num_cols] = {
	{'Q', 'W', 'R', 'T'},
	{'1', '2', '3', 'U'}, 
	{'4', '5', '6', 'D'},
	{'7', '8', '9', '.'},
	{'<', '0', '>', '#'}
};

char 
prev_keys,
keys;
//-------------------------------


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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	
	/*LCD CONFIGURATION*/
	//----------------------------
	lcd_init(&hi2c2);
	select_menu = BOOTING_MENU;
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_Delay(500);
	//----------------------------
	
	
	/*EEPROM CONFIGURATION*/
	//-------------------------------------------------
	EEPROM_Init(&hi2c1, &eeprom, MEM_SIZE_256Kb, 0xA0);
	HAL_Delay(500);
	//-------------------------------------------------
	
	
	/*RS232 COM CONFIGURATION*/
	//-------------------------
	RS232_Init(&huart1);
	Start_get_command();
	Get_command(&command);
	HAL_Delay(500);
	//-------------------------
	
	
	/*KEYPAD CONFIGURATION*/	
	//----------------------------------------------------------
	keypad.row_port[0] = GPIOA, keypad.row_pin[0] = GPIO_PIN_1;
	keypad.row_port[1] = GPIOA, keypad.row_pin[1] = GPIO_PIN_2;
	keypad.row_port[2] = GPIOA, keypad.row_pin[2] = GPIO_PIN_3;
	keypad.row_port[3] = GPIOA, keypad.row_pin[3] = GPIO_PIN_4;
	keypad.row_port[4] = GPIOA, keypad.row_pin[4] = GPIO_PIN_5;
	
	keypad.col_port[0] = GPIOC, keypad.col_pin[0] = GPIO_PIN_13;
	keypad.col_port[1] = GPIOC, keypad.col_pin[1] = GPIO_PIN_14;
	keypad.col_port[2] = GPIOC, keypad.col_pin[2] = GPIO_PIN_15;
	keypad.col_port[3] = GPIOA, keypad.col_pin[3] = GPIO_PIN_0;
	
	Keypad_Init(&keypad, makeKeymap(key), num_rows, num_cols);
	HAL_Delay(500);
	//----------------------------------------------------------
	
	
	/*ALL PREPARATION COMPLETED - BOOTING DONE*/
	//------------------------------------------
	select_menu = HOME_MENU;
	lcd_clear();
	//------------------------------------------
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		ui_handler();
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
  hi2c1.Init.ClockSpeed = 100000;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 287;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 49999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*--- USER INTERFACE HANDLER ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void ui_handler(void){
	keys = Keypad_Read(&keypad);
	if(keys != prev_keys && keys != 0x00) lcd_clear();
	
	if(select_menu == AUTO_HOME_MENU){
		while(1){
			keys = Keypad_Read(&keypad);
			if(keys == '#' && prev_keys != keys){
				lcd_clear();
				break;
			}
		}
		
		while(1){
			Send_auto_home();
			select_menu = HOME_MENU;
			if(command.feedback == AUTO_HOME_DONE){
				HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
				break;
			}
		}
	}
	
	else if(select_menu == HOME_MENU){
		// AUTO HOME
		if(keys == 'Q' && prev_keys != keys){
			select_menu = AUTO_HOME_MENU;
		}
		
		// CHANGE MOVE CONTROL
		if(keys == 'W' && prev_keys != keys){
			ctrl_mode_counter+=1;
			if(ctrl_mode_counter > 1) ctrl_mode_counter = 0;
		}
		
		// SELECT MOVE VARIABLE
		if(keys == 'R' && prev_keys != keys){
			move_var_counter+=1;
			if(ctrl_mode_counter == 0 && move_var_counter > 2) move_var_counter = 0;
			else if(ctrl_mode_counter == 1 && move_var_counter > 5) move_var_counter = 0;
		}
		
		// SELECT MOVE VALUE
		if(keys == 'T' && prev_keys != keys){
			change_value_counter+=1;
			if(change_value_counter > 2) change_value_counter = 0;
		}
		
		// MOVE VALUE BY DISTANCE
		if((change_value_counter == 1 && check_numkeys_pressed() == true) || keys == '<'){				
			while(1){
				if(keys != prev_keys && keys != 0x00) lcd_clear();
				
				// INSERT VALUE
				if(check_numkeys_pressed() == true && prev_keys != keys){
					string_distance[add_col] = keys;
					add_col+=1;
				}
				
				// DELETE VALUE
				if(keys == '<' && prev_keys != keys){
					if(add_col > 0) add_col-=1;
					
					if(add_col == 0) string_distance[add_col] = '0';
					else string_distance[add_col] = ' ';
				}
				
				// SAVE VALUE
				if((keys == '>') && prev_keys != keys){
					sscanf(string_distance, "%f", &distance_val);
					if(distance_val >= max_distance || distance_val <= -max_distance){
						char max_dist[] = "300";
						memcpy(string_distance, max_dist, sizeof(max_dist));
						distance_val = max_distance;
					}
					break;
				}
	
				prev_keys = keys;
			}
		}
		
		// MOVE VALUE BY STEP
		else if(change_value_counter == 2) increase_decrease_value = 0.5;
		
		// MOVE VALUE CONTINUOUS
		else increase_decrease_value = 2.5;
		
		// INCLREASE VALUE
		if(keys == 'U' && HAL_GetTick() - prev_tick > debounce){
			if(ctrl_mode_counter == 0){
				for(int i=0; i<3; i++) if(move_var_counter == i){
					if(change_value_counter != 1) pos_value[i] += increase_decrease_value;
					else pos_value[i] += distance_val;
				}
			}
			else{
				for(int i=0; i<6; i++) if(move_var_counter == i){
					if(change_value_counter != 1) angle_value[i] += increase_decrease_value;
					else angle_value[i] += distance_val;
				}
			}
			prev_tick = HAL_GetTick();
		}
		
		// DECREASE VALUE
		if(keys == 'D' && HAL_GetTick() - prev_tick > debounce){
			if(ctrl_mode_counter == 0){
				for(int i=0; i<3; i++) if(move_var_counter == i){
					if(change_value_counter != 1) pos_value[i] -= increase_decrease_value;
					else pos_value[i] -= distance_val;
				}
			}
			else{
				for(int i=0; i<6; i++) if(move_var_counter == i){
					if(change_value_counter != 1) angle_value[i] -= increase_decrease_value;
					else angle_value[i] -= distance_val;
				}
			}
			prev_tick = HAL_GetTick();
		}
		
		// MOVE TO PREPARATION MENU
		if(keys == '#' && prev_keys != keys){
			select_menu = PREP_MENU;
		}
	}
	
	else if(select_menu == PREP_MENU){
		// BACT TO HOME MENU
		if(keys == '.' && prev_keys != keys){
			select_menu = HOME_MENU;
		}
	}
	
	prev_keys = keys;
	prev_select_menu = select_menu;
	prev_sub_menu = sub_menu;
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- LCD MENU ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void show_menu(LCD_Menu_t menu){
	if(menu == BOOTING_MENU){
		lcd_set_cursor(3, 1);
		lcd_printstr("SYSTEM BOOTING");
		lcd_set_cursor(4, 2);
		lcd_printstr("PLEASE  WAIT");
	}
	
	else if(menu == AUTO_HOME_MENU){
		lcd_set_cursor(3, 1);
		lcd_printstr("PRESS  ENTER");
		lcd_set_cursor(1, 2);
		lcd_printstr("TO START AUTO HOME");
	}
	
	else if(menu == HOME_MENU){		
		if(ctrl_mode_counter == 0){
			sub_menu = 2;
			
			lcd_set_cursor(9, move_var_counter);
			lcd_printstr("<");
			
			lcd_set_cursor(0, 0);	
			lcd_printstr("XP:");
			lcd_printfloat(pos_value[0], 1);
			lcd_set_cursor(0, 1);
			lcd_printstr("YP:");
			lcd_printfloat(pos_value[1], 1);
			lcd_set_cursor(0, 2);
			lcd_printstr("ZP:");
			lcd_printfloat(pos_value[2], 1);
			
			lcd_set_cursor(17, 3);
			lcd_printstr(pos_ctrl);
		}
		
		else if(ctrl_mode_counter == 1){
			sub_menu = 3;
			
			if(move_var_counter < 3){
				lcd_set_cursor(9, move_var_counter);
				lcd_printstr("<");
			}
			
			else if(move_var_counter > 2){
				lcd_set_cursor(19, move_var_counter-3);
				lcd_printstr("<");
			}
			
			lcd_set_cursor(0, 0);			
			lcd_printstr("A1:");			
			lcd_printfloat(angle_value[0], 1);				
			lcd_set_cursor(0, 1);			
			lcd_printstr("A2:");			
			lcd_printfloat(angle_value[1], 1);				
			lcd_set_cursor(0, 2);			
			lcd_printstr("A3:");			
			lcd_printfloat(angle_value[2], 1);				
			
			lcd_set_cursor(10, 0);
			lcd_printstr("A4:");
			lcd_printfloat(angle_value[3], 1);
			lcd_set_cursor(10, 1);
			lcd_printstr("A5:");
			lcd_printfloat(angle_value[4], 1);
			lcd_set_cursor(10, 2);
			lcd_printstr("A6:");
			lcd_printfloat(angle_value[5], 1);
			
			lcd_set_cursor(17, 3);
			lcd_printstr(angle_ctrl);
		}
		
		lcd_set_cursor(0, 3);	
		if(change_value_counter == 0x00) lcd_printstr(cont_change);
		if(change_value_counter == 0x01){
			lcd_printstr(dist_change);
			lcd_set_cursor(5, 3);	
			lcd_printstr(string_distance);
		}
		if(change_value_counter == 0x02) lcd_printstr(step_change);
		
		lcd_set_cursor(8, 3);
		if(change_value_counter != 0x01) lcd_printstr("HOME");
	}
	
	else if(menu == PREP_MENU){
		sub_menu = 4;
		
		lcd_set_cursor(0, 0);
		lcd_printstr("Memory");
		lcd_set_cursor(0, 1);
		lcd_printstr("Speed");
		
		lcd_set_cursor(0, 3);
		if(run_mode_counter == 0) lcd_printstr(pattern_mode);
		if(run_mode_counter == 1) lcd_printstr(repeat_mode);
	}
	
	else if(menu == RUNNING_MENU_1){
	}
	
	else if(menu == RUNNING_MENU_2){
	}
	
	else if(menu == RUNNING_MENU_3){
	}
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*--- CHECK PRESSED NUMKEYS ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool check_numkeys_pressed(void){
	keys = Keypad_Read(&keypad);
	
	for(int i=0; i<sizeof(num_keys); i++) if(keys == num_keys[i]) return true;
	return false;
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
