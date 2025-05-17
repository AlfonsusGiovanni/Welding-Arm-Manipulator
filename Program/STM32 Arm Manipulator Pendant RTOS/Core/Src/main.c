/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "LCD_I2C.h"
#include "Keypad_Driver.h"
#include "RS232_Driver.h"
#include "EEPROM_lib.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticEventGroup_t osStaticEventGroupDef_t;
/* USER CODE BEGIN PTD */

//--- LCD MENU TYPEDEF ---//
typedef enum{
	EMERGENCY_MENU1 = 0x01,
	EMERGENCY_MENU2,
	EMERGENCY_MENU3,
	EMERGENCY_MENU4,
	BOOTING_MENU,
	SETTING_MENU,
	CAL1_MENU,
	CAL2_MENU,
	ZEROING_MENU,
	CHANGE_SPEED_MENU,
	HOME_MENU,
	PREP_MENU,
	MAPPING_MENU,
	PREVIEW_MENU,
	WELDING_MENU,
	PAUSE_MENU,
	STOP_MENU,
	HOLD_MENU,
	MAPPING_ERR_MENU,
	PREVIEW_ERR_MENU,
	MAPPING_SAVE_MENU,
	POINT_SET_MENU,
	POINT_UNSET_MENU,
	EEPROM_ERR_MENU,
	RS232_ERR_MENU,
	KEYPAD_ERR_MENU,
	ANGLE_LIMIT_MENU,
}LCD_Menu_t;
LCD_Menu_t select_menu;


//--- KEYPAD TYPEDEF ---//
Keypad_t keypad;


//--- EEPROM TYPEDEF ---//
EEPROM_t eeprom1;

typedef enum{
	UPDATE_ALL = 0x01,
	SELECTED_ONLY,
}Mem_Update_t;


//--- RS232 TYPEDEF ---//
Data_Get_t command;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*WELDING POINT SET*/
//-------------------
#define MAX_POINT 300


/*EEPROM ADDRESS SET*/
//-----------------------------------
#define EEPROM1_ADDRESS					0xA0
#define DATA_BYTE_SHIFT					0x02
#define DATA_PAGE_SHIFT					0x01


/*ERROR WARNING SET*/
//----------------------------
#define MISS_STEP_ERR			0xA0
#define SOFT_LIMIT_ERR		0xB0
#define HARD_LIMIT_ERR		0xC0


/*FUNCTION CONFIG SET*/
//---------------------
#define USE_LCD
#define USE_EEPROM
#define USE_RS232
#define USE_KEYPAD


/*TESTING MODE SET*/
//----------------------
//#define KEYPAD_TEST
//#define RS232_TEST
//#define EEPROM_TEST
//#define EMERGENCY_TEST

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* Definitions for interfaceTask */
osThreadId_t interfaceTaskHandle;
uint32_t interfaceTaskBuffer[ 256 ];
osStaticThreadDef_t interfaceTaskControlBlock;
const osThreadAttr_t interfaceTask_attributes = {
  .name = "interfaceTask",
  .cb_mem = &interfaceTaskControlBlock,
  .cb_size = sizeof(interfaceTaskControlBlock),
  .stack_mem = &interfaceTaskBuffer[0],
  .stack_size = sizeof(interfaceTaskBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for receiveTask */
osThreadId_t receiveTaskHandle;
uint32_t receiveTaskBuffer[ 256 ];
osStaticThreadDef_t receiveTaskControlBlock;
const osThreadAttr_t receiveTask_attributes = {
  .name = "receiveTask",
  .cb_mem = &receiveTaskControlBlock,
  .cb_size = sizeof(receiveTaskControlBlock),
  .stack_mem = &receiveTaskBuffer[0],
  .stack_size = sizeof(receiveTaskBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for lcdTask */
osThreadId_t lcdTaskHandle;
uint32_t lcdTaskBuffer[ 128 ];
osStaticThreadDef_t lcdTaskControlBlock;
const osThreadAttr_t lcdTask_attributes = {
  .name = "lcdTask",
  .cb_mem = &lcdTaskControlBlock,
  .cb_size = sizeof(lcdTaskControlBlock),
  .stack_mem = &lcdTaskBuffer[0],
  .stack_size = sizeof(lcdTaskBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for lcdUpdate */
osEventFlagsId_t lcdUpdateHandle;
osStaticEventGroupDef_t lcdUpdateControlBlock;
const osEventFlagsAttr_t lcdUpdate_attributes = {
  .name = "lcdUpdate",
  .cb_mem = &lcdUpdateControlBlock,
  .cb_size = sizeof(lcdUpdateControlBlock),
};
/* Definitions for getData */
osEventFlagsId_t getDataHandle;
osStaticEventGroupDef_t getDataControlBlock;
const osEventFlagsAttr_t getData_attributes = {
  .name = "getData",
  .cb_mem = &getDataControlBlock,
  .cb_size = sizeof(getDataControlBlock),
};
/* USER CODE BEGIN PV */

/*PENDANT GLOBAL VARIABLE*/
//////////////////////////////
uint16_t
prev_getdata;

bool 
saved_dist    		= true,
saved_prev				= true,
prep_done 				= false,
mapped_start_array[MAX_POINT],
mapped_end_array[MAX_POINT];

uint8_t
lcd_error_value,
current_running_mode,
stored_welding_data[3],
mapped_array[MAX_POINT],
mapped_points[MAX_POINT],
mapped_pattern[MAX_POINT],
mapped_speed[MAX_POINT];

uint16_t 
mapping_point,
total_mapped_points,
preview_point, welding_point,
preview_pattern, welding_pattern,
preview_speed, welding_speed,
max_welding_point = MAX_POINT;


float
angle_value[6], move_angle_value[6],
pos_value[3], move_pos_value[3],
rot_value[3], move_rot_value[3],
A1, A2, A3, A4, A5, A6,
moveX, moveY, moveZ,
distance_val,
max_axis_distance = 100,
max_joint_distance = 90;


char
string_distance[20],
string_preview[20],
string_speed[20];

char 
prev_keys,
keys;


/*MENU MODE VARIABLE*/
///////////////////////////////////
bool
emergency_state = false,
prev_emergency_state = false;

float
increase_decrease_value = 2.5;

char
home_menu[]					= "(HOME)",	
preparation[]     	= "(PREP)",
mapping_menu[]    	= "(MAP)",
preview_menu[] 			= "(PREV)",
welding_menu[]			= "(WELD)",
mem_menu[]        	= "(MEM)",
set_menu[]					= "(SET)",

world_ctrl[]				= "(W)",
joint_ctrl[] 				= "(J)",

cont_change[] 			= "(CON)",
dist_change[] 			= "(DIS)",
step_change[] 			= "(STP)",

mapping_mode[]			= "(MAPPING)",
welding_mode[] 			= "(WELDING)",
preview_mode[] 			= "(PREVIEW)",
cal_mode[]					= "(CALIBRATE)",
zero_mode[]					= "(ZEROING)",
speed_mode[]				= "(SPEED)",

low_speed[] 				= "LOW",
med_speed[] 				= "MED",
high_speed[] 				= "HIGH",

not_set_mode[]			= "NOTSET",
dot_mode[]					= "DOT",
linear_mode[]				= "LINEAR",
circular_mode[] 		= "CIRCULAR",
wave_mode[]					= "WAVE",

string_max_dist1[] 	= "100      ",
string_max_dist2[] 	= "90       ",
string_max_point[] 	= "360      ";

int16_t 
ctrl_mode_counter,			// Counter Mode Kontrol - (World, Joint)
move_mode_counter,			// Counter Mode Gerak - (Continuous, Distance, Step)
move_var_counter,				// Counter Variabel Gerak - (X, Y, Z, Rx, Ry, Rz, J1-J6)
run_mode_counter,				// Counter Mode Running - (Welding, Preview)
speed_mode_counter,			// Counter Mode Kecepatan Pada Welding Point - (LOW, MED, HIGH)
mapping_menu_counter,		// Counter Menu Mapping - (Menu 1, Menu 2, Menu 3)
mapped_point_counter,		// Counter Titik Mapping
start_point_counter,		// Counter Titik Start Pengelasan
end_point_counter,			// Counter Titik End Pengelasan
pattern_sel_counter,		// Counter Pemilihan Pola Pengelasan
welding_menu_counter,		// Counter Menu Welding
cal_mode_counter, 			// Counter Mode Kalibrasi
setting_menu_counter,		// Counter Menu Setting
joint_sel_counter,			// Counter Pemilihan Joint Yang Akan Dilakukan Zeroing
global_speed_counter,		// Counter Mode Kecepatan Global - (LOW, MED, HIGH)
add_col;

char num_keys[10] = {
'1', '2', '3', '4', '5',
'6', '7', '8', '9', '0',
};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
void StartInterfaceTask(void *argument);
void StartReceiveTask(void *argument);
void StartLCDTask(void *argument);

/* USER CODE BEGIN PFP */

// LED BLINK FUNCTION PROTOTYPE
void blink_led(uint8_t delay_time, uint8_t count);

// MENU HANDLER FUNCTION PROTOTYPE
void ui_handler(void);
void show_menu(LCD_Menu_t menu);
void reset_counter(LCD_Menu_t menu);
void lcd_update(void);
void change_menu(LCD_Menu_t menu);
bool check_numkeys_pressed(void);

// EEPROM HANDLER FUNCTION PROTOTYPE
void Save_WeldingData(uint8_t welding_point, uint8_t welding_pattern, uint8_t welding_speed);		// SAVING WELDING DATA
void Delete_WeldingData(uint8_t welding_point);																									// DELETE WELDING DATA
void Read_WeldingData(uint16_t welding_point, uint8_t* stored_data);														// READ WELDING DATA
void Update_Data(Mem_Update_t update_type);																											// UPDATE WELDING DATA
void Format_mem(void);																																					// MEMORY FORMAT

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/*EMERGENCY STOP BUTTON INTERRUPT*/
//----------------------------------------------------------------------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == EMERGENCY_Pin){
		emergency_state = (HAL_GPIO_ReadPin(EMERGENCY_GPIO_Port, EMERGENCY_Pin) == GPIO_PIN_RESET);
		if(emergency_state == true) HAL_GPIO_WritePin(SIG_GPIO_Port, SIG_Pin, GPIO_PIN_RESET);
		else HAL_GPIO_WritePin(SIG_GPIO_Port, SIG_Pin, GPIO_PIN_SET);
	}
}
//----------------------------------------------------------------------------------------------


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
//-------------------------------


/*RS232 CONFIGURATION*/
//------------------------------------------------------
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == USART1){
		osEventFlagsSet(getDataHandle, 0x01);
		prev_getdata = HAL_GetTick();
	}
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
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

	HAL_Delay(2000);
	
	/*LCD CONFIGURATION*/
	//---------------------------------
	#ifdef USE_LCD
	lcd_error_value = lcd_init(&hi2c2);
	if(lcd_error_value != 0){
		while(1){
			blink_led(50, 1);
		}
	};
	lcd_clear();
	HAL_Delay(10);
	show_menu(BOOTING_MENU);
	blink_led(100, 2);
	#endif
	//---------------------------------
	
	
	/*CHECK EMERGENCY*/
	//-----------------------------------------------------------------------
	if(HAL_GPIO_ReadPin(EMERGENCY_GPIO_Port, EMERGENCY_Pin) == GPIO_PIN_SET){
		HAL_GPIO_WritePin(SIG_GPIO_Port, SIG_Pin, GPIO_PIN_SET);
	}
	else HAL_GPIO_WritePin(SIG_GPIO_Port, SIG_Pin, GPIO_PIN_RESET);
	//-----------------------------------------------------------------------
	
	
	/*RS232 COM CONFIGURATION*/
	//----------------------------------------
	#ifdef USE_RS232
	RS232_Init(&huart1);
	Start_get_command(&command);
	for(int i=0; i<50; i++){
		Send_feedback(&command, PENDANT_ONLINE, 0);
		HAL_Delay(10);
	}
	blink_led(100, 2);
	#endif
	//----------------------------------------

	
	/*EEPROM CONFIGURATION*/
	//---------------------------------------------------------
	#ifdef USE_EEPROM
	EEPROM_Init(&hi2c1, &eeprom1, MEM_SIZE_256Kb, 0xA0);
	EEPROM_ByteRead(&eeprom1, 511, 0, eeprom1.dummy_byte, 1);
	if(eeprom1.status != EEPROM_OK){
		lcd_clear();
		HAL_Delay(10);
		while(1){ 
			show_menu(EEPROM_ERR_MENU);
			blink_led(50, 1);
		}
	}
	Update_Data(UPDATE_ALL);
	blink_led(100, 2);
	#endif
	//---------------------------------------------------------
	
	
	/*KEYPAD CONFIGURATION*/	
	//---------------------------------------------------------
	#ifdef USE_KEYPAD
	keypad.col_port[0] = GPIOB, keypad.col_pin[0] = GPIO_PIN_1;
	keypad.col_port[1] = GPIOB, keypad.col_pin[1] = GPIO_PIN_0;
	keypad.col_port[2] = GPIOA, keypad.col_pin[2] = GPIO_PIN_7;
	keypad.col_port[3] = GPIOA, keypad.col_pin[3] = GPIO_PIN_6;
	
	keypad.row_port[0] = GPIOA, keypad.row_pin[0] = GPIO_PIN_1;
	keypad.row_port[1] = GPIOA, keypad.row_pin[1] = GPIO_PIN_2;
	keypad.row_port[2] = GPIOA, keypad.row_pin[2] = GPIO_PIN_3;
	keypad.row_port[3] = GPIOA, keypad.row_pin[3] = GPIO_PIN_4;
	keypad.row_port[4] = GPIOA, keypad.row_pin[4] = GPIO_PIN_5;
	
	Keypad_Init(&keypad, makeKeymap(key), num_rows, num_cols);
	blink_led(100, 2);
	#endif
	//---------------------------------------------------------
	
	
	/*ALL PREPARATION COMPLETED - BOOTING DONE*/
	//------------------------------------------
	lcd_clear();
	HAL_Delay(5);
	select_menu = HOME_MENU;
	HAL_Delay(500);
	prep_done = true;
	//------------------------------------------
	
	
	#ifdef EEPROM_TEST
	Save_WeldingData(0, LINEAR, 50);
	HAL_Delay(100);
	Read_WeldingData(0, stored_welding_data);
	#endif
	

  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of interfaceTask */
  interfaceTaskHandle = osThreadNew(StartInterfaceTask, NULL, &interfaceTask_attributes);

  /* creation of receiveTask */
  receiveTaskHandle = osThreadNew(StartReceiveTask, NULL, &receiveTask_attributes);

  /* creation of lcdTask */
  lcdTaskHandle = osThreadNew(StartLCDTask, NULL, &lcdTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of lcdUpdate */
  lcdUpdateHandle = osEventFlagsNew(&lcdUpdate_attributes);

  /* creation of getData */
  getDataHandle = osEventFlagsNew(&getData_attributes);

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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
  htim3.Init.Prescaler = 143;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 49999;
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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_Pin|SIG_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_Pin SIG_Pin */
  GPIO_InitStruct.Pin = LED_Pin|SIG_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : EMERGENCY_Pin */
  GPIO_InitStruct.Pin = EMERGENCY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EMERGENCY_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*--- LED BLINK HANDLER ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void blink_led(uint8_t delay_time, uint8_t count){
	for(int i=0; i<count; i++){
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		HAL_Delay(delay_time);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
		HAL_Delay(delay_time);
	}
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- USER INTERFACE HANDLER ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void ui_handler(void){
	// Read Keypad ***********************
	keys = Keypad_Read(&keypad);
	if(keys != prev_keys && keys != 0x00){
		lcd_update();
	}
	
	/* EMERGENCY MENU HANDLER *//////////////////////////////////////////////////////////////////////////////////
	if(select_menu == EMERGENCY_MENU1){
		if(HAL_GPIO_ReadPin(EMERGENCY_GPIO_Port, EMERGENCY_Pin) == GPIO_PIN_SET){
			change_menu(CAL2_MENU);
		}
	}
	
	else if(select_menu == EMERGENCY_MENU2 || select_menu == EMERGENCY_MENU3 || select_menu == EMERGENCY_MENU4){
		if(keys == '.' && prev_keys != keys){
			for(int i=0; i<50; i++){
				Send_state_reset(&command);
				osDelay(5);
			}
			change_menu(HOME_MENU);
		}
	}
	
	
	/* SETTING MENU HANDLER *//////////////////////////
	else if(select_menu == SETTING_MENU){
		keys = Keypad_Read(&keypad);
		
		// CHANGE SETTING MENU  ********************************
		if(keys == 'W' && prev_keys != keys){
			setting_menu_counter++;
			if(setting_menu_counter > 2) setting_menu_counter = 0;
			cal_mode_counter = 0;
			joint_sel_counter = 0;
			global_speed_counter = 0;
		}
		
		// MOVE TO HOME MENU *********************
		else if(keys == '.' && prev_keys != keys){
			change_menu(HOME_MENU);
		}
		
		// SETTING MENU 1 ***********************************
		if(setting_menu_counter == 0){
			// CALIBRATION MODE SELECT ---------------------
			if(keys == 'R' && prev_keys != keys){
				cal_mode_counter++;
				if(cal_mode_counter > 1) cal_mode_counter = 0;
			}
			
			// START CALIBRATION ------------------------------
			else if(keys == '#' && prev_keys != keys){
				command.feedback = NO_FEEDBACK;
				if(cal_mode_counter == 0) change_menu(CAL1_MENU);
				else change_menu(CAL2_MENU);
			}
		}
		
		// SETTING MENU 2 **********************************
		else if(setting_menu_counter == 1){
			// JOINT SELECT ----------------------------------
			if(keys == 'R' && prev_keys != keys){
				joint_sel_counter++;
				if(joint_sel_counter > 5) joint_sel_counter = 0;
			}
			
			// SEND ZEROING JOINT --------------------
			else if(keys == '#' && prev_keys != keys){
				command.feedback = NO_FEEDBACK;
				change_menu(ZEROING_MENU);
			}
		}
		
		// SETTING MENU 3 *****************************************
		else if(setting_menu_counter == 2){
			// SPEED SELECT ----------------------------------------
			if(keys == 'R' && prev_keys != keys){
				global_speed_counter++;
				if(global_speed_counter > 2) global_speed_counter = 0;
			}
			
			// SEND GLOBAL SPEED ---------------------
			else if(keys == '#' && prev_keys != keys){
				command.feedback = NO_FEEDBACK;
				change_menu(CHANGE_SPEED_MENU);
			}
		}
	}
	
	/* CALIBRATION MODE 1 MENU HANDLER *//////////////////////////////////////////////////////////
	else if(select_menu == CAL1_MENU){
		if(command.feedback == CALIBRATION_DONE){
			command.feedback = NO_FEEDBACK;
			change_menu(HOME_MENU);
		}
		else Send_setting_data(&command, JOINT_CALIBRATION, CALIBRATE_ONLY, NO_SELECTION, NO_SPEED);
		
		if(keys == '.' && prev_keys != keys){
			for(int j=0; j<100; j++){
				Send_running(&command, RUNNING_STOP, SETTING_MODE, 0);
				osDelay(5);
			}
			change_menu(HOME_MENU);
		}
	}
	
	/* CALIBRATION MODE 2 MENU HANDLER *////////////////////////////////////////////////////////////
	else if(select_menu == CAL2_MENU){
		if(command.feedback == CALIBRATION_DONE){
			command.feedback = NO_FEEDBACK;
			change_menu(HOME_MENU);
		}
		else Send_setting_data(&command, JOINT_CALIBRATION, CALIBRATE_HOMING, NO_SELECTION, NO_SPEED);
		
		if(keys == '.' && prev_keys != keys){
			for(int j=0; j<100; j++){
				Send_running(&command, RUNNING_STOP, SETTING_MODE, 0);
				osDelay(5);
			}
			change_menu(HOME_MENU);
		}
	}
	
	/* ZEROING MANU HANDLER *////////////////////////////////////////////////////////////////////////////////////////////
	else if(select_menu == ZEROING_MENU){
		if(command.feedback == ZEROING_DONE || (keys == '.' && prev_keys != keys)){
			command.feedback = NO_FEEDBACK;
			change_menu(HOME_MENU);
		}
		else Send_setting_data(&command, JOINT_ZEROING, NO_CALIBRATION, (Zeroing_Select_t)(joint_sel_counter+1), NO_SPEED);
	}
	
	
	/* CHANGE SPEED MENU HANDLER *///////////////////////////////////////////////////////////////////////////////////
	else if(select_menu == CHANGE_SPEED_MENU){
		if(command.feedback == SPEED_CHANGE_DONE || (keys == '.' && prev_keys != keys)){
			command.feedback = NO_FEEDBACK;
			change_menu(HOME_MENU);
		}
		else Send_setting_data(&command, JOINT_SPEED, NO_CALIBRATION, NO_SELECTION, (Speed_t)(global_speed_counter+1));
	}
	
	
	/* HOME MENU HANDLER *////////////////////////////////////////////////////////////////
	else if(select_menu == HOME_MENU){
		// SETTING MENU *********************
		if(keys == 'Q' && prev_keys != keys){
			change_menu(SETTING_MENU);
		}
	
		// CHANGE MOVE CONTROL ***************************
		else if(keys == 'W' && prev_keys != keys){
			move_var_counter = 0;
			ctrl_mode_counter++;
			
			if(ctrl_mode_counter > 1) ctrl_mode_counter = 0;
		}
		
		// CHANGE CURSOR *******************************
		else if(keys == 'R' && prev_keys != keys){
			move_var_counter++;
			if(move_var_counter > 5) move_var_counter = 0;
		}
		
		// SELECT MOVE MODE ******************************
		else if(keys == 'T' && prev_keys != keys){
			move_mode_counter++;
			
			if(move_mode_counter > 2) move_mode_counter = 0;
		}
		
		// CANCEL STEPPER MOVE ************************************
		else if(keys == '.' && prev_keys != keys){
			Send_running(&command, RUNNING_STOP, CONTROL_MODE, 0);
		}
		
		// MOVE VALUE BY DISTANCE **********************************************************
		else if((move_mode_counter == 1 && check_numkeys_pressed() == true) || keys == '<'){
			add_col = 0;
			distance_val = 0;
			
			while(1){
				saved_dist = false;
				if(keys != prev_keys && keys != 0x00){
					lcd_update();
				}
				
				// INSERT VALUE -----------------------------------------
				if(check_numkeys_pressed() == true && prev_keys != keys){
					string_distance[add_col] = keys;
					add_col++;
				}
				
				// DELETE VALUE ----------------------------------
				if(keys == '<' && prev_keys != keys){
					if(add_col > 0) add_col-=1;
					
					if(add_col == 0) string_distance[add_col] = '0';
					else string_distance[add_col] = ' ';
				}
				
				// SAVE VALUE ------------------------------------------------------------
				if(keys == '>' && prev_keys != keys){
					sscanf(string_distance, "%f", &distance_val);
					if(ctrl_mode_counter == 0){
						if(distance_val >= max_axis_distance){
							memcpy(string_distance, string_max_dist1, sizeof(string_max_dist1));
							distance_val = max_axis_distance;
						}
					}
					else{
						if(distance_val >= max_joint_distance){
							memcpy(string_distance, string_max_dist2, sizeof(string_max_dist2));
							distance_val = max_joint_distance;
						}
					}
					saved_dist = true;
					break;
				}
				prev_keys = keys;
			}
		}
		// *********************************************************************************
		
		
		// MOVE VALUE BY STEP *****************************************
		else if(move_mode_counter == 2) increase_decrease_value = 0.25;
		
		
		// MOVE VALUE CONTINUOUS **********
		else increase_decrease_value = 0.0;
		
		
		// INCLREASE VALUE ***************************************************************************************************************
		if(keys == 'U' && HAL_GetTick() - prev_tick > debounce){
			// WORLD INCREASE VALUE CONTROL ------------------------------------------------------------------------------------------------
			if(ctrl_mode_counter == 0){
				for(int i=0; i<6; i++){
					if(move_var_counter == i){
						if(move_mode_counter != 1) move_pos_value[i] = increase_decrease_value;
						else move_pos_value[i] = distance_val;
						Send_move(&command, WORLD_CTRL, (Move_Mode_t)(move_mode_counter+1), (Move_Var_t)(i+1), UNSIGNED_VAR, move_pos_value[i]);
					}
				}
			}
			
			// JOINT INCREASE VALUE CONTROL ------------------------------------------------------------------------------------------------
			else{
				for(int i=0; i<6; i++){
					if(move_var_counter == i){
						if(move_mode_counter != 1) move_angle_value[i] = increase_decrease_value;
						else move_angle_value[i] = distance_val;
						Send_move(&command, JOINT_CTRL, (Move_Mode_t)(move_mode_counter+1), (Move_Var_t)(i+7), UNSIGNED_VAR, move_angle_value[i]);
					}
				}
			}
			prev_tick = HAL_GetTick();
		}
		
		
		// DECREASE VALUE *****************************************************************************************************************
		else if(keys == 'D' && HAL_GetTick() - prev_tick > debounce){
			// WORLD DECREASE VALUE CONTROL --------------------------------------------------------------------------------------------
			if(ctrl_mode_counter == 0){
				for(int i=0; i<6; i++){
					if(move_var_counter == i){
						if(move_mode_counter != 1) move_pos_value[i] = -1*(increase_decrease_value);
						else move_pos_value[i] = -1*(distance_val);
						Send_move(&command, WORLD_CTRL, (Move_Mode_t)(move_mode_counter+1), (Move_Var_t)(i+1), SIGNED_VAR, move_pos_value[i]);
					}
				}
			}
			// JOINT DECREASE VALUE CONTROL ----------------------------------------------------------------------------------------------
			else{
				for(int i=0; i<6; i++){
					if(move_var_counter == i){
						if(move_mode_counter != 1) move_angle_value[i] = -1*(increase_decrease_value);
						else move_angle_value[i] = -1*(distance_val);
						Send_move(&command, JOINT_CTRL, (Move_Mode_t)(move_mode_counter+1), (Move_Var_t)(i+7), SIGNED_VAR, move_angle_value[i]);
					}
				}
			}
			prev_tick = HAL_GetTick();
		}		
		
		// MOVE TO PREPARATION MENU *********
		if(keys == '#' && prev_keys != keys){
			change_menu(PREP_MENU);
		}
	}
	///////////////////////////////////////////////////////////////////////////////////////
	
	
	/* PREPARATION MENU HANDLER *////////////////////////////////////////////////////////////////
	else if(select_menu == PREP_MENU){		
		// BACK TO HOME MENU *****************
		if(keys == '.' && prev_keys != keys){
			change_menu(HOME_MENU);
		}
		
		
		// CHANGE RUNNING MODE *************************
		else if(keys == 'Q' && prev_keys != keys){
			run_mode_counter++;
			if(run_mode_counter > 2) run_mode_counter = 0;
			strcpy(string_preview, "-");
		}
		
		
		// MAPPING PREPARATION MENU ***************************************************************
		if(run_mode_counter == 0){
			// CHECK NEXT MAPPED POINT
			if(keys == 'U' && prev_keys != keys){
				mapped_point_counter++;
				if(mapped_point_counter >= max_welding_point) mapped_point_counter = max_welding_point;
			}
			
			// CHECK PREVIOUS MAPPED POINT
			else if(keys == 'D' && prev_keys != keys){
				mapped_point_counter--;
				if(mapped_point_counter <= 0) mapped_point_counter = 0;
			}
			
			// MAPPING RUN
			else if(keys == '#' && prev_keys != keys){
				change_menu(MAPPING_MENU);
				mapping_menu_counter = 0;
			}
		}
		
		
		// PREVIEW PREPARATION MENU **************************************************
		else if(run_mode_counter == 1){
			// CHECK FIRST NUMKEY INPUT ------------------------------------------------
			if(check_numkeys_pressed() == true || keys == '<'){
				add_col = 0;
				*string_preview = '0';
				
				while(1){
					if(keys != prev_keys && keys != 0x00){
						lcd_update();
					}
					saved_prev = false;
					
					// INSERT VALUE 
					if(check_numkeys_pressed() == true && prev_keys != keys && add_col < 3){
						string_preview[add_col] = keys;
						add_col++;
					}
					
					// SAVE VALUE 
					else if((keys == '>') && prev_keys != keys){
						preview_point = atoi(string_preview);
						if(preview_point > max_welding_point){
							memcpy(string_preview, string_max_point, sizeof(string_max_point));
							preview_point = max_welding_point;
						}
						saved_prev = true;
						break;
					}
					
					// DELETE VALUE 
					else if(keys == '<' && prev_keys != keys){
						if(add_col > 0) add_col-=1;
						
						if(add_col == 0) string_preview[add_col] = '0';
						else string_preview[add_col] = ' ';
					}
					prev_keys = keys;
				}
			}
			
			// PREVIEW RUN ---------------------------
			else if(keys == '#' && prev_keys != keys){
				for(int i=0; i<20; i++){
					
				}
				change_menu(PREVIEW_MENU);
				mapping_menu_counter = 0;
			}
		}
		// **************************************************************************
		
		
		// WELDING PREPARATION MENU *******************************
		else if(run_mode_counter == 2){			
			// WELDING RUN
			if(keys == '#' && prev_keys != keys){
				Send_running(&command, RUNNING_START, WELDING_MODE, 0);
				
				change_menu(WELDING_MENU);
				
				move_var_counter = 0;
				mapping_menu_counter = 0;
				ctrl_mode_counter = 0;
				
				preview_point = command.welding_point_num;
				preview_pattern = command.pattern_type;
				preview_speed = command.running_speed;
			}
		}
	}
	/////////////////////////////////////////////////////////////////////////////////////////////
	
	
	/* MAPPING MENU UI HANDLER */////////////////////////////////////////////////////////////////////////////////
	else if(select_menu == MAPPING_MENU){
		// BACK TO PREPARATION MENU **************************************
		if(keys == '.' && prev_keys != keys && mapping_menu_counter == 0){
			change_menu(PREP_MENU);
		}
		
		
		// CHANGE MAPPING MENU *********************************
		else if(keys == 'Q' && prev_keys != keys){
			move_var_counter = 0;
			mapping_menu_counter++;
			if(mapping_menu_counter > 2) mapping_menu_counter = 0;
		}
		
		
		// MAPPING MENU 1 **************************************************************************************************************************
		if(mapping_menu_counter == 0){
			// CHANGE MOVE CONTROL ---------------------------
			if(keys == 'W' && prev_keys != keys){
				move_var_counter = 0;
				ctrl_mode_counter++;
				if(ctrl_mode_counter > 1) ctrl_mode_counter = 0;
				distance_val = 0;
			}
			
			// CHANGE CURSOR FOR SELECT MOVE VARIABLE -------------------------------------
			else if(keys == 'R' && prev_keys != keys){
				move_var_counter++;
				if(ctrl_mode_counter == 0 && move_var_counter > 2) move_var_counter = 0;
				else if(ctrl_mode_counter == 1 && move_var_counter > 5) move_var_counter = 0;
			}
			
			// SELECT MOVE VALUE -----------------------------
			else if(keys == 'T' && prev_keys != keys){
				move_mode_counter++;
				if(move_mode_counter > 2) move_mode_counter = 0;
			}
			
			// CANCEL STEPPER MOVE ------------------------------------
			else if(keys == '.' && prev_keys != keys){
				Send_running(&command, RUNNING_STOP, CONTROL_MODE, 0);
			}
			
			// MOVE VALUE BY DISTANCE ----------------------------------------------------------
			else if((move_mode_counter == 1 && check_numkeys_pressed() == true) || keys == '<'){
				add_col = 0;
				*string_distance = '0';
				
				while(1){
					saved_dist = false;
					if(keys != prev_keys && keys != 0x00){
						lcd_update();
					}
					
					// INSERT VALUE
					if(check_numkeys_pressed() == true && prev_keys != keys){
						string_distance[add_col] = keys;
						add_col++;
					}
					
					// SAVE VALUE
					else if(keys == '>' && prev_keys != keys){
						sscanf(string_distance, "%f", &distance_val);
						if(ctrl_mode_counter == 0){
							if(distance_val >= max_axis_distance){
								memcpy(string_distance, string_max_dist1, sizeof(string_max_dist1));
								distance_val = max_axis_distance;
							}
						}
						else{
							if(distance_val >= max_joint_distance){
								memcpy(string_distance, string_max_dist2, sizeof(string_max_dist2));
								distance_val = max_joint_distance;
							}
						}
						saved_dist = true;
						break;
					}
					
					// DELETE VALUE
					else if(keys == '<' && prev_keys != keys){
						if(add_col > 0) add_col-=1;
						
						if(add_col == 0) string_distance[add_col] = '0';
						else string_distance[add_col] = ' ';
					}
			
					prev_keys = keys;
				}
			}
			
			// MOVE VALUE BY STEP -----------------------------------------
			else if(move_mode_counter == 2) increase_decrease_value = 0.25;
			
			
			// MOVE VALUE CONTINUOUS ----------
			else increase_decrease_value = 0.0;
			
			
			// INCLREASE VALUE 
			if(keys == 'U' && HAL_GetTick() - prev_tick > debounce){
				// World Increase Value Control
				if(ctrl_mode_counter == 0){
					for(int i=0; i<6; i++){
						if(move_var_counter == i){
							if(move_mode_counter != 1) move_pos_value[i] = increase_decrease_value;
							else move_pos_value[i] = distance_val;
							Send_move(&command, WORLD_CTRL, (Move_Mode_t)(move_mode_counter+1), (Move_Var_t)(i+1), UNSIGNED_VAR, move_pos_value[i]);
						}
					}
				}
				
				// Joint Increase Value Control
				else{
					for(int i=0; i<6; i++){
						if(move_var_counter == i){
							if(move_mode_counter != 1) move_angle_value[i] = increase_decrease_value;
							else move_angle_value[i] = distance_val;
							Send_move(&command, JOINT_CTRL, (Move_Mode_t)(move_mode_counter+1), (Move_Var_t)(i+7), UNSIGNED_VAR, move_angle_value[i]);
						}
					}
				}
				prev_tick = HAL_GetTick();
			}
			
			// DECREASE VALUE 
			else if(keys == 'D' && HAL_GetTick() - prev_tick > debounce){
				// World Decrease Value Control
				if(ctrl_mode_counter == 0){
					for(int i=0; i<6; i++){
						if(move_var_counter == i){
							if(move_mode_counter != 1) move_pos_value[i] = -1*(increase_decrease_value);
							else move_pos_value[i] = -1*(distance_val);
							Send_move(&command, WORLD_CTRL, (Move_Mode_t)(move_mode_counter+1), (Move_Var_t)(i+1), SIGNED_VAR, move_pos_value[i]);
						}
					}
				}
				
				// Joint Decrease Value Control
				else{
					for(int i=0; i<6; i++){
						if(move_var_counter == i){
							if(move_mode_counter != 1) move_angle_value[i] = -1*(increase_decrease_value);
							else move_angle_value[i] = -1*(distance_val);
							Send_move(&command, JOINT_CTRL, (Move_Mode_t)(move_mode_counter+1), (Move_Var_t)(i+7), SIGNED_VAR, move_angle_value[i]);
						}
					}
				}
				prev_tick = HAL_GetTick();
			}
		}
		// *****************************************************************************************************************************************
		
		
		// MAPPING MENU 2 ***************************************************************************
		else if(mapping_menu_counter == 1){
			// CHANGE CURSOR FOR SELECT WELDING POINT
			if(keys == 'R' && prev_keys != keys){
				move_var_counter++;
				if(move_var_counter > 1) move_var_counter = 0;
			}
			
			// BACK TO MAPPING MENU 1
			else if(keys == '.' && prev_keys != keys){
				mapping_menu_counter = 0;
			}
			
			// INCLREASE VALUE
			if(keys == 'U' && HAL_GetTick() - prev_tick > debounce){
				if(move_var_counter == 0){
					start_point_counter++;
					if(start_point_counter > max_welding_point) start_point_counter = 0;
				}
				
				else if(move_var_counter ==1){
					end_point_counter++;
					if(end_point_counter > max_welding_point) end_point_counter = 0;
				}
				prev_tick = HAL_GetTick();
			}
			
			// DECREASE VALUE
			else if(keys == 'D' && HAL_GetTick() - prev_tick > debounce){
				if(move_var_counter == 0){
					start_point_counter--;
					if(start_point_counter < 0) start_point_counter = max_welding_point;
				}
				
				else if(move_var_counter ==1){
					end_point_counter--;
					if(end_point_counter < 0) end_point_counter = max_welding_point;
				}
				prev_tick = HAL_GetTick();
			}
				
			// SET MAPPING POINT
			else if(keys == '>' && prev_keys != keys && mapping_menu_counter != 0){
				if(move_var_counter == 0 && start_point_counter != 0){
					mapped_start_array[start_point_counter] = true;
					for(int i=0; i<25; i++){
						Send_mapping(&command, start_point_counter, START_POINT, NO_PATTERN, LOW, SAVE_VALUE);
						osDelay(5);
					}
				}
				else if(move_var_counter == 1 && end_point_counter != 0){
					mapped_end_array[end_point_counter] = true;
					for(int i=0; i<25; i++){
						Send_mapping(&command, end_point_counter, END_POINT, NO_PATTERN, LOW, SAVE_VALUE);
						osDelay(5);
					}
				}
				
				change_menu(POINT_SET_MENU);
				
				osDelay(1500);
				
				change_menu(MAPPING_MENU);
			}
			
			// UNSET MAPPING POINT 
			else if(keys == '<' && prev_keys != keys && mapping_menu_counter != 0){
				if(move_var_counter == 0){
					mapped_start_array[start_point_counter] = false;
					for(int i=0; i<25; i++){
						Delete_WeldingData(start_point_counter);
						Send_mapping(&command, start_point_counter, START_POINT, NO_PATTERN, LOW, DELETE_VALUE);
						osDelay(5);
					}
				}
				else if(move_var_counter == 1){
					mapped_end_array[end_point_counter] = false;
					for(int i=0; i<25; i++){
						Delete_WeldingData(end_point_counter);
						Send_mapping(&command, end_point_counter, END_POINT, NO_PATTERN, LOW, DELETE_VALUE);
						osDelay(5);
					}
				}
				
				change_menu(POINT_UNSET_MENU);
				
				Update_Data(UPDATE_ALL);
				osDelay(1500);
				
				change_menu(MAPPING_MENU);
			}
		}
		// *******************************************************************************************
		
		
		// MAPPING MENU 3 ********************************************************************************************************************
		else if(mapping_menu_counter == 2){
			// CHANGE PATTERN
			if(keys == 'W' && prev_keys != keys){
				pattern_sel_counter++;
				if(pattern_sel_counter > 2) pattern_sel_counter = 0;
			}
			
			// CHANGE MAPPED POINT MOVE SPEED
			else if(keys == 'R' && prev_keys != keys){
				speed_mode_counter++;
				if(speed_mode_counter > 3) speed_mode_counter = 0;
			}
			
			// BACK TO MAPPING MENU 1
			else if(keys == '.' && prev_keys != keys){
				mapping_menu_counter = 0;
			}
			
			// SAVE ALL MAPPING POINT DATA PATTERN
			else if(keys == '#' && prev_keys != keys){
				if((mapped_start_array[start_point_counter] & mapped_end_array[end_point_counter]) == true){					
					for(int i=0; i<50; i++){
						Send_mapping(&command, start_point_counter&end_point_counter, PATTERN, (Welding_Pattern_t) pattern_sel_counter, (Speed_t)speed_mode_counter, SAVE_VALUE);
						osDelay(5);
					}
					Save_WeldingData(start_point_counter, pattern_sel_counter, speed_mode_counter);
					
					change_menu(MAPPING_SAVE_MENU);
					
					Update_Data(UPDATE_ALL);
					osDelay(1500);
					
					change_menu(MAPPING_MENU);
					mapping_menu_counter = 0;
				}
				
				else{
					change_menu(MAPPING_ERR_MENU);
					
					osDelay(3000);
					
					change_menu(MAPPING_MENU);
					mapping_menu_counter = 1;
				}
			}
		}
		// ***********************************************************************************************************************************
	}
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	
	/* PREVIEW MENU HANDLER */////////////////////////////////////////////////////////////////////////////////////////////////
	else if(select_menu == PREVIEW_MENU){
		preview_pattern = mapped_pattern[preview_point-1];
		preview_speed = mapped_speed[preview_point-1];
		current_running_mode = PREVIEW_MODE;
		
		if((preview_point & mapped_points[preview_point-1]) == preview_point && preview_point != 0 && preview_speed != 0){
			while(1){
				keys = Keypad_Read(&keypad);
				
				if(keys != prev_keys && keys != 0x00){
					lcd_update();
				}
				
				if(command.feedback == CURRENT_POINT_DONE){
					move_mode_counter = 0;
					ctrl_mode_counter = 0;
					move_var_counter = 0;
					command.feedback = NO_FEEDBACK;
					change_menu(HOME_MENU);
					break;
				}
				
				if(keys == '.' && prev_keys != keys){
					for(int i=0; i<20; i++){
						
					}
					change_menu(STOP_MENU);
				}
				
				prev_keys = keys;
			}
		}
		
		else if((preview_point & mapped_points[preview_point-1]) != preview_point || preview_point == 0 || preview_speed == 0){
			change_menu(PREVIEW_ERR_MENU);
			
			osDelay(2000);
			
			change_menu(PREP_MENU);
			run_mode_counter = 1;
		}
	}
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	
	
	/* WELDING MENU HANDLER *//////////////////////////////////
	else if(select_menu == WELDING_MENU){
		preview_point = command.welding_point_num;
		preview_pattern = command.pattern_type;
		preview_speed = command.running_speed;
		current_running_mode = WELDING_MODE;
		
		// PAUSE MENU ***********************
		if(keys == 'Q' && prev_keys != keys){
			change_menu(PAUSE_MENU);
		}
		// **********************************
		

		// CHANGE WELDING MENU *********************************
		else if(keys == 'W' && prev_keys != keys){
			ctrl_mode_counter++;
			if(ctrl_mode_counter > 1) ctrl_mode_counter = 0;
		}
		// *****************************************************
		
		
		// CHANGE WELDING MENU *********************************
		else if(keys == 'R' && prev_keys != keys){
			welding_menu_counter++;
			if(welding_menu_counter > 1) welding_menu_counter = 0;
		} 
		// *****************************************************
		
		
		// STOP MENU *****************************
		else if(keys == '.' && prev_keys != keys){
			Send_running(&command, RUNNING_STOP, WELDING_MODE, 0);
			change_menu(STOP_MENU);
		}
		// ***************************************
		
		if(emergency_state == true){
			lcd_update();
			change_menu(EMERGENCY_MENU1);
		}
	}
	//////////////////////////////////////////////////////////
	
	
	/* PAUSE MENU HANDLER */////////////////////////////////////////////////////
	else if(select_menu == PAUSE_MENU){
		// CONTINUE MENU ********************************************************
		while(1){
			keys = Keypad_Read(&keypad);
			if(keys != prev_keys && keys != 0x00){
				lcd_update();
			}
			
			if(current_running_mode == WELDING_MODE){
				Send_running(&command, RUNNING_PAUSE, WELDING_MODE, 0);
			}
			else if(current_running_mode == PREVIEW_MODE){
				Send_running(&command, RUNNING_PAUSE, PREVIEW_MODE, preview_point);
			}
			
			if(keys == '#' && prev_keys != keys){
				if(current_running_mode == WELDING_MODE){
					Send_running(&command, RUNNING_PAUSE, WELDING_MODE, 0);
					change_menu(WELDING_MENU);
				}
				else if(current_running_mode == PREVIEW_MODE){
					Send_running(&command, RUNNING_PAUSE, PREVIEW_MODE, preview_point);
					change_menu(PREVIEW_MENU);
				}
			}
			
			prev_keys = keys;
		}
		// **********************************************************************
	}
	///////////////////////////////////////////////////////////////////////////
	
	
	/* STOP MENU HANDLER */////////////////////
	else if(select_menu == STOP_MENU){
		// WAIT FOR AUTO HOMING DONE ************
		while(1){
			if(current_running_mode == WELDING_MODE){
				Send_running(&command, RUNNING_PAUSE, WELDING_MODE, 0);
			}
			else if(current_running_mode == PREVIEW_MODE){
				Send_running(&command, RUNNING_PAUSE, PREVIEW_MODE, preview_point);
			}
			
			if(command.feedback == CALIBRATION_DONE){
				ctrl_mode_counter = 0,
				move_mode_counter = 0,
				move_var_counter = 0,
				run_mode_counter = 0,
				command.feedback = NO_FEEDBACK;
				
				change_menu(HOME_MENU);
				break;
			}
		}
		// **************************************
	}
	////////////////////////////////////////////
	
	prev_keys = keys;
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- LCD MENU ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void show_menu(LCD_Menu_t menu){
	/* EMERGENCY MENU 1 *////////////////
	if(menu == EMERGENCY_MENU1){
		lcd_set_cursor(1, 0);
		lcd_printstr("EMERGENCY PRESSED!");
		lcd_set_cursor(3, 2);
		lcd_printstr("RELEASE BUTTON");
		lcd_set_cursor(5, 3);
		lcd_printstr("FOR HOMING");
	}
	/////////////////////////////////////
	
	
	/* EMERGENCY MENU 2 */////////////////////////////////
	if(menu == EMERGENCY_MENU2){
		lcd_set_cursor(1, 0);
		lcd_printstr("<JOINT MISS STEPS>");
		lcd_set_cursor(3, 1);
		lcd_printstr("Error Code: ");
		lcd_printint(MISS_STEP_ERR | command.feedback_num); 
		lcd_set_cursor(0, 3);
		lcd_printstr("PRESS ESC TO RESET!");
	}
	//////////////////////////////////////////////////////
	
	
	/* EMERGENCY MENU 3 */////////////////////////////////
	if(menu == EMERGENCY_MENU3){
		lcd_set_cursor(1, 1);
		lcd_printstr("<ANGLE SOFT LIMIT>");
		lcd_set_cursor(3, 1);
		lcd_printstr("Error Code: ");
		lcd_printint(SOFT_LIMIT_ERR | command.feedback_num); 
		lcd_set_cursor(0, 2);
		lcd_printstr("PRESS ESC TO RESET!");
	}
	//////////////////////////////////////////////////////
	
	
	/* EMERGENCY MENU 4 */////////////////////////////////
	if(menu == EMERGENCY_MENU4){
		lcd_set_cursor(1, 0);
		lcd_printstr("<ANGLE HARD LIMIT>");
		lcd_set_cursor(3, 1);
		lcd_printstr("Error Code: ");
		lcd_printint(HARD_LIMIT_ERR | command.feedback_num); 
		lcd_set_cursor(0, 3);
		lcd_printstr("PRESS ESC TO RESET!");
	}
	//////////////////////////////////////////////////////
	
	
	/* BOOTING MENU *////////////////
	else if(menu == BOOTING_MENU){
		lcd_set_cursor(3, 1);
		lcd_printstr("SYSTEM BOOTING");
		lcd_set_cursor(4, 2);
		lcd_printstr("PLEASE  WAIT");
	}
	/////////////////////////////////
	
	
	/* SETTING MENU *////////////////////
	else if(menu == SETTING_MENU){
		// SETTING MENU 1 ******************
		if(setting_menu_counter == 0){
			lcd_set_cursor(0, 0);
			lcd_printstr("Select Mode: ");
			lcd_set_cursor(13, 0);			
			lcd_printint(cal_mode_counter+1);
			
			lcd_set_cursor(0, 3);
			lcd_printstr(cal_mode);
		}
		// *********************************
		
		
		// SETTING MENU 2 ******************
		else if(setting_menu_counter == 1){
			lcd_set_cursor(0, 0);
			lcd_printstr("Select Joint: J");
			lcd_set_cursor(15, 0);			
			lcd_printint(joint_sel_counter+1);
			
			lcd_set_cursor(0, 3);
			lcd_printstr(zero_mode);
		}
		// *********************************
		
		
		// SETTING MENU 3 ********************************************
		else if(setting_menu_counter == 2){
			lcd_set_cursor(0, 0);
			lcd_printstr("Select Speed: ");
			lcd_set_cursor(14, 0);			
			if(global_speed_counter == 0) lcd_printstr(low_speed);
			else if(global_speed_counter == 1) lcd_printstr(med_speed);
			else if(global_speed_counter == 2) lcd_printstr(high_speed);
			
			lcd_set_cursor(0, 3);
			lcd_printstr(speed_mode);
		}
		// ***********************************************************
		
		
		// SHOW CURRENT MENU **
		lcd_set_cursor(15, 3);
		lcd_printstr(set_menu);
		// ********************
	}
	///////////////////////////////////////
	
	
	/* CALIBRATION MENU *////////////////////////////
	else if(menu == CAL1_MENU || menu == CAL2_MENU){
		lcd_set_cursor(1, 0);	
		lcd_printstr("JOINTS CALIBRATION");
		lcd_set_cursor(0, 2);
		lcd_printstr("Mode: ");
		lcd_set_cursor(0, 3);
		lcd_printstr("Home: ");
		
		if(menu == CAL1_MENU){
			lcd_set_cursor(6, 2);
			lcd_printint(cal_mode_counter+1);
			lcd_set_cursor(6, 3);
			lcd_printstr("No");
		}
		
		else if(menu == CAL2_MENU){
			lcd_set_cursor(6, 2);
			lcd_printint(cal_mode_counter+1);
			lcd_set_cursor(6, 3);
			lcd_printstr("Yes");
		}
	}
	/////////////////////////////////////////////////
	
	
	/* ZEROING MENU *////////////////
	else if(menu == ZEROING_MENU){
		lcd_set_cursor(3, 1);
		lcd_printstr("JOINT  ZEROING");
		lcd_set_cursor(4, 2);
		lcd_printstr("PLEASE  WAIT");
	}
	/////////////////////////////////
	
	
	/* CHANGE SPEED MENU */////////////
	else if(menu == CHANGE_SPEED_MENU){
		lcd_set_cursor(3, 1);
		lcd_printstr("UPDATING SPEED");
		lcd_set_cursor(4, 2);
		lcd_printstr("PLEASE  WAIT");
	}
	
	
	/* HOME MENU *//////////////////////////////////////////////
	else if(menu == HOME_MENU){		
		// SHOW POS CTRL *********************
		if(ctrl_mode_counter == 0){
			if(move_var_counter < 3){
				lcd_set_cursor(9, move_var_counter);
				lcd_printstr("<");
			}
			
			else if(move_var_counter > 2){
				lcd_set_cursor(19, move_var_counter-3);
				lcd_printstr("<");
			}
			
			lcd_set_cursor(0, 0);	
			lcd_printstr("XP:");
			lcd_printfloat(pos_value[0], 1);
			lcd_set_cursor(0, 1);
			lcd_printstr("YP:");
			lcd_printfloat(pos_value[1], 1);
			lcd_set_cursor(0, 2);
			lcd_printstr("ZP:");
			lcd_printfloat(pos_value[2], 1);
			
			lcd_set_cursor(10, 0);
			lcd_printstr("RX:");
			lcd_printfloat(rot_value[0], 1);
			lcd_set_cursor(10, 1);
			lcd_printstr("RY:");
			lcd_printfloat(rot_value[1], 1);
			lcd_set_cursor(10, 2);
			lcd_printstr("RZ:");
			lcd_printfloat(rot_value[2], 1);
			
			lcd_set_cursor(17, 3);
			lcd_printstr(world_ctrl);
		}
		// ***********************************
		
		
		// SHOW ANGLE CTRL **************************
		else if(ctrl_mode_counter == 1){
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
			lcd_printstr(joint_ctrl);
		}
		// ******************************************
		
		
		// CONTINUES MOVE ***********************************
		lcd_set_cursor(0, 3);	
		if(move_mode_counter == 0) lcd_printstr(cont_change);
		// **************************************************
		
		
		// DISTANCE MOVE ***********************
		else if(move_mode_counter == 1){
			lcd_printstr(dist_change);
			lcd_set_cursor(6, 3);	
			lcd_printstr(string_distance);
			
			if(!saved_dist && add_col != 0){
				lcd_set_cursor(6+add_col, 3);	
				lcd_printstr("* ");
			} 
			else if(!saved_dist && add_col == 0){
				lcd_set_cursor(7, 3);	
				lcd_printstr("* ");
			}
		}
		// *************************************
		
		
		// STEP MOVE *********************************************
		else if(move_mode_counter == 2) lcd_printstr(step_change);
		// *******************************************************
		
		
		// SHOW CURRENT MENU *********
		if(move_mode_counter != 1){
			lcd_set_cursor(8, 3);
			lcd_printstr(home_menu);
		}
		// ***************************
	}
	////////////////////////////////////////////////////////////
	
	
	/* PREPARATION MENU *///////////////////////////////////////
	else if(menu == PREP_MENU){
		// MAPPING PREP ******************************************
		if(run_mode_counter == 0){
			lcd_set_cursor(0, 0);
			lcd_printstr("Mapped Point");
			lcd_set_cursor(17, 0);
			if(mapped_points[mapped_point_counter] == 0){
				lcd_printstr("-");
				}
			else lcd_printint(mapped_points[mapped_point_counter]);
			
			lcd_set_cursor(0, 3);
			lcd_printstr(mapping_mode);
		}
		// *******************************************************
		
		
		// PREVIEW PREP ********************
		if(run_mode_counter == 1){
			lcd_set_cursor(0, 0);
			lcd_printstr("Select Point");
			if(!saved_prev) lcd_printstr("*");
			else lcd_printstr(" ");
				
			lcd_set_cursor(17, 0); 
			lcd_printstr(string_preview);
			
			lcd_set_cursor(0, 3);
			lcd_printstr(preview_mode);
		}
		// *********************************
		
		
		// WELDING PREP ********************
		if(run_mode_counter == 2){	
			lcd_set_cursor(0, 0);
			lcd_printstr("Total Mapped");
			lcd_set_cursor(17, 0);
			lcd_printint(total_mapped_points);
			
			lcd_set_cursor(0, 3);
			lcd_printstr(welding_mode);
		}
		// *********************************
		
		
		// SHOW CURRENT MENU ******
		lcd_set_cursor(14, 3);
		lcd_printstr(preparation);
		// ************************
	}
	////////////////////////////////////////////////////////////
	
	
	/* MAPPING MENU 1 *////////////////////////////////////////////
	else if(menu == MAPPING_MENU && mapping_menu_counter == 0){
		// SHOW CARTESIAN COORDINATE **********
		if(ctrl_mode_counter == 0){
			if(move_var_counter < 3){
				lcd_set_cursor(9, move_var_counter);
				lcd_printstr("<");
			}
			
			else if(move_var_counter > 2){
				lcd_set_cursor(19, move_var_counter-3);
				lcd_printstr("<");
			}
			
			lcd_set_cursor(0, 0);	
			lcd_printstr("XP:");
			lcd_printfloat(pos_value[0], 1);
			lcd_set_cursor(0, 1);
			lcd_printstr("YP:");
			lcd_printfloat(pos_value[1], 1);
			lcd_set_cursor(0, 2);
			lcd_printstr("ZP:");
			lcd_printfloat(pos_value[2], 1);
			
			lcd_set_cursor(10, 0);
			lcd_printstr("RX:");
			lcd_printfloat(rot_value[0], 1);
			lcd_set_cursor(10, 1);
			lcd_printstr("RY:");
			lcd_printfloat(rot_value[1], 1);
			lcd_set_cursor(10, 2);
			lcd_printstr("RZ:");
			lcd_printfloat(rot_value[2], 1);
			
			lcd_set_cursor(17, 3);
			lcd_printstr(world_ctrl);
		}
		// ************************************
		
		
		// SHOW JOINT ANGLE ***********************
		else if(ctrl_mode_counter == 1){
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
			lcd_printstr(joint_ctrl);
		}
		// ****************************************
		
		
		// CONTINUES MOVE **************************************
		lcd_set_cursor(0, 3);	
		if(move_mode_counter == 0) lcd_printstr(cont_change);
		
		else if(move_mode_counter == 1){
			lcd_printstr(dist_change);
			lcd_set_cursor(6, 3);	
			lcd_printstr(string_distance);
			
			if(!saved_dist && add_col != 0){
				lcd_set_cursor(6+add_col, 3);	
				lcd_printstr("* ");
			} 
			else if(!saved_dist && add_col == 0){
				lcd_set_cursor(7, 3);	
				lcd_printstr("* ");
			}
		}
		// *****************************************************
		
		 
		// DISTANCE MOVE ***********************
		else if(move_mode_counter == 1){
			lcd_printstr(dist_change);
			lcd_set_cursor(6, 3);	
			lcd_printstr(string_distance);
			
			if(!saved_dist && add_col != 0){
				lcd_set_cursor(6+add_col, 3);	
				lcd_printstr("* ");
			} 
			else if(!saved_dist && add_col == 0){
				lcd_set_cursor(7, 3);	
				lcd_printstr("* ");
			}
		}
		// *************************************
		
		
		// STEP MOVE ************************************************
		else if(move_mode_counter == 2) lcd_printstr(step_change);
		// **********************************************************
		
		
		// SHOW CURRENT MENU *********
		if(move_mode_counter != 1){
			lcd_set_cursor(8, 3);
			lcd_printstr(mapping_menu);
		}
		// ***************************
	}
	///////////////////////////////////////////////////////////////
	
	
	/* MAPPING MENU 2 *//////////////////////////////////////////////
	else if(menu == MAPPING_MENU && mapping_menu_counter == 1){
		// SHOW CURRENT WELDING INFO **********************************
		lcd_set_cursor(19, move_var_counter);
		lcd_printstr("<");
		
		lcd_set_cursor(0, 0);
		lcd_printstr("Strt Point:");
		if(start_point_counter != 0) lcd_printint(start_point_counter);
		else lcd_printstr("-");
		
		lcd_set_cursor(0, 1);
		lcd_printstr("End Point :");
		if(end_point_counter != 0)  lcd_printint(end_point_counter);
		else lcd_printstr("-");
	
		lcd_set_cursor(0, 3);
		lcd_printstr(mapping_menu);
		
		lcd_set_cursor(15 , 3);
		lcd_printstr(mem_menu);
		// ************************************************************
	}
	/////////////////////////////////////////////////////////////////
	
	
	/* MAPPING MENU 3 *//////////////////////////////////////////////
	else if(menu == MAPPING_MENU && mapping_menu_counter == 2){
		// SHOW CURRENT WELDING INFO **********************************
		lcd_set_cursor(0, 0);
		lcd_printstr("Pattern");
		
		if(pattern_sel_counter == 0){
			lcd_set_cursor(14, 0);
			lcd_printstr(not_set_mode);
		}
		else if(pattern_sel_counter == 1){
			lcd_set_cursor(17, 0);
			lcd_printstr(dot_mode);
		}
		else if(pattern_sel_counter == 2){
			lcd_set_cursor(14, 0);
			lcd_printstr(linear_mode);
		}
		else if(pattern_sel_counter == 3){
			lcd_set_cursor(12, 0);
			lcd_printstr(circular_mode);
		}
		else if(pattern_sel_counter == 4){
			lcd_set_cursor(16, 0);
			lcd_printstr(wave_mode);
		}
		
		lcd_set_cursor(0, 1);
		lcd_printstr("Move Spd");
		
		if(speed_mode_counter == 0){
			lcd_set_cursor(14, 1);
			lcd_printstr(not_set_mode);
		}
		else if(speed_mode_counter == LOW){
			lcd_set_cursor(17, 1);
			lcd_printstr(low_speed);
		}
		else if(speed_mode_counter == MED){
			lcd_set_cursor(17, 1);
			lcd_printstr(med_speed);
		}
		else if(speed_mode_counter == HIGH){
			lcd_set_cursor(16, 1);
			lcd_printstr(high_speed);
		}
		
		lcd_set_cursor(0, 3);
		lcd_printstr(mapping_menu);
		
		lcd_set_cursor(15 , 3);
		lcd_printstr(mem_menu);
		// ************************************************************
	}
	/////////////////////////////////////////////////////////////////
	
	
	/* PREVIEW MENU *///////////////////////////////////
	else if(menu == PREVIEW_MENU){
		lcd_set_cursor(0, 0);
		lcd_printstr("Point Pos");
		lcd_set_cursor(17, 0);
		lcd_printint(preview_point);
		
		lcd_set_cursor(0, 1);
		lcd_printstr("Welding Speed");
		lcd_set_cursor(17, 1);
		if(preview_speed == LOW) lcd_printstr(low_speed);
		else if(preview_speed == MED) lcd_printstr(low_speed);
		else if(preview_speed == HIGH) lcd_printstr(high_speed);
		
		lcd_set_cursor(0, 3);
		lcd_printstr(preview_menu);
		
		if(preview_pattern == 0){
			lcd_set_cursor(12, 2);
			lcd_printstr("(");
			lcd_printstr(not_set_mode);
			lcd_printstr(")");
		}
		
		else if(preview_pattern == 1){
			lcd_set_cursor(15, 3);
			lcd_printstr("(");
			lcd_printstr(dot_mode);
			lcd_printstr(")");
		}
		else if(preview_pattern == 2){
			lcd_set_cursor(12, 3);
			lcd_printstr("(");
			lcd_printstr(linear_mode);
			lcd_printstr(")");
		}
		else if(preview_pattern == 3){
			lcd_set_cursor(10, 3);
			lcd_printstr("(");
			lcd_printstr(circular_mode);
			lcd_printstr(")");
		}
		else if(preview_pattern == 4){
			lcd_set_cursor(14, 3);
			lcd_printstr("(");
			lcd_printstr(wave_mode);
			lcd_printstr(")");
		}
	}
	///////////////////////////////////////////////////

	
	/* WELDING MENU 1*/////////////////////////////////////////
	else if(menu == WELDING_MENU && welding_menu_counter == 0){
		// SHOW CARTESIAN COORDINATE *********
		if(ctrl_mode_counter == 0){
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
			
			lcd_set_cursor(10, 0);
			lcd_printstr("RX:");
			lcd_printfloat(rot_value[0], 1);
			lcd_set_cursor(10, 1);
			lcd_printstr("RY:");
			lcd_printfloat(rot_value[1], 1);
			lcd_set_cursor(10, 2);
			lcd_printstr("RZ:");
			lcd_printfloat(rot_value[2], 1);
			
			lcd_set_cursor(17, 3);
			lcd_printstr(world_ctrl);
		}
		// ***********************************
		
		
		// SHOW JOINT ANGLE ***********************
		else if(ctrl_mode_counter == 1){
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
			lcd_printstr(joint_ctrl);
		}
		// ****************************************
		
		
		// SHOW CURRENT MENU *********
		lcd_set_cursor(0, 3);
		lcd_printstr(welding_menu);
		// ***************************
	}
	///////////////////////////////////////////////////////////

	
	/* WELDING MENU 2*/////////////////////////////////////////
	else if(menu == WELDING_MENU && welding_menu_counter == 1){
		lcd_set_cursor(0, 0);
		lcd_printstr("Current Pts");
		lcd_set_cursor(17, 0);
		lcd_printint(preview_point);
		
		lcd_set_cursor(0, 1);
		lcd_printstr("Current Spd");
		if(preview_speed == 0){
			lcd_set_cursor(17, 1);
			lcd_printstr(low_speed);
		}
		else if(preview_speed == 1){
			lcd_set_cursor(17, 1);
			lcd_printstr(med_speed);
		}
		else if(preview_speed == 2){
			lcd_set_cursor(16, 1);
			lcd_printstr(high_speed);
		}
		
		// SHOW CURRENT PATTERN ******
		lcd_set_cursor(0, 2);
		lcd_printstr("Current Ptn");
		
		if(preview_pattern == 0){
			lcd_set_cursor(14, 2);
			lcd_printstr(not_set_mode);
		}
		else if(preview_pattern == 1){
			lcd_set_cursor(17, 2);
			lcd_printstr(dot_mode);
		}
		else if(preview_pattern == 2){
			lcd_set_cursor(14, 2);
			lcd_printstr(linear_mode);
		}
		else if(preview_pattern == 3){
			lcd_set_cursor(12, 2);
			lcd_printstr(circular_mode);
		}
		else if(preview_pattern == 4){
			lcd_set_cursor(16, 2);
			lcd_printstr(wave_mode);
		}
		// ***************************
	}
	///////////////////////////////////////////////////////////
	
	
	/* MAPPING SAVE MENU */////////////
	else if(menu == MAPPING_SAVE_MENU){
		lcd_set_cursor(2, 1);
		lcd_printstr("SAVING PARAMETER");
		lcd_set_cursor(4, 2);
		lcd_printstr("PLEASE  WAIT");
	}
	///////////////////////////////////
	
	
	/* MAPPING ERROR MENU *////////////////
	else if(menu == MAPPING_ERR_MENU){
		lcd_set_cursor(3, 0);
		lcd_printstr("WELDING  POINT");
		lcd_set_cursor(2, 1);
		lcd_printstr("NO SET CORRECTLY");
		lcd_set_cursor(3, 3);
		lcd_printstr("PLEASE REPEAT");
	}
	///////////////////////////////////////
	
	
	/* PREVIEW ERROR MENU *//////////////
	else if(menu == PREVIEW_ERR_MENU){
		lcd_set_cursor(3, 1);
		lcd_printstr("SELECTED POINT");
		lcd_set_cursor(5, 2);
		lcd_printstr("NOT  FOUND");
	}
	/////////////////////////////////////
	
	
	/* POINT SET MENU*///////////////
	else if(menu == POINT_SET_MENU){
		lcd_set_cursor(4, 1);
		lcd_printstr("WELDING POINT");
		lcd_set_cursor(7, 2);
		lcd_printstr("SETTED");
	}
	/////////////////////////////////
	
	
	/* POINT UNSET MENU *//////////////
	else if(menu == POINT_UNSET_MENU){
		lcd_set_cursor(4, 1);
		lcd_printstr("WELDING POINT");
		lcd_set_cursor(6, 2);
		lcd_printstr("UNSETTED");
	}
	///////////////////////////////////
	
	
	/* PAUSE MENU *////////////////////
	else if(menu == PAUSE_MENU){
		lcd_set_cursor(2, 1);
		lcd_printstr("WELDING  PROCESS");
		lcd_set_cursor(6, 2);
		lcd_printstr("PAUSED");
	}
	///////////////////////////////////
	
	
	/* STOP MENU */////////////////////
	else if(menu == STOP_MENU){
		if(run_mode_counter == 1){
			lcd_set_cursor(2, 0);
			lcd_printstr("PREVIEW  PROCESS");
			lcd_set_cursor(6, 1);
			lcd_printstr("STOPED");
			lcd_set_cursor(4, 3);
			lcd_printstr("ROBOT HOMING");
		}
		
		else if(run_mode_counter == 2){
			lcd_set_cursor(2, 0);
			lcd_printstr("WELDING  PROCESS");
			lcd_set_cursor(6, 1);
			lcd_printstr("STOPED");
			lcd_set_cursor(4, 3);
			lcd_printstr("ROBOT HOMING");
		}
	}
	///////////////////////////////////
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- RESET COUNTER ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void reset_counter(LCD_Menu_t menu){
	if(menu == HOME_MENU){
		ctrl_mode_counter = 0;
		move_mode_counter = 0;
		move_var_counter = 0;
		distance_val = 0;
		strcpy(string_distance, "0");
	}
	
	else if(menu == PREP_MENU){
		run_mode_counter = 0;
		mapped_point_counter = 0;  
		strcpy(string_preview, "-");
	}
	
	else if(menu == MAPPING_MENU){
		mapping_menu_counter = 0;
		start_point_counter = 0;
		end_point_counter = 0;
	}
	
	else if(menu == SETTING_MENU){
		cal_mode_counter = 0; 			
		setting_menu_counter = 0;
		joint_sel_counter = 0;
	}
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- LCD UPDATE ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void lcd_update(void){
	osEventFlagsSet(lcdUpdateHandle, 0x01);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- LCD MENU CHANGE ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void change_menu(LCD_Menu_t menu){
	lcd_update();
	select_menu = menu;
	reset_counter(menu);
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


/*--- SAVE WELDING DATA ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Save_WeldingData(uint8_t welding_point, uint8_t welding_pattern, uint8_t welding_speed){
	uint8_t save_data[3] = {welding_point, welding_pattern, welding_speed};
	EEPROM_PageWrite(&eeprom1, welding_point-1, 0, save_data, sizeof(save_data));
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- DELETE WELDING DATA ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Delete_WeldingData(uint8_t welding_point){
	EEPROM_PageReset(&eeprom1, welding_point-1, 0);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- READ WELDING DATA ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Read_WeldingData(uint16_t welding_point, uint8_t* stored_data){
	EEPROM_PageRead(&eeprom1, welding_point-1, 0, stored_data, sizeof(stored_data));
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- UPDATE EEPROM STORED DATA ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Update_Data(Mem_Update_t update_type){
	if(update_type == UPDATE_ALL){
		total_mapped_points = 0; 
		for(uint16_t i=1; i<max_welding_point; i++){
			Read_WeldingData(i, stored_welding_data);
			
			if(stored_welding_data[0] != 0){
				total_mapped_points++;
				mapped_points[i-1] 	= stored_welding_data[0];
				mapped_pattern[i-1] = stored_welding_data[1];
				mapped_speed[i-1] 	= stored_welding_data[2];
			}
			else{
				mapped_points[i-1] 	= 0;
				mapped_pattern[i-1] = 0;
				mapped_speed[i-1] 	= 0;
			}
			if(prep_done) osDelay(1);
			else HAL_Delay(1);
		}
	}
	
	else if(update_type == SELECTED_ONLY){
		if(prep_done) osDelay(1000);
		else HAL_Delay(1000);
		
		Read_WeldingData(start_point_counter, stored_welding_data);
		mapped_points[start_point_counter-1] 	= stored_welding_data[0];
		mapped_pattern[start_point_counter-1] = stored_welding_data[1];
		mapped_speed[start_point_counter-1] 	= stored_welding_data[2];
	}
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- FORMAT EEPROM MEMORY ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Format_mem(void){
	for(uint8_t j=0; j<eeprom1.page_size; j++){
		EEPROM_PageReset(&eeprom1, j, 0);
	}
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartInterfaceTask */
/**
  * @brief  Function implementing the interfaceTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartInterfaceTask */
void StartInterfaceTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		ui_handler();
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartReceiveTask */
/**
* @brief Function implementing the receiveTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartReceiveTask */
void StartReceiveTask(void *argument)
{
  /* USER CODE BEGIN StartReceiveTask */
  /* Infinite loop */
  for(;;)
  {
		uint32_t flags = osEventFlagsWait(getDataHandle, 0x01, osFlagsWaitAny, 0);
		Start_get_command(&command);
		
		if(flags & 0x01){
			Get_command(&command);
			osDelay(5);
			osEventFlagsClear(getDataHandle, 0x01);
		} 
		
		if(command.type == FEEDBACK){
			if(command.feedback == JOINT_MISS_STEP){
				lcd_update();
				select_menu = EMERGENCY_MENU2;
				command.feedback = NO_FEEDBACK;
			}
			else if(command.feedback == ANGLE_SOFT_LIMIT){
				lcd_update();
				select_menu = EMERGENCY_MENU3;
				command.feedback = NO_FEEDBACK;
			}
			else if(command.feedback == ANGLE_HARD_LIMIT){
				lcd_update();
				select_menu = EMERGENCY_MENU4;
				command.feedback = NO_FEEDBACK;
			}
		}
		
		else if(command.type == SEND_REQ){
			for(int i=0; i<6; i++){
				if(i<3){
					pos_value[i] = command.World_pos_req[i];
					rot_value[i] = command.World_rot_req[i];
				}
				angle_value[i] = command.Joint_angle_req[i];
			}
		}
		
		if(HAL_GetTick() - prev_getdata > 250){
			Reset_command(&command);
			prev_getdata = HAL_GetTick();
		}
		
    osDelay(5);
  }
  /* USER CODE END StartReceiveTask */
}

/* USER CODE BEGIN Header_StartLCDTask */
/**
* @brief Function implementing the lcdTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLCDTask */
void StartLCDTask(void *argument)
{
  /* USER CODE BEGIN StartLCDTask */
  /* Infinite loop */
  for(;;)
  {
		uint8_t update_flag;
		
		update_flag = osEventFlagsGet(lcdUpdateHandle);
		
		if(update_flag & 0x01){
			lcd_clear();
			osDelay(10);
			osEventFlagsClear(lcdUpdateHandle, 0x01);
		}
		
		show_menu(select_menu);
    osDelay(50);
  } 
  /* USER CODE END StartLCDTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
