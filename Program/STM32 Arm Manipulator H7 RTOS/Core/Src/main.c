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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "EEPROM_lib.h"
#include "RS232_Driver.h"
#include "ArmRobot_Math.h"
#include "ssd1306.h"
#include "stdbool.h"
#include "stdlib.h"
#include "math.h"
#include "PID_Driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef StaticTask_t osStaticThreadDef_t;
typedef StaticSemaphore_t osStaticSemaphoreDef_t;
typedef StaticEventGroup_t osStaticEventGroupDef_t;
/* USER CODE BEGIN PTD */

/* EEPROM TYPEDEF */
// -----------------
EEPROM_t eeprom1;
EEPROM_t eeprom2;
// -----------------


/* RS232 TYPEDEF */
// ----------------
Data_Get_t command;
// ----------------


/* KINEMATICS TYPEDEF */
// ---------------------
Kinematics_t kinematics;
// ---------------------


/* PID TYPEDEF*/
// -------------------
PIDController pos_pid;
// -------------------


/* OLED TYPEDEF */
// --------------------
typedef enum{
	BOOT_MENU,
	COM_INIT_MENU,
	MAIN_MENU,
	EEPROM1_ERROR_MENU,
	EEPROM2_ERROR_MENU,
}Oled_Menu_t;
// --------------------


/* STEPPER DIR TYPEDEF*/
// ---------------------
typedef enum{
	DIR_CW,
	DIR_CCW
}Stepper_Dir_t;
// ---------------------


/*  LIMIT TYPE TYPEDEF */
// ----------------------
typedef enum{
	ALL_LIMIT_CHECK = 0x01,
	SOFT_LIMIT_CHECK,
	HARD_LIMIT_CHECK,
}Limit_t;
// ----------------------


/* HOMING MODE TYPEDEF*/
// ---------------------
typedef enum{
	ENCODER_LIMIT,
	SWITCH_LIMIT,
}Homing_Mode_t;
// ---------------------


/* POWER STATUS TYPEDEF */
// -----------------------
typedef enum{
	UNDER_VOLTAGE,
	NORM_VOLTAGE,
}Power_Status_t;
// -----------------------


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* SYSTEM CONFIGURATION */
//----------------------
#define USE_OLED
#define USE_EEPROM
#define USE_RS232
#define USE_STEPPER
#define USE_ENCODER
//#define USE_KINEMATICS
//#define USE_PID

//#define TEST_EEPROM
//#define TEST_RS232
//#define TEST_OLED
//#define TEST_STEPPER
//#define TEST_ENCODER
//#define TEST_KINEMATICS

//#define MAIN_PROGRAM
//----------------------

/* EEPROM ADDRESS SET */
//-----------------------------------
#define EEPROM1_ADDRESS					0xA0	// PRIMARY EEPROM ADDRESS
#define EEPROM2_ADDRESS					0xA1	// SECONDARY EEPROM ADDRESS

#define SP_POS_BYTE_ADDR				0x00	// START POINT POSITION ADDR (12 Bytes)
#define SP_ANG_BYTE_ADDR				0x0C	// START POINT ANGLE ADDR	(24 Bytes)
#define EP_POS_BYTE_ADDR				0x24	// END POINT POSITION ADDR (12 Bytes)
#define EP_ANG_BYTE_ADDR				0x30	// END POINT ANGLE ADDR	(24 Bytes)
#define PATTERN_BYTE_ADDR				0x48	// PATTERN ADDR (1 Byte)
//-----------------------------------

/* EEPROM PATTERN SET */
//-----------------------------------
#define LINEAR_PATTERN					0x01
#define CIRCULAR_PATTERN				0x02
#define ZIGZAG_PATTERN					0x03
//-----------------------------------

/* STEPPER SET */
//-----------------------------
#define MICROSTEP_FACTOR1	1
#define MICROSTEP_FACTOR2	2
#define MICROSTEP_FACTOR3	4
#define MICROSTEP_FACTOR4	8
#define MICROSTEP_FACTOR5	16
#define MICROSTEP_FACTOR6	32

#define MICROSTEP_VALUE1 	200
#define MICROSTEP_VALUE2 	400
#define MICROSTEP_VALUE3 	800
#define MICROSTEP_VALUE4 	1600
#define MICROSTEP_VALUE5	3200
#define MICROSTEP_VALUE6 	6400

#define STEPPER1_RATIO		90
#define	STEPPER2_RATIO		36
#define	STEPPER3_RATIO		36
#define	STEPPER4_RATIO		9
#define	STEPPER5_RATIO		90
#define	STEPPER6_RATIO		9

#define STEPPER1_MAXSPD		420
#define STEPPER2_MAXSPD		315
#define STEPPER3_MAXSPD		420
#define STEPPER4_MAXSPD		420
#define STEPPER5_MAXSPD		420
#define STEPPER6_MAXSPD		420

#define SET_DUTY_CYCLE		0.25f
#define BASE_FREQ					1000

#define STOPPING					0x01
#define ACCELERATING			0X02
#define AT_TOP_SPEED			0X03
#define DECELERATING			0X04
//-----------------------------


/*ROBOT SET*/
//-------------------------------
#define JOINT_NUM					6
#define AXIS_NUM					3

#define JOINT1_MAX_ANGLE	110.0f
#define JOINT2_MAX_ANGLE	90.0f
#define JOINT3_MAX_ANGLE	60.0f
#define JOINT4_MAX_ANGLE	165.0f
#define JOINT5_MAX_ANGLE	105.0f
#define JOINT6_MAX_ANGLE	155.0f

#define JOINT1_MIN_ANGLE	-110.0f
#define JOINT2_MIN_ANGLE	-60.0f
#define JOINT3_MIN_ANGLE	-90.0f
#define JOINT4_MIN_ANGLE	-165.0f
#define JOINT5_MIN_ANGLE	-105.0f
#define JOINT6_MIN_ANGLE	-155.0f
//-------------------------------


/* RUNNING SET */
//------------------------
#define WELDING_FLAG	0X01
#define PREVIEW_FLAG	0X02
//------------------------

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart4;

/* Definitions for encoderRead */
osThreadId_t encoderReadHandle;
uint32_t encoderReadBuffer[ 128 ];
osStaticThreadDef_t encoderReadControlBlock;
const osThreadAttr_t encoderRead_attributes = {
  .name = "encoderRead",
  .cb_mem = &encoderReadControlBlock,
  .cb_size = sizeof(encoderReadControlBlock),
  .stack_mem = &encoderReadBuffer[0],
  .stack_size = sizeof(encoderReadBuffer),
  .priority = (osPriority_t) osPriorityRealtime,
};
/* Definitions for limitCheck */
osThreadId_t limitCheckHandle;
uint32_t limitCheckBuffer[ 128 ];
osStaticThreadDef_t limitCheckControlBlock;
const osThreadAttr_t limitCheck_attributes = {
  .name = "limitCheck",
  .cb_mem = &limitCheckControlBlock,
  .cb_size = sizeof(limitCheckControlBlock),
  .stack_mem = &limitCheckBuffer[0],
  .stack_size = sizeof(limitCheckBuffer),
  .priority = (osPriority_t) osPriorityHigh1,
};
/* Definitions for stepperControl */
osThreadId_t stepperControlHandle;
uint32_t stepperControlBuffer[ 256 ];
osStaticThreadDef_t stepperControlControlBlock;
const osThreadAttr_t stepperControl_attributes = {
  .name = "stepperControl",
  .cb_mem = &stepperControlControlBlock,
  .cb_size = sizeof(stepperControlControlBlock),
  .stack_mem = &stepperControlBuffer[0],
  .stack_size = sizeof(stepperControlBuffer),
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for commandHandler */
osThreadId_t commandHandlerHandle;
uint32_t commandHandlerBuffer[ 256 ];
osStaticThreadDef_t commandHandlerControlBlock;
const osThreadAttr_t commandHandler_attributes = {
  .name = "commandHandler",
  .cb_mem = &commandHandlerControlBlock,
  .cb_size = sizeof(commandHandlerControlBlock),
  .stack_mem = &commandHandlerBuffer[0],
  .stack_size = sizeof(commandHandlerBuffer),
  .priority = (osPriority_t) osPriorityAboveNormal1,
};
/* Definitions for runningHandler */
osThreadId_t runningHandlerHandle;
uint32_t runningHandlerBuffer[ 512 ];
osStaticThreadDef_t runningHandlerControlBlock;
const osThreadAttr_t runningHandler_attributes = {
  .name = "runningHandler",
  .cb_mem = &runningHandlerControlBlock,
  .cb_size = sizeof(runningHandlerControlBlock),
  .stack_mem = &runningHandlerBuffer[0],
  .stack_size = sizeof(runningHandlerBuffer),
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for posUpdate */
osThreadId_t posUpdateHandle;
uint32_t posUpdateBuffer[ 128 ];
osStaticThreadDef_t posUpdateControlBlock;
const osThreadAttr_t posUpdate_attributes = {
  .name = "posUpdate",
  .cb_mem = &posUpdateControlBlock,
  .cb_size = sizeof(posUpdateControlBlock),
  .stack_mem = &posUpdateBuffer[0],
  .stack_size = sizeof(posUpdateBuffer),
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for lcdMenu */
osThreadId_t lcdMenuHandle;
uint32_t lcdMenuBuffer[ 128 ];
osStaticThreadDef_t lcdMenuControlBlock;
const osThreadAttr_t lcdMenu_attributes = {
  .name = "lcdMenu",
  .cb_mem = &lcdMenuControlBlock,
  .cb_size = sizeof(lcdMenuControlBlock),
  .stack_mem = &lcdMenuBuffer[0],
  .stack_size = sizeof(lcdMenuBuffer),
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for encoderRW */
osSemaphoreId_t encoderRWHandle;
osStaticSemaphoreDef_t encoderRWControlBlock;
const osSemaphoreAttr_t encoderRW_attributes = {
  .name = "encoderRW",
  .cb_mem = &encoderRWControlBlock,
  .cb_size = sizeof(encoderRWControlBlock),
};
/* Definitions for stepperRW */
osSemaphoreId_t stepperRWHandle;
osStaticSemaphoreDef_t stepperRWControlBlock;
const osSemaphoreAttr_t stepperRW_attributes = {
  .name = "stepperRW",
  .cb_mem = &stepperRWControlBlock,
  .cb_size = sizeof(stepperRWControlBlock),
};
/* Definitions for commandRW */
osSemaphoreId_t commandRWHandle;
osStaticSemaphoreDef_t commandRWControlBlock;
const osSemaphoreAttr_t commandRW_attributes = {
  .name = "commandRW",
  .cb_mem = &commandRWControlBlock,
  .cb_size = sizeof(commandRWControlBlock),
};
/* Definitions for runningFlag */
osEventFlagsId_t runningFlagHandle;
osStaticEventGroupDef_t runningFlagControlBlock;
const osEventFlagsAttr_t runningFlag_attributes = {
  .name = "runningFlag",
  .cb_mem = &runningFlagControlBlock,
  .cb_size = sizeof(runningFlagControlBlock),
};
/* USER CODE BEGIN PV */

// PARAMETER VARIABLE -------
double
joint_positive_axisLim[6] = {
	JOINT1_MAX_ANGLE,
	JOINT2_MAX_ANGLE,
	JOINT3_MAX_ANGLE,
	JOINT4_MAX_ANGLE,
	JOINT5_MAX_ANGLE,
	JOINT6_MAX_ANGLE,
	
},
joint_negative_axisLim[6] = {
	JOINT1_MIN_ANGLE,
	JOINT2_MIN_ANGLE,
	JOINT3_MIN_ANGLE,
	JOINT4_MIN_ANGLE,
	JOINT5_MIN_ANGLE,
	JOINT6_MIN_ANGLE,
	
},
joint_max_traveldist[6],
joint_step_per_deg[6],
joint_deg_per_step[6];

bool
home = false,
soft_limit[6],
hard_limit[6],
positive_limit[6],
negative_limit[6],
joint_limit = false,
joint_limit_bypass = false,
limit_switch_state[6];

uint8_t
robot_axis[6] = {
	AXIS_X,
	AXIS_Y,
	AXIS_Z
},

robot_joint[6] = {
	JOINT_1,
	JOINT_2,
	JOINT_3,
	JOINT_4,
	JOINT_5,
	JOINT_6
};

double
joint_d[6] = {
	191.0, 0.0, 0.0,
	575.0, 0.0, 65.0,
},

joint_a[6] = {
	23.5, 650.0, 0.0,
	0.0, 0.0, 0.0, 
},

joint_alpha[6] = {
	-90.0, 0.0, 90.0,
	-90.0, 90.0, 0.0,
};
//---------------------------


// EEPROM VARIABLE ---
float 
array_pos_start[3],
array_pos_end[3],
array_ang_start[6],
array_ang_end[6];

uint8_t
page_data_byte[128],

saved_pos[12], 
read_saved_pos[12],
read_saved_posX[4],
read_saved_posY[4],
read_saved_posZ[4],

saved_angle[24], 
read_saved_angle[24],
read_saved_angle1[4],
read_saved_angle2[4],
read_saved_angle3[4],
read_saved_angle4[4],
read_saved_angle5[4],
read_saved_angle6[4],

welding_pattern,
read_saved_pattern[1];
//--------------------


// INVERSE KINEMATICS VARIABLE
double
ik_pos_input[6],
ik_angle_output[6];

// FORWARD KINEMATICS VARIABLE
double
fk_angle_input[6],
fk_pos_output[6];

// MOVE VARIABLE ---
double
current_position[3],
current_rotation[3],
delta_move_pos[3],
prev_position[3],

current_angle[6],
delta_move_ang[6],
prev_angle[6],
prev_input_angle[6];

uint8_t
direction,
current_point,
current_speed;
//------------------


// STEPPER VARIABLE -------
TIM_TypeDef*
stepper_tim_instance[6] = {
	TIM16,
	TIM17,
	TIM12,
	TIM13,
	TIM14,
	TIM15
};

TIM_HandleTypeDef*
stepper_tim_handler[6] = {
	&htim16,
	&htim17,
	&htim12,
	&htim13,
	&htim14,
	&htim15
};

GPIO_TypeDef*
stepper_gpio_port[12] = {
	PUL1_GPIO_Port,
	PUL2_GPIO_Port,
	PUL3_GPIO_Port,
	PUL4_GPIO_Port,
	PUL5_GPIO_Port,
	PUL6_GPIO_Port,
	
	DIR1_GPIO_Port,
	DIR2_GPIO_Port,
	DIR3_GPIO_Port,
	DIR4_GPIO_Port,
	DIR5_GPIO_Port,
	DIR6_GPIO_Port,
};

unsigned long
step_timer,
rpm_timer;

bool
step_start[6],
step_limit[6],
step_pos_dir[6],
step_dir_inv[6],
step_home_dir[6];

uint8_t
step_state[6];

uint16_t
rpm_counter,
stepper_rpm[6],
stepper_freq[6],
t_on[6],
t_off[6],
step_input[6],
step_output[6],
step_freq[6],
step_period[6],
step_accel_period[6],
stepper_stepfactor[6],

stepper_gpio_pin[12] = {
	PUL1_Pin,
	PUL2_Pin,
	PUL3_Pin,
	PUL4_Pin,
	PUL5_Pin,
	PUL6_Pin,
	
	DIR1_Pin,
	DIR2_Pin,
	DIR3_Pin,
	DIR4_Pin,
	DIR5_Pin,
	DIR6_Pin,
},

stepper_microstep[6] = {
	MICROSTEP_VALUE3,
	MICROSTEP_VALUE3,
	MICROSTEP_VALUE3,
	MICROSTEP_VALUE3,
	MICROSTEP_VALUE3,
	MICROSTEP_VALUE3,
},

stepper_ratio[6] = {
	STEPPER1_RATIO,
	STEPPER2_RATIO,
	STEPPER3_RATIO,
	STEPPER4_RATIO,
	STEPPER5_RATIO,
	STEPPER6_RATIO,
};

uint32_t
timer_counter[6],
step_counter[6],
current_step[6];

float
joint_rpm;

double
joint_delta_angle[6];
//-----------------------


// RS232 VARIABLE -------
unsigned long
prev_time_send,
prev_time_get;

float
tx_pos[3],
tx_angle[6],
tx_welding_point,
tx_welding_pattern,

rx_move_position[3],
rx_move_rotation[3],
rx_move_angle[6];

uint8_t
send_data_interval = 50,
get_buff[80],
rx_savepoint,
rx_patterntype[200],
rx_pointtype,
rx_rotate_mode,
rx_rotate_value[8];

uint8_t
struct_size;
//----------------------


// OLED LCD VARIABLE ----------
uint8_t 
refresh_counter,
selected_menu;

char 
title[]  = "   WELDING ARM V1",
error1[] = "EEPROM1 ERROR    ",
error2[] = "EEPROM2 ERROR    ";
//-----------------------------


// ENCODER VARIABLE -----
TIM_TypeDef*
enc_tim_instance[6] = {
	TIM4,
	TIM8,
	TIM2,
	TIM1,
	TIM3,
	TIM5
};

TIM_HandleTypeDef*
enc_tim_handler[6] = {
	&htim4,
	&htim8,
	&htim2,
	&htim1,
	&htim3,
	&htim5
};

GPIO_TypeDef*
enc_gpio_port[12] = {
	ENC1A_GPIO_Port,
	ENC2A_GPIO_Port,
	ENC3A_GPIO_Port,
	ENC4A_GPIO_Port,
	ENC5A_GPIO_Port,
	ENC6A_GPIO_Port,
	
	ENC1B_GPIO_Port,
	ENC2B_GPIO_Port,
	ENC3B_GPIO_Port,
	ENC4B_GPIO_Port,
	ENC5B_GPIO_Port,
	ENC6B_GPIO_Port
};

uint16_t
enc_gpio_pin[12] = {
	ENC1A_Pin,
	ENC2A_Pin,
	ENC3A_Pin,
	ENC4A_Pin,
	ENC5A_Pin,
	ENC6A_Pin,
	
	ENC1B_Pin,
	ENC2B_Pin,
	ENC3B_Pin,
	ENC4B_Pin,
	ENC5B_Pin,
	ENC6B_Pin
};

uint8_t
enc_mag_status[6];

uint32_t 
pwm_freq[6],
high_time[6],
period_time[6];

int
enc_counter[6];

double
duty_cycle[6],
min_duty_cycle = 2.9,
max_duty_cycle = 97.1,
diff_angle[6],
enc_angle[6],
cal_value[6],
cal_enc_angle[6],
prev_cal_enc_angle[6],
reduced_cal_enc_angle[6],
enc_joint_angle[6],
prev_enc_joint_angle[6],

max_joint_angle[6] = {
	JOINT1_MAX_ANGLE,
	JOINT2_MAX_ANGLE,
	JOINT3_MAX_ANGLE,
	JOINT4_MAX_ANGLE,
	JOINT5_MAX_ANGLE,
	JOINT6_MAX_ANGLE
},
min_joint_angle[6] = {
	JOINT1_MIN_ANGLE,
	JOINT2_MIN_ANGLE,
	JOINT3_MIN_ANGLE,
	JOINT4_MIN_ANGLE,
	JOINT5_MIN_ANGLE,
	JOINT6_MIN_ANGLE,
};
//-----------------------


// POWER VARIABLE ---
uint8_t pwr_status;
//-------------------

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM17_Init(void);
void StartEncoderRead(void *argument);
void StartLimitCheck(void *argument);
void StartStepperControl(void *argument);
void StartCommandHandler(void *argument);
void StartRunningHandler(void *argument);
void StartPosUpdate(void *argument);
void StartLCDMenu(void *argument);

/* USER CODE BEGIN PFP */

// EEPROM Function
void save_weldingpos_world(uint16_t welding_point, uint8_t start_addr, float* pos_data, size_t size);
void read_weldingpos_world(uint16_t welding_point, uint8_t start_addr, float* stored_data, size_t size);
void save_weldingpos_angle(uint16_t welding_point, uint8_t start_addr, float* angle_data, size_t size);
void read_weldingpos_angle(uint16_t welding_point, uint8_t start_addr, float* stored_data, size_t size);
void save_weldingpos_pattern(uint16_t welding_point, uint8_t select_pattern);
void read_weldingpos_pattern(uint16_t welding_point, uint8_t store_pattern);
void read_all_weldingdata(uint16_t welding_point);
uint16_t check_weldingpoint(void);

// Stepper Control Function
void disable_stepper(void);
void enable_stepper(void);
void move_stepper(uint8_t select_joint, uint16_t step, Stepper_Dir_t dir, uint16_t input_freq);

// Calculation Function
uint32_t calculate_exponential_step_interval(uint8_t select_joint, uint32_t step);

// Main Function
void move_joint(uint8_t select_joint, double input_angle, Speed_t set_speed);
void move_world(uint8_t move_var, double move_pos, Speed_t set_speed);
void welder_on(void);
void welder_off(void);
void welding_preview(void);
void welding_start(void);
void update_value(void);
void check_limit(Limit_t check_mode);
bool homing(Homing_Mode_t mode);

// Encoder Function
void encoder_init(void);
void encoder_reset(void);

// Custom RS232 Function
void send_mapped_data(void);
void send_ang_pos_data(void);
void receive_data(void);

// OLED Function
void show_menu(Oled_Menu_t sel_menu);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* RS232 CALLBACK */
//------------------------------------------------------
uint32_t RS232_state, RS232_err_status;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == UART4){
		RS232_state = check_state();
		command.msg_get = true;
		prev_time_get = HAL_GetTick();
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == UART4){
		command.msg_sent = true;
	}
}
//------------------------------------------------------


/* INPUT CAPTURE CALLBACK */
//----------------------------------------------------------------------------------------------------
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	for(int i=0; i<JOINT_NUM; i++){
		if(htim->Instance == enc_tim_instance[i]){
			
			if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
				high_time[i] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			}
			
			if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
				period_time[i] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
			}
		}
	}
	osSemaphoreRelease(encoderRWHandle);
}
//----------------------------------------------------------------------------------------------------


/* EXTI BUTTON CALLBACK */
//---------------------------------------------------------------------------------------------
void HAL_GPIO_EXTI_CALLBACK(uint16_t GPIO_PIN){
	// Limit 1 
	if(GPIO_PIN == LIMIT1_Pin){
		limit_switch_state[0] = (HAL_GPIO_ReadPin(LIMIT1_GPIO_Port, LIMIT1_Pin) == GPIO_PIN_RESET);
	}
	
	// Limit 2
	else if(GPIO_PIN == LIMIT2_Pin){
		limit_switch_state[1] = (HAL_GPIO_ReadPin(LIMIT2_GPIO_Port, LIMIT2_Pin) == GPIO_PIN_RESET);
	}
	
	// Limit 3 
	else if(GPIO_PIN == LIMIT3_Pin){
		limit_switch_state[2] = (HAL_GPIO_ReadPin(LIMIT3_GPIO_Port, LIMIT3_Pin) == GPIO_PIN_RESET);
	}
	
	// Limit 4 
	else if(GPIO_PIN == LIMIT4_Pin){
		limit_switch_state[3] = (HAL_GPIO_ReadPin(LIMIT4_GPIO_Port, LIMIT4_Pin) == GPIO_PIN_RESET);
	}
	
	// Limit 5 
	else if(GPIO_PIN == LIMIT5_Pin){
		limit_switch_state[4] = (HAL_GPIO_ReadPin(LIMIT5_GPIO_Port, LIMIT5_Pin) == GPIO_PIN_RESET);
	}
	
	// Limit 6 
	else if(GPIO_PIN == LIMIT6_Pin){
		limit_switch_state[5] = (HAL_GPIO_ReadPin(LIMIT6_GPIO_Port, LIMIT6_Pin) == GPIO_PIN_RESET);
	}
}
//---------------------------------------------------------------------------------------------

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

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
  MX_UART4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM17_Init();
  /* USER CODE BEGIN 2 */
	
	/* OLED LCD SETUP */
	#ifdef USE_OLED
	SSD1306_Init();
	selected_menu = BOOT_MENU;
	show_menu((Oled_Menu_t)selected_menu);
	HAL_Delay(1000);
	#endif
	
	
	/* EEPROM SETUP */
	#ifdef USE_EEPROM
	EEPROM_Init(&hi2c1, &eeprom1, MEM_SIZE_512Kb, EEPROM1_ADDRESS);
	EEPROM_ByteRead(&eeprom1, 511, 0, eeprom1.dummy_byte, 1);
	if(eeprom1.status != EEPROM_OK){
		while(1){ 
			selected_menu = EEPROM1_ERROR_MENU;
		}
	}
	
	EEPROM_Init(&hi2c1, &eeprom2, MEM_SIZE_512Kb, EEPROM2_ADDRESS);
	EEPROM_ByteRead(&eeprom2, 511, 0, eeprom1.dummy_byte, 1);
	if(eeprom2.status != EEPROM_OK){
		while(1){ 
			selected_menu = EEPROM2_ERROR_MENU;
		}
	}
	#endif
	
	
	/* RS-232 COMMUNICATION SETUP */
	#ifdef USE_RS232
	RS232_Init(&huart4);
	show_menu(COM_INIT_MENU);
	
	uint32_t timeout = HAL_GetTick() + 5000;
	while(HAL_GetTick() < timeout){
		receive_data(); 
		if(command.type == FEEDBACK && command.feedback == PENDANT_ONLINE){
			SSD1306_Clear();
			break;
		}
	}
	
	show_menu(MAIN_MENU);
	command.msg_sent = true;
	command.msg_get = true;
	#endif
	
	
	/* ENCODER SETUP */
	#ifdef USE_ENCODER
	encoder_init();
	HAL_Delay(500);
	encoder_reset();
	HAL_Delay(1000);
	#endif
	
	
	/* USE KINEMATICS */
	#ifdef USE_KINEMATICS
	DHparam_init(&kinematics, joint_d, joint_a, joint_alpha);
	tollframe_init(&kinematics, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	forward_transform_matrix(&kinematics);
	calculate_all_link(&kinematics);
	
	run_forward_kinematic(&kinematics, enc_joint_angle);
	#endif
	
	
	/* STEPPER SETUP */
	#ifdef USE_STEPPER
	enable_stepper();
	
	for(int i=0; i<JOINT_NUM; i++){
		if(stepper_microstep[i] == MICROSTEP_VALUE1) stepper_stepfactor[i] = MICROSTEP_FACTOR1;
		else if(stepper_microstep[i] == MICROSTEP_VALUE2) stepper_stepfactor[i] = MICROSTEP_FACTOR2;
		else if(stepper_microstep[i] == MICROSTEP_VALUE3) stepper_stepfactor[i] = MICROSTEP_FACTOR3;
		else if(stepper_microstep[i] == MICROSTEP_VALUE4) stepper_stepfactor[i] = MICROSTEP_FACTOR4;
		else if(stepper_microstep[i] == MICROSTEP_VALUE5) stepper_stepfactor[i] = MICROSTEP_FACTOR5;
		else if(stepper_microstep[i] == MICROSTEP_VALUE6) stepper_stepfactor[i] = MICROSTEP_FACTOR6;
		
		joint_max_traveldist[i] = joint_positive_axisLim[i] + joint_negative_axisLim[i];
		joint_step_per_deg[i] = (stepper_microstep[i] * stepper_ratio[i]) / 360.0;
		joint_deg_per_step[i] = 360.0 / (stepper_microstep[i] * stepper_ratio[i]);
		
		step_state[i] = STOPPING;
	}
	
	step_pos_dir[0] = DIR_CW;
	step_pos_dir[1] = DIR_CW;
	step_pos_dir[2] = DIR_CW;
	step_pos_dir[3] = DIR_CW;
	step_pos_dir[4] = DIR_CW;
	step_pos_dir[5] = DIR_CW;
	#endif
	
	
	/* PID SETUP */
	#ifdef USE_PID
	pos_pid.Kp = 0.0;
	pos_pid.Ki = 0.0;
	pos_pid.Kd = 0.0;
	pos_pid.limMax = 0.0;
	pos_pid.limMin = 0.0;
	pos_pid.limMaxInt = 0.0;
	pos_pid.limMinInt = 0.0;
	pos_pid.tau = 0.2;
	pos_pid.T_sample = 0.01f;
	#endif
	
	
	#ifdef TEST_EEPROM
	float 
	test_pos_start[3] = {-40.96, 12.15, -37.42},
	test_pos_end[3] = {-20.96, 2.15, -37.42},
	test_angle_start[6] = {70.55, -45.5, 90.58, 150.45, 55.17, 178.77},
	test_angle_end[6] = {60, -20, 70.25, 115.62, 66.18, 122.76};
	
	save_weldingpos_world(0x00, SP_POS_BYTE_ADDR, test_pos_start, sizeof(test_pos_start));
	save_weldingpos_world(0x00, EP_POS_BYTE_ADDR, test_pos_end, sizeof(test_pos_end));
	save_weldingpos_angle(0x00, SP_ANG_BYTE_ADDR, test_angle_start, sizeof(test_angle_start));
	save_weldingpos_angle(0x00, EP_ANG_BYTE_ADDR, test_angle_end, sizeof(test_angle_end));
	save_weldingpos_pattern(0x00, LINEAR_PATTERN);
	HAL_Delay(500);
	read_weldingpos_world(0x00, SP_POS_BYTE_ADDR, array_pos_start, sizeof(array_pos_start));
	read_weldingpos_world(0x00, EP_POS_BYTE_ADDR, array_pos_end, sizeof(array_pos_end));
	read_weldingpos_angle(0x00, SP_ANG_BYTE_ADDR, array_ang_start, sizeof(array_ang_start));
	read_weldingpos_angle(0x00, EP_ANG_BYTE_ADDR, array_ang_end, sizeof(array_ang_end));
	read_weldingpos_pattern(0x00, welding_pattern);
	HAL_Delay(500);
	read_all_weldingdata(0x00);
	#endif
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of encoderRW */
  encoderRWHandle = osSemaphoreNew(1, 1, &encoderRW_attributes);

  /* creation of stepperRW */
  stepperRWHandle = osSemaphoreNew(1, 1, &stepperRW_attributes);

  /* creation of commandRW */
  commandRWHandle = osSemaphoreNew(1, 1, &commandRW_attributes);

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
  /* creation of encoderRead */
  encoderReadHandle = osThreadNew(StartEncoderRead, NULL, &encoderRead_attributes);

  /* creation of limitCheck */
  limitCheckHandle = osThreadNew(StartLimitCheck, NULL, &limitCheck_attributes);

  /* creation of stepperControl */
  stepperControlHandle = osThreadNew(StartStepperControl, NULL, &stepperControl_attributes);

  /* creation of commandHandler */
  commandHandlerHandle = osThreadNew(StartCommandHandler, NULL, &commandHandler_attributes);

  /* creation of runningHandler */
  runningHandlerHandle = osThreadNew(StartRunningHandler, NULL, &runningHandler_attributes);

  /* creation of posUpdate */
  posUpdateHandle = osThreadNew(StartPosUpdate, NULL, &posUpdate_attributes);

  /* creation of lcdMenu */
  lcdMenuHandle = osThreadNew(StartLCDMenu, NULL, &lcdMenu_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of runningFlag */
  runningFlagHandle = osEventFlagsNew(&runningFlag_attributes);

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV4;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

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
  hi2c1.Init.Timing = 0x00401959;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 99;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim1, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
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
  if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
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
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 99;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 99;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim4, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 99;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim5, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 65535;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_TI1FP1;
  sSlaveConfig.TriggerPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sSlaveConfig.TriggerPrescaler = TIM_ICPSC_DIV1;
  sSlaveConfig.TriggerFilter = 0;
  if (HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_FALLING;
  sConfigIC.ICSelection = TIM_ICSELECTION_INDIRECTTI;
  if (HAL_TIM_IC_ConfigChannel(&htim8, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 99;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim12, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 0;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 99;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 0;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 99;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 0;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 99;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim15, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 99;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 0;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 99;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, ENABLE1_Pin|ENABLE2_Pin|DIR5_Pin|ENC2B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, ENC6B_Pin|DIR6_Pin|PUL6_Pin|ENC5B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PUL5_Pin|DIR4_Pin|PUL2_Pin|DIR1_Pin
                          |PUL1_Pin|ENC3B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, PUL4_Pin|DIR3_Pin|ENC4B_Pin|PUL3_Pin
                          |DIR2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, MIG_SW_IN_Pin|ENC1B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LIMIT1_Pin LIMIT2_Pin LIMIT3_Pin LIMIT4_Pin
                           LIMIT5_Pin */
  GPIO_InitStruct.Pin = LIMIT1_Pin|LIMIT2_Pin|LIMIT3_Pin|LIMIT4_Pin
                          |LIMIT5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LIMIT6_Pin */
  GPIO_InitStruct.Pin = LIMIT6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(LIMIT6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : ENABLE1_Pin ENABLE2_Pin DIR5_Pin ENC2B_Pin */
  GPIO_InitStruct.Pin = ENABLE1_Pin|ENABLE2_Pin|DIR5_Pin|ENC2B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : ENC6B_Pin DIR6_Pin ENC5B_Pin */
  GPIO_InitStruct.Pin = ENC6B_Pin|DIR6_Pin|ENC5B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PUL6_Pin */
  GPIO_InitStruct.Pin = PUL6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(PUL6_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PUL5_Pin PUL2_Pin PUL1_Pin */
  GPIO_InitStruct.Pin = PUL5_Pin|PUL2_Pin|PUL1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR4_Pin DIR1_Pin ENC3B_Pin */
  GPIO_InitStruct.Pin = DIR4_Pin|DIR1_Pin|ENC3B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PUL4_Pin PUL3_Pin */
  GPIO_InitStruct.Pin = PUL4_Pin|PUL3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : DIR3_Pin ENC4B_Pin DIR2_Pin */
  GPIO_InitStruct.Pin = DIR3_Pin|ENC4B_Pin|DIR2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : MIG_SW_IN_Pin ENC1B_Pin */
  GPIO_InitStruct.Pin = MIG_SW_IN_Pin|ENC1B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PC8 PC9 PC10 PC11
                           PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDIO1;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF12_SDIO1;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 14, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 14, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 14, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 14, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 14, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/*--- SAVE WELDING POINT POSITION VALUE ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void save_weldingpos_world(uint16_t welding_point, uint8_t start_addr, float* pos_data, size_t size){
	for(int i=0; i<12; i++) saved_pos[i] = 0;
	for(size_t i=0; i<size; i++) memcpy(&saved_pos[i*4], &pos_data[i], sizeof(float));
	
	EEPROM_PageWrite(&eeprom1, welding_point, start_addr, saved_pos, sizeof(saved_pos));
	HAL_Delay(10);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- READ WELDING POINT POSITION VALUE ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void read_weldingpos_world(uint16_t welding_point, uint8_t start_addr, float* stored_data, size_t size){
	for(int i=0; i<12; i++) read_saved_pos[i] = 0;
	for(size_t i=0; i<size; i++) stored_data[i] = 0;
	
	EEPROM_PageRead(&eeprom1, welding_point, start_addr, read_saved_pos, sizeof(read_saved_pos));
	
	for(int i=0; i<4; i++){
		read_saved_posX[i] = read_saved_pos[i];
		read_saved_posY[i] = read_saved_pos[i+4];
		read_saved_posZ[i] = read_saved_pos[i+8];
	}
	
	memcpy(&stored_data[0], read_saved_posX, sizeof(float));
	memcpy(&stored_data[1], read_saved_posY, sizeof(float));
	memcpy(&stored_data[2], read_saved_posZ, sizeof(float));
	HAL_Delay(10);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- SAVE WELDING POINT ANGLE VALUE ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void save_weldingpos_angle(uint16_t welding_point, uint8_t start_addr, float* angle_data, size_t size){
	for(int i=0; i<24; i++) saved_angle[i] = 0;
	for(size_t i=0; i<size; i++) memcpy(&saved_angle[i*4], &angle_data[i], sizeof(float));
	
	EEPROM_PageWrite(&eeprom1, welding_point, start_addr, saved_angle, sizeof(saved_angle));
	HAL_Delay(10);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- SAVE WELDING POINT ANGLE VALUE ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void read_weldingpos_angle(uint16_t welding_point, uint8_t start_addr, float* stored_data, size_t size){
	for(int i=0; i<24; i++) read_saved_angle[i] = 0;
	for(size_t i=0; i<size; i++) stored_data[i] = 0;
	
	EEPROM_PageRead(&eeprom1, welding_point, start_addr, read_saved_angle, sizeof(read_saved_angle));
	
	for(int i=0; i<4; i++){
		read_saved_angle1[i] = read_saved_angle[i];
		read_saved_angle2[i] = read_saved_angle[i+4];
		read_saved_angle3[i] = read_saved_angle[i+8];
		read_saved_angle4[i] = read_saved_angle[i+12];
		read_saved_angle5[i] = read_saved_angle[i+16];
		read_saved_angle6[i] = read_saved_angle[i+20];
	}
	
	memcpy(&stored_data[0], read_saved_angle1, sizeof(float));
	memcpy(&stored_data[1], read_saved_angle2, sizeof(float));
	memcpy(&stored_data[2], read_saved_angle3, sizeof(float));
	memcpy(&stored_data[3], read_saved_angle4, sizeof(float));
	memcpy(&stored_data[4], read_saved_angle5, sizeof(float));
	memcpy(&stored_data[5], read_saved_angle6, sizeof(float));
	HAL_Delay(10);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- SAVE WELDING POINT PATTERN ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void save_weldingpos_pattern(uint16_t welding_point, uint8_t select_pattern){
	EEPROM_ByteWrite(&eeprom1, welding_point, PATTERN_BYTE_ADDR, select_pattern, sizeof(select_pattern));
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- READ WELDING POINT PATTERN ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void read_weldingpos_pattern(uint16_t welding_point, uint8_t store_pattern){
	EEPROM_ByteRead(&eeprom1, welding_point, PATTERN_BYTE_ADDR, read_saved_pattern, sizeof(store_pattern));
	welding_pattern = read_saved_pattern[0]; 
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- READ ALL WELDING POINT DATA IN BYTE ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void read_all_weldingdata(uint16_t welding_point){
	EEPROM_PageRead(&eeprom1, welding_point, 0x00, page_data_byte, sizeof(page_data_byte));
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- CHECK EEPROM FREE WLEDING POINT ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint16_t check_weldingpoint(void){
	return 0;
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- STEPPER DISABLE --- */
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void disable_stepper(void){
	HAL_GPIO_WritePin(ENABLE1_GPIO_Port, ENABLE1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ENABLE2_GPIO_Port, ENABLE2_Pin, GPIO_PIN_SET);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- STEPPER ENABLE --- */
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void enable_stepper(void){
	HAL_GPIO_WritePin(ENABLE1_GPIO_Port, ENABLE1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ENABLE2_GPIO_Port, ENABLE2_Pin, GPIO_PIN_RESET);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- MOVE STEPPER FUNCTION ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void move_stepper(uint8_t select_joint, uint16_t step, Stepper_Dir_t dir, uint16_t input_freq){		
	for(int i=0; i<JOINT_NUM; i++){
		if(select_joint == robot_joint[i]){
			// Duty Cycle Calculation
			step_freq[i] = input_freq;
			step_period[i] = 1000000/step_freq[i];
			
			// Set Stepper Direction
			if(dir == DIR_CW && step_state[i] == STOPPING){
				HAL_GPIO_WritePin(stepper_gpio_port[i+6], stepper_gpio_pin[i+6], GPIO_PIN_RESET);
			}
			else if(dir == DIR_CCW && step_state[i] == STOPPING){
				HAL_GPIO_WritePin(stepper_gpio_port[i+6], stepper_gpio_pin[i+6], GPIO_PIN_SET);
			}
			
			// Pulse Start
			step_input[i] = step;
			if((step_input[i] > 0 && step_counter[i] == 0 && step_state[i] == STOPPING) || (step_start[i] == true && step_state[i] == STOPPING) || joint_limit == false){
				if(step_input[i] != 0) step_state[i] = ACCELERATING;
				else{
					step_state[i] = AT_TOP_SPEED;
					t_on[i] = step_period[i] * SET_DUTY_CYCLE;
					t_off[i] = step_period[i] - t_on[i];
				}
				HAL_TIM_Base_Start_IT(stepper_tim_handler[i]);
			}
		}
	}
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- EXPONENTIAL SPEED RAMP CALCULATION ---*/
uint32_t calculate_exponential_step_interval(uint8_t select_joint, uint32_t step){
	if (step == 0) return 100000 / BASE_FREQ;  // Initial delay

	float freq = BASE_FREQ + (step_freq[select_joint] - BASE_FREQ) * (1 - expf(-0.001 * step));
	return (uint32_t)(100000.0f / freq);  // Convert frequency to step delay
}


/* --- JOINT BASE MOVE FUNCTION ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void move_joint(uint8_t select_joint, double input_angle, Speed_t set_speed){
	// Joint RPM Set
	if(set_speed == LOW) joint_rpm = 1;
	else if(set_speed == MED) joint_rpm = 2;
	else if(set_speed == HIGH) joint_rpm = 4;
	
	if((input_angle > 0 && input_angle < 5) || (input_angle > -5 && input_angle < 0)){
		joint_rpm = 0.5;
	}
	
	for(int i=0; i<JOINT_NUM; i++){
		if(select_joint == robot_joint[i]){
			// Calculate Step
			if(step_limit[i] == true){
				step_output[i] = abs((int)(input_angle / joint_deg_per_step[i]));
			}
			
			// Synchronize Joint RPM
			stepper_rpm[i] = joint_rpm * stepper_ratio[i];
			stepper_freq[i] = stepper_rpm[i] * stepper_microstep[i] / 60;
			
			// Step Move
			if(input_angle > 0){
				move_stepper(robot_joint[i], step_output[i], DIR_CW, stepper_freq[i]);
			}
			else if(input_angle < 0){
				move_stepper(robot_joint[i], step_output[i], DIR_CCW, stepper_freq[i]);
			}
			
			// Continuous Move
			if(step_start[i] == true){
				if(command.move_sign == UNSIGNED_VAR) move_stepper(robot_joint[i], 0, DIR_CW, stepper_freq[i]);
				else move_stepper(robot_joint[i], 0, DIR_CCW, stepper_freq[i]);
			}
		}
	}
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- WORLD BASE MOVE FUNCTION ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void move_world(uint8_t move_var, double move_pos, Speed_t set_speed){
	// Joint RPM Set
	if(set_speed == LOW) joint_rpm = 2;
	else if(set_speed == MED) joint_rpm = 3;
	else if(set_speed == HIGH) joint_rpm = 4;
	
	run_inverse_kinematic(&kinematics, rx_move_position[0], rx_move_position[1], rx_move_position[2], 
	kinematics.axis_rot_out[0], kinematics.axis_rot_out[1], kinematics.axis_rot_out[2]);	
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- WELDER TURN ON FUNCTION ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void welder_on(void){
	HAL_GPIO_WritePin(MIG_SW_IN_GPIO_Port, MIG_SW_IN_Pin, GPIO_PIN_SET);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- WELDER TURN OFF FUNCTION ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void welder_off(void){
	HAL_GPIO_WritePin(MIG_SW_IN_GPIO_Port, MIG_SW_IN_Pin, GPIO_PIN_RESET);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- WELDING POINT PREVIEW FUNCTION ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void welding_preview(void){

}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- WELDING START FUNCTION ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void welding_start(void){

}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- UPDATE JOINT ANGLE AND POSITION VALUE FUNCTION ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void update_value(void){
	run_forward_kinematic(&kinematics, enc_joint_angle);
	
	for(int i=0; i<JOINT_NUM; i++){
		if(i<3){
			current_position[i] = kinematics.axis_pos_out[i];
			current_rotation[i] = kinematics.axis_rot_out[i];
		}
		current_angle[i] = enc_joint_angle[i];
	}
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- LIMIT SWITCH STATE CHECK FUNCTION ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void check_limit(Limit_t check_mode){
	if(check_mode == ALL_LIMIT_CHECK){
		
	}
	
	else if(check_mode == SOFT_LIMIT_CHECK){
		
	}
	
	else if(check_mode == HARD_LIMIT_CHECK){
		for(int i=0; i<JOINT_NUM; i++){
			if(enc_joint_angle[i] > joint_negative_axisLim[i] && enc_joint_angle[i] < joint_positive_axisLim[i]){
				hard_limit[i] = false;
			}
			else{
				if(joint_limit_bypass == false){
					hard_limit[i] = true;
					for(int i=0; i<10; i++){
						Send_feedback(&command, ANGLE_HARD_LIMIT);
					}
					if(enc_joint_angle[i] <= joint_negative_axisLim[i]) negative_limit[i] = true;
					else if(enc_joint_angle[i] >= joint_positive_axisLim[i]) positive_limit[i] = true;
				}
			}
			
			if(negative_limit[i] == true && enc_joint_angle[i] > joint_negative_axisLim[i]){
				negative_limit[i] = false;
				joint_limit_bypass = false;
			}
			else if(positive_limit[i] == true && enc_joint_angle[i] < joint_positive_axisLim[i]){
				positive_limit[i] = false;
				joint_limit_bypass = false;
			}
			
			joint_limit |= hard_limit[i];
		}
	}
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- JOINT ANGLE HOMING FUNCTION ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool homing(Homing_Mode_t mode){
	bool state;
	joint_rpm = 4;	
	
	// Calculate each joint delta angle
	
	while(1){
		if(mode == ENCODER_LIMIT){
			for(int i=0; i<JOINT_NUM; i++){
				if(enc_joint_angle[i] >= -0.01 || enc_joint_angle[i] <= 0.01){
					state = true;
					break;
				}
			}
		}
		
		else if(mode == SWITCH_LIMIT){
			if(limit_switch_state[0] & limit_switch_state[1] & limit_switch_state[2] & limit_switch_state[3] & limit_switch_state[4] & limit_switch_state[5]){
				state = true;
				break;
			}
		}
	}
	
	return state;
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- ENCODER INITIALIZE FUNCTION ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void encoder_init(void){
	for(int i=0; i<JOINT_NUM; i++){
		HAL_TIM_IC_Start_IT(enc_tim_handler[i], TIM_CHANNEL_1);
		HAL_TIM_IC_Start_IT(enc_tim_handler[i], TIM_CHANNEL_2);
		
		if(step_pos_dir[i] == DIR_CW) HAL_GPIO_WritePin(enc_gpio_port[i+6], enc_gpio_pin[i+6], GPIO_PIN_SET);
		else HAL_GPIO_WritePin(enc_gpio_port[i+6], enc_gpio_pin[i+6], GPIO_PIN_RESET);
	}
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- ENCODER COUNTER RESET FUNCTION ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void encoder_reset(void){
	for(int i=0; i<JOINT_NUM; i++){
		if(period_time[i] != 0){
			duty_cycle[i] = high_time[i] * 100 / period_time[i];
			pwm_freq[i] = (1000000/period_time[i]);
			enc_angle[i] = (((duty_cycle[i] - min_duty_cycle) * 360) / (max_duty_cycle - min_duty_cycle));
			
			cal_value[i] = enc_angle[i];
			enc_joint_angle[i] = 0.0;
		}
	}
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


// --- SEND MAPPED WELDING DATA FUNCTION --- */
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void send_mapped_data(void){
	if(command.requested_data == MAPPED_START_POINT){
		read_weldingpos_world(command.welding_point_num, SP_POS_BYTE_ADDR, array_pos_start, sizeof(array_ang_start));
		HAL_Delay(1);
		read_weldingpos_angle(command.welding_point_num, SP_ANG_BYTE_ADDR, array_ang_start, sizeof(array_ang_start));
		HAL_Delay(1);
		
		tx_pos[0] = array_pos_start[0];
		tx_pos[1] = array_pos_start[1];
		tx_pos[2] = array_pos_start[2];
		
		tx_angle[0] = array_ang_start[0];
		tx_angle[1] = array_ang_start[1];
		tx_angle[2] = array_ang_start[2];
		tx_angle[3] = array_ang_start[3];
		tx_angle[4] = array_ang_start[4];
		tx_angle[5] = array_ang_start[5];	
	}
	else if(command.requested_data == MAPPED_END_POINT){
		read_weldingpos_world(command.welding_point_num, EP_POS_BYTE_ADDR, array_pos_end, sizeof(array_pos_end));
		HAL_Delay(1);
		read_weldingpos_angle(command.welding_point_num, EP_ANG_BYTE_ADDR, array_ang_end, sizeof(array_ang_end));
		HAL_Delay(1);
		
		tx_pos[0] = array_pos_end[0];
		tx_pos[1] = array_pos_end[1];
		tx_pos[2] = array_pos_end[2];
		
		tx_angle[0] = array_ang_end[0];
		tx_angle[1] = array_ang_end[1];
		tx_angle[2] = array_ang_end[2];
		tx_angle[3] = array_ang_end[3];
		tx_angle[4] = array_ang_end[4];
		tx_angle[5] = array_ang_end[5];	
	}
	
	if(HAL_GetTick() - prev_time_send > send_data_interval){
		Send_requested_data(&command, tx_pos, tx_angle, current_point, LINEAR, LOW);
		prev_time_send = HAL_GetTick();
	}
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- CUSTON RS232 TRANSMIT FUNCTION ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void send_ang_pos_data(void){
//	tx_pos[0] = (float)kinematics.axis_pos_out[0];
//	tx_pos[1] = (float)kinematics.axis_pos_out[1];
//	tx_pos[2] = (float)kinematics.axis_pos_out[2];
//	
	tx_angle[0] = (float)enc_joint_angle[0];
//	tx_angle[1] = (float)enc_joint_angle[1];
//	tx_angle[2] = (float)enc_joint_angle[2];
//	tx_angle[3] = (float)enc_joint_angle[3];
//	tx_angle[4] = (float)enc_joint_angle[4];
//	tx_angle[5] = (float)enc_joint_angle[5];	
	
	tx_pos[0] = 12.3;
	tx_pos[1] = 12.3;
	tx_pos[2] = 12.3;
	
//	tx_angle[0] = 90.5;
	tx_angle[1] = 90.5;
	tx_angle[2] = 90.5;
	tx_angle[3] = 90.5;
	tx_angle[4] = 90.5;
	tx_angle[5] = 90.5;
	
	if(HAL_GetTick() - prev_time_send > send_data_interval){
		for(int i=0; i<25; i++){
			Send_requested_data(&command, tx_pos, tx_angle, current_point, LINEAR, LOW);
		}
		prev_time_send = HAL_GetTick();
	}
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- CUSTON RS232 RECEIVE FUNCTION ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void receive_data(void){
	Start_get_command(&command);
	
	if(command.msg_get == true){
		Get_command(&command);
		command.msg_get = false;
	}
	
	if(HAL_GetTick() - prev_time_get > 300 && command.msg_get == false){
		command.type = NONE;
		prev_time_get = HAL_GetTick();
	}
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- SHOW OLED MENU FUNCTION ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void show_menu(Oled_Menu_t sel_menu){
	if(sel_menu == BOOT_MENU){
		SSD1306_GotoXY(0,0);
		SSD1306_Puts("BOOTING...", &Font_7x10, SSD1306_COLOR_WHITE);
		
		SSD1306_UpdateScreen();
	}
	
	else if(sel_menu == COM_INIT_MENU){
		SSD1306_GotoXY(0,0);
		SSD1306_Puts("RS232 Init...", &Font_7x10, SSD1306_COLOR_WHITE);
		
		SSD1306_UpdateScreen();
	}
	
	else if(sel_menu == MAIN_MENU){
		SSD1306_GotoXY(0,0);
		SSD1306_Puts(title, &Font_7x10, SSD1306_COLOR_WHITE);
		
		SSD1306_GotoXY(0,25);
		SSD1306_Puts("RS232 :", &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY(50,25);
		if(RS232_state == 0x00) SSD1306_Puts("Reset       ", &Font_7x10, SSD1306_COLOR_WHITE);
		else if(RS232_state == HAL_UART_STATE_READY) SSD1306_Puts("Ready       ", &Font_7x10, SSD1306_COLOR_WHITE);
		else if(RS232_state == HAL_UART_STATE_BUSY) SSD1306_Puts("Busy        ", &Font_7x10, SSD1306_COLOR_WHITE);
		else if(RS232_state == HAL_UART_STATE_BUSY_TX) SSD1306_Puts("Busy tx     ", &Font_7x10, SSD1306_COLOR_WHITE);
		else if(RS232_state == HAL_UART_STATE_BUSY_RX) SSD1306_Puts("Busy rx     ", &Font_7x10, SSD1306_COLOR_WHITE);
		else if(RS232_state == HAL_UART_STATE_BUSY_TX_RX) SSD1306_Puts("Busy all    ", &Font_7x10, SSD1306_COLOR_WHITE);
		else if(RS232_state == HAL_UART_STATE_TIMEOUT) SSD1306_Puts("Timeout     ", &Font_7x10, SSD1306_COLOR_WHITE);
		else if(RS232_state == HAL_UART_STATE_ERROR) SSD1306_Puts("Error       ", &Font_7x10, SSD1306_COLOR_WHITE);
		
		SSD1306_GotoXY(0,37);
		SSD1306_Puts("GETCOM:", &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY(50,37);
		if(command.type == AUTO_HOME) SSD1306_Puts("Homing      ", &Font_7x10, SSD1306_COLOR_WHITE);
		else if(command.type == MAPPING) SSD1306_Puts("Mapping     ", &Font_7x10, SSD1306_COLOR_WHITE);
		else if(command.type == PREVIEW) SSD1306_Puts("Preview     ", &Font_7x10, SSD1306_COLOR_WHITE);
		else if(command.type == MOVE) SSD1306_Puts("Move        ", &Font_7x10, SSD1306_COLOR_WHITE);
		else if(command.type == RUN) SSD1306_Puts("Run         ", &Font_7x10, SSD1306_COLOR_WHITE);
		else if(command.type == REQ_DATA) SSD1306_Puts("Req Data    ", &Font_7x10, SSD1306_COLOR_WHITE);
		else if(command.type == SEND_REQ) SSD1306_Puts("Send Req    ", &Font_7x10, SSD1306_COLOR_WHITE);
		else if(command.type == MOTOR_STATE) SSD1306_Puts("Step State  ", &Font_7x10, SSD1306_COLOR_WHITE);
		else if(command.type == WELDER_STATE) SSD1306_Puts("Weld State  ", &Font_7x10, SSD1306_COLOR_WHITE);
		else if(command.type == FEEDBACK) SSD1306_Puts("Feedback    ", &Font_7x10, SSD1306_COLOR_WHITE);
		else if(command.type == RESET_STATE) SSD1306_Puts("Reset State ", &Font_7x10, SSD1306_COLOR_WHITE);
		else SSD1306_Puts("None        ", &Font_7x10, SSD1306_COLOR_WHITE);
	
		SSD1306_GotoXY(0,49);
		SSD1306_Puts("POWER :", &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_GotoXY(50,49);
		pwr_status = 1;
		if(pwr_status == UNDER_VOLTAGE) SSD1306_Puts("Under       ", &Font_7x10, SSD1306_COLOR_WHITE);
		else if(pwr_status == NORM_VOLTAGE) SSD1306_Puts("Normal      ", &Font_7x10, SSD1306_COLOR_WHITE);
		
		SSD1306_UpdateScreen();
	}
	
	else if(sel_menu == EEPROM1_ERROR_MENU){
		SSD1306_GotoXY(0,0);
		SSD1306_Puts(error1, &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();
	}
	
	else if(sel_menu == EEPROM2_ERROR_MENU){
		SSD1306_GotoXY(0,0);
		SSD1306_Puts(error2, &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();
	}
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartEncoderRead */
/**
  * @brief  Function implementing the encoderRead thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartEncoderRead */
void StartEncoderRead(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		if(osSemaphoreAcquire(encoderRWHandle, 0) == osOK){
			for(int i=0; i<JOINT_NUM; i++){
				if(period_time[i] != 0){
					duty_cycle[i] = high_time[i] * 100 / period_time[i];
					pwm_freq[i] = (1000000/period_time[i]);
					enc_angle[i] = (((duty_cycle[i] - min_duty_cycle) * 360) / (max_duty_cycle - min_duty_cycle));
					cal_enc_angle[i] = fmod((enc_angle[i] - cal_value[i] + 360), 360);
					
					reduced_cal_enc_angle[i] = cal_enc_angle[i] / stepper_ratio[i];
					
					if(prev_cal_enc_angle[i] > 350 && cal_enc_angle[i] < 10) enc_counter[i]++;
					else if(prev_cal_enc_angle[i] < 10 && cal_enc_angle[i] > 350) enc_counter[i]--;
					
					enc_joint_angle[i] = reduced_cal_enc_angle[i] + (enc_counter[i] * (360 / stepper_ratio[i]));
					
					prev_cal_enc_angle[i] = cal_enc_angle[i];
				}
			}
		}
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartLimitCheck */
/**
* @brief Function implementing the limitCheck thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLimitCheck */
void StartLimitCheck(void *argument)
{
  /* USER CODE BEGIN StartLimitCheck */
  /* Infinite loop */
  for(;;)
  {
		if(command.control_mode == CARTESIAN_CTRL) check_limit(ALL_LIMIT_CHECK);
		else if(command.control_mode == JOINT_CTRL) check_limit(HARD_LIMIT_CHECK);
  }
  /* USER CODE END StartLimitCheck */
}

/* USER CODE BEGIN Header_StartStepperControl */
/**
* @brief Function implementing the stepperControl thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartStepperControl */
void StartStepperControl(void *argument)
{
  /* USER CODE BEGIN StartStepperControl */
  /* Infinite loop */
  for(;;)
  {
		if(osSemaphoreAcquire(stepperRWHandle, 0) == osOK){
			for(int i=0; i<JOINT_NUM; i++){
				if(step_state[i] == ACCELERATING){
					if(step_counter[i] < step_input[i]*0.25) step_accel_period[i] = calculate_exponential_step_interval(i, step_counter[i]/2);
					else step_accel_period[i] = calculate_exponential_step_interval(i, step_input[i]-step_counter[i]/2);
					
					t_on[i] = step_accel_period[i] * SET_DUTY_CYCLE;
					t_off[i] = step_accel_period[i] - t_on[i];
				}
				
				if(timer_counter[i] < t_on[i]) HAL_GPIO_WritePin(stepper_gpio_port[i], stepper_gpio_pin[i], GPIO_PIN_SET);
				else if(timer_counter[i] >= t_on[i] && timer_counter[i] < step_period[i]-1) HAL_GPIO_WritePin(stepper_gpio_port[i], stepper_gpio_pin[i], GPIO_PIN_RESET);
				else if(timer_counter[i] == step_period[i]-1){
					timer_counter[i] = 0;
					step_counter[i]++;
				}
				
				if(((step_counter[i] >= step_input[i]) && step_limit[i] == true) || (step_limit[i] == false && step_start[i] == false) || joint_limit == true){
					HAL_GPIO_WritePin(stepper_gpio_port[i], stepper_gpio_pin[i], GPIO_PIN_RESET);
					HAL_TIM_Base_Stop_IT(stepper_tim_handler[i]);
					timer_counter[i] = 0;
					step_counter[i] = 0;
					step_state[i] = STOPPING;
				}
			}
		}
  }
  /* USER CODE END StartStepperControl */
}

/* USER CODE BEGIN Header_StartCommandHandler */
/**
* @brief Function implementing the commandHandler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartCommandHandler */
void StartCommandHandler(void *argument)
{
  /* USER CODE BEGIN StartCommandHandler */
  /* Infinite loop */
  for(;;)
  {
		// Receive RS-232 Data
		receive_data();
		
		// Check Auto Home Command
		if(command.type == AUTO_HOME){
			if(homing(ENCODER_LIMIT) == true){
				home = true;
				Send_feedback(&command, AUTO_HOME_DONE);
			}
		}
		
		// Check Mapping Command
		else if(command.type == MAPPING){
			if(command.mapping_state == SAVE_VALUE){
				if(command.welding_point_type == START_POINT){
					save_weldingpos_world(command.welding_point_num, SP_POS_BYTE_ADDR, (float*)current_position, sizeof(current_position));
					save_weldingpos_angle(command.welding_point_num, SP_ANG_BYTE_ADDR, (float*)current_angle, sizeof(current_angle));
				}
				else if(command.welding_point_type == END_POINT){
					save_weldingpos_world(command.welding_point_num, EP_POS_BYTE_ADDR, (float*)current_position, sizeof(current_position));
					save_weldingpos_angle(command.welding_point_num, EP_ANG_BYTE_ADDR, (float*)current_angle, sizeof(current_angle));
					save_weldingpos_pattern(command.welding_point_num, command.pattern_type);
				}
			}
			else if(command.mapping_state == DELETE_VALUE){
				if(command.welding_point_type == START_POINT){
					uint8_t sp_reset[36];
					memset(sp_reset, 0x00, 36);
					EEPROM_PageWrite(&eeprom1, command.welding_point_num, SP_POS_BYTE_ADDR, sp_reset, sizeof(sp_reset));
				}
				else if(command.welding_point_type == END_POINT){
					uint8_t ep_reset[37];
					memset(ep_reset, 0x00, 37);
					EEPROM_PageWrite(&eeprom1, command.welding_point_num, EP_POS_BYTE_ADDR, ep_reset, sizeof(ep_reset));
				}
			}
		}
		
		// Check Preview Command
		else if(command.type == PREVIEW){
			osEventFlagsSet(runningFlagHandle, PREVIEW_FLAG);
		}
		
		// Check Move Command
		else if(command.type == MOVE){
			if(command.control_mode == CARTESIAN_CTRL){			
				for(int i=0; i<AXIS_NUM; i++){
					if(command.move_variable == robot_axis[i]){
						if(command.move_mode == CONTINUOUS){
							move_world(command.move_variable, rx_move_position[i], MED);
						}
						else if(command.move_mode == DISTANCE){
							step_limit[i] = true;
							step_start[i] = false;
							rx_move_position[i] = command.move_value;
							move_world(command.move_variable, rx_move_position[i], MED);
						}
						else if(command.move_mode == STEP){
							
						}
					}
					else rx_move_position[i] = 0.0;
				}
			}
			
			else if(command.control_mode == JOINT_CTRL){	
				for(int i=0; i<JOINT_NUM; i++){
					if(command.move_variable == robot_joint[i]){
						if(command.move_mode == CONTINUOUS){
							step_limit[i] = false;
							step_start[i] = true;
							rx_move_angle[i] = 0.0;
						}
						else if(command.move_mode == DISTANCE || command.move_mode == STEP){
							step_limit[i] = true;
							step_start[i] = false;
							rx_move_angle[i] = command.move_value;
						}
						
						if(positive_limit[i] == true && command.move_sign == SIGNED_VAR){
							move_joint(robot_joint[i], rx_move_angle[i], LOW);
						}
						
						else if(negative_limit[i] == true && command.move_sign == UNSIGNED_VAR){
							move_joint(robot_joint[i], rx_move_angle[i], LOW);
						}
						
						else if(positive_limit[i] == false && negative_limit[i] == false){
							move_joint(robot_joint[i], rx_move_angle[i], LOW);
						}
					}
					else rx_move_angle[i] = 0.0;
				}
			}
		}
		
		// Check Run Command
		else if(command.type == RUN){
			osEventFlagsSet(runningFlagHandle, WELDING_FLAG);
		}
		
		// Check Motor State Command
		else if(command.type == MOTOR_STATE){
			if(command.motor_state == MOTOR_OFF) disable_stepper();
			else if(command.motor_state == MOTOR_ON) enable_stepper();
		}
		
		// Check Welder State Command
		else if(command.type == WELDER_STATE){
			if(command.welder_state == WELDER_OFF) welder_off();
			else if(command.welder_state == WELDER_ON) welder_on();
		}
		
		// Check Reset State Command
		else if(command.type == RESET_STATE){
			for(int i=0; i<JOINT_NUM; i++){
				soft_limit[i] = false;
				hard_limit[i] = false;
			}
			joint_limit = false;
			joint_limit_bypass = true;
			kinematics.singularity = false;
		}
		
		// Check None Command
		else if(command.type == NONE){
			for(int i=0; i<JOINT_NUM; i++){
				step_start[i] = false;
				rx_move_position[i] = 0.0;
				rx_move_angle[i] = 0.0;
			}
		}
		
		// Check Request Command
		if(command.type == REQ_DATA){
			send_mapped_data();
		}
		else{
			send_ang_pos_data();
		}
  }
  /* USER CODE END StartCommandHandler */
}

/* USER CODE BEGIN Header_StartRunningHandler */
/**
* @brief Function implementing the runningHandler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartRunningHandler */
void StartRunningHandler(void *argument)
{
  /* USER CODE BEGIN StartRunningHandler */
  /* Infinite loop */
  for(;;)
  {
		uint8_t flags;
		flags = osEventFlagsWait(runningFlagHandle, WELDING_FLAG | PREVIEW_FLAG, osFlagsWaitAny, 0);
		
		// Welding Algorithm
		if(flags & WELDING_FLAG){
		}
		
		// Preview Algorithm
		if(flags & PREVIEW_FLAG){
		}
		
    osDelay(1);
  }
  /* USER CODE END StartRunningHandler */
}

/* USER CODE BEGIN Header_StartPosUpdate */
/**
* @brief Function implementing the posUpdate thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartPosUpdate */
void StartPosUpdate(void *argument)
{
  /* USER CODE BEGIN StartPosUpdate */
  /* Infinite loop */
  for(;;)
  {
		update_value();
		send_ang_pos_data();
    osDelay(1);
  }
  /* USER CODE END StartPosUpdate */
}

/* USER CODE BEGIN Header_StartLCDMenu */
/**
* @brief Function implementing the lcdMenu thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLCDMenu */
void StartLCDMenu(void *argument)
{
  /* USER CODE BEGIN StartLCDMenu */
  /* Infinite loop */
  for(;;)
  {
		show_menu(MAIN_MENU);
    osDelay(1);
  }
  /* USER CODE END StartLCDMenu */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
	for(int i=0; i<JOINT_NUM; i++){
		if(htim->Instance == stepper_tim_instance[i]){
			timer_counter[i]++;
		}
	}
	
	osSemaphoreRelease(stepperRWHandle);
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
