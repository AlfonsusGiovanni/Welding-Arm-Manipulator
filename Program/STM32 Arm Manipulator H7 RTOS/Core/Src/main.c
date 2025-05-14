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
#include "fatfs.h"

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


/* SDCARD TYPEDEF */
// -----------------
FATFS fs;
FIL file;
FRESULT res;
// -----------------


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
	SDCARD_ERROR_MENU,
	SAVE_MENU,
	DELETE_MENU,
}Oled_Menu_t;
// --------------------


/* STEPPER DIR TYPEDEF */
// ----------------------
typedef enum{
	DIR_CCW,
	DIR_CW,
}Stepper_Dir_t;
// ----------------------


/* JOINT DIR TYPEDEF */
// --------------------
typedef enum{
	NEGATIVE_DIR,
	POSITIVE_DIR,
}Joint_Dir_t;
// --------------------


/*  LIMIT TYPE TYPEDEF */
// ----------------------
typedef enum{
	ALL_LIMIT_CHECK,
	SOFT_LIMIT_CHECK,
	HARD_LIMIT_CHECK,
}Limit_t;
// ----------------------


/* WELDING DATA STATUS TYPEDEF */
// ----------------------------
typedef enum{
	NO_WELDING_DATA,
	WELDING_DATA_INVALID,
	WELDING_DATA_VALID,
}Welding_Data_Status_t;
// ----------------------------


/* EEPROM SELECT TYPEDEF */
// ------------------------
typedef enum{
	EEPROM_ID1 = 0x01,
	EEPROM_ID2,
}EEPROM_ID_t;
// ------------------------


/* SDCARD STATUS TYPEDEF */
// ------------------------
typedef enum{
	SD_OK, 
	SD_MOUNT_ERR,
	SD_CREATE_ERR,
	SD_OPEN_ERR,
	SD_READ_ERR,
	SD_WRITE_ERR,
}SD_Status_t;
// ------------------------


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
//#define USE_SDCARD
#define USE_EEPROM
#define USE_RS232
#define USE_OLED
#define USE_STEPPER
#define USE_ENCODER
//#define USE_KINEMATICS
//#define USE_PID

//#define ENCODER_TASK_ON
//#define LIMIT_CHECK_TASK_ON
//#define STEPPER_CTRL_TASK_ON
//#define CMD_HANDLER_TASK_ON
//#define RUNNING_HANDLER_TASK_ON
//#define POS_UPDATE_TASK_ON
//#define LCD_MENU_TASK_ON

//#define USE_DEBUG_PROGRAM
//#define USE_MISS_STEP_CHECK
//#define USE_LIMIT_CHECK

//#define MAIN_PROGRAM
//----------------------

/* EEPROM ADDRESS SET */
//-----------------------------------
#define EEPROM1_ADDRESS					0xA0	// Primary EEPROM Address
#define EEPROM2_ADDRESS					0xA2	// Secondary EEPROM Address

#define START_POINT_ADDR				0x01	// Start Point EEPROM select
#define END_POINT_ADDR					0x02	// End Point EEPROM select

#define POS_BYTES_ADDR					0x00	// World Position Value Address
#define ANG_BYTES_ADDR					0x30	// Joint Angle Value Address

#define PATTERN_BYTE_ADDR				0x60	// Welding Pattern Address (1 Byte)
#define SPEED_BYTE_ADDR					0x61	// Welding Speed Address (1 Byte)

#define CHECKSUM_H_ADDR					0x62	// Checksum H Byte Address
#define CHECKSUM_L_ADDR					0x63	// Checksum L Byte Address
//-----------------------------------

/* EEPROM PATTERN SET */
//-----------------------------------
#define LINEAR_PATTERN					0x01
#define CIRCULAR_PATTERN				0x02
#define ZIGZAG_PATTERN					0x03
//-----------------------------------

/* STEPPER SET */
//---------------------------------
#define MICROSTEP_FACTOR1			1
#define MICROSTEP_FACTOR2			2
#define MICROSTEP_FACTOR3			4
#define MICROSTEP_FACTOR4			8
#define MICROSTEP_FACTOR5			16
#define MICROSTEP_FACTOR6			32

#define MICROSTEP_VALUE1 			200
#define MICROSTEP_VALUE2 			400
#define MICROSTEP_VALUE3 			800
#define MICROSTEP_VALUE4 			1600
#define MICROSTEP_VALUE5			3200
#define MICROSTEP_VALUE6 			6400  

#define STEPPER1_RATIO				162
#define	STEPPER2_RATIO				144
#define	STEPPER3_RATIO				36
#define	STEPPER4_RATIO				20
#define	STEPPER5_RATIO				90
#define	STEPPER6_RATIO				9

#define STEPPER1_MAXSPD				420
#define STEPPER2_MAXSPD				315
#define STEPPER3_MAXSPD				420
#define STEPPER4_MAXSPD				420
#define STEPPER5_MAXSPD				420
#define STEPPER6_MAXSPD				420

#define SET_DUTY_CYCLE				0.25f
#define BASE_FREQ							1000

#define STOPPING							0x01
#define ACCELERATING					0X02
#define AT_TOP_SPEED					0X03
#define DECELERATING					0X04

#define MISS_STEP_TOLLERANCE	1
#define MISS_STEP_SAFE_ANGLE	10.0f
//---------------------------------


/*ROBOT SET*/
//-------------------------------
#define MAX_POINTS				360
#define JOINT_NUM					6
#define AXIS_NUM					6

#define JOINT1_MAX_ANGLE	170.0
#define JOINT2_MAX_ANGLE	90.0
#define JOINT3_MAX_ANGLE	60.0
#define JOINT4_MAX_ANGLE	145.0
#define JOINT5_MAX_ANGLE	105.0
#define JOINT6_MAX_ANGLE	155.0

#define JOINT1_MIN_ANGLE	-170.0
#define JOINT2_MIN_ANGLE	-60.0
#define JOINT3_MIN_ANGLE	-90.0
#define JOINT4_MIN_ANGLE	-145.0
#define JOINT5_MIN_ANGLE	-90.0
#define JOINT6_MIN_ANGLE	-155.0

#define JOINT1_CAL_ANGLE 	-90.0
#define JOINT2_CAL_ANGLE 	-60.0
#define JOINT3_CAL_ANGLE 	60.0
#define JOINT4_CAL_ANGLE 	0.0
#define JOINT5_CAL_ANGLE 	-90.0
#define JOINT6_CAL_ANGLE 	0.0
//-------------------------------


/*INTERPOLATION SET*/
//--------------------------
#define MAX_UPDATE_STEP		50


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

SD_HandleTypeDef hsd1;

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
  .priority = (osPriority_t) osPriorityLow1,
};
/* Definitions for setupTask */
osThreadId_t setupTaskHandle;
uint32_t setupTaskBuffer[ 128 ];
osStaticThreadDef_t setupTaskControlBlock;
const osThreadAttr_t setupTask_attributes = {
  .name = "setupTask",
  .cb_mem = &setupTaskControlBlock,
  .cb_size = sizeof(setupTaskControlBlock),
  .stack_mem = &setupTaskBuffer[0],
  .stack_size = sizeof(setupTaskBuffer),
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
/* Definitions for setupFlag */
osEventFlagsId_t setupFlagHandle;
osStaticEventGroupDef_t setupFlagControlBlock;
const osEventFlagsAttr_t setupFlag_attributes = {
  .name = "setupFlag",
  .cb_mem = &setupFlagControlBlock,
  .cb_size = sizeof(setupFlagControlBlock),
};
/* Definitions for msgGet */
osEventFlagsId_t msgGetHandle;
osStaticEventGroupDef_t msgGetControlBlock;
const osEventFlagsAttr_t msgGet_attributes = {
  .name = "msgGet",
  .cb_mem = &msgGetControlBlock,
  .cb_size = sizeof(msgGetControlBlock),
};
/* Definitions for msgSent */
osEventFlagsId_t msgSentHandle;
osStaticEventGroupDef_t msgSentControlBlock;
const osEventFlagsAttr_t msgSent_attributes = {
  .name = "msgSent",
  .cb_mem = &msgSentControlBlock,
  .cb_size = sizeof(msgSentControlBlock),
};
/* USER CODE BEGIN PV */


// PARAMETER VARIABLE
const double
joint_positive_axisLim[JOINT_NUM] = {
	JOINT1_MAX_ANGLE,
	JOINT2_MAX_ANGLE,
	JOINT3_MAX_ANGLE,
	JOINT4_MAX_ANGLE,
	JOINT5_MAX_ANGLE,
	JOINT6_MAX_ANGLE,
	
},
joint_negative_axisLim[JOINT_NUM] = {
	JOINT1_MIN_ANGLE,
	JOINT2_MIN_ANGLE,
	JOINT3_MIN_ANGLE,
	JOINT4_MIN_ANGLE,
	JOINT5_MIN_ANGLE,
	JOINT6_MIN_ANGLE,
	
},
joint_cal_angle[JOINT_NUM] = {
	JOINT1_CAL_ANGLE,
	JOINT2_CAL_ANGLE,
	JOINT3_CAL_ANGLE,
	JOINT4_CAL_ANGLE,
	JOINT5_CAL_ANGLE,
	JOINT6_CAL_ANGLE,
};

double
joint_max_traveldist[JOINT_NUM],
joint_ang_per_step[JOINT_NUM],
joint_miss_step_angle[JOINT_NUM];

bool
clear_to_update = false,
home = false,
robot_stop = false,
welding_run = false,
preview_run = false,
joint_limit = false,
joint_miss_step = false,
joint_error_bypass = false,
calibrated_joint[JOINT_NUM],
soft_limit[JOINT_NUM],
hard_limit[JOINT_NUM],
positive_limit[JOINT_NUM],
negative_limit[JOINT_NUM],
positive_miss[JOINT_NUM],
negative_miss[JOINT_NUM],
miss_step[JOINT_NUM];

const uint8_t
robot_axis[AXIS_NUM] = {
	AXIS_X,
	AXIS_Y,
	AXIS_Z,
	AXIS_RX,
	AXIS_RY,
	AXIS_RZ
},

robot_joint[JOINT_NUM] = {
	JOINT_1,
	JOINT_2,
	JOINT_3,
	JOINT_4,
	JOINT_5,
	JOINT_6
};

const double
joint_d[JOINT_NUM] = {
	191.0, 0.0, 0.0,
	575.0, 0.0, 65.0,
},

joint_a[JOINT_NUM] = {
	23.5, 650.0, 0.0,
	0.0, 0.0, 0.0, 
},

joint_alpha[JOINT_NUM] = {
	-90.0, 0.0, 90.0,
	-90.0, 90.0, 0.0,
};

uint8_t
invalid_welding_point;

uint16_t
total_valid_points,
total_mapped_points,
joint_home_offset[JOINT_NUM];

Welding_Data_Status_t
point_status[MAX_POINTS];

Joint_Dir_t
joint_current_dir[JOINT_NUM],
calibration_dir[JOINT_NUM],
home_dir[JOINT_NUM];


// EEPROM VARIABLE
double 
array_pos_start[AXIS_NUM],
array_pos_end[AXIS_NUM],
array_ang_start[JOINT_NUM],
array_ang_end[JOINT_NUM];

uint8_t
page_data_byte[EEPROM_512Kb_PAGE_SIZE],

saved_pos[48], 
read_saved_pos[48],
saved_angle[48], 
read_saved_angle[48],

#ifdef USE_DEBUG_PROGRAM
read_saved_posX[8],
read_saved_posY[8],
read_saved_posZ[8],
read_saved_rotX[8],
read_saved_rotY[8],
read_saved_rotZ[8],

read_saved_angle1[8],
read_saved_angle2[8],
read_saved_angle3[8],
read_saved_angle4[8],
read_saved_angle5[8],
read_saved_angle6[8],
#endif
welding_point;

uint16_t
chcksum_sp, chcksum_ep,
chcksum_sp_validation,
chcksum_ep_validation;

Speed_t welding_speed;
Welding_Pattern_t welding_pattern;
Welding_Data_Status_t wd_data_status;


// INVERSE KINEMATICS VARIABLE
double
ik_pos_input[AXIS_NUM],
ik_angle_output[JOINT_NUM];


// FORWARD KINEMATICS VARIABLE
double
fk_angle_input[JOINT_NUM],
fk_pos_output[AXIS_NUM];


// MOVE VARIABLE
double
input_position[AXIS_NUM],
current_position[AXIS_NUM],
target_position[AXIS_NUM],
delta_move_pos[AXIS_NUM],
prev_position[AXIS_NUM],

current_angle[JOINT_NUM],
target_angle[JOINT_NUM],
delta_ang[JOINT_NUM],
prev_angle[JOINT_NUM];


// INTERPOLATION VARIABLE
double
axis_delta[AXIS_NUM],
joint_delta[JOINT_NUM],
int_x_out, int_y_out, int_z_out,
int_rx_out, int_ry_out, int_rz_out;

uint8_t
int_counter;


// STEPPER VARIABLE 
TIM_TypeDef*
stepper_tim_instance[JOINT_NUM] = {
	TIM16,
	TIM17,
	TIM12,
	TIM13,
	TIM14,
	TIM15
};

TIM_HandleTypeDef*
stepper_tim_handler[JOINT_NUM] = {
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
step_start[JOINT_NUM],
step_limit[JOINT_NUM],
step_positive_dir[JOINT_NUM];

uint8_t
step_state[JOINT_NUM];

uint16_t
stepper_rpm[JOINT_NUM],
stepper_freq[JOINT_NUM],
t_on[JOINT_NUM],
t_off[JOINT_NUM],
target_step[JOINT_NUM],
step_count[JOINT_NUM],
step_freq[JOINT_NUM],
step_period[JOINT_NUM],
step_accel_period[JOINT_NUM],
stepper_stepfactor[JOINT_NUM];

const uint16_t
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

stepper_microstep[JOINT_NUM] = {
	MICROSTEP_VALUE3,
	MICROSTEP_VALUE3,
	MICROSTEP_VALUE3,
	MICROSTEP_VALUE3,
	MICROSTEP_VALUE3,
	MICROSTEP_VALUE3,
},

stepper_ratio[JOINT_NUM] = {
	STEPPER1_RATIO,
	STEPPER2_RATIO,
	STEPPER3_RATIO,
	STEPPER4_RATIO,
	STEPPER5_RATIO,
	STEPPER6_RATIO,
};

uint32_t
timer_counter[JOINT_NUM],
step_counter[JOINT_NUM],
current_step[JOINT_NUM];

float
angular_rate,
stepper_joint_rpm;


// RS232 VARIABLE
unsigned long
prev_time_send,
prev_time_get;

float
tx_pos[3],
tx_rot[3],
tx_angle[6],
tx_welding_point,
tx_welding_pattern,

rx_move_position[3],
rx_move_rotation[3],
rx_move_angle[6];

const uint8_t
send_data_interval = 50;

uint8_t
prev_command,
rx_savepoint,
rx_patterntype[MAX_POINTS],
rx_pointtype,
rx_rotate_mode,
rx_rotate_value[8];

uint8_t
struct_size;


// OLED LCD VARIABLE
uint8_t 
refresh_counter,
selected_menu;

char 
title[] = "   WELDING ARM V1",
eeprom1_err[] = "EEPROM1 ERROR    ",
eeprom2_err[] = "EEPROM2 ERROR    ",
sdcard_err[] = "SDCARD ERROR     ",
save_data[]		= "SAVING DATA...   ",
delete_data[] = "DELTING DATA...  ";


// ENCODER VARIABLE
TIM_TypeDef*
enc_tim_instance[JOINT_NUM] = {
	TIM4,
	TIM8,
	TIM2,
	TIM1,
	TIM3,
	TIM5
};

TIM_HandleTypeDef*
enc_tim_handler[JOINT_NUM] = {
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

const uint16_t
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
enc_mag_status[JOINT_NUM];

uint32_t
pwm_freq[JOINT_NUM],
high_time[JOINT_NUM],
period_time[JOINT_NUM];

int
enc_counter[JOINT_NUM];

const double
min_duty_cycle = 2.9,
max_duty_cycle = 97.1;

double
duty_cycle[JOINT_NUM],
diff_angle[JOINT_NUM],
enc_angle[JOINT_NUM],
cal_value[JOINT_NUM],
cal_enc_angle[JOINT_NUM],
prev_cal_enc_angle[JOINT_NUM],
reduced_cal_enc_angle[JOINT_NUM],
enc_joint_angle[JOINT_NUM],
prev_enc_joint_angle[JOINT_NUM],

max_joint_angle[JOINT_NUM] = {
	JOINT1_MAX_ANGLE,
	JOINT2_MAX_ANGLE,
	JOINT3_MAX_ANGLE,
	JOINT4_MAX_ANGLE,
	JOINT5_MAX_ANGLE,
	JOINT6_MAX_ANGLE
},
min_joint_angle[JOINT_NUM] = {
	JOINT1_MIN_ANGLE,
	JOINT2_MIN_ANGLE,
	JOINT3_MIN_ANGLE,
	JOINT4_MIN_ANGLE,
	JOINT5_MIN_ANGLE,
	JOINT6_MIN_ANGLE,
};


// LIMIT SWITCH VARIABLE
bool
switch_pressed[JOINT_NUM];

GPIO_TypeDef*
switch_gpio_port[JOINT_NUM] = {
	LIMIT1_GPIO_Port,
	LIMIT2_GPIO_Port,
	LIMIT3_GPIO_Port,
	LIMIT4_GPIO_Port,
	LIMIT5_GPIO_Port,
	LIMIT6_GPIO_Port,
};

const uint16_t switch_gpio_pin[JOINT_NUM] = {
	LIMIT1_Pin,
	LIMIT2_Pin,
	LIMIT3_Pin,
	LIMIT4_Pin,
	LIMIT5_Pin,
	LIMIT6_Pin,
};

uint8_t
calibrate_step = 0,
select_count = 0,
calibrated_count = 0;

// SDCARD VARIABLE
SD_Status_t
sdcard_status;

// POWER VARIABLE
uint8_t pwr_status;


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
static void MX_SDMMC1_SD_Init(void);
void StartEncoderRead(void *argument);
void StartLimitCheck(void *argument);
void StartStepperControl(void *argument);
void StartCommandHandler(void *argument);
void StartRunningHandler(void *argument);
void StartPosUpdate(void *argument);
void StartLCDMenu(void *argument);
void StartSetup(void *argument);

/* USER CODE BEGIN PFP */

// EEPROM Function ------------------------------------------------------------------------------------------
void format_mem(EEPROM_ID_t id);
void save_welding_point(uint16_t welding_point, uint8_t point_type, double *pos_data, double *angle_data);
void read_welding_point(uint16_t welding_point, uint8_t point_type, double *stored_pos, double *stored_angle);
void save_welding_pattern(uint16_t welding_point, uint8_t select_pattern);
void read_welding_pattern(uint16_t welding_point);
void save_welding_speed(uint16_t welding_point, uint8_t select_speed);
void read_welding_speed(uint16_t welding_point);
void delete_welding_data(uint16_t welding_point, uint8_t point_type);
Welding_Data_Status_t weldingpoint_validate(uint16_t welding_point);
//-----------------------------------------------------------------------------------------------------------


// SDCARD Function
//------------------------------------------
SD_Status_t mount_sdcard(void);
SD_Status_t create_txt_file(char *filename);
//------------------------------------------


// Stepper Control Function -------------------------------------------------------------------
void disable_stepper(void);
void enable_stepper(void);
void move_stepper(uint8_t select_joint, uint16_t step, Joint_Dir_t dir, uint16_t input_freq);
//---------------------------------------------------------------------------------------------


// Calculation Function --------------------------------------------------------------------------------------
uint32_t calc_exp_step_interval(uint8_t select_joint, uint32_t step);
uint16_t calc_stepper_freq(uint8_t select_joint, float rpm_input);
uint16_t calc_stepper_step(uint8_t select_joint, double angle);
void calc_linear_parametric(double start_pos[6], double end_pos[6], uint8_t input_step, uint8_t current_step);
//------------------------------------------------------------------------------------------------------------


// Main Function ----------------------------------------------------------------------------------------------------
void move_joint(uint8_t select_joint, double input_angle, Speed_t set_speed);
void move_world(uint8_t select_axis, double move_pos, Speed_t set_speed);
void linear_move(double end_pos[6], Speed_t speed);
void welder_on(void);
void welder_off(void);
void welding_preview(void);
void welding_start(void);
void update_value(void);
void check_limit(Limit_t check_mode);
bool joint_calibrate(Cal_Mode_t mode, bool j1_cal, bool j2_cal, bool j3_cal, bool j4_cal, bool j5_cal, bool j6_cal);
//------------------------------------------------------------------------------------------------------------------


// Encoder Function ----------------------------------------------------
void encoder_init(void);
void encoder_read(uint32_t *buffer, uint8_t buff_size, uint8_t enc_sel);
void encoder_reset(void);
void encoder_zero(void);
//----------------------------------------------------------------------


// Safety Function ------------
void check_miss_step(void);
//-----------------------------


// Custom RS232 Function -----
void send_mapped_data(void);
void send_ang_pos_data(void);
void receive_data(void);
//----------------------------


// OLED Function --------------------
void show_menu(Oled_Menu_t sel_menu);
//-----------------------------------

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
//-----------------------------------------------------------------------
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
//-----------------------------------------------------------------------


/* EXTI BUTTON CALLBACK */
//------------------------------------------------------------------------------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN){
	for(int i=0; i<JOINT_NUM; i++){
		if(GPIO_PIN == switch_gpio_pin[i]){
			switch_pressed[i] = (HAL_GPIO_ReadPin(switch_gpio_port[i], switch_gpio_pin[i]) == GPIO_PIN_RESET);
		}
	}
}
//------------------------------------------------------------------------------------------------------

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
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
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
			show_menu(EEPROM1_ERROR_MENU);
		}
	}
	
	EEPROM_Init(&hi2c1, &eeprom2, MEM_SIZE_512Kb, EEPROM2_ADDRESS);
	EEPROM_ByteRead(&eeprom2, 511, 0, eeprom2.dummy_byte, 1);
	if(eeprom2.status != EEPROM_OK){
		while(1){ 
			show_menu(EEPROM2_ERROR_MENU);
		}
	}
	#endif
	
	
	/* SDCARD SETUP */
	#ifdef USE_SDCARD
	if(mount_sdcard() != SD_OK){
		while(1){
			show_menu(SDCARD_ERROR_MENU);
		}
	}
	#endif
	
	
	/* RS-232 COMMUNICATION SETUP */
	#ifdef USE_RS232
	RS232_Init(&huart4);
	#endif
	
	
	/* ENCODER SETUP */
	#ifdef USE_ENCODER
	encoder_init();
	HAL_Delay(500);
//	encoder_reset();
	encoder_zero();
	HAL_Delay(1000);
	#endif
	
	
	/* USE KINEMATICS */
	#ifdef USE_KINEMATICS
	DHparam_init(&kinematics, joint_d, joint_a, joint_alpha);
	tollframe_init(&kinematics, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	forward_transform_matrix(&kinematics);
	calc_all_link(&kinematics);
	
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
		joint_ang_per_step[i] = 360.0 / (stepper_microstep[i] * stepper_ratio[i]);
		
		step_state[i] = STOPPING;
	}
	
	step_positive_dir[0] = DIR_CCW;
	step_positive_dir[1] = DIR_CCW;
	step_positive_dir[2] = DIR_CCW;
	step_positive_dir[3] = DIR_CW;
	step_positive_dir[4] = DIR_CW;
	step_positive_dir[5] = DIR_CCW;
	
	calibration_dir[0] = NEGATIVE_DIR; 	// Calibration pos -> -90 degree
	calibration_dir[1] = NEGATIVE_DIR;	// Calibration pos -> -60 degree
	calibration_dir[2] = POSITIVE_DIR;	// Calibration pos -> 60 Degree
	calibration_dir[3] = NEGATIVE_DIR; 	// Calibration pos -> 0 Degree
	calibration_dir[4] = POSITIVE_DIR; 	// Calibration pos -> 90 Degree
	
	home_dir[0] = POSITIVE_DIR;
	home_dir[1] = POSITIVE_DIR;
	home_dir[2] = NEGATIVE_DIR;
	home_dir[3] = POSITIVE_DIR;
	home_dir[4] = NEGATIVE_DIR;
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
	
	
	/* LIMIT SWITHC SETUP */
	for(int i=0; i<JOINT_NUM; i++){
		switch_pressed[i] = (HAL_GPIO_ReadPin(switch_gpio_port[i], switch_gpio_pin[i]) == GPIO_PIN_RESET);
	}
	
	
	#ifdef TEST_EEPROM 	
	double test_pos_start[6] = {-40.96, 12.15, -37.42, 12.27, -0.55, 5.72};
	double test_angle_start[6] = {70.55, -45.5, 90.58, 150.45, 55.17, 178.77};
	double test_pos_end[6] = {-20.96, 2.15, -37.42, 2.27, -10.55, 15.72};
	double test_angle_end[6] = {60, -20, 70.25, 115.62, 66.18, 122.76};

	save_welding_point(0x01, START_POINT, test_pos_start, test_angle_start);
	save_welding_point(0x01, END_POINT, test_pos_end, test_angle_end);
	save_welding_pattern(0x01, LINEAR);
	save_welding_speed(0x01, LOW);
	
	HAL_Delay(500);
	
	read_welding_point(0x01, START_POINT, array_pos_start, array_ang_start);
	read_welding_point(0x01, END_POINT, array_pos_end, array_ang_end);
	read_welding_pattern(0x01);
	read_welding_speed(0x01);

	HAL_Delay(500);
	#endif
	
	#ifdef TEST_SDCARD
	
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

  /* creation of setupTask */
  setupTaskHandle = osThreadNew(StartSetup, NULL, &setupTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the event(s) */
  /* creation of runningFlag */
  runningFlagHandle = osEventFlagsNew(&runningFlag_attributes);

  /* creation of setupFlag */
  setupFlagHandle = osEventFlagsNew(&setupFlag_attributes);

  /* creation of msgGet */
  msgGetHandle = osEventFlagsNew(&msgGet_attributes);

  /* creation of msgSent */
  msgSentHandle = osEventFlagsNew(&msgSent_attributes);

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
  RCC_OscInitStruct.PLL.PLLQ = 8;
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
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_4B;
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd1.Init.ClockDiv = 4;
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

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

  /*Configure GPIO pin : SDMMC_BSP_Pin */
  GPIO_InitStruct.Pin = SDMMC_BSP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDMMC_BSP_GPIO_Port, &GPIO_InitStruct);

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

/*--- FORMAT MEMORY FUNCTION ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void format_mem(EEPROM_ID_t id){
	if(id == EEPROM_ID1){
		for(uint8_t j=0; j<eeprom1.page_size; j++){
			EEPROM_PageReset(&eeprom1, j, 0);
		}
	}
	
	else if(id == EEPROM_ID2){
		for(uint8_t j=0; j<eeprom2.page_size; j++){
			EEPROM_PageReset(&eeprom2, j, 0);
		}
	}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- SAVE WELDING POINT VALUE ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void save_welding_point(uint16_t welding_point, uint8_t point_type, double* pos_data, double*angle_data){
	uint8_t chcksum_H, chcksum_L;
	uint16_t chcksum_bytes;
	
	// Save World Position & Joint Angle Value
	for(int i=0; i<48; i++){
		saved_pos[i] = 0;
		saved_angle[i] = 0;
	}
	for(size_t i=0; i<6; i++){
		memcpy(&saved_pos[i*8], &pos_data[i], sizeof(double));
		memcpy(&saved_angle[i*8], &angle_data[i], sizeof(double));
	}
	
	// Calculate Checksum
	chcksum_bytes = 0;
	for(int i=0; i<48; i++){
		chcksum_bytes += (saved_pos[i] + saved_angle[i]);
	}
	
	chcksum_H = (uint8_t)((chcksum_bytes >> 8) & 0xFF);
	chcksum_L = (uint8_t)(chcksum_bytes & 0xFF);
	
	if(point_type == START_POINT){
		// Save World Position & Joint Angle Value
		EEPROM_PageWrite(&eeprom1, welding_point-1, POS_BYTES_ADDR, saved_pos, sizeof(saved_pos));
		HAL_Delay(2);	
		EEPROM_PageWrite(&eeprom1, welding_point-1, ANG_BYTES_ADDR, saved_angle, sizeof(saved_angle));
		HAL_Delay(2);
		
		// Save Calculated Checksum Value
		EEPROM_ByteWrite(&eeprom1, welding_point-1, CHECKSUM_H_ADDR, chcksum_H, 1);
		HAL_Delay(2);
		EEPROM_ByteWrite(&eeprom1, welding_point-1, CHECKSUM_L_ADDR, chcksum_L, 1);
		HAL_Delay(2);
	}
	
	else if(point_type == END_POINT){
		// Save World Position & Joint Angle Value
		EEPROM_PageWrite(&eeprom2, welding_point-1, POS_BYTES_ADDR, saved_pos, sizeof(saved_pos));
		HAL_Delay(2);	
		EEPROM_PageWrite(&eeprom2, welding_point-1, ANG_BYTES_ADDR, saved_angle, sizeof(saved_angle));
		HAL_Delay(2);
		
		// Save Calculated Checksum Value
		EEPROM_ByteWrite(&eeprom2, welding_point-1, CHECKSUM_H_ADDR, chcksum_H, 1);
		HAL_Delay(2);
		EEPROM_ByteWrite(&eeprom2, welding_point-1, CHECKSUM_L_ADDR, chcksum_L, 1);
		HAL_Delay(2);
	}
}

void read_welding_point(uint16_t welding_point, uint8_t point_type, double* stored_pos, double*stored_angle){
	uint8_t
	read_chcksum_H[1], 
	read_chcksum_L[1];
	
	// Read World Position & Joint Angle Value
	for(int i=0; i<48; i++){
		read_saved_pos[i] = 0;
		read_saved_angle[i] = 0;
	}
	for(size_t i=0; i<6; i++){
		stored_pos[i] = 0;
		stored_angle[i] = 0;
	}
	
	if(point_type == START_POINT){
		// Read World Position & Joint Angle Value
		EEPROM_PageRead(&eeprom1, welding_point-1, POS_BYTES_ADDR, read_saved_pos, sizeof(read_saved_pos));
		HAL_Delay(5);
		EEPROM_PageRead(&eeprom1, welding_point-1, ANG_BYTES_ADDR, read_saved_angle, sizeof(read_saved_angle));
		HAL_Delay(5);
		
		// Read Calculated Checksum Value
		EEPROM_ByteRead(&eeprom1, welding_point-1, CHECKSUM_H_ADDR, read_chcksum_H, 1);
		HAL_Delay(5);
		EEPROM_ByteRead(&eeprom1, welding_point-1, CHECKSUM_L_ADDR, read_chcksum_L, 1);
		HAL_Delay(5);
		
		chcksum_sp_validation = (read_chcksum_H[0] << 8) | read_chcksum_L[0];	
		chcksum_sp = 0;
		
		// Calculate Saved World Position & Angle Joint Value Checksum
		for(int i=0; i<48; i++){
			chcksum_sp += (read_saved_pos[i] + read_saved_angle[i]);
		}
	}
	
	else if(point_type == END_POINT){
		// Read World Position & Joint Angle Value
		EEPROM_PageRead(&eeprom2, welding_point-1, POS_BYTES_ADDR, read_saved_pos, sizeof(read_saved_pos));
		HAL_Delay(5);
		EEPROM_PageRead(&eeprom2, welding_point-1, ANG_BYTES_ADDR, read_saved_angle, sizeof(read_saved_angle));
		HAL_Delay(5);
		
		// Read Calculated Checksum Value
		EEPROM_ByteRead(&eeprom2, welding_point-1, CHECKSUM_H_ADDR, read_chcksum_H, 1);
		HAL_Delay(5);
		EEPROM_ByteRead(&eeprom2, welding_point-1, CHECKSUM_L_ADDR, read_chcksum_L, 1);
		HAL_Delay(5);
		
		chcksum_ep_validation = (read_chcksum_H[0] << 8) | read_chcksum_L[0];
		chcksum_ep = 0;
		
		// Calculate Saved World Position & Angle Joint Value Checksum
		for(int i=0; i<48; i++){
			chcksum_ep += (read_saved_pos[i] + read_saved_angle[i]);
		}
	}
	
	// Process World Position & Joint Angle Value
	#ifdef USE_DEBUG_PROGRAM
	for(int i=0; i<8; i++){
		read_saved_posX[i] = read_saved_pos[i];
		read_saved_posY[i] = read_saved_pos[i+8];
		read_saved_posZ[i] = read_saved_pos[i+16];
		read_saved_rotX[i] = read_saved_pos[i+24];
		read_saved_rotY[i] = read_saved_pos[i+32];
		read_saved_rotZ[i] = read_saved_pos[i+40];
		
		read_saved_angle1[i] = read_saved_angle[i];
		read_saved_angle2[i] = read_saved_angle[i+8];
		read_saved_angle3[i] = read_saved_angle[i+16];
		read_saved_angle4[i] = read_saved_angle[i+24];
		read_saved_angle5[i] = read_saved_angle[i+32];
		read_saved_angle6[i] = read_saved_angle[i+40];
	}
	
	memcpy(&stored_pos[0], read_saved_posX, sizeof(double));
	memcpy(&stored_pos[1], read_saved_posY, sizeof(double));
	memcpy(&stored_pos[2], read_saved_posZ, sizeof(double));
	memcpy(&stored_pos[3], read_saved_rotX, sizeof(double));
	memcpy(&stored_pos[4], read_saved_rotY, sizeof(double));
	memcpy(&stored_pos[5], read_saved_rotZ, sizeof(double));
	
	memcpy(&stored_angle[0], read_saved_angle1, sizeof(double));
	memcpy(&stored_angle[1], read_saved_angle2, sizeof(double));
	memcpy(&stored_angle[2], read_saved_angle3, sizeof(double));
	memcpy(&stored_angle[3], read_saved_angle4, sizeof(double));
	memcpy(&stored_angle[4], read_saved_angle5, sizeof(double));
	memcpy(&stored_angle[5], read_saved_angle6, sizeof(double));
	#else
	memcpy(&stored_pos[0], &read_saved_pos[0], sizeof(double));
	memcpy(&stored_pos[1], &read_saved_pos[8], sizeof(double));
	memcpy(&stored_pos[2], &read_saved_pos[16], sizeof(double));
	memcpy(&stored_pos[3], &read_saved_pos[24], sizeof(double));
	memcpy(&stored_pos[4], &read_saved_pos[32], sizeof(double));
	memcpy(&stored_pos[5], &read_saved_pos[40], sizeof(double));
	
	memcpy(&stored_angle[0], &read_saved_angle[0], sizeof(double));
	memcpy(&stored_angle[1], &read_saved_angle[8], sizeof(double));
	memcpy(&stored_angle[2], &read_saved_angle[16], sizeof(double));
	memcpy(&stored_angle[3], &read_saved_angle[24], sizeof(double));
	memcpy(&stored_angle[4], &read_saved_angle[32], sizeof(double));
	memcpy(&stored_angle[5], &read_saved_angle[40], sizeof(double));
	#endif
}


/*--- SAVE WELDING POINT PATTERN ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void save_welding_pattern(uint16_t welding_point, uint8_t select_pattern){
	EEPROM_ByteWrite(&eeprom2, welding_point-1, PATTERN_BYTE_ADDR, select_pattern, sizeof(select_pattern));
	HAL_Delay(5);
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- READ WELDING POINT PATTERN ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void read_welding_pattern(uint16_t welding_point){
	uint8_t read_pattern[1];
	EEPROM_ByteRead(&eeprom2, welding_point-1, PATTERN_BYTE_ADDR, read_pattern, sizeof(read_pattern));
	welding_pattern = (Welding_Pattern_t)read_pattern[0];
	HAL_Delay(5);
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- WRITE WELDING POINT SPEED ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void save_welding_speed(uint16_t welding_point, uint8_t select_speed){
	EEPROM_ByteWrite(&eeprom2, welding_point-1, SPEED_BYTE_ADDR, select_speed, sizeof(select_speed));
	HAL_Delay(5);
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- READ WELDING POINT SPEED ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void read_welding_speed(uint16_t welding_point){
	uint8_t read_speed[1];
	EEPROM_ByteRead(&eeprom2, welding_point-1, SPEED_BYTE_ADDR, read_speed, sizeof(read_speed));
	welding_speed = (Speed_t)read_speed[0];
	HAL_Delay(5);
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- WELDING POINT DELETE FUNCTION*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void delete_welding_data(uint16_t welding_point, uint8_t point_type){
	if(point_type == START_POINT){
		EEPROM_PageReset(&eeprom1, welding_point-1, 0x00);
		HAL_Delay(1);
	}
	else if(point_type == END_POINT){
		EEPROM_PageReset(&eeprom2, welding_point-1, 0x00);
		HAL_Delay(1);
	}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- WELDING POINT VALIDATION ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Welding_Data_Status_t weldingpoint_validate(uint16_t welding_point){
	// Read Welding Point Data
	read_welding_point(welding_point, START_POINT, array_pos_start, array_ang_start);
	read_welding_point(welding_point, END_POINT, array_pos_end, array_ang_end);
	read_welding_pattern(welding_point);
	read_welding_speed(welding_point);
	
	// Data Validation
	if(chcksum_sp_validation != 0 && chcksum_ep_validation != 0){
		if(chcksum_sp == chcksum_sp_validation && chcksum_ep == chcksum_ep_validation && welding_pattern > 0x00 && welding_speed > 0x00){
			return WELDING_DATA_VALID;
		}
		else if(chcksum_sp != chcksum_sp_validation || chcksum_ep != chcksum_ep_validation || welding_pattern == 0x00 || welding_speed == 0x00){
			return WELDING_DATA_INVALID;
		}
	}
	
	return NO_WELDING_DATA;
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- SDCARD MOUNT ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
SD_Status_t mount_sdcard(void){
	res = f_mount(&fs, "", 1);
	if(res == FR_OK) return SD_OK;
	else return SD_MOUNT_ERR;
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- SDCARD TXT FILE CREATE FUNCTION ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
SD_Status_t create_txt_file(char *filename){
	res = f_open(&file, filename, FA_CREATE_ALWAYS | FA_WRITE);
	if (res != FR_OK) return SD_CREATE_ERR;
	else return SD_OK;
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- STEPPER DISABLE --- */
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void disable_stepper(void){
	HAL_GPIO_WritePin(ENABLE1_GPIO_Port, ENABLE1_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(ENABLE2_GPIO_Port, ENABLE2_Pin, GPIO_PIN_SET);
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- STEPPER ENABLE --- */
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void enable_stepper(void){
	HAL_GPIO_WritePin(ENABLE1_GPIO_Port, ENABLE1_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(ENABLE2_GPIO_Port, ENABLE2_Pin, GPIO_PIN_RESET);
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- MOVE STEPPER FUNCTION --- */
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void move_stepper(uint8_t select_joint, uint16_t step, Joint_Dir_t dir, uint16_t input_freq){		
	for(int i=0; i<JOINT_NUM; i++){
		if(select_joint == robot_joint[i]){
			// Stepper Joint Current Direction
			joint_current_dir[i] = dir;
			
			// Duty Cycle Calculation
			step_freq[i] = input_freq;
			step_period[i] = 1000000/step_freq[i];
			
			// Set Stepper Direction
			if(dir == POSITIVE_DIR && step_state[i] == STOPPING){
				if(step_positive_dir[i] == DIR_CW) HAL_GPIO_WritePin(stepper_gpio_port[i+6], stepper_gpio_pin[i+6], GPIO_PIN_SET);
				else HAL_GPIO_WritePin(stepper_gpio_port[i+6], stepper_gpio_pin[i+6], GPIO_PIN_RESET);
			}
			else if(dir == NEGATIVE_DIR && step_state[i] == STOPPING){
				if(step_positive_dir[i] == DIR_CW) HAL_GPIO_WritePin(stepper_gpio_port[i+6], stepper_gpio_pin[i+6], GPIO_PIN_RESET);
				else HAL_GPIO_WritePin(stepper_gpio_port[i+6], stepper_gpio_pin[i+6], GPIO_PIN_SET);
			}
			
			// Pulse Start
			target_step[i] = step;
			
			if((target_step[i] > 0 && step_counter[i] == 0 && step_state[i] == STOPPING) || (step_start[i] && step_state[i] == STOPPING) || !joint_limit){
				step_state[i] = AT_TOP_SPEED;
				t_on[i] = step_period[i] * SET_DUTY_CYCLE;
				t_off[i] = step_period[i] - t_on[i];
				HAL_TIM_Base_Start_IT(stepper_tim_handler[i]);
			}
		}
	}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- EXPONENTIAL SPEED RAMP CALCULATION ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint32_t calc_exp_step_interval(uint8_t select_joint, uint32_t step){
	if (step == 0) return 100000 / BASE_FREQ;  // Initial delay

	float freq = BASE_FREQ + (step_freq[select_joint] - BASE_FREQ) * (1 - expf(-0.001 * step));
	return (uint32_t)(100000.0f / freq);  // Convert frequency to step delay
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- STEPPER RPM CALCULATION --- */ 
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint16_t calc_stepper_freq(uint8_t select_joint, float rpm_input){
	// Synchronize Joint RPM
	stepper_rpm[select_joint] = rpm_input * stepper_ratio[select_joint];
	stepper_freq[select_joint] = stepper_rpm[select_joint] * stepper_microstep[select_joint] / 60;
	
	return stepper_freq[select_joint];
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- STEPPER STEP CALCULATION --- */
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint16_t calc_stepper_step(uint8_t select_joint, double angle){
	if(step_limit[select_joint] == true){
		step_count[select_joint] = abs((int)(angle / joint_ang_per_step[select_joint]));
	}
	return step_count[select_joint];
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- LINEAR INTERPOLATION CALCULATION --- */
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void calc_linear_parametric(double start_pos[6], double end_pos[6], uint8_t input_step, uint8_t current_step){	
	axis_delta[0] = (end_pos[0] - start_pos[0]) / input_step;
	axis_delta[1] = (end_pos[1] - start_pos[1]) / input_step;
	axis_delta[2] = (end_pos[2] - start_pos[2]) / input_step;
	axis_delta[3] = (end_pos[3] - start_pos[3]) / input_step;
	axis_delta[4] = (end_pos[4] - start_pos[4]) / input_step;
	axis_delta[5] = (end_pos[5] - start_pos[5]) / input_step;
	
	int_x_out = start_pos[0] + current_step * axis_delta[0];
	int_y_out = start_pos[1] + current_step * axis_delta[1];
	int_z_out = start_pos[2] + current_step * axis_delta[2];
	int_rx_out = start_pos[3] + current_step * axis_delta[3];
	int_ry_out = start_pos[4] + current_step * axis_delta[4];
	int_rz_out = start_pos[5] + current_step * axis_delta[5];
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- JOINT BASE MOVE FUNCTION --- */
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void move_joint(uint8_t select_joint, double input_angle, Speed_t set_speed){
	// Joint RPM Set
	if(set_speed == LOW) stepper_joint_rpm = 0.5f;
	else if(set_speed == MED) stepper_joint_rpm = 1.0f;
	else if(set_speed == HIGH) stepper_joint_rpm = 4.0f;
	
	if((input_angle > 0 && input_angle < 1) || (input_angle > -1 && input_angle < 0)){
		stepper_joint_rpm = 0.25;
	}
	
	for(int i=0; i<JOINT_NUM; i++){
		if(select_joint == robot_joint[i]){
			target_angle[i] = input_angle + enc_joint_angle[i];
			
			// Step or Distance Move
			if(input_angle > 0){
				move_stepper(robot_joint[i], calc_stepper_step(i, input_angle), POSITIVE_DIR, calc_stepper_freq(i, stepper_joint_rpm));
			}
			else if(input_angle < 0){
				move_stepper(robot_joint[i], calc_stepper_step(i, input_angle), NEGATIVE_DIR, calc_stepper_freq(i, stepper_joint_rpm));
			}
			
			// Continuous Move
			if(step_start[i] == true){
				if(command.move_sign == UNSIGNED_VAR) move_stepper(robot_joint[i], 0, POSITIVE_DIR, calc_stepper_freq(i, stepper_joint_rpm));
				else move_stepper(robot_joint[i], 0, NEGATIVE_DIR, calc_stepper_freq(i, stepper_joint_rpm));
			}
		}
	}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- WORLD BASE MOVE FUNCTION --- */
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void move_world(uint8_t select_axis, double move_pos, Speed_t set_speed){
	// Check Target Position
	for(int i=0; i<AXIS_NUM; i++){
		if(step_state[i] == STOPPING && select_axis == robot_axis[i]){
			if(move_pos == 0.0) target_position[i] = current_position[i] + 1.0;
			else target_position[i] = current_position[i] + move_pos;
		}
	}
	
	// Calculate Inverse Kinematics
	run_inverse_kinematic(&kinematics, target_position[0], target_position[1], target_position[2],
	target_position[3], target_position[4], target_position[5]);
	
	// Move Joint
	for(int i=0; i<JOINT_NUM; i++){
		move_joint(robot_joint[i], kinematics.joint_ang_out[i], set_speed);
	}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- LINEAR MOVE FUNCTION --- */
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void linear_move(double end_pos[6], Speed_t speed){	
	uint8_t steps = (speed == LOW) ? 50 :	(speed == MED) ? 30 : 10;
	
	if(int_counter > steps) int_counter = 0;
	
	if(int_counter <= steps){
		clear_to_update = true;
		for(int i=0; i<JOINT_NUM; i++){
			if(step_state[i] != STOPPING){
				clear_to_update = false;
				break;
			}
		}
		
		if(clear_to_update){
			// Calculate Linear Parametric
			calc_linear_parametric(current_position, end_pos, steps, int_counter);
			
			// Calculate Inverse Kinematics
			run_inverse_kinematic(&kinematics, int_x_out, int_y_out, int_z_out, int_rx_out, int_ry_out, int_rz_out);
			
			for(int i=0; i<JOINT_NUM; i++){
				move_joint(robot_joint[i], kinematics.joint_ang_out[i], speed);
			}
	
			int_counter++;
		}
	}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- WELDER TURN ON FUNCTION ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void welder_on(void){
	HAL_GPIO_WritePin(MIG_SW_IN_GPIO_Port, MIG_SW_IN_Pin, GPIO_PIN_SET);
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- WELDER TURN OFF FUNCTION ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void welder_off(void){
	HAL_GPIO_WritePin(MIG_SW_IN_GPIO_Port, MIG_SW_IN_Pin, GPIO_PIN_RESET);
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- WELDING POINT PREVIEW FUNCTION ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void welding_preview(void){
	while(1){
		
	}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- WELDING START FUNCTION ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void welding_start(void){	
	// Count valid welding points data
	for(int i=1; i<=MAX_POINTS; i++){		
		point_status[i-1] = weldingpoint_validate(i);
		if(point_status[i-1] == WELDING_DATA_VALID) total_valid_points++;
	}
	
	// Check welding points data align
	for(int i=0; i<MAX_POINTS; i++){
		if(point_status[i] != WELDING_DATA_VALID){
			invalid_welding_point = i+1;
			break;
		}
		else total_mapped_points++;
	}
	
	// Check welding run state
	if(total_valid_points != total_mapped_points){
		welding_run = false;
		for(int i=0; i<10; i++){
			Send_feedback(&command, POINT_INVALID, invalid_welding_point);
		}
	}
	else{
		welding_run = true;
	}

	// Start welding algorithm when all mapped welding point is valid
	if(welding_run){
		// Robot Homing
		
		// Home -> First Welding Point
		
		// Run Welding
		for(int i=0; i<total_mapped_points-1; i++){
			// Current Point Handler
			
			// Welding Point Transition
		}
		
		// Last Point Handler
		
		// Last Welding Point -> Home
	}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- UPDATE JOINT ANGLE AND POSITION VALUE FUNCTION ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void update_value(void){
	run_forward_kinematic(&kinematics, enc_joint_angle);
	
	for(int i=0; i<JOINT_NUM; i++){
		if(i<3) current_position[i] = kinematics.axis_pos_out[i];
		else current_position[i] = kinematics.axis_rot_out[i];
		
		#ifdef USE_ENCODER
		current_angle[i] = enc_joint_angle[i];
		#endif
	}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- LIMIT SWITCH STATE CHECK FUNCTION ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void check_limit(Limit_t check_mode){
	if(check_mode == ALL_LIMIT_CHECK){
		
	}
	
	// Software Limit
	else if(check_mode == SOFT_LIMIT_CHECK){
		
	}
	
	// Hardware Limit
	else if(check_mode == HARD_LIMIT_CHECK){
		for(int i=0; i<JOINT_NUM; i++){
			if(enc_joint_angle[i] > joint_negative_axisLim[i] && enc_joint_angle[i] < joint_positive_axisLim[i]){
				hard_limit[i] = false;
			}
			else{
				if(joint_error_bypass == false){
					hard_limit[i] = true;
					for(int i=0; i<10; i++){
						Send_feedback(&command, ANGLE_HARD_LIMIT, (uint16_t)i);
					}
					if(enc_joint_angle[i] <= joint_negative_axisLim[i]) negative_limit[i] = true;
					else if(enc_joint_angle[i] >= joint_positive_axisLim[i]) positive_limit[i] = true;
				}
			}
			
			if(negative_limit[i] == true && enc_joint_angle[i] > joint_negative_axisLim[i]){
				negative_limit[i] = false;
				joint_error_bypass = false;
			}
			else if(positive_limit[i] == true && enc_joint_angle[i] < joint_positive_axisLim[i]){
				positive_limit[i] = false;
				joint_error_bypass = false;
			}
			
			joint_limit |= hard_limit[i];
		}
	}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- JOINT ANGLE HOMING FUNCTION ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool joint_calibrate(Cal_Mode_t mode, bool j1_cal, bool j2_cal, bool j3_cal, bool j4_cal, bool j5_cal, bool j6_cal){
	bool selected_joint[6] = {j1_cal, j2_cal, j3_cal, j4_cal, j5_cal, j6_cal};
	
	// Count Calibrated Joint
	if(calibrate_step == 0){
		for(int i=0; i<JOINT_NUM; i++){
			if(selected_joint[i]){
				calibrated_joint[i] = false;
				select_count++;
			}
		}
		calibrate_step = 1;
	}
	
	// Move All Joints Until Each Limit Switch Pressed
	else if(calibrate_step == 1){		
		for(int i=0; i<JOINT_NUM; i++){
			if(!selected_joint[i]) continue;
			
			if(!calibrated_joint[i]){
				for(int i=0; i<50; i++){
					switch_pressed[i] &= switch_pressed[i];
				}
						
				if(!switch_pressed[i]){
					step_start[i] = true;
					move_stepper(robot_joint[i], 0, calibration_dir[i], calc_stepper_freq(i, 0.5f));
				}
				else if(switch_pressed[i]){
					step_start[i] = false;
					calibrated_joint[i] = true;
				}
			}
		}
		calibrated_count = calibrated_joint[0] + calibrated_joint[1] + calibrated_joint[2] + calibrated_joint[3] + calibrated_joint[4] + calibrated_joint[5];	
		if(calibrated_count == select_count) calibrate_step = 2;
	}
	
	else if(calibrate_step == 2){
		calibrate_step = 0;
		calibrated_count = 0;
		select_count = 0;
		command.type = NONE;
		return true;
	}
	
	return false;
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- ENCODER INITIALIZE FUNCTION ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void encoder_init(void){	
	for(int i=0; i<JOINT_NUM; i++){
		HAL_TIM_IC_Start_IT(enc_tim_handler[i], TIM_CHANNEL_1);
		HAL_TIM_IC_Start_IT(enc_tim_handler[i], TIM_CHANNEL_2);
		
		if(step_positive_dir[i] == DIR_CW) HAL_GPIO_WritePin(enc_gpio_port[i+6], enc_gpio_pin[i+6], GPIO_PIN_RESET);
		else HAL_GPIO_WritePin(enc_gpio_port[i+6], enc_gpio_pin[i+6], GPIO_PIN_SET);
	}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- ENCODER COUNTER RESET FUNCTION --- */
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void encoder_reset(void){
	for(int i=0; i<JOINT_NUM; i++){
		cal_value[i] = enc_angle[i];
		enc_joint_angle[i] = joint_cal_angle[i];
	}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- ENCODER ANGLE ZEROING FUNCTION --- */
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void encoder_zero(void){
	for(int i=0; i<JOINT_NUM; i++){
		cal_value[i] = enc_angle[i];
		enc_joint_angle[i] = 0.0;
	}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- MISS STEP FUNCTION --- */
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void check_miss_step(void){
	for(int i=0; i<JOINT_NUM; i++){	
		int32_t delta_joint_angle = enc_joint_angle[i] - prev_enc_joint_angle[i];
		
		if(abs(delta_joint_angle) > MISS_STEP_TOLLERANCE){
			if(joint_error_bypass == false){
				miss_step[i] = true;
				for(int i=0; i<10; i++){
					Send_feedback(&command, JOINT_MISS_STEP, (uint16_t)i);
				}
				
				if(joint_current_dir[i] == NEGATIVE_DIR) negative_miss[i] = true;
				else if(joint_current_dir[i] == POSITIVE_DIR) positive_miss[i] = true;
				
				joint_miss_step_angle[i] = enc_joint_angle[i];
			}
		}
		else miss_step[i] = false;
	
		if(negative_miss[i] == true && enc_joint_angle[i] > joint_miss_step_angle[i] + MISS_STEP_SAFE_ANGLE){
			negative_miss[i] = false;
			joint_error_bypass = false;
		}
		else if(positive_miss[i] == true && enc_joint_angle[i] < joint_miss_step_angle[i] - MISS_STEP_SAFE_ANGLE){
			positive_miss[i] = false;
			joint_error_bypass = false;
		}
		
		prev_enc_joint_angle[i] = enc_joint_angle[i];
		joint_miss_step |= miss_step[i];
	}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


// --- SEND MAPPED WELDING DATA FUNCTION --- */
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void send_mapped_data(void){
	if(command.requested_data == MAPPED_START_POINT){
		
	}
	else if(command.requested_data == MAPPED_END_POINT){
		
	}
	
	// On Pregress
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- CUSTON RS232 TRANSMIT FUNCTION ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void send_ang_pos_data(void){
	tx_pos[0] = 12.3;
	tx_pos[1] = 12.3;
	tx_pos[2] = 12.3;
	
	tx_rot[0] = 11.1;
	tx_rot[1] = 11.1;
	tx_rot[2] = 11.1;
	
	tx_angle[0] = 90.5;
	tx_angle[1] = 90.5;
	tx_angle[2] = 90.5;
	tx_angle[3] = 90.5;
	tx_angle[4] = 90.5;
	tx_angle[5] = 90.5;
	
//	tx_pos[0] = (float)kinematics.axis_pos_out[0];
//	tx_pos[1] = (float)kinematics.axis_pos_out[1];
//	tx_pos[2] = (float)kinematics.axis_pos_out[2];
	
//	tx_rot[0] = (float)kinematics.axis_rot_out[0];
//	tx_rot[1] = (float)kinematics.axis_rot_out[1];
//	tx_rot[2] = (float)kinematics.axis_rot_out[2];
	
//	tx_angle[0] = (float)enc_joint_angle[0];
//	tx_angle[1] = (float)enc_joint_angle[1];
//	tx_angle[2] = (float)enc_joint_angle[2];
//	tx_angle[3] = (float)enc_joint_angle[3];
//	tx_angle[4] = (float)enc_joint_angle[4];
//	tx_angle[5] = (float)enc_joint_angle[5];
	
	if(HAL_GetTick() - prev_time_send > send_data_interval){
		for(int i=0; i<25; i++){
			Send_requested_data(&command, tx_pos, tx_rot, tx_angle, welding_point, welding_pattern, welding_speed);
		}
		prev_time_send = HAL_GetTick();
	}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- CUSTON RS232 RECEIVE FUNCTION ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void receive_data(void){
	HAL_UART_Receive_IT(&huart4, command.data_buff, sizeof(command.data_buff));
	
	if(command.msg_get == true){
		Get_command(&command);
		command.msg_get = false;
	}
	
	if(command.type != CALIBRATION && HAL_GetTick() - prev_time_get > 300 && command.msg_get == false){
		command.type = NONE;
		prev_time_get = HAL_GetTick();
	}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- SHOW OLED MENU FUNCTION ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
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
		if(command.type == CALIBRATION) SSD1306_Puts("Calibrate   ", &Font_7x10, SSD1306_COLOR_WHITE);
		else if(command.type == MAPPING) SSD1306_Puts("Mapping     ", &Font_7x10, SSD1306_COLOR_WHITE);
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
		SSD1306_Puts(eeprom1_err, &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();
	}
	
	else if(sel_menu == EEPROM2_ERROR_MENU){
		SSD1306_GotoXY(0,0);
		SSD1306_Puts(eeprom2_err, &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();
	}
	
	else if(sel_menu == SDCARD_ERROR_MENU){
		SSD1306_GotoXY(0,0);
		SSD1306_Puts(sdcard_err, &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();
	}
	
	else if(sel_menu == SAVE_MENU){
		SSD1306_GotoXY(0,0);
		SSD1306_Puts(save_data, &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();
	}
	
	else if(sel_menu == DELETE_MENU){
		SSD1306_GotoXY(0,0);
		SSD1306_Puts(delete_data, &Font_7x10, SSD1306_COLOR_WHITE);
		SSD1306_UpdateScreen();
	}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------

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
		#ifdef ENCODER_TASK_ON
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
		#else
		osDelay(1);
		#endif
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
		#ifdef LIMIT_CHECK_TASK_ON
		if(command.control_mode == WORLD_CTRL) check_limit(ALL_LIMIT_CHECK);
		else if(command.control_mode == JOINT_CTRL) check_limit(HARD_LIMIT_CHECK);
		#else
		osDelay(1);
		#endif
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
		#ifdef STEPPER_CTRL_TASK_ON
		if(osSemaphoreAcquire(stepperRWHandle, 0) == osOK){
			for(int i=0; i<JOINT_NUM; i++){
				if(step_state[i] == ACCELERATING){
					if(step_counter[i] < target_step[i]*0.25) step_accel_period[i] = calc_exp_step_interval(i, step_counter[i]/2);
					else step_accel_period[i] = calc_exp_step_interval(i, target_step[i]-step_counter[i]/2);
					
					t_on[i] = step_accel_period[i] * SET_DUTY_CYCLE;
					t_off[i] = step_accel_period[i] - t_on[i];
				}
				
				if(timer_counter[i] < t_on[i]) HAL_GPIO_WritePin(stepper_gpio_port[i], stepper_gpio_pin[i], GPIO_PIN_SET);
				else if(timer_counter[i] >= t_on[i] && timer_counter[i] < step_period[i]-1) HAL_GPIO_WritePin(stepper_gpio_port[i], stepper_gpio_pin[i], GPIO_PIN_RESET);
				else if(timer_counter[i] == step_period[i]-1){
					timer_counter[i] = 0;
					step_counter[i]++;
				}
				
				if(((step_counter[i] >= target_step[i]) && step_limit[i]) || (!step_limit[i] && !step_start[i]) || joint_limit || robot_stop){
					HAL_GPIO_WritePin(stepper_gpio_port[i], stepper_gpio_pin[i], GPIO_PIN_RESET);
					HAL_TIM_Base_Stop_IT(stepper_tim_handler[i]);
					timer_counter[i] = 0;
					step_counter[i] = 0;
					step_state[i] = STOPPING;
				}
			}
		}
		#else
		osDelay(1);
		#endif
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
		#ifdef CMD_HANDLER_TASK_ON
		// Receive RS-232 Data
		receive_data();
		
		// Check Auto Home Command
		if(command.type == CALIBRATION){
			if(joint_calibrate(command.calibration_mode, true, false, true, false, false, false) == true){
				home = true;
				Send_feedback(&command, CALIBRATION_DONE, 0x00);
			}
		}
		
		// Check Mapping Command
		else if(command.type == MAPPING){
			if(command.mapping_state == SAVE_VALUE){
				if(command.welding_point_type == START_POINT){
					show_menu(SAVE_MENU);
					save_welding_point(command.welding_point_num, START_POINT, current_position, current_angle);
				
					HAL_Delay(1000);
				}
				else if(command.welding_point_type == END_POINT){
					show_menu(SAVE_MENU);
					save_welding_point(command.welding_point_num, END_POINT, current_position, current_angle);
					
					HAL_Delay(1000);
				}
			}
			else if(command.mapping_state == DELETE_VALUE){
				if(command.welding_point_type == START_POINT){
					show_menu(DELETE_MENU);
					delete_welding_data(command.welding_point_num, START_POINT);
					
					HAL_Delay(1000);
				}
				else if(command.welding_point_type == END_POINT){
					show_menu(DELETE_MENU);
					delete_welding_data(command.welding_point_num, END_POINT);
					
					HAL_Delay(1000);
				}
			}
		}
		
		// Check Move Command
		else if(command.type == MOVE){
			robot_stop = false;
			
			if(command.control_mode == WORLD_CTRL){			
				for(int i=0; i<AXIS_NUM; i++){
					if(command.move_variable == robot_axis[i]){
						step_limit[i] = true;
						step_start[i] = false;
						
						if(command.move_mode == CONTINUOUS){
							move_world(command.move_variable, rx_move_position[i], LOW);
						}
						else if(command.move_mode == DISTANCE){
							rx_move_position[i] = command.move_value;
							move_world(command.move_variable, rx_move_position[i], LOW);
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
						
						if(positive_limit[i] && command.move_sign == SIGNED_VAR){
							move_joint(robot_joint[i], rx_move_angle[i], LOW);
						}
						
						else if(negative_limit[i] && command.move_sign == UNSIGNED_VAR){
							move_joint(robot_joint[i], rx_move_angle[i], LOW);
						}
						
						else if(!positive_limit[i] && !negative_limit[i]){
							move_joint(robot_joint[i], rx_move_angle[i], LOW);
						}
					}
					else rx_move_angle[i] = 0.0;
				}
			}
		}
		
		// Check Run Command
		else if(command.type == RUN){
			if(command.running_mode == CONTROL_MODE){
				if(command.running_state == RUNNING_STOP){
					robot_stop = true;
				}
			}
			
			else if(command.running_mode == WELDING_MODE){
				// ON PROGRESS
			}
			
			else if(command.running_mode == PREVIEW_MODE){
				// ON PROGRESS
			}
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
			joint_miss_step = false;
			joint_error_bypass = true;
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
		#else
		osDelay(1);
		#endif
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
		#ifdef RUNNING_HANDLER_TASK_ON
		uint8_t flags;
		flags = osEventFlagsWait(runningFlagHandle, WELDING_FLAG | PREVIEW_FLAG, osFlagsWaitAny, 0);
		
		// Welding Algorithm
		if(flags & WELDING_FLAG){
		}
		
		// Preview Algorithm
		if(flags & PREVIEW_FLAG){
		}
		#else
    osDelay(1);
		#endif
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
		#ifdef POS_UPDATE_TASK_ON
		update_value();
		send_ang_pos_data();
    osDelay(1);
		#else
		osDelay(1);
		#endif
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
		#ifdef LCD_MENU_TASK_ON
		uint32_t flags = osEventFlagsWait(setupFlagHandle, 0x01, osFlagsWaitAny, 0);
		
		if(flags & 0x01){
			show_menu(MAIN_MENU);
		}
		#endif
    osDelay(1);
  }
  /* USER CODE END StartLCDMenu */
}

/* USER CODE BEGIN Header_StartSetup */
/**
* @brief Function implementing the setupTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSetup */
void StartSetup(void *argument)
{
  /* USER CODE BEGIN StartSetup */
  /* Infinite loop */
  for(;;)
  {
		receive_data();
    osDelay(1);
  }
  /* USER CODE END StartSetup */
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
