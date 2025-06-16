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
#include "EEPROM_lib.h"
#include "RS232_Driver.h"
#include "ArmRobot_Math.h"
#include "ssd1306.h"
#include "stdbool.h"
#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "math.h"
#include "PID_Driver.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
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
UINT bw;
// -----------------


/* PID TYPEDEF*/
// -------------------
PIDController pos_pid;
// -------------------


/* OLED TYPEDEF */
// --------------------
typedef enum{
	BOOT_MENU,						// STM32 Booting menu
	COM_INIT_MENU,				// RS-232 initialize menu
	MAIN_MENU,						// Main menu
	EEPROM1_ERROR_MENU,		// EEPROM 1 error menu
	EEPROM2_ERROR_MENU,		// EEPROM 2 error menu
	SDCARD_ERROR_MENU,		// SDCARD mount error menu
	ENCODER_ERROR_MENU,		// Encoder init error menu
	SAVE_MENU,						// Welding data save menu
	DELETE_MENU,					// Welding data delete meneu
}Oled_Menu_t;
// --------------------


/* STEPPER DIR TYPEDEF */
// ----------------------
typedef enum{
	DIR_CCW,		// Counterclockwise direction
	DIR_CW,			// Clockwise direction
}Stepper_Dir_t;
// ----------------------


/* JOINT DIR TYPEDEF */
// --------------------
typedef enum{
	NEGATIVE_DIR,	// Joint negative angle direction
	POSITIVE_DIR,	// Joint positive angle direction
}Joint_Dir_t;
// --------------------


/*  LIMIT TYPE TYPEDEF */
// ----------------------
typedef enum{
	NO_CHECK,
	SOFT_LIMIT_CHECK,	// Software only check
	HARD_LIMIT_CHECK,	// Hardware only check
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


/* RUNNING STEP TYPEDEF */
// -----------------------
typedef enum{
	NO_STEP,
	MOVE_TO_HOME,
	MOVE_TO_START_POINT,
	MOVE_TO_END_POINT,
	CHANGE_POINT,
	MOVE_STOP,
}Running_Step_t; 	
// -----------------------


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
	SD_OK = 0x01, 
	SD_MOUNT_ERR,
	SD_CREATE_ERR,
	SD_OPEN_ERR,
	SD_READ_ERR,
	SD_WRITE_ERR,
}SD_Status_t;
// ------------------------


/* CALIBRATION STEP TYPEDEF */
// ---------------------------
typedef enum{
	COUNT_JOINT,				// Count selected joint
	J1_J3_CAL,					// J1 to J3 first calibration
	J1_J3_OFFSET_MOVE,	// Move J1 to J3 to angle offset
	J1_J3_RECAL,				// J1 to J3 re-calibration
	
	J4_J6_CAL,					// J4 to J6 first calibration
	J4_J6_OFFSET_MOVE,	// Move J4 to J6 to angle offset
	J4_J6_RECAL,				// J4 to J6 re-calibration
	
	ALL_JOINT_HOME,
	
	CALIBRATION_OK,			// Calibration Done
}Cal_Step_t;
// ---------------------------


/* LINEAR INTERPOLATION MODE TYPEDEF */
// ------------------------------------
typedef enum{
	ALL_AXES,
	POS_AXES,
	ROT_AXES,
}Linear_Move_Mode_t;
// ------------------------------------


/* POWER STATUS TYPEDEF */
// -----------------------
typedef enum{
	UNDER_VOLTAGE,
	NORM_VOLTAGE,
}Power_Status_t;
// -----------------------


/* WELDING POINT DATA STRUCT */
// ------------------------------------
typedef struct{
	float 
	array_pos_start[6],
	array_pos_end[6],
	array_ang_start[6],
	array_ang_end[6];
	
	uint16_t welding_point;
	
	Speed_t welding_speed;
	Welding_Pattern_t welding_pattern;
	Welding_Data_Status_t wd_data_status;
	Axis_Offset_t welding_axis_offset;
}Welding_Data_t;
Welding_Data_t welding_data;
// ------------------------------------

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* SYSTEM CONFIGURATION */
//----------------------
//#define USE_OLED
//#define USE_SDCARD
#define USE_EEPROM
#define USE_RS232
#define USE_STEPPER
#define USE_ENCODER
#define USE_KINEMATICS
//#define USE_PID

//#define USE_DEBUG_PROGRAM
//#define USE_MISS_STEP_CHECK
#define USE_LIMIT_CHECK

//#define TEST_SDCARD
//#define TEST_EEPROM
//#define TEST_RS232
//#define TEST_OLED
//#define TEST_STEPPER
//#define TEST_ENCODER
//#define TEST_LIMIT_SWITCH
//#define TEST_KINEMATICS
//#define TEST_MOTION

//#define RECORD_RESPONS_DATA

#define MAIN_PROGRAM
//----------------------

/* EEPROM ADDRESS SET */
//-----------------------------------
#define EEPROM1_ADDRESS				0xA0	// Primary EEPROM Address
#define EEPROM2_ADDRESS				0xA2	// Secondary EEPROM Address

#define START_POINT_ADDR			0x01	// Start Point EEPROM select
#define END_POINT_ADDR				0x02	// End Point EEPROM select

#define POS_BYTES_ADDR				0x00	// World Position Value Address
#define ANG_BYTES_ADDR				0x18	// Joint Angle Value Address

#define PATTERN_BYTE_ADDR			0x30	// Welding Pattern Address (1 Byte)
#define SPEED_BYTE_ADDR				0x31	// Welding Speed Address (1 Byte)
#define OFFSET_BYTE_ADDR			0x32	// Welding Spline Axis Offset Address (5 Byte)

#define CHECKSUM_H_ADDR				0x37	// Checksum H Byte Address
#define CHECKSUM_L_ADDR				0x38	// Checksum L Byte Address
//-----------------------------------

/* EEPROM PATTERN SET */
//-----------------------------------
#define DOT_PATTERN							0x01
#define LINEAR_PATTERN					0x02
#define CIRCULAR_PATTERN				0x03
#define ZIGZAG_PATTERN					0x04
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

#define STEPPER1_RATIO				162.0f
#define	STEPPER2_RATIO				200.0f
#define	STEPPER3_RATIO				50.0f
#define	STEPPER4_RATIO				96.0f 
#define	STEPPER5_RATIO				22.5f
#define	STEPPER6_RATIO				25.0f

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

#define JOINT1_MAX_ANGLE	170.0f
#define JOINT2_MAX_ANGLE	90.0f
#define JOINT3_MAX_ANGLE	60.0f
#define JOINT4_MAX_ANGLE	145.0f
#define JOINT5_MAX_ANGLE	120.0f
#define JOINT6_MAX_ANGLE	155.0f

#define JOINT1_MIN_ANGLE	-170.0f
#define JOINT2_MIN_ANGLE	-60.0f
#define JOINT3_MIN_ANGLE	-90.0f
#define JOINT4_MIN_ANGLE	-145.0f
#define JOINT5_MIN_ANGLE	-105.0f
#define JOINT6_MIN_ANGLE	-155.0f

#define JOINT1_CAL_ANGLE 	-90.0f
#define JOINT2_CAL_ANGLE 	-60.0f
#define JOINT3_CAL_ANGLE 	60.0f
#define JOINT4_CAL_ANGLE 	0.0f
#define JOINT5_CAL_ANGLE 	-90.0f
#define JOINT6_CAL_ANGLE 	0.0f

#define JOINT1_BIT_VAL		0x01
#define JOINT2_BIT_VAL		0x02
#define JOINT3_BIT_VAL		0x04
#define JOINT4_BIT_VAL		0x08
#define JOINT5_BIT_VAL		0x10
#define JOINT6_BIT_VAL		0x20

#define FILTER_FACTOR			0.5f
//-------------------------------


/*INTERPOLATION SET*/
//--------------------------
#define MAX_UPDATE_STEP		51
//--------------------------

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

/* USER CODE BEGIN PV */

// PARAMETER VARIABLE
Speed_t
global_speed;

float
max_delta_angle,
input_points[2][6];

const float
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

float
home_pos[AXIS_NUM] = {
	598.5f, 0.0f, 776.0f,
	180.0f, 0.0f, 180.0f
};

float
joint_max_traveldist[JOINT_NUM],
joint_ang_res[JOINT_NUM],
joint_miss_step_angle[JOINT_NUM];

float
joint_rpm = 0.0f,
global_feedrate = 0.0f,
delta_angle[JOINT_NUM],
sync_joint_rpm[JOINT_NUM],
joint_input_angle[JOINT_NUM];

bool
calibrating = false,
clear_to_update = false,
cont_world_move = false,
end_point_reach = false,
home = false,
robot_stop = false,
get_welding_cmd = false,
welding_run = false,
get_preview_cmd = false,
preview_run = false,
point_checked = false,
joint_limit = false,
joint_miss_step = false,
joint_error_bypass = false,
check_welding_point = false,
calculate_parameter = false,
step_reached[JOINT_NUM],
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

float
joint_d[JOINT_NUM] = {
	191.0f, 0.0f, 0.0f,
	575.0f, 0.0f, 65.0f,
},

joint_a[JOINT_NUM] = {
	23.5f, 650.0f, 0.0f,
	0.0f, 0.0f, 0.0f, 
},

joint_alpha[JOINT_NUM] = {
	-90.0f, 0.0f, 90.0f,
	-90.0f, 90.0f, 0.0f,
};

uint8_t
limit_bit_sum,
singularity_counter,
joint_limit_bit[6] = {
	JOINT1_BIT_VAL,
	JOINT2_BIT_VAL,
	JOINT3_BIT_VAL,
	JOINT4_BIT_VAL,
	JOINT5_BIT_VAL,
	JOINT6_BIT_VAL,
},

joint_zero_sel,
invalid_welding_point;

uint16_t
current_running_point,
total_valid_points,
total_mapped_points;

Welding_Data_Status_t
point_status[MAX_POINTS];

Running_Step_t
running_step;

Joint_Dir_t
joint_current_dir[JOINT_NUM],
calibration_dir[JOINT_NUM],
home_dir[JOINT_NUM];



// EEPROM VARIABLE
uint8_t
page_data_byte[EEPROM_512Kb_PAGE_SIZE],

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

saved_pos[24], 
read_saved_pos[24],
saved_angle[24], 
read_saved_angle[24];

uint16_t
chcksum_sp, chcksum_ep,
chcksum_sp_validation,
chcksum_ep_validation;


// INVERSE KINEMATICS VARIABLE
float
ik_pos_input[AXIS_NUM],
ik_angle_output[JOINT_NUM];


// FORWARD KINEMATICS VARIABLE
float
fk_angle_input[JOINT_NUM],
fk_pos_output[AXIS_NUM];


// MOVE VARIABLE
float
current_position[AXIS_NUM],
target_position[AXIS_NUM],
delta_move_pos[AXIS_NUM],
prev_move_position[AXIS_NUM],

current_angle[JOINT_NUM],
target_angle[JOINT_NUM],
delta_move_ang[JOINT_NUM],
prev_move_angle[JOINT_NUM];


// INTERPOLATION VARIABLE
float
middle_point[AXIS_NUM],
axis_delta[AXIS_NUM],
joint_delta[JOINT_NUM],

int_x_out[MAX_UPDATE_STEP], 
int_y_out[MAX_UPDATE_STEP], 
int_z_out[MAX_UPDATE_STEP],
int_rx_out[MAX_UPDATE_STEP], 
int_ry_out[MAX_UPDATE_STEP], 
int_rz_out[MAX_UPDATE_STEP],
int_axes_out[AXIS_NUM],

int_j1_out[MAX_UPDATE_STEP],
int_j2_out[MAX_UPDATE_STEP],
int_j3_out[MAX_UPDATE_STEP],
int_j4_out[MAX_UPDATE_STEP],
int_j5_out[MAX_UPDATE_STEP],
int_j6_out[MAX_UPDATE_STEP],
int_angle_out[JOINT_NUM];

uint8_t
current_point,
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
step_cont_run_j[JOINT_NUM],
step_cont_run_w[JOINT_NUM],
step_cal_run[JOINT_NUM],
step_limit[JOINT_NUM];

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
stepper_stepfactor[JOINT_NUM],

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
};

const float
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

Stepper_Dir_t
step_positive_dir[JOINT_NUM];


// RS232 VARIABLE
unsigned long
prev_time_send,
prev_time_get;

float
tx_pos[3],
tx_rot[3],
tx_angle[6],
rx_move_position[6],
rx_move_angle[6];

const uint8_t
send_data_interval = 50;


// OLED LCD VARIABLE
uint8_t 
refresh_counter,
selected_menu;

char 
title[] = "   WELDING ARM V1",
eeprom1_err[] = "EEPROM1 ERROR    ",
eeprom2_err[] = "EEPROM2 ERROR    ",
sdcard_err[] = "SDCARD ERROR     ",
encoder_err[] = "ENCODER ERROR    ",
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

bool
reset_enc = false;

uint8_t
enc_mag_status[JOINT_NUM];

volatile uint32_t
pwm_freq[JOINT_NUM],
high_time[JOINT_NUM],
period_time[JOINT_NUM];

int
enc_counter[JOINT_NUM];

const float
min_duty_cycle = 2.9,
max_duty_cycle = 97.1;

float
duty_cycle[JOINT_NUM],
diff_angle[JOINT_NUM],
enc_angle[JOINT_NUM],
cal_value[JOINT_NUM],
cal_enc_angle[JOINT_NUM],
prev_cal_enc_angle[JOINT_NUM],
reduced_cal_enc_angle[JOINT_NUM],
raw_joint_angle[JOINT_NUM],
filtered_joint_angle[JOINT_NUM],
prev_filtered_joint_angle[JOINT_NUM],

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
},

enc_tollerance[JOINT_NUM] = {
	0.0, 0.0, 0.0,
	0.0, 0.0, 0.0,
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

Cal_Step_t
calibration_step;

uint8_t
group1_count = 0,
group2_count = 0,
group1_calibrated = 0,
group2_calibrated = 0;

// SDCARD VARIABLE
SD_Status_t sdcard_status;
char 
file1[] = "Singularity.txt",
file2[] = "Respons.txt",
file3[] = "Noname.txt";

// POWER VARIABLE
uint8_t pwr_status;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM15_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM13_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_TIM17_Init(void);
/* USER CODE BEGIN PFP */

// EEPROM Function ---------------------------------------------------------------------------------
void format_mem(EEPROM_ID_t id);
void save_welding_point(uint16_t point, uint8_t point_type, float *pos_data, float *angle_data);
void read_welding_point(uint16_t point, uint8_t point_type, float *stored_pos, float *stored_angle);
void save_welding_pattern(uint16_t point, uint8_t select_pattern);
void read_welding_pattern(uint16_t point);
void save_welding_speed(uint16_t point, uint8_t select_speed);
void read_welding_speed(uint16_t point);
void save_welding_axis_offset(uint16_t point, uint8_t select_axis);
void read_welding_axis_offset(uint16_t point);
void delete_welding_data(uint16_t point, uint8_t point_type);
Welding_Data_Status_t weldingpoint_validate(uint16_t point);
//--------------------------------------------------------------------------------------------------


// SDCARD Function
//---------------------------------------------------
SD_Status_t mount_sdcard(void);
SD_Status_t create_txt_file(char *filename);
SD_Status_t save_respons(float value, int time);
//---------------------------------------------------


// Stepper Control Function -------------------------------------------------------------------
void disable_stepper(void);
void enable_stepper(void);
void move_stepper(uint8_t select_joint, uint16_t step, Joint_Dir_t dir, uint16_t input_freq);
//---------------------------------------------------------------------------------------------


// Calculation Function -------------------------------------------------------------------------------
uint32_t calc_exp_step_interval(uint8_t select_joint, uint32_t step);
uint16_t calc_stepper_freq(uint8_t select_joint, float rpm_input);
uint16_t calc_stepper_step(uint8_t select_joint, float angle);
void calc_middle_point(float *first_end, float *second_start, Axis_Offset_t axis_select, float offset);
float calc_cubic_spline(float *x_point, float *y_points, int n, float x);
void calc_spline_parametric(float *t, float points[3][3], int num_points);
void calc_linear_points(float points[2][6], int steps, Linear_Move_Mode_t mode);
void calc_spline_points(float points[3][3], int num_points, int steps);
//-----------------------------------------------------------------------------------------------------


// Main Function -----------------------------------------------------------------------------------------------------------
void move_joint(uint8_t select_joint, float input_angle, Speed_t set_speed);
bool move_all_joint(float *input_angle, Speed_t set_speed);
bool move_world(uint8_t select_axis, float move_pos, Speed_t set_speed);
bool linear_move(float *start_pos, float *end_pos, Speed_t speed);
bool spline_move(float *start_pos, float *mid_pos, float *end_pos, Axis_Offset_t axis_select, float offset, Speed_t speed);
void welder_on(void);
void welder_off(void);
void welding_preview(void);
void welding_start(void);
void update_value(void);
void check_limit(Limit_t check_mode);
bool joint_calibrate(Cal_Mode_t mode, bool j1_cal, bool j2_cal, bool j3_cal, bool j4_cal, bool j5_cal, bool j6_cal);
//-------------------------------------------------------------------------------------------------------------------------


// Encoder Function ------------------------------------------------------------
uint8_t encoder_init(void);
void encoder_stop(void);
void encoder_reset(bool state, Zeroing_Select_t sel_joint, uint8_t current_tim);
//------------------------------------------------------------------------------


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


/*STEPPER TIMER CALLBACK*/
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	for(int i=0; i<JOINT_NUM; i++){
		if(htim->Instance != stepper_tim_instance[i]) continue;
			
		// Cache Variable
		uint16_t cnt = ++timer_counter[i];
		uint16_t t_on_val = t_on[i];
		uint16_t period = step_period[i];
		GPIO_TypeDef *port = stepper_gpio_port[i];
		uint16_t pin = stepper_gpio_pin[i];
		
		// Step Signal
		if(step_counter[i] <= target_step[i] || step_cont_run_j[i] || step_cal_run[i]){
			if(cnt == 1) port->BSRR = (uint32_t)pin;
			else if(cnt == t_on_val) port->BSRR = (uint32_t)pin << 16;
			else if (cnt >= period){
				timer_counter[i] = 0;
				step_counter[i]++;
			}
		}
		
		// Step Reached 
		if(step_counter[i] > target_step[i] && step_cont_run_w[i]){
			step_counter[i] = 0;
			step_reached[i] = true;
		}
		
		// Stop Timer 
		if(((step_counter[i] >= target_step[i]) && step_limit[i]) || (!step_limit[i] && !step_cont_run_j[i] && !step_cont_run_w[i] && !step_cal_run[i]) || joint_limit || robot_stop || end_point_reach){		
			port->BSRR = (uint32_t)pin << 16;
			HAL_TIM_Base_Stop_IT(stepper_tim_handler[i]);
			timer_counter[i] = 0;
			step_counter[i] = 0;
			target_step[i] = 0;
			step_state[i] = STOPPING;
		}
	}
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* INPUT CAPTURE CALLBACK */
//--------------------------------------------------------------------------------------------------------------------------
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	for(int i=0; i<JOINT_NUM; i++){
		if(htim->Instance != enc_tim_instance[i]) continue;
		
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2){
			high_time[i] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
		}
		
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1){
			period_time[i] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		}
		
		if(period_time[i] != 0){
			duty_cycle[i] = high_time[i] * 100.0f / period_time[i];
			pwm_freq[i] = (1000000.0f/period_time[i]);
			enc_angle[i] = (((duty_cycle[i] - min_duty_cycle) * 360.0f) / (max_duty_cycle - min_duty_cycle));
			
			encoder_reset(reset_enc, (Zeroing_Select_t)joint_zero_sel, i);
			
			cal_enc_angle[i] = fmod((enc_angle[i] - cal_value[i] + 360.0f), 360.0f);
			
			reduced_cal_enc_angle[i] = cal_enc_angle[i] / stepper_ratio[i];
			
			if(prev_cal_enc_angle[i] > 355.0f && cal_enc_angle[i] < 5.f) enc_counter[i]++;
			else if(prev_cal_enc_angle[i] < 5.0f && cal_enc_angle[i] > 355.0f) enc_counter[i]--;
			
			prev_cal_enc_angle[i] = cal_enc_angle[i];
			
			raw_joint_angle[i] = reduced_cal_enc_angle[i] + (enc_counter[i] * (360.0f / stepper_ratio[i])) + enc_tollerance[i];
			if(raw_joint_angle[i] >= -0.01f && raw_joint_angle[i] <= 0.01f) raw_joint_angle[i] = 0.00f;
			filtered_joint_angle[i] = FILTER_FACTOR*raw_joint_angle[i] + (1-FILTER_FACTOR) * filtered_joint_angle[i];
		}
	}
}
//-----------------------------------------------------------------------------------------------------------------------


/* EXTI BUTTON CALLBACK */
//----------------------------------------------------------------------------------------------------------
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_PIN){
	for(int i=0; i<JOINT_NUM; i++){
		if(GPIO_PIN == switch_gpio_pin[i]){
			switch_pressed[i] = (HAL_GPIO_ReadPin(switch_gpio_port[i], switch_gpio_pin[i]) == GPIO_PIN_RESET);
		}
	}
}
//----------------------------------------------------------------------------------------------------------

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
  MX_TIM14_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM15_Init();
  MX_TIM16_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_SDMMC1_SD_Init();
  MX_FATFS_Init();
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
	sdcard_status = mount_sdcard();
	if(sdcard_status != SD_OK){
		while(1){
			show_menu(SDCARD_ERROR_MENU);
		}
	}
	#endif
	
	
	/* RS-232 COMMUNICATION SETUP */
	#ifdef USE_RS232
	RS232_Init(&huart4);
	show_menu(COM_INIT_MENU);
//	while(command.type != FEEDBACK && command.feedback != PENDANT_ONLINE){
//		receive_data();
//		Send_feedback(&command, MAIN_ONLINE, 0x00);
//	}
	SSD1306_Clear();
	command.msg_sent = true;
	command.msg_get = true;
	
	global_speed = LOW;
	#endif
	
	
	/* KINEMATICS SETUP */
	#ifdef USE_KINEMATICS
	DHparam_init(&kinematics, joint_d, joint_a, joint_alpha);
	tollframe_init(&kinematics, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
	forward_transform_matrix(&kinematics);
	calculate_all_link(&kinematics);
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
		joint_ang_res[i] = 360.0f / (stepper_microstep[i] * stepper_ratio[i]);
		
		step_state[i] = STOPPING;
	}
	
	step_positive_dir[0] = DIR_CCW;
	step_positive_dir[1] = DIR_CCW;
	step_positive_dir[2] = DIR_CCW;
	step_positive_dir[3] = DIR_CCW;
	step_positive_dir[4] = DIR_CW;
	step_positive_dir[5] = DIR_CCW;
	
	calibration_dir[0] = NEGATIVE_DIR; 	// Calibration pos -> -90 degree
	calibration_dir[1] = NEGATIVE_DIR;	// Calibration pos -> -60 degree
	calibration_dir[2] = POSITIVE_DIR;	// Calibration pos -> 60 Degree
	calibration_dir[3] = NEGATIVE_DIR; 	// Calibration pos -> 0 Degree
	calibration_dir[4] = POSITIVE_DIR; 	// Calibration pos -> -90 Degree
	
	home_dir[0] = POSITIVE_DIR;
	home_dir[1] = POSITIVE_DIR;
	home_dir[2] = NEGATIVE_DIR;
	home_dir[3] = POSITIVE_DIR;
	home_dir[4] = NEGATIVE_DIR;
	#endif
	
	
	/* ENCODER SETUP */
	#ifdef USE_ENCODER
	if(encoder_init() != 0){
		while(1){
			show_menu(ENCODER_ERROR_MENU);
		}
	}
	HAL_Delay(500);
	reset_enc = true;
	HAL_Delay(1000);
	reset_enc = false;
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
	#ifdef USE_LIMIT_CHECK
	for(int i=0; i<JOINT_NUM; i++){
		switch_pressed[i] = (HAL_GPIO_ReadPin(switch_gpio_port[i], switch_gpio_pin[i]) == GPIO_PIN_RESET);
	}
	#endif
	
	
	/* ALL SETUP DONE SHOW MAIN MENU*/
	#ifdef USE_OLED
	show_menu(MAIN_MENU);
	#endif
	
	
	#ifdef TEST_EEPROM 		
//	float test_pos_start[6] = {-40.96, 12.15, -37.42, 12.27, -0.55, 5.72};
//	float test_angle_start[6] = {70.55, -45.5, 90.58, 150.45, 55.17, 178.77};
//	float test_pos_end[6] = {-20.96, 2.15, -37.42, 2.27, -10.55, 15.72};
//	float test_angle_end[6] = {60, -20, 70.25, 115.62, 66.18, 122.76};

//	save_welding_point(0x01, START_POINT, test_pos_start, test_angle_start);
//	save_welding_point(0x01, END_POINT, test_pos_end, test_angle_end);
//	save_welding_pattern(0x01, LINEAR);
//	save_welding_speed(0x01, LOW);
//	save_welding_axis_offset(0x01, OFFSET_ON_Z_AXIS);
	
//	HAL_Delay(500);

//	read_welding_point(0x01, START_POINT, welding_data.array_pos_start, welding_data.array_ang_start);
//	read_welding_point(0x01, END_POINT, welding_data.array_pos_end, welding_data.array_ang_end);
//	read_welding_pattern(0x01);
//	read_welding_speed(0x01);
//	read_welding_axis_offset(0x01);

//	HAL_Delay(500);

	point_status[1] = weldingpoint_validate(1);
//	calculate_parameter = true;
	#endif
	
	#ifdef TEST_SDCARD
	
	#endif
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		#ifdef TEST_RS232 
		receive_data();
		send_ang_pos_data();
		#endif
		
		#ifdef TEST_OLED
		show_menu(MAIN_MENU);
		#endif
		
		#ifdef TEST_STEPPER
//		target_step[0] = calc_stepper_step(0, 0.001);
		#endif
		
		#ifdef TEST_ENCODER
		HAL_GPIO_WritePin(enc_gpio_port[6], enc_gpio_pin[6], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(enc_gpio_port[7], enc_gpio_pin[7], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(enc_gpio_port[8], enc_gpio_pin[8], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(enc_gpio_port[9], enc_gpio_pin[9], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(enc_gpio_port[10], enc_gpio_pin[10], GPIO_PIN_RESET);
		HAL_GPIO_WritePin(enc_gpio_port[11], enc_gpio_pin[11], GPIO_PIN_RESET);
		
		HAL_Delay(2000);
		
		HAL_GPIO_WritePin(enc_gpio_port[6], enc_gpio_pin[6], GPIO_PIN_SET);
		HAL_GPIO_WritePin(enc_gpio_port[7], enc_gpio_pin[7], GPIO_PIN_SET);
		HAL_GPIO_WritePin(enc_gpio_port[8], enc_gpio_pin[8], GPIO_PIN_SET);
		HAL_GPIO_WritePin(enc_gpio_port[9], enc_gpio_pin[9], GPIO_PIN_SET);
		HAL_GPIO_WritePin(enc_gpio_port[10], enc_gpio_pin[10], GPIO_PIN_SET);
		HAL_GPIO_WritePin(enc_gpio_port[11], enc_gpio_pin[11], GPIO_PIN_SET);
		
		HAL_Delay(2000);
		#endif
		
		#ifdef TEST_LIMIT_SWITCH
		for(int i=0; i<JOINT_NUM; i++) switch_pressed[i] = HAL_GPIO_ReadPin(switch_gpio_port[i], switch_gpio_pin[i]);
		#endif
		
		#ifdef TEST_KINEMATICS
//		float dummy_pos[] = {598.5, 0.0, 776.0, 180.0, 0.0, 180.0};
		kinematics.j5_enc_angle = 90.0;
		run_inverse_kinematic(&kinematics, home_pos);
		
//		float dummy_ang[] = {0.0, 19.9, 20.3, -0.2, -40.0, -0.3};
//		kinematics.j5_enc_angle = dummy_ang[4];
//		run_forward_kinematic(&kinematics, dummy_ang);
//		
//		find_jacobian_variable(&kinematics);
//		check_singularity(&kinematics);
		#endif
		
		#ifdef TEST_MOTION
//		linear_move(welding_data.array_pos_start, welding_data.array_pos_end, LOW);

//		joint_rpm = 0.5;

//		target_position[0] = 751.289f;
//		target_position[1] = 100.0f;
//		target_position[2] = 432.197f;
//		target_position[3] = 132.183f;
//		target_position[4] = 90.0f;
//		target_position[5] = 132.183f;
//		
//		// Calculate Inverse Kinematics
//		run_inverse_kinematic(&kinematics, target_position);
//		
//		float dummy_joint_angle[6] = {0.0f, 20.0f, 20.0f, 0.0f, -40.0f, 0.0f};
//		
//		// Calulate Max Delta Angle
//		for(int i=0; i<JOINT_NUM; i++){
//			delta_move_ang[i] = kinematics.joint_ang_out[i] - dummy_joint_angle[i];
//			
//			delta_angle[i] = fabs(delta_move_ang[i]);
//			if(delta_angle[i] > max_delta_angle){
//				max_delta_angle = delta_angle[i];
//			}
//		}
//		
//		// Calculate Each Joint Speed & Run Stepper
//		if(max_delta_angle > 0.1f){	
//			for(int i = 0; i<JOINT_NUM; i++){
//				if(delta_angle[i] > 0.01f){
//					sync_joint_rpm[i] = joint_rpm * delta_angle[i] / max_delta_angle;
//					Joint_Dir_t dir = (delta_move_ang[i] > 0.0f) ? POSITIVE_DIR : NEGATIVE_DIR;
//					move_stepper(robot_joint[i], calc_stepper_step(i, delta_move_ang[i]), dir, calc_stepper_freq(i, sync_joint_rpm[i]));
//				}
//				else{
//					sync_joint_rpm[i] = 0.0f;
//				}
//			}
//		}
		
//		kinematics.j5_enc_angle = 90;
//		linear_move(home_pos, welding_data.array_pos_start, LOW);
		#endif
		
		#ifdef MAIN_PROGRAM	
		// CHECK MISS STEP =======
		#ifdef USE_MISS_STEP_CHECK
		check_miss_step();
		#endif
		
		// CHECK JOINT LIMIT ===========
		#ifdef USE_LIMIT_CHECK
		if(!calibrating){
			check_limit(HARD_LIMIT_CHECK);
		}
		#endif
		
		// UPDATE IK AND FK VALUE
		update_value();
		
		// RECEIVE RS232 DATA
		receive_data();
		
		// CHECK SETTING COMMAND ==============================================================
		if(command.type == SETTING){
			// Joint calibratino command --------------------------------------------------------
			if(command.setting_mode == JOINT_CALIBRATION){
				robot_stop = false;
				if(joint_calibrate(command.calibration_mode, true, true, true, true, true, false) == true){
					for(int i=0; i<50; i++){
						Send_feedback(&command, CALIBRATION_DONE, 0x00);
						HAL_Delay(10);
					}
					HAL_Delay(250);
					home = true;
					command.type = NO_COMMAND;
					calibration_step = COUNT_JOINT;
				}
			}
			
			// Joint zeroing command -----------------------
			else if(command.setting_mode == JOINT_ZEROING){
				reset_enc = true;
				joint_zero_sel = command.joint_zeroing;
				HAL_Delay(500);
				reset_enc = false;
				joint_zero_sel = 0;
				command.joint_zeroing = NO_SELECTION;
				HAL_Delay(500);
				
				for(int i=0; i<50; i++){
					Send_feedback(&command, ZEROING_DONE, 0x00);
					HAL_Delay(10);
				}
			}
			
			// Joint speed setting command ----------------------
			else if(command.setting_mode == JOINT_SPEED){
				global_speed = command.running_speed;
				HAL_Delay(500);
				
				for(int i=0; i<50; i++){
					Send_feedback(&command, SPEED_CHANGE_DONE, 0x00);
					HAL_Delay(10);
				}
			}
		}
		
		// CHECK MAPPING COMMAND =========================================================================
		else if(command.type == MAPPING){
			// Save current position value -----------------------------------------------------------------
			if(command.mapping_state == SAVE_VALUE){
				show_menu(SAVE_MENU);
				
				if(command.data_type == START_POINT){
					save_welding_point(command.welding_point_num, START_POINT, current_position, current_angle);
					HAL_Delay(1000);
				}
				else if(command.data_type == END_POINT){	
					save_welding_point(command.welding_point_num, END_POINT, current_position, current_angle);
					HAL_Delay(1000);
				}
				else if(command.data_type == PATTERN){
					save_welding_pattern(command.welding_point_num, command.pattern_type);
					save_welding_speed(command.welding_point_num, command.running_speed);
					save_welding_axis_offset(command.welding_point_num, command.axis_offset);
				}
			}
			
			// Delete current position value -------------------------------
			else if(command.mapping_state == DELETE_VALUE){
				if(command.data_type == START_POINT){
					show_menu(DELETE_MENU);
					delete_welding_data(command.welding_point_num, START_POINT);
					
					HAL_Delay(1000);
				}
				else if(command.data_type == END_POINT){
					show_menu(DELETE_MENU);
					delete_welding_data(command.welding_point_num, END_POINT);
					
					HAL_Delay(1000);
				}
			}
			command.type = NO_COMMAND;
		}
		
		// CHECK MOVE COMMAND ==============================================================================================================
		else if(command.type == MOVE){
			robot_stop = false;
	
			// World move control ----------------------------------------------------------
			if(command.control_mode == WORLD_CTRL){			
				for(int i=0; i<AXIS_NUM-3; i++){	
					if(command.move_variable == robot_axis[i]){			
						// Continuous mode
						if(command.move_mode == CONTINUOUS){
							for(int j=0; j<JOINT_NUM; j++){
								step_limit[j] = false;
								step_cont_run_w[j] = true;
							}
							if(command.move_sign == UNSIGNED_VAR) rx_move_position[i] = 1.0f;
							else if(command.move_sign == SIGNED_VAR) rx_move_position[i] = -1.0f;
						}
						
						// Distance or Step mode
						else if(command.move_mode == DISTANCE || command.move_mode == STEP){
							for(int j=0; j<JOINT_NUM; j++){
								step_reached[j] = false;
								step_limit[j] = true;
								step_cont_run_j[j] = false;
							}
							rx_move_position[i] = command.move_value;
						}
						
						move_world(command.move_variable, rx_move_position[i], global_speed);
					}
					else rx_move_position[i] = 0.0f;
				}
			}
			
			// Joint move control ------------------------------------------------------------------------------------------------------------
			else if(command.control_mode == JOINT_CTRL){	
				for(int i=0; i<JOINT_NUM; i++){
					if(command.move_variable == robot_joint[i]){
						// Continuous mode
						if(command.move_mode == CONTINUOUS){
							step_limit[i] = false;
							step_cont_run_j[i] = true;
							rx_move_angle[i] = 0.0f;
						}
						
						// Distance or Step mode
						else if(command.move_mode == DISTANCE || command.move_mode == STEP){
							step_limit[i] = true;
							step_cont_run_j[i] = false;
							rx_move_angle[i] = command.move_value;
						}
						
						// Joint one direction movement when limit
						if(positive_limit[i] && command.move_sign == SIGNED_VAR) move_joint(robot_joint[i], rx_move_angle[i], global_speed);						
						else if(negative_limit[i] && command.move_sign == UNSIGNED_VAR) move_joint(robot_joint[i], rx_move_angle[i], global_speed);
						else if(!positive_limit[i] && !negative_limit[i]) move_joint(robot_joint[i], rx_move_angle[i], global_speed);
					}
					else rx_move_angle[i] = 0.0f;
				}
			}
		}
		
		// CHECK RUN COMMAND ================================================================
		else if(command.type == RUN){
			
			// Setting mode ----------------------------
			if(command.running_mode == SETTING_MODE){
				if(command.running_state == RUNNING_STOP){
					for(int i=0; i<JOINT_NUM; i++){
						step_cal_run[i] = false;
					}
					robot_stop = true;
					calibrating = false;
					calibration_step = COUNT_JOINT;
				}
			}
			
			// Setting mode ------------------------------
			else if(command.running_mode == CONTROL_MODE){
				if(command.running_state == RUNNING_STOP){
					robot_stop = true;
				}
			}
			
			// Welding mode ---------------------------------------------------------
			else if(command.running_mode == WELDING_MODE){
				if(command.running_state == RUNNING_START){
					check_welding_point = true;
					get_welding_cmd = true;
				}
				else if(command.running_state == RUNNING_STOP) get_welding_cmd = false;
			}
			  
			// Preview mode ------------------------------------------------------------------
			else if(command.running_mode == PREVIEW_MODE){
				if(command.running_state == RUNNING_START){
					check_welding_point = true;
					get_preview_cmd = true;
				}
				else if(command.running_state == RUNNING_STOP){
					get_preview_cmd = false;
					if(joint_calibrate(HOMING_ONLY, true, true, true, true, true, false) == true){
						for(int i=0; i<50; i++){
							Send_feedback(&command, CALIBRATION_DONE, 0x00);
							HAL_Delay(10);
						}
						command.type = NO_COMMAND;
						calibration_step = COUNT_JOINT;
					}
				}
			}
		}
		
		// CHECK MOTOR STATE COMMAND -------------------------------
		else if(command.type == MOTOR_STATE){
			if(command.motor_state == MOTOR_OFF) disable_stepper();
			else if(command.motor_state == MOTOR_ON) enable_stepper();
		}
		
		// CHECK WELDER STATE COMMAND ---------------------------
		else if(command.type == WELDER_STATE){
			if(command.welder_state == WELDER_OFF) welder_off();
			else if(command.welder_state == WELDER_ON) welder_on();
		}
		
		// CHECK RESET STATE COMMAND --------
		else if(command.type == RESET_STATE){
			for(int i=0; i<JOINT_NUM; i++){
				soft_limit[i] = false;
				hard_limit[i] = false;
			}
			singularity_counter = 0;
			joint_limit = false;
			joint_miss_step = false;
			joint_error_bypass = true;
			kinematics.singularity = false;
		}
		
		// CHECK NONE COMMAND --------------
		else if(command.type == NO_COMMAND){
			for(int i=0; i<JOINT_NUM; i++){
				step_cont_run_j[i] = false;
				step_cont_run_w[i] = false;
				rx_move_position[i] = 0.0f;
				rx_move_angle[i] = 0.0f;
			}
		}
		
		// RUN WELDING -----
		if(get_welding_cmd){
			
		}
		
		// RUN PREVIEW -----
		if(get_preview_cmd){
			welding_preview();
		}
		
		// SEND ALL DATA ---
		send_ang_pos_data();
		
		// SHOW MENU --------
		show_menu(MAIN_MENU);
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
  sSlaveConfig.TriggerFilter = 15;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
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
  sSlaveConfig.TriggerFilter = 15;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
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
  sSlaveConfig.TriggerFilter = 15;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
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
  sSlaveConfig.TriggerFilter = 15;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
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
  sSlaveConfig.TriggerFilter = 15;
  if (HAL_TIM_SlaveConfigSynchro(&htim5, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
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
  htim8.Init.Prescaler = 99;
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
  sSlaveConfig.TriggerFilter = 15;
  if (HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 15;
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
  htim12.Init.Period = 999;
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
  htim13.Init.Period = 999;
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
  htim14.Init.Period = 999;
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
  htim15.Init.Period = 999;
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
  htim16.Init.Period = 999;
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
  htim17.Init.Period = 999;
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
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huart4.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
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

  /*Configure GPIO pin : SDMMC1_BSP_Pin */
  GPIO_InitStruct.Pin = SDMMC1_BSP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDMMC1_BSP_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 11, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 11, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 11, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 11, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 11, 0);
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
void save_welding_point(uint16_t point, uint8_t point_type, float* pos_data, float*angle_data){
	uint8_t chcksum_H, chcksum_L;
	uint16_t chcksum_bytes;
	
	// Save World Position & Joint Angle Value
	for(int i=0; i<24; i++){
		saved_pos[i] = 0;
		saved_angle[i] = 0;
	}
	for(size_t i=0; i<JOINT_NUM; i++){
		memcpy(&saved_pos[i*4], &pos_data[i], sizeof(float));
		memcpy(&saved_angle[i*4], &angle_data[i], sizeof(float));
	}
	
	// Calculate Checksum
	chcksum_bytes = 0;
	for(int i=0; i<24; i++){
		chcksum_bytes += (saved_pos[i] + saved_angle[i]);
	}
	
	chcksum_H = (uint8_t)((chcksum_bytes >> 8) & 0xFF);
	chcksum_L = (uint8_t)(chcksum_bytes & 0xFF);
	
	if(point_type == START_POINT){
		// Save World Position & Joint Angle Value
		EEPROM_PageWrite(&eeprom1, point-1, POS_BYTES_ADDR, saved_pos, sizeof(saved_pos));
		HAL_Delay(2);	
		EEPROM_PageWrite(&eeprom1, point-1, ANG_BYTES_ADDR, saved_angle, sizeof(saved_angle));
		HAL_Delay(2);
		
		// Save Calculated Checksum Value
		EEPROM_ByteWrite(&eeprom1, point-1, CHECKSUM_H_ADDR, chcksum_H, 1);
		HAL_Delay(2);
		EEPROM_ByteWrite(&eeprom1, point-1, CHECKSUM_L_ADDR, chcksum_L, 1);
		HAL_Delay(2);
	}
	
	else if(point_type == END_POINT){
		// Save World Position & Joint Angle Value
		EEPROM_PageWrite(&eeprom2, point-1, POS_BYTES_ADDR, saved_pos, sizeof(saved_pos));
		HAL_Delay(2);	
		EEPROM_PageWrite(&eeprom2, point-1, ANG_BYTES_ADDR, saved_angle, sizeof(saved_angle));
		HAL_Delay(2);
		
		// Save Calculated Checksum Value
		EEPROM_ByteWrite(&eeprom2, point-1, CHECKSUM_H_ADDR, chcksum_H, 1);
		HAL_Delay(2);
		EEPROM_ByteWrite(&eeprom2, point-1, CHECKSUM_L_ADDR, chcksum_L, 1);
		HAL_Delay(2);
	}
}

void read_welding_point(uint16_t point, uint8_t point_type, float* stored_pos, float*stored_angle){
	uint8_t
	read_chcksum_H[1], 
	read_chcksum_L[1];
	
	// Read World Position & Joint Angle Value
	for(int i=0; i<24; i++){
		read_saved_pos[i] = 0;
		read_saved_angle[i] = 0;
	}
	for(size_t i=0; i<JOINT_NUM; i++){
		stored_pos[i] = 0;
		stored_angle[i] = 0;
	}
	
	if(point_type == START_POINT){
		// Read World Position & Joint Angle Value
		EEPROM_PageRead(&eeprom1, point-1, POS_BYTES_ADDR, read_saved_pos, sizeof(read_saved_pos));
		HAL_Delay(5);
		EEPROM_PageRead(&eeprom1, point-1, ANG_BYTES_ADDR, read_saved_angle, sizeof(read_saved_angle));
		HAL_Delay(5);
		
		// Read Calculated Checksum Value
		EEPROM_ByteRead(&eeprom1, point-1, CHECKSUM_H_ADDR, read_chcksum_H, 1);
		HAL_Delay(5);
		EEPROM_ByteRead(&eeprom1, point-1, CHECKSUM_L_ADDR, read_chcksum_L, 1);
		HAL_Delay(5);
		
		chcksum_sp_validation = (read_chcksum_H[0] << 8) | read_chcksum_L[0];	
		chcksum_sp = 0;
		
		// Calculate Saved World Position & Angle Joint Value Checksum
		for(int i=0; i<24; i++){
			chcksum_sp += (read_saved_pos[i] + read_saved_angle[i]);
		}
	}
	
	else if(point_type == END_POINT){
		// Read World Position & Joint Angle Value
		EEPROM_PageRead(&eeprom2, point-1, POS_BYTES_ADDR, read_saved_pos, sizeof(read_saved_pos));
		HAL_Delay(5);
		EEPROM_PageRead(&eeprom2, point-1, ANG_BYTES_ADDR, read_saved_angle, sizeof(read_saved_angle));
		HAL_Delay(5);
		
		// Read Calculated Checksum Value
		EEPROM_ByteRead(&eeprom2, point-1, CHECKSUM_H_ADDR, read_chcksum_H, 1);
		HAL_Delay(5);
		EEPROM_ByteRead(&eeprom2, point-1, CHECKSUM_L_ADDR, read_chcksum_L, 1);
		HAL_Delay(5);
		
		chcksum_ep_validation = (read_chcksum_H[0] << 8) | read_chcksum_L[0];
		chcksum_ep = 0;
		
		// Calculate Saved World Position & Angle Joint Value Checksum
		for(int i=0; i<24; i++){
			chcksum_ep += (read_saved_pos[i] + read_saved_angle[i]);
		}
	}
	
	// Process World Position & Joint Angle Value
	#ifdef USE_DEBUG_PROGRAM
	for(int i=0; i<8; i++){
		read_saved_posX[i] = read_saved_pos[i];
		read_saved_posY[i] = read_saved_pos[i+4];
		read_saved_posZ[i] = read_saved_pos[i+8];
		read_saved_rotX[i] = read_saved_pos[i+12];
		read_saved_rotY[i] = read_saved_pos[i+16];
		read_saved_rotZ[i] = read_saved_pos[i+20];
		
		read_saved_angle1[i] = read_saved_angle[i];
		read_saved_angle2[i] = read_saved_angle[i+4];
		read_saved_angle3[i] = read_saved_angle[i+8];
		read_saved_angle4[i] = read_saved_angle[i+12];
		read_saved_angle5[i] = read_saved_angle[i+16];
		read_saved_angle6[i] = read_saved_angle[i+20];
	}
	
	memcpy(&stored_pos[0], read_saved_posX, sizeof(float));
	memcpy(&stored_pos[1], read_saved_posY, sizeof(float));
	memcpy(&stored_pos[2], read_saved_posZ, sizeof(float));
	memcpy(&stored_pos[3], read_saved_rotX, sizeof(float));
	memcpy(&stored_pos[4], read_saved_rotY, sizeof(float));
	memcpy(&stored_pos[5], read_saved_rotZ, sizeof(float));
	
	memcpy(&stored_angle[0], read_saved_angle1, sizeof(float));
	memcpy(&stored_angle[1], read_saved_angle2, sizeof(float));
	memcpy(&stored_angle[2], read_saved_angle3, sizeof(float));
	memcpy(&stored_angle[3], read_saved_angle4, sizeof(float));
	memcpy(&stored_angle[4], read_saved_angle5, sizeof(float));
	memcpy(&stored_angle[5], read_saved_angle6, sizeof(float));
	#else
	memcpy(&stored_pos[0], &read_saved_pos[0], sizeof(float));
	memcpy(&stored_pos[1], &read_saved_pos[4], sizeof(float));
	memcpy(&stored_pos[2], &read_saved_pos[8], sizeof(float));
	memcpy(&stored_pos[3], &read_saved_pos[12], sizeof(float));
	memcpy(&stored_pos[4], &read_saved_pos[16], sizeof(float));
	memcpy(&stored_pos[5], &read_saved_pos[20], sizeof(float));
	
	memcpy(&stored_angle[0], &read_saved_angle[0], sizeof(float));
	memcpy(&stored_angle[1], &read_saved_angle[4], sizeof(float));
	memcpy(&stored_angle[2], &read_saved_angle[8], sizeof(float));
	memcpy(&stored_angle[3], &read_saved_angle[12], sizeof(float));
	memcpy(&stored_angle[4], &read_saved_angle[16], sizeof(float));
	memcpy(&stored_angle[5], &read_saved_angle[20], sizeof(float));
	#endif
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- SAVE WELDING POINT PATTERN ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void save_welding_pattern(uint16_t point, uint8_t select_pattern){
	EEPROM_ByteWrite(&eeprom2, point-1, PATTERN_BYTE_ADDR, select_pattern, sizeof(select_pattern));
	HAL_Delay(5);
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- READ WELDING POINT PATTERN ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void read_welding_pattern(uint16_t point){
	uint8_t read_pattern[1];
	EEPROM_ByteRead(&eeprom2, point-1, PATTERN_BYTE_ADDR, read_pattern, sizeof(read_pattern));
	welding_data.welding_pattern = (Welding_Pattern_t)read_pattern[0];
	HAL_Delay(5);
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- WRITE WELDING POINT SPEED ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void save_welding_speed(uint16_t point, uint8_t select_speed){
	EEPROM_ByteWrite(&eeprom2, point-1, SPEED_BYTE_ADDR, select_speed, sizeof(select_speed));
	HAL_Delay(5);
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- READ WELDING POINT SPEED ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void read_welding_speed(uint16_t point){
	uint8_t read_speed[1];
	EEPROM_ByteRead(&eeprom2, point-1, SPEED_BYTE_ADDR, read_speed, sizeof(read_speed));
	welding_data.welding_speed = (Speed_t)read_speed[0];
	HAL_Delay(5);
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- SAVE WELDING SPLINE AXIS OFFSET ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void save_welding_axis_offset(uint16_t point, uint8_t select_axis){
	EEPROM_ByteWrite(&eeprom2, point-1, OFFSET_BYTE_ADDR, select_axis, sizeof(select_axis));
	HAL_Delay(5);
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- READ WELDING SPLINE AXIS OFFSET ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void read_welding_axis_offset(uint16_t point){
	uint8_t read_axis[1];
	EEPROM_ByteRead(&eeprom2, point-1, OFFSET_BYTE_ADDR, read_axis, sizeof(read_axis));
	welding_data.welding_axis_offset = (Axis_Offset_t)read_axis[0];
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
	read_welding_point(welding_point, START_POINT, welding_data.array_pos_start, welding_data.array_ang_start);
	read_welding_point(welding_point, END_POINT, welding_data.array_pos_end, welding_data.array_ang_end);
	read_welding_pattern(welding_point);
	read_welding_speed(welding_point);
	read_welding_axis_offset(welding_point);
	
	// Data Validation
	if(chcksum_sp_validation != 0 && chcksum_ep_validation != 0){
		if(chcksum_sp == chcksum_sp_validation && chcksum_ep == chcksum_ep_validation && welding_data.welding_pattern > 0x00 && welding_data.welding_speed > 0x00){
			return WELDING_DATA_VALID;
		}
		else if(chcksum_sp != chcksum_sp_validation || chcksum_ep != chcksum_ep_validation || welding_data.welding_pattern == 0x00 || welding_data.welding_speed == 0x00){
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


/*--- SDCARD SAVE RESPONS DATA FUNCTION ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
SD_Status_t save_respons(float value, int time){
	char text[64];
	
	res = f_open(&file, file1, FA_CREATE_ALWAYS | FA_WRITE);
	if(res == FR_OK){
		snprintf(text, sizeof(text), "Float: %.3f, Int: %d\r\n", value, time);
		
		res = f_write(&file, text, strlen(text), &bw);
		if(res != FR_OK || bw == 0){
			return SD_WRITE_ERR;
		}
		
		f_close(&file);
		return SD_OK;
	}
	else return SD_OPEN_ERR;
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
			step_period[i] = 100000/step_freq[i];
			
			// Set Stepper Direction
			if(dir == POSITIVE_DIR){
				if(step_positive_dir[i] == DIR_CW) HAL_GPIO_WritePin(stepper_gpio_port[i+6], stepper_gpio_pin[i+6], GPIO_PIN_SET);
				else HAL_GPIO_WritePin(stepper_gpio_port[i+6], stepper_gpio_pin[i+6], GPIO_PIN_RESET);
			}
			else if(dir == NEGATIVE_DIR){
				if(step_positive_dir[i] == DIR_CW) HAL_GPIO_WritePin(stepper_gpio_port[i+6], stepper_gpio_pin[i+6], GPIO_PIN_RESET);
				else HAL_GPIO_WritePin(stepper_gpio_port[i+6], stepper_gpio_pin[i+6], GPIO_PIN_SET);
			}
			
			// Pulse Start
			target_step[i] = step;
			if(target_step[i] > 0 || step_cont_run_j[i] || step_cal_run[i]){
				t_on[i] = step_period[i] * SET_DUTY_CYCLE;
				t_off[i] = step_period[i] - t_on[i];
				
				if(step_state[i] == STOPPING){
					step_state[i] = AT_TOP_SPEED;
					HAL_TIM_Base_Start_IT(stepper_tim_handler[i]);
				}
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
uint16_t calc_stepper_step(uint8_t select_joint, float angle){
	step_count[select_joint] = abs((int)(angle / joint_ang_res[select_joint]));
	return step_count[select_joint];
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- SPLINE MIDDLE POINT CALCULATION FUNCTION --- */
//----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void calc_middle_point(float *first_end, float *second_start, Axis_Offset_t axis_select, float offset){
	float delta_value[3];
	
	for(int i=0; i<3; i++){
		middle_point[i] = 0.0f;
		delta_value[i] = second_start[i] - first_end[i];
		middle_point[i] = (delta_value[i] / 2) + first_end[i];
	}
	
	if(axis_select == OFFSET_ON_X_AXIS) middle_point[0] += offset;
	else if(axis_select == OFFSET_ON_Y_AXIS) middle_point[1] += offset;
	else if(axis_select == OFFSET_ON_Z_AXIS) middle_point[2] += offset;
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- CUBIC SPLINE CALCULATION */
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
float calc_cubic_spline(float *x_points, float *y_points, int n, float x){
	float h[MAX_UPDATE_STEP], A[MAX_UPDATE_STEP], B[MAX_UPDATE_STEP], M[MAX_UPDATE_STEP];
	
	for(int i=0; i<n; i++){
		h[i] = x_points[i+1] - x_points[i];
	}
	
	A[0] = 0.0f; 	// Reset Nilai
	A[n] = 0.0f;	// Reset Nilai
	
	for(int i=0; i<n; i++){
		A[i] = 2.0f * (h[i-1] + h[i]);
	}
	
	for(int i=0; i<=n; i++) B[i] = 0.0f; // Reset Nilai
	for(int i=0; i<n; i++){
			B[i] = (6.0f / h[i]) * (y_points[i+1] - y_points[i]) - (6.0f / h[i-1]) * (y_points[i] - y_points[i-1]);
	}
	
	// Eliminasi Maju Menggunakan Algoritma Thomas
	for(int i=1; i<n; i++){
		float temp = h[i-1] / A[i];
		A[i+1] -= temp * h[i-1];
		B[i+1] -= temp * B[i];
	}
	
	// Subtitusi Mundur
	for(int i=0; i<=n ; i++) M[i] = 0.0f; // Reset Nilai
	for(int i=n-1; i>0; i++){
		M[i] = (B[i] - h[i] * M[i+1]) / A[i];
	}
	
	// Mencari Interval
	for(int i=0; i<n; i++){
		if(x_points[i] <= x && x <= x_points[i+1]){
			float dx = x - x_points[i];
			float a = (M[i+1] - M[i]) / (6.0f * h[i]);
			float b = M[i] / 2.0f;
			float c = (y_points[i+1] - y_points[i]) / h[i] - (M[i+1] + 2.0f * M[i]) * h[i] / 6.0f;
			float d = y_points[i];
			
			return a*pow(dx, 3) + b*pow(dx, 2) + c*dx + d;
		}
	}
	
	return 0.0f;
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- CALCULATE SPLINE PARAMETRIC --- */
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void calc_spline_parametric(float *t, float points[3][3], int num_points){
	 t[0] = 0.0f;
	for(int i=1; i<num_points; i++){
		float dx = points[i][0] - points[i-1][0];
		float dy = points[i][1] - points[i-1][1];
		float dz = points[i][2] - points[i-1][2];
		float dist = sqrtf(dx * dx + dy * dy + dz * dz);
		t[i] = t[i-1] + dist;
	}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- LINEAR MOVE POINTS CALCULATION --- */
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void calc_linear_points(float points[2][6], int steps, Linear_Move_Mode_t mode){
	if(mode == ALL_AXES){
		for(int i=0; i<AXIS_NUM; i++){
			axis_delta[i] = (points[1][i] - points[0][i]) / steps;
		}
		
		for(int i=0; i<=steps; i++){
			int_x_out[i] = points[0][0] + i*axis_delta[0];
			int_y_out[i] = points[0][1] + i*axis_delta[1];
			int_z_out[i] = points[0][2] + i*axis_delta[2];
			
			int_rx_out[i] = points[0][3] + i*axis_delta[3];
			int_ry_out[i] = points[0][4] + i*axis_delta[4];
			int_rz_out[i] = points[0][5] + i*axis_delta[5];
		}
	}
	
	else if(mode == POS_AXES){
		for(int i=0; i<AXIS_NUM-3; i++){
			axis_delta[i] = (points[1][i] - points[0][i]) / steps;
		}
		
		for(int i=0; i<steps+1; i++){
			int_x_out[i] = points[0][0] + i*axis_delta[0];
			int_y_out[i] = points[0][1] + i*axis_delta[1];
			int_z_out[i] = points[0][2] + i*axis_delta[2];
		}
	}
	
	else if(mode == ROT_AXES){
		for(int i=3; i<AXIS_NUM; i++){
			axis_delta[i] = (points[1][i] - points[0][i]) / steps;
		}
		
		for(int i=0; i<steps+1; i++){
			int_rx_out[i] = points[0][3] + i*axis_delta[3];
			int_ry_out[i] = points[0][4] + i*axis_delta[4];
			int_rz_out[i] = points[0][5] + i*axis_delta[5];
		}
	}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* SPLINE MOVE POINTS CALCULATION */
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void calc_spline_points(float points[3][3], int num_points, int steps){
	float t[MAX_UPDATE_STEP];
	float xs[MAX_UPDATE_STEP], ys[MAX_UPDATE_STEP], zs[MAX_UPDATE_STEP];
	
	calc_spline_parametric(t, points, num_points);
	
	for(int i=0; i<num_points; i++){
		xs[i] = points[i][0];
		ys[i] = points[i][1];
		zs[i] = points[i][2];
	}
	
	float t_min = t[0];
	float t_max = t[num_points-1];
	int total_steps = steps * (num_points-1);
	
	for(int i=0; i<=total_steps; i++){
		float ti = t_min + (float)i * (t_max - t_min) / total_steps;
		int_x_out[i] = calc_cubic_spline(t, xs, num_points - 1, ti);
		int_y_out[i] = calc_cubic_spline(t, ys, num_points - 1, ti);
		int_z_out[i] = calc_cubic_spline(t, zs, num_points - 1, ti);
	}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- JOINT BASE MOVE FUNCTION --- */
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void move_joint(uint8_t select_joint, float input_angle, Speed_t set_speed){
	// Joint RPM Set
	if(set_speed == LOW) joint_rpm = 0.5f;
	else if(set_speed == MED) joint_rpm = 0.75f;
	else if(set_speed == HIGH) joint_rpm = 1.0f;
	
	if((input_angle > 0 && input_angle < 1) || (input_angle > -1 && input_angle < 0)){
		joint_rpm = 0.25f;
	}
	
	for(int i=0; i<JOINT_NUM; i++){
		if(select_joint != robot_joint[i]) continue;
		else{						
			// Step or Distance Move			
			if(input_angle > 0){
				move_stepper(robot_joint[i], calc_stepper_step(i, input_angle), POSITIVE_DIR, calc_stepper_freq(i, joint_rpm));
			}
			else if(input_angle < 0){
				move_stepper(robot_joint[i], calc_stepper_step(i, input_angle), NEGATIVE_DIR, calc_stepper_freq(i, joint_rpm));
			}
			
			// Continuous Move
			if(step_cont_run_j[i] == true){
				if(command.move_sign == UNSIGNED_VAR) move_stepper(robot_joint[i], 0, POSITIVE_DIR, calc_stepper_freq(i, joint_rpm));
				else move_stepper(robot_joint[i], 0, NEGATIVE_DIR, calc_stepper_freq(i, joint_rpm));
			}
		}
	}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- MOVE ALL JOINT FUNCTION --- */
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool move_all_joint(float *input_angle, Speed_t set_speed){
	bool all_step_reached = true;
	float max_delta = 0.0f;
	
	// Set Minimum Joint Speed
	if(set_speed == LOW) joint_rpm = 0.5f;
	else if(set_speed == MED) joint_rpm = 0.75f;
	else if(set_speed == HIGH) joint_rpm = 1.0f;
	
	// Calulate Max Delta Angle
	for(int i=0; i<JOINT_NUM; i++){
		if(step_counter[i] == 0){
			delta_move_ang[i] = input_angle[i] - filtered_joint_angle[i];
			
			delta_angle[i] = fabs(delta_move_ang[i]);
			if(delta_angle[i] > max_delta){
				max_delta = delta_angle[i];
			}
		}
		else continue;
	}
	
	// Check Step Reached To Update Point Counter
	if(!step_reached[0] || !step_reached[1] || !step_reached[2] || !step_reached[3] || !step_reached[4] || !step_reached[5]){
		all_step_reached = false;
	}
	
	if(all_step_reached) return true;
	
	// Calculate Each Joint Speed & Run Stepper
	max_delta_angle = max_delta;
	if(max_delta > 0.02){					
		for(int i=0; i<JOINT_NUM; i++){
			if(delta_angle[i] >= joint_ang_res[i] && step_counter[i] == 0){
				step_reached[i] = false;
				sync_joint_rpm[i] = joint_rpm * (delta_angle[i] / max_delta);
				Joint_Dir_t dir = (delta_move_ang[i] > 0.0f) ? POSITIVE_DIR : NEGATIVE_DIR;
				move_stepper(robot_joint[i], calc_stepper_step(i, delta_move_ang[i]), dir, calc_stepper_freq(i, sync_joint_rpm[i]));
			}
			else continue;
		}
	}
	
	return false;
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- WORLD BASE MOVE FUNCTION --- */
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool move_world(uint8_t select_axis, float move_pos, Speed_t set_speed){
	bool all_step_stopped = true;
	float max_delta = 0.0f;
	
	// Set Minimum Joint Speed
	if(set_speed == LOW) joint_rpm = 0.5f;
	else if(set_speed == MED) joint_rpm = 0.75f;
	else if(set_speed == HIGH) joint_rpm = 1.0f;
	
	// Move World Continuous Mode
	if(command.move_mode == CONTINUOUS){
		// Check Target Position
		for(int i=0; i<AXIS_NUM; i++){
			if(select_axis == robot_axis[i]){
				target_position[i] = current_position[i] + move_pos;
			}
			else target_position[i] = current_position[i];
		}
		
		// Calculate Inverse Kinematics
		run_inverse_kinematic(&kinematics, target_position);
		
		// Check Position Singularity
		#ifdef USE_LIMIT_CHECK
		run_forward_kinematic(&kinematics, kinematics.joint_ang_out);
		check_limit(SOFT_LIMIT_CHECK);
		#endif
		
		if(!kinematics.singularity){
			if(move_all_joint(kinematics.joint_ang_out, set_speed) == true) return true;
		}
	}
	
	// Move World Distance Or Step Mode
	else if(command.move_mode == DISTANCE || command.move_mode == STEP){
		// Check if all joints are stopped
		if(step_state[0] != 0x01 || step_state[1] != 0x01 || step_state[2] != 0x01 || step_state[3] != 0x01 || step_state[4] != 0x01 || step_state[5] != 0x01){
			all_step_stopped = false;
		}
		
		if(all_step_stopped){				
			// Check Target Position
			for(int i=0; i<AXIS_NUM; i++){
				if(select_axis == robot_axis[i]){
					target_position[i] = current_position[i] + move_pos;
				}
				else target_position[i] = current_position[i];
			}
			
			// Calculate Inverse Kinematics
			run_inverse_kinematic(&kinematics, target_position);
			
			// Check Position Singularity
			#ifdef USE_LIMIT_CHECK
			run_forward_kinematic(&kinematics, kinematics.joint_ang_out);
			check_limit(SOFT_LIMIT_CHECK);
			#endif
			
			if(!kinematics.singularity){
				if(move_all_joint(kinematics.joint_ang_out, set_speed) == true) return true;
			}
		}
	}
	
	return false;
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- LINEAR MOVE FUNCTION ---- */
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool linear_move(float *start_pos, float *end_pos, Speed_t speed){
	uint8_t steps = (speed == LOW) ? 20 :	
									(speed == MED) ? 15 : 10;
	
	for(int i=0; i<AXIS_NUM; i++){
		input_points[0][i] = start_pos[i];
		input_points[1][i] = end_pos[i];
		step_cont_run_w[i] = true;
	}
	
	if(calculate_parameter){
		calc_linear_points(input_points, steps, ALL_AXES);
		calculate_parameter = false;
	}
	
	if(current_point < steps){
		int_axes_out[0] = int_x_out[current_point+1];
		int_axes_out[1] = int_y_out[current_point+1];
		int_axes_out[2] = int_z_out[current_point+1];
		int_axes_out[3] = int_rx_out[current_point+1];
		int_axes_out[4] = int_ry_out[current_point+1];
		int_axes_out[5] = int_rz_out[current_point+1];
		
		run_inverse_kinematic(&kinematics, int_axes_out);
		end_point_reach = false;
		
		if(move_all_joint(kinematics.joint_ang_out, speed) == true){
			current_point++;
		}
	}
	else{
		end_point_reach = true;
		current_point = 0;
		for(int i=0; i<JOINT_NUM; i++){
			step_cont_run_w[i] = false;
			HAL_TIM_Base_Stop_IT(stepper_tim_handler[i]);
			timer_counter[i] = 0;
			step_counter[i] = 0;
			step_state[i] = STOPPING;
		}
		return true;
	}
	
	return false;
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- SPLINE MOVE FUNCTION ---- */
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool spline_move(float *start_pos, float *mid_pos, float *end_pos, Axis_Offset_t axis_select, float offset, Speed_t speed){
	uint8_t steps = (speed == LOW) ? 50 :	
									(speed == MED) ? 30 : 10;
	
	static float rot_points[2][6];
	for(int i=0; i<AXIS_NUM; i++){
		rot_points[0][i] = start_pos[i];
		rot_points[1][i] = end_pos[i];
	}
	
	calc_middle_point(start_pos, end_pos, axis_select, offset);
	calc_linear_points(rot_points, steps, ROT_AXES);
	
	static float points[3][3];
	for(int i=0; i<AXIS_NUM; i++){
		points[0][i] = start_pos[i];
		points[1][i] = middle_point[i];
		points[2][i] = end_pos[i];
	}
	
	calc_spline_points(points, 3, steps);
	
	if(current_point <= steps){
		int_axes_out[0] = int_x_out[current_point];
		int_axes_out[1] = int_y_out[current_point];
		int_axes_out[2] = int_z_out[current_point];
		int_axes_out[3] = int_rx_out[current_point];
		int_axes_out[4] = int_ry_out[current_point];
		int_axes_out[5] = int_rz_out[current_point];
		
		run_inverse_kinematic(&kinematics, int_axes_out);
		end_point_reach = false;
		
		if(move_all_joint(kinematics.joint_ang_out, speed) == true){
			for(int i=0; i<JOINT_NUM; i++) step_counter[i] = 0;
			current_point++;
		}
	}
	else{
		end_point_reach = true;
		current_point = 0;
		return true;
	}
	
	return false;
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
	// Check Welding Point
	if(check_welding_point){
		point_status[command.preview_point_num] = weldingpoint_validate(command.preview_point_num);
		if(point_status[command.preview_point_num] == WELDING_DATA_VALID) total_valid_points = 1;
		
		if(total_valid_points > 0){
			running_step = MOVE_TO_HOME;
			preview_run = true;
		}
		else{
			preview_run = false;
			get_preview_cmd = false;
			for(int i=0; i<10; i++){
				Send_feedback(&command, POINT_INVALID, invalid_welding_point);
			}
		}
		check_welding_point = false;
	}
	
	// Run Preview
	if(preview_run){
		// Move to home
		if(running_step == MOVE_TO_HOME){
			if(joint_calibrate(HOMING_ONLY, true, true, true, true, true, false) == true){
				HAL_Delay(1000);
				calculate_parameter = true;
				running_step = MOVE_TO_START_POINT;
			}
		}
		
		// Move to start point
		if(running_step == MOVE_TO_START_POINT){
			if(linear_move(current_position, welding_data.array_pos_start, LOW) == true){
				running_step = MOVE_TO_END_POINT;
			}
		}
		
		// From start point move to end point
		else if(running_step == MOVE_TO_END_POINT){
			if(welding_data.welding_pattern == DOT){
				// Do Nothing
			}
			else if(welding_data.welding_pattern == LINEAR){
				if(linear_move(welding_data.array_ang_start, welding_data.array_pos_end, LOW) == true){
					running_step = MOVE_STOP;
				}
			}
		}
		
		// Move stop
		else if(running_step == MOVE_STOP){
			if(linear_move(welding_data.array_pos_end, home_pos, LOW) == true){
				preview_run = false;
				get_preview_cmd = false;
				running_step = NO_STEP;
			}
		}
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
		
	}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- UPDATE JOINT ANGLE AND POSITION VALUE FUNCTION ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void update_value(void){
	if(!calibrating){
		run_forward_kinematic(&kinematics, filtered_joint_angle);
		kinematics.j5_enc_angle = filtered_joint_angle[4];
	}
	
	for(int i=0; i<JOINT_NUM; i++){
		if(i<3) current_position[i] = kinematics.axis_pos_out[i];
		else current_position[i] = kinematics.axis_rot_out[i-3];
		
		#ifdef USE_ENCODER
		current_angle[i] = filtered_joint_angle[i];
		#endif
	}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- LIMIT SWITCH STATE CHECK FUNCTION ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void check_limit(Limit_t check_mode){
	// Software Limit
	if(check_mode == SOFT_LIMIT_CHECK){
		find_jacobian_variable(&kinematics);
		check_singularity(&kinematics);
		
		if(joint_error_bypass == false && kinematics.singularity == true){
			robot_stop = true;
			for(int i=0; i<10; i++){
				Send_feedback(&command, ANGLE_SOFT_LIMIT, 0x0000);
			}
		}
		
		if(joint_error_bypass == true && kinematics.singularity == false){
			joint_error_bypass = false;
		}
	}
	
	// Hardware Limit
	else if(check_mode == HARD_LIMIT_CHECK){
		limit_bit_sum = 0;
		
		for(int i=0; i<JOINT_NUM; i++){
			if(filtered_joint_angle[i] > joint_negative_axisLim[i] && filtered_joint_angle[i] < joint_positive_axisLim[i]){
				hard_limit[i] = false;
			}
			else{
				if(joint_error_bypass == false){
					hard_limit[i] = true;
					limit_bit_sum += joint_limit_bit[i];
					
					for(int i=0; i<10; i++){
						Send_feedback(&command, ANGLE_HARD_LIMIT, limit_bit_sum);
					}
					if(filtered_joint_angle[i] <= joint_negative_axisLim[i]) negative_limit[i] = true;
					else if(filtered_joint_angle[i] >= joint_positive_axisLim[i]) positive_limit[i] = true;
				}
			}
			
			if(negative_limit[i] == true && filtered_joint_angle[i] > joint_negative_axisLim[i]){
				negative_limit[i] = false;
				joint_error_bypass = false;
			}
			else if(positive_limit[i] == true && filtered_joint_angle[i] < joint_positive_axisLim[i]){
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
	bool 
	selected_group1[3] = {j1_cal, j2_cal, j3_cal},
	selected_group2[3] = {j4_cal, j5_cal, j6_cal};

	calibrating = true;
	for(int i=0; i<JOINT_NUM; i++) step_limit[i] = false;
	
	if(mode == CALIBRATE_ONLY || mode == CALIBRATE_HOMING){
		home = false;
		
		// Count Calibrated Joint In Group 1 And 2
		if(calibration_step == COUNT_JOINT){
			group1_count = 0;
			group2_count = 0;
			
			for(int i=0; i<3; i++){
				if(selected_group1[i]){
					calibrated_joint[i] = switch_pressed[i];
					group1_count++;
				}
				
				if(selected_group2[i]){
					calibrated_joint[i+3] = switch_pressed[i+3];
					group2_count++;
				}
			}
			
			if(group1_count > 0) calibration_step = J1_J3_CAL;
			else calibration_step = J4_J6_CAL;
		}
		
		// Move First 3 Joints Until Each Limit Switch Pressed
		else if(calibration_step == J1_J3_CAL){
			for(int i=0; i<3; i++){
				if(!selected_group1[i]) continue;
				
				if(!calibrated_joint[i]){				
					if(!switch_pressed[i]){
						step_cal_run[i] = true;
						move_stepper(robot_joint[i], 0, calibration_dir[i], calc_stepper_freq(i, 0.5f));
					}
					else{
						step_cal_run[i] = false;
						calibrated_joint[i] = true;
					}
				}
			}
			group1_calibrated = calibrated_joint[0] + calibrated_joint[1] + calibrated_joint[2];
			if(group1_calibrated == group1_count){
				HAL_Delay(500);
				reset_enc = true;
				HAL_Delay(500);
				reset_enc = false;
				calibration_step = J1_J3_OFFSET_MOVE;
			}
		}
		
		// Move First 3 Joints To Offset Angle
		else if(calibration_step == J1_J3_OFFSET_MOVE){
			float offset_angle[3] = {10.0, 10.0, -10.0};
			
			for(int i=0; i<2; i++){
				if(filtered_joint_angle[i] < offset_angle[i]){
					step_cal_run[i] = true;
					move_stepper(robot_joint[i], 0, home_dir[i], calc_stepper_freq(i, 0.5f));
				}
				else{
					calibrated_joint[i] = false;
					step_cal_run[i] = false;
				}
			}
			
			if(filtered_joint_angle[2] > offset_angle[2]){
				step_cal_run[2] = true;
				move_stepper(robot_joint[2], 0, home_dir[2], calc_stepper_freq(2, 0.5f));
			}
			else{
				step_cal_run[2] = false;
				calibrated_joint[2] = false;
			}
			
			group1_calibrated = calibrated_joint[0] + calibrated_joint[1] + calibrated_joint[2];
			if(group1_calibrated == 0){
				calibration_step = J1_J3_RECAL;
			}
		}
		
		// Recalibrate First 3 Joints Until Each Limit Switch Pressed
		else if(calibration_step == J1_J3_RECAL){
			for(int i=0; i<3; i++){
				if(!selected_group1[i]) continue;
				
				if(!calibrated_joint[i]){							
					if(!switch_pressed[i]){
						step_cal_run[i] = true;
						move_stepper(robot_joint[i], 0, calibration_dir[i], calc_stepper_freq(i, 0.25f));
					}
					else{
						step_cal_run[i] = false;
						calibrated_joint[i] = true;
					}
				}
			}
			group1_calibrated = calibrated_joint[0] + calibrated_joint[1] + calibrated_joint[2];
			if(group1_calibrated == group1_count){
				HAL_Delay(500);
				reset_enc = true;
				HAL_Delay(500);
				reset_enc = false;
				calibration_step = J4_J6_CAL;
			}
		}
		
		// Move Last 3 Joint Until Each Limit Switch Pressed
		else if(calibration_step == J4_J6_CAL){
			for(int i=3; i<5; i++){
				if(!selected_group2[i]) continue;
				
				if(!calibrated_joint[i]){				
					if(!switch_pressed[i]){
						step_cal_run[i] = true;
						move_stepper(robot_joint[i], 0, calibration_dir[i], calc_stepper_freq(i, 0.5f));
					}
					else{
						step_cal_run[i] = false;
						calibrated_joint[i] = true;
					}
				}
			}
			group2_calibrated = calibrated_joint[3] + calibrated_joint[4] + calibrated_joint[5];
			if(group2_calibrated == group2_count){
				HAL_Delay(500);
				reset_enc = true;
				HAL_Delay(500);
				reset_enc = false;
				calibration_step = J4_J6_OFFSET_MOVE;
			}
		}
		
		// Move Last 3 Joints To Offset Angle
		else if(calibration_step == J4_J6_OFFSET_MOVE){
			float offset_angle[2] = {10.0, -10.0};
			
			if(filtered_joint_angle[3] < offset_angle[0]){
				step_cal_run[3] = true;
				move_stepper(robot_joint[3], 0, home_dir[3], calc_stepper_freq(3, 0.5f));
			}
			else{
				calibrated_joint[3] = false;
				step_cal_run[3] = false;
			}
			
			if(filtered_joint_angle[4] > offset_angle[1]){
				step_cal_run[4] = true;
				move_stepper(robot_joint[4], 0, home_dir[4], calc_stepper_freq(4, 0.5f));
			}
			else{
				calibrated_joint[4] = false;
				step_cal_run[4] = false;
			}
			
			group2_calibrated = calibrated_joint[3] + calibrated_joint[4];
			if(group2_calibrated == 0){
				calibration_step = J4_J6_RECAL;
			}
		}
		
		// Move Last 3 Joints Until Each Limit Switch Pressed
		if(calibration_step == J4_J6_RECAL){
			for(int i=3; i<5; i++){
				if(!selected_group2[i]) continue;
				
				if(!calibrated_joint[i]){				
					if(!switch_pressed[i]){
						step_cal_run[i] = true;
						move_stepper(robot_joint[i], 0, calibration_dir[i], calc_stepper_freq(i, 0.5f));
					}
					else{
						step_cal_run[i] = false;
						calibrated_joint[i] = true;
					}
				}
			}
			group2_calibrated = calibrated_joint[3] + calibrated_joint[4] + calibrated_joint[5];
			if(group2_calibrated == group2_count){
				HAL_Delay(500);
				reset_enc = true;
				HAL_Delay(500);
				reset_enc = false;
				
				if(mode == CALIBRATE_ONLY){
					calibration_step = CALIBRATION_OK;
				}
				else if(mode == CALIBRATE_HOMING){
					calibration_step = ALL_JOINT_HOME;
				}
			}
		}
		
		// Move All Joints To Home Position
		else if(calibration_step == ALL_JOINT_HOME){
			float offset_angle[6] = {90, 45, -45, 90, -120, 0};
			
			for(int i=0; i<JOINT_NUM-1; i++){
				if(offset_angle[i] < 0){
					if(filtered_joint_angle[i] > offset_angle[i]){
						step_cal_run[i] = true;
						move_stepper(robot_joint[i], 0, home_dir[i], calc_stepper_freq(i, 0.5f));
					}
					else{
						calibrated_joint[i] = false;
						step_cal_run[i] = false;
					}
				}
				else if(offset_angle[i] > 0){
					if(filtered_joint_angle[i] < offset_angle[i]){
						step_cal_run[i] = true;
						move_stepper(robot_joint[i], 0, home_dir[i], calc_stepper_freq(i, 0.5f));
					}
					else{
						calibrated_joint[i] = false;
						step_cal_run[i] = false;
					}
				}
			}
			
			group1_calibrated = calibrated_joint[0] + calibrated_joint[1] + calibrated_joint[2];
			group2_calibrated = calibrated_joint[3] + calibrated_joint[4] + calibrated_joint[5];
			
			if(group1_calibrated == 0 && group2_calibrated == 0){
				HAL_Delay(500);
				reset_enc = true;
				HAL_Delay(500);
				reset_enc = false;
				home = true;
				calibration_step = CALIBRATION_OK;
			}
		}
	}
	
	else if(mode == HOMING_ONLY){
		if(home){
			if(calibration_step == COUNT_JOINT){
				group1_count = 0;
				group2_count = 0;
				group1_calibrated = 0;
				group2_calibrated = 0;
				
				for(int i=0; i<3; i++){
					if(selected_group1[i]){
						if(filtered_joint_angle[i] >= -1.0f && filtered_joint_angle[i] <= 1.0f){
							calibrated_joint[i] = true;
						}
						else{
							calibrated_joint[i] = false;
						}
						group1_count++;
					}
					
					if(selected_group2[i]){
						if(filtered_joint_angle[i+3] >= -1.0f && filtered_joint_angle[i+3] <= 1.0f){
							calibrated_joint[i+3] = true;
						}
						else{
							calibrated_joint[i+3] = false;
						}
						group2_count++;
					}
				}
				calibration_step = ALL_JOINT_HOME;
			}
			
			else if(calibration_step == ALL_JOINT_HOME){
				for(int i=0; i<JOINT_NUM; i++){
					if(i != 4){
						if(!calibrated_joint[i]){
							if(filtered_joint_angle[i] > 0.2f){
								step_cal_run[i] = true;
								calibrated_joint[i] = false;
								move_stepper(robot_joint[i], 0, NEGATIVE_DIR, calc_stepper_freq(i, 0.5f));
							}
							else if(filtered_joint_angle[i] < -0.2){
								step_cal_run[i] = true;
								calibrated_joint[i] = false;
								move_stepper(robot_joint[i], 0, POSITIVE_DIR, calc_stepper_freq(i, 0.5f));
							}
							else{
								step_cal_run[i] = false;
								calibrated_joint[i] = true;
							}
						}
					}
				}
				
				if(filtered_joint_angle[4] > 90.2f){
					step_cal_run[4] = true;
					calibrated_joint[4] = false;
					move_stepper(robot_joint[4], 0, NEGATIVE_DIR, calc_stepper_freq(4, 0.5f));
				}
				else if(filtered_joint_angle[4] < 89.8f){
					step_cal_run[4] = true;
					calibrated_joint[4] = false;
					move_stepper(robot_joint[4], 0, POSITIVE_DIR, calc_stepper_freq(4, 0.5f));
				}
				else{
					step_cal_run[4] = false;
					calibrated_joint[4] = true;
				}
				
				group1_calibrated = calibrated_joint[0] + calibrated_joint[1] + calibrated_joint[2];
				group2_calibrated = calibrated_joint[3] + calibrated_joint[4];
				
				if(group1_calibrated == group1_count && group2_calibrated == group2_count){
					HAL_Delay(500);
					home = true;
					calibration_step = CALIBRATION_OK;
				}
			}
		}
		else{
			HAL_Delay(500);
			calibration_step = CALIBRATION_OK;
		}
	}
	
	if(calibration_step == CALIBRATION_OK){
		calibrating = false;
		group1_count = 0;
		group1_calibrated = 0;
		group2_count = 0;
		group2_calibrated = 0;
		return true;
	}
	
	return false;
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- ENCODER INITIALIZE FUNCTION --- */
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint8_t encoder_init(void){		
	for(int i=0; i<JOINT_NUM; i++){
		if(HAL_TIM_IC_Start_IT(enc_tim_handler[i], TIM_CHANNEL_1) == HAL_OK && HAL_TIM_IC_Start_IT(enc_tim_handler[i], TIM_CHANNEL_2) == HAL_OK){
			if(step_positive_dir[i] == DIR_CW) HAL_GPIO_WritePin(enc_gpio_port[i+6], enc_gpio_pin[i+6], GPIO_PIN_RESET);
			else HAL_GPIO_WritePin(enc_gpio_port[i+6], enc_gpio_pin[i+6], GPIO_PIN_SET);
		}
		else return i+1;
	}
	return 0;
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- ENCODER STOP FUNCTION --- */
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void encoder_stop(void){
	for(int i=0; i<JOINT_NUM; i++){
		HAL_TIM_IC_Stop_IT(enc_tim_handler[i], TIM_CHANNEL_1);
		HAL_TIM_IC_Stop_IT(enc_tim_handler[i], TIM_CHANNEL_2);
	}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- ENCODER COUNTER RESET FUNCTION --- */
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void encoder_reset(bool state, Zeroing_Select_t sel_joint, uint8_t current_tim){
	if(state == false) return;
	else if(state == true && sel_joint == 0){
		cal_value[current_tim] = enc_angle[current_tim];
		enc_counter[current_tim] = 0;
		filtered_joint_angle[current_tim] = 0.0;
	}
	else if(state == true && sel_joint > 0){
		cal_value[sel_joint-1] = enc_angle[sel_joint-1];
		enc_counter[sel_joint-1] = 0;
		filtered_joint_angle[sel_joint-1] = 0.0;
	}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- MISS STEP FUNCTION --- */
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void check_miss_step(void){
	for(int i=0; i<JOINT_NUM; i++){	
		int32_t delta_joint_angle = filtered_joint_angle[i] - prev_filtered_joint_angle[i];
		
		if(abs(delta_joint_angle) > MISS_STEP_TOLLERANCE){
			if(joint_error_bypass == false){
				miss_step[i] = true;
				for(int i=0; i<10; i++){
					Send_feedback(&command, JOINT_MISS_STEP, (uint16_t)i);
				}
				
				if(joint_current_dir[i] == NEGATIVE_DIR) negative_miss[i] = true;
				else if(joint_current_dir[i] == POSITIVE_DIR) positive_miss[i] = true;
				
				joint_miss_step_angle[i] = filtered_joint_angle[i];
			}
		}
		else miss_step[i] = false;
	
		if(negative_miss[i] == true && filtered_joint_angle[i] > joint_miss_step_angle[i] + MISS_STEP_SAFE_ANGLE){
			negative_miss[i] = false;
			joint_error_bypass = false;
		}
		else if(positive_miss[i] == true && filtered_joint_angle[i] < joint_miss_step_angle[i] - MISS_STEP_SAFE_ANGLE){
			positive_miss[i] = false;
			joint_error_bypass = false;
		}
		
		prev_filtered_joint_angle[i] = filtered_joint_angle[i];
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
//	tx_pos[0] = 12.3;
//	tx_pos[1] = 12.3;
//	tx_pos[2] = 12.3;
//	
//	tx_rot[0] = 11.1;
//	tx_rot[1] = 11.1;
//	tx_rot[2] = 11.1;
	
//	tx_angle[0] = 90.5;
//	tx_angle[1] = 90.5;
//	tx_angle[2] = 90.5;
//	tx_angle[3] = 90.5;
//	tx_angle[4] = 90.5;
//	tx_angle[5] = 90.5;
	
	tx_pos[0] = (float)kinematics.axis_pos_out[0];
	tx_pos[1] = (float)kinematics.axis_pos_out[1];
	tx_pos[2] = (float)kinematics.axis_pos_out[2];
	
	tx_rot[0] = (float)kinematics.axis_rot_out[0];
	tx_rot[1] = (float)kinematics.axis_rot_out[1];
	tx_rot[2] = (float)kinematics.axis_rot_out[2];
	
	tx_angle[0] = (float)filtered_joint_angle[0];
	tx_angle[1] = (float)filtered_joint_angle[1];
	tx_angle[2] = (float)filtered_joint_angle[2];
	tx_angle[3] = (float)filtered_joint_angle[3];
	tx_angle[4] = (float)filtered_joint_angle[4];
	tx_angle[5] = (float)filtered_joint_angle[5];
	
	if(HAL_GetTick() - prev_time_send > send_data_interval){
		for(int i=0; i<25; i++){
			Send_requested_data(&command, tx_pos, tx_rot, tx_angle, welding_data.welding_point, welding_data.welding_pattern, welding_data.welding_speed);
		}
		prev_time_send = HAL_GetTick();
	}
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- CUSTOM RS232 RECEIVE FUNCTION ---*/
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void receive_data(void){
	Start_get_command(&command);
	
	if(command.msg_get == true){
		Get_command(&command);
		command.msg_get = false;
	}
	
	if(HAL_GetTick() - prev_time_get > 300 && command.msg_get == false){
		Reset_command(&command);
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
		if(command.type == SETTING) SSD1306_Puts("Setting     ", &Font_7x10, SSD1306_COLOR_WHITE);
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
	
	else if(sel_menu == ENCODER_ERROR_MENU){
		SSD1306_GotoXY(0,0);
		SSD1306_Puts(encoder_err, &Font_7x10, SSD1306_COLOR_WHITE);
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
