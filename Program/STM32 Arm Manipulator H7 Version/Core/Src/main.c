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
#include "EEPROM_lib.h"
#include "RS232_Driver.h"
#include "ssd1306.h"
#include "stdbool.h"
#include "stdlib.h"
#include "math.h"
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


/* JOINT TYPEDEF */
// ----------------
typedef enum{
	J1 = 0x01,	// TIMER 6
	J2,					// TIMER 7
	J3,					// TIMER 12
	J4,					// TIMER 13
	J5,					// TIMER 14
	J6,					// TIMER 15
}Joint_Num_t;
// ----------------


/* STEPPER DIR TYPEDEF*/
// ---------------------
typedef enum{
	DIR_CW,
	DIR_CCW
}Stepper_Dir_t;
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
#define USE_EEPROM
#define USE_RS232
#define USE_OLED
#define USE_STEPPER
//#define USE_ENCODER

//#define TEST_EEPROM
//#define TEST_RS232
//#define TEST_OLED
//#define TEST_STEPPER
//#define TEST_ENCODER

#define MAIN_PROGRAM
//----------------------

/* EEPROM ADDRESS SET */
//-----------------------------------
#define EEPROM1_ADDRESS					0xA0	// PRIMARY EEPROM ADDRESS
#define EEPROM2_ADDRESS					0xA1	// SECONDARY EEPROM ADDRESS

#define SP_POS_BYTE_ADDR				0x00	// START POINT POSITION ADDR
#define EP_POS_BYTE_ADDR				0x18	// END POINT POSITION ADDR
#define SP_ANG_BYTE_ADDR				0x30	// START POINT ANGLE ADDR
#define EP_ANG_BYTE_ADDR				0x48	// END POINT ANGLE ADDR
#define PATTERN_BYTE_ADDR				0x60	// PATTERN ADDR
//-----------------------------------

/* EEPROM PATTERN SET */
//-----------------------------------
#define LINEAR_PATTERN					0x01
#define CIRCULAR_PATTERN				0x02
#define ZIGZAG_PATTERN					0x03
//-----------------------------------

/* STEPPER SET */
//----------------------------
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

#define SET_DUTY_CYCLE		0.25
#define DOF_NUM						6
//----------------------------

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
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

// PARAMETER VARIABLE ----
double
joint_positive_axisLim[6],
joint_negative_axisLim[6],
joint_max_traveldist[6],
joint_step_per_deg[6];

bool
limit_switch_state[6];
//------------------------


// EEPROM VARIABLE ---
double 
array_pos_start[3],
array_pos_end[3],
array_ang_start[6],
array_ang_end[6];

uint8_t
page_data_byte[128],

saved_pos[24], 
read_saved_pos[24],
read_saved_posX[8],
read_saved_posY[8],
read_saved_posZ[8],

saved_angle[48], 
read_saved_angle[48],
read_saved_angle1[8],
read_saved_angle2[8],
read_saved_angle3[8],
read_saved_angle4[8],
read_saved_angle5[8],
read_saved_angle6[8],

welding_pattern,
read_saved_pattern[1];
//--------------------


// INVERSE KINEMATICS VARIABLE


// FORWARD KINEMATICS VARIABLE


// MOVE VARIABLE ---
double
current_position[3],
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


// STEPPER VARIABLE -----
unsigned long
rpm_timer;

bool
step_start[6],
step_limit[6],
step_dir[6],
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
stepper_stepfactor[6],

stepper_microstep[6]={
	MICROSTEP_VALUE3,
	MICROSTEP_VALUE3,
	MICROSTEP_VALUE3,
	MICROSTEP_VALUE3,
	MICROSTEP_VALUE3,
	MICROSTEP_VALUE3,
},

stepper_ratio[6]={
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

double
max_joint_speed,
joint_rpm,
joint_delta_angle[6];
//-----------------------


// RS232 VARIABLE ---
bool
data_get = false;

double
tx_current_pos[3],
tx_current_angle[6],
tx_welding_point,
tx_welding_pattern,

rx_move_position[3],
rx_move_angle[6];

uint8_t
send_data_counter,
rx_savepoint,
rx_patterntype[200],
rx_pointtype,
rx_rotate_mode,
rx_rotate_value[8];

uint8_t
struct_size;
//-------------------


// OLED LCD VARIABLE ----------
uint8_t 
refresh_counter,
selected_menu;

char 
title[]  = "   WELDING ARM V1",
error1[] = "EEPROM1 ERROR    ",
error2[] = "EEPROM2 ERROR    ";
//-----------------------------


// ENCODER VARIABLE ---
uint32_t 
enc_counter[6],
pwm_freq[6], 
cycle_time[6];

uint8_t
enc_mag_status[6];

float
min_duty_cycle = 2.9,
max_duty_cycle = 97.1,
duty_cycle[6],
enc_angle[6],
cal_value[6],
cal_enc_angle[6],
prev_cal_enc_angle[6],
reduced_cal_enc_angle[6],
enc_joint_angle[6],
prev_enc_joint_angle[6];
//---------------------


// POWER VARIABLE ---
uint8_t pwr_status;
//-------------------


// TIMER VARIABLE 
unsigned long
prev_data_get;
//---------------

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM8_Init(void);
static void MX_I2C1_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM15_Init(void);
/* USER CODE BEGIN PFP */

// EEPROM Function
void save_weldingpos_world(uint16_t welding_point, uint8_t start_addr, double* pos_data, size_t size);
void read_weldingpos_world(uint16_t welding_point, uint8_t start_addr, double* stored_data, size_t size);
void save_weldingpos_angle(uint16_t welding_point, uint8_t start_addr, double* angle_data, size_t size);
void read_weldingpos_angle(uint16_t welding_point, uint8_t start_addr, double* stored_data, size_t size);
void save_weldingpos_pattern(uint16_t welding_point, uint8_t select_pattern);
void read_weldingpos_pattern(uint16_t welding_point, uint8_t store_pattern);
void read_all_weldingdata(uint16_t welding_point);
uint16_t check_weldingpoint(void);

// Stepper Control Function
void disable_stepper(void);
void enable_stepper(void);
void move_stepper(Joint_Num_t select_joint, uint16_t step, Stepper_Dir_t dir, uint16_t input_freq);

// Calculation Function
void forward_kinematics(double J1_input, double J2_input, double J3_input, double J4_input, double J5_input, double J6_input);
void inverse_kinematics(double Xpos_input, double Ypos_input, double Zpos_input);

// Main Function
void move_joint(Joint_Num_t select_joint, double input_angle, Speed_t set_speed);
void move_world(uint8_t move_var, double move_pos, Speed_t set_speed);
bool homing(void);

// Encoder Function
void encoder_init(void);
void encoder_reset(void);
void encoder_read(void);

// Custom RS232 Function
void transmit_req_data(void);
void receive_data(void);

// OLED Function
void show_menu(Oled_Menu_t sel_menu);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* RS232 CALLBACK */
//--------------------------------------------------------------------------------------------------------------------------------
uint32_t RS232_state, RS232_err_status;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == UART4){
		RS232_state = check_state();
		command.msg_get = true;
		prev_data_get = HAL_GetTick();
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart->Instance == UART4){
		RS232_state = check_state();
		command.msg_sent = true;
	}
}
//--------------------------------------------------------------------------------------------------------------------------------

/* TIMER CALLBACK */
//---------------------------------------------------------------------------------------------------------------------------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	// Joint 1
	if(htim->Instance == TIM6){
		step_state[0] = true;
		timer_counter[0]++;
		if(timer_counter[0] < t_on[0]) HAL_GPIO_WritePin(PUL1_GPIO_Port, PUL1_Pin, GPIO_PIN_SET);
		else if(timer_counter[0] >= t_on[0] && timer_counter[0] < step_period[0]-1) HAL_GPIO_WritePin(PUL1_GPIO_Port, PUL1_Pin, GPIO_PIN_RESET);
		else if(timer_counter[0] == step_period[0]-1){
			timer_counter[0] = 0;
			step_counter[0]++;
		}
		
		if(((step_counter[0] >= step_input[0]) && step_limit[0] == true) || step_start[0] == false){
			HAL_TIM_Base_Stop_IT(&htim6);
			timer_counter[0] = 0;
			step_counter[0] = 0;
			step_state[0] = false;
		}
	}
	
	// Joint 2
	else if(htim->Instance == TIM7){
		step_state[1] = true;
		timer_counter[1]++;
		if(timer_counter[1] < t_on[1]) HAL_GPIO_WritePin(PUL2_GPIO_Port, PUL2_Pin, GPIO_PIN_SET);
		else if(timer_counter[1] >= t_on[1] && timer_counter[1] < step_period[1]-1) HAL_GPIO_WritePin(PUL2_GPIO_Port, PUL2_Pin, GPIO_PIN_RESET);
		else if(timer_counter[1] == step_period[1]-1){
			timer_counter[1] = 0;
			step_counter[1]++;
		}
		
		if(((step_counter[1] >= step_input[0]) && step_limit[1] == true) || step_start[1] == false){
			HAL_TIM_Base_Stop_IT(&htim7);
			timer_counter[1] = 0;
			step_counter[1] = 0;
			step_state[1] = false;
		}
	}
	
	// Joint 3
	else if(htim->Instance == TIM12){
		timer_counter[2]++;
	}
	
	// Joint 4
	else if(htim->Instance == TIM13){
		timer_counter[3]++;
	}
	
	// Joint 5
	else if(htim->Instance == TIM14){
		timer_counter[4]++;
	}
	
	// Joint 6
	else if(htim->Instance == TIM15){
		timer_counter[5]++;
	}
}
//---------------------------------------------------------------------------------------------------------------------------------------------

/* INPUT CAPTURE CALLBACK */
//-------------------------------------------------------------------------------------------
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim){
	// Joint 1 Encoder
	if(htim->Instance == TIM4){
		cycle_time[0] = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		if(cycle_time[0] != 0){
			duty_cycle[0] = (HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2)*100.0) / cycle_time[0];
			pwm_freq[0] = (100000000/cycle_time[0]);
			enc_angle[0] = (((duty_cycle[0] - min_duty_cycle) * 360) / (max_duty_cycle - min_duty_cycle));
			cal_enc_angle[0] = fmod((enc_angle[0] - cal_value[0] + 360), 360);
			reduced_cal_enc_angle[0] = cal_enc_angle[0] / stepper_ratio[0];
			
			if(prev_cal_enc_angle[0] > 350 && cal_enc_angle[0] < 10) enc_counter[0]++;
			else if(prev_cal_enc_angle[0] < 10 && cal_enc_angle[0] > 350) enc_counter[0]--;
			
			enc_joint_angle[0] = reduced_cal_enc_angle[0] + (enc_counter[0] * 360);
		}
	}
	
	// Joint 2 Encoder
	else if(htim->Instance == TIM8){
	}
	
	// Joint 3 Encoder
	else if(htim->Instance == TIM2){
	}
	
	// Joint 4 Encoder
	else if(htim->Instance == TIM1){
	}
	
	// Joint 5 Encoder
	else if(htim->Instance == TIM3){
	}
	
	// Joint 6 Encoder
	else if(htim->Instance == TIM5){
	}
}
//-------------------------------------------------------------------------------------------

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
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_TIM14_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */
	
	/* OLED LCD SETUP */
	//-----------------------------
	#ifdef USE_OLED
	SSD1306_Init();
	selected_menu = BOOT_MENU;
	show_menu((Oled_Menu_t)selected_menu);
	HAL_Delay(1000);
	#endif
	//-----------------------------
	
	/* EEPROM SETUP */
	// ------------------------------------------------------------
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
	// ------------------------------------------------------------
	
	
	/* RS-232 COMMUNICATION SETUP */
	// ---------------------------------------------------------------------------------------------------------------------------
	#ifdef USE_RS232
	RS232_Init(&huart4);
	selected_menu = COM_INIT_MENU;
	show_menu((Oled_Menu_t)selected_menu);
	while(1){
		receive_data();
		Send_feedback(&command, MAIN_ONLINE);
		if(command.type == FEEDBACK && command.feedback == PENDANT_ONLINE){
			SSD1306_Clear();
			break;
		}
	}
	command.msg_sent = true;
	command.msg_get = true;
	#endif
	// ---------------------------------------------------------------------------------------------------------------------------
	
	/* ENCODER SETUP */
	#ifdef USE_ENCODER
	
	#endif
	
	
	/* STEPPER SETUP */
	//--------------------------------------------------------------------------------------
	#ifdef USE_STEPPER
	enable_stepper();
	HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_SET);
	
	for(int i=0; i<DOF_NUM; i++){
		if(stepper_microstep[i] == MICROSTEP_VALUE1) stepper_stepfactor[i] = MICROSTEP_FACTOR1;
		else if(stepper_microstep[i] == MICROSTEP_VALUE2) stepper_stepfactor[i] = MICROSTEP_FACTOR2;
		else if(stepper_microstep[i] == MICROSTEP_VALUE3) stepper_stepfactor[i] = MICROSTEP_FACTOR3;
		else if(stepper_microstep[i] == MICROSTEP_VALUE4) stepper_stepfactor[i] = MICROSTEP_FACTOR4;
		else if(stepper_microstep[i] == MICROSTEP_VALUE5) stepper_stepfactor[i] = MICROSTEP_FACTOR5;
		else if(stepper_microstep[i] == MICROSTEP_VALUE6) stepper_stepfactor[i] = MICROSTEP_FACTOR6;
	}
	
	step_dir[0] = 0;	
	step_dir[1] = 0;
	step_dir[2] = 0;
	step_dir[3] = 0;
	step_dir[4] = 0;
	step_dir[5] = 0;
	
	for(int i=0; i<DOF_NUM; i++){
		joint_max_traveldist[i] = joint_positive_axisLim[i] + joint_negative_axisLim[i];
		joint_step_per_deg[i] = (stepper_microstep[i] * stepper_ratio[i]) / 360;
	}
	#endif
	//--------------------------------------------------------------------------------------
	
	
	#ifdef TEST_EEPROM
	double 
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
	
	
	/* ALL SETUP DONE */
	//------------------------
	selected_menu = MAIN_MENU;
	show_menu((Oled_Menu_t)selected_menu);
	//------------------------
	
	struct_size = sizeof(Data_Get_t)%8;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		#ifdef TEST_RS232 
		receive_data();
		
		tx_current_angle[0] = 70.28;
		tx_current_angle[1] = 90.55;
		tx_current_angle[2] = 45.78;
		tx_current_angle[3] = 100.11;
		tx_current_angle[4] = 30.17;
		tx_current_angle[5] = 15.99;
		
		tx_current_pos[0] = 100.55;
		tx_current_pos[1] = 20.67;
		tx_current_pos[2] = 45.22;
		#endif
		
		#ifdef TEST_OLED
		#endif
		
		#ifdef TEST_STEPPER
		move_stepper(J1, 8000, DIR_CW, 3600);
		move_stepper(J2, 8000, DIR_CW, 1000);
		#endif
		
		#ifdef TEST_ENCODER
		#endif
		
		#ifdef MAIN_PROGRAM
		// Listening For Pendant Command
		receive_data();
		
		// Check Auto Home Command
		if(command.type == AUTO_HOME){
			while(1){
				if(homing() == false) continue;
				else{
					Send_feedback(&command, AUTO_HOME_DONE);
				}
			}
		}
		
		// Check Mapping Command
		else if(command.type == MAPPING){
		}
		
		// Check Preview Command
		else if(command.type == PREVIEW){
		}
		
		// Check Move Command
		else if(command.type == MOVE){
			if(command.control_mode == CARTESIAN_CTRL){

			}
			
			else if(command.control_mode == JOINT_CTRL){	
				// Joint 1 Command Control
				if(command.move_variable == JOINT_1){
					if(command.move_mode == CONTINUOUS){
						step_limit[0] = false;
						step_start[0] = true;
						rx_move_angle[0] = 0.0;
					}
					
					else if(command.move_mode == DISTANCE || command.move_mode == STEP){
						step_limit[0] = true;
						step_limit[0] = false;
						rx_move_angle[0] = command.move_value;
					}
					move_joint(J1, rx_move_angle[0], MED);
				}
				
				// Joint 2 Command Control
				else if(command.move_variable == JOINT_2){
					if(command.move_mode == CONTINUOUS){
						step_limit[1] = false;
						step_start[1] = true;
						rx_move_angle[1] = 0.0;
					}
					
					else if(command.move_mode == DISTANCE || command.move_mode == STEP){
						step_limit[1] = true;
						step_start[1] = false;
						rx_move_angle[1] = command.move_value;
					}
					move_joint(J2, rx_move_angle[1], MED);
				}
			}
		}
		
		// Check Run Command
		else if(command.type == RUN){
		}
		
		// Check Send Requested Data Command
		else if(command.type == SEND_REQ){
		}
		
		// Check Motor State Command
		else if(command.type == MOTOR_STATE){
			if(command.motor_state == MOTOR_OFF) disable_stepper();
			else if(command.motor_state == MOTOR_ON) enable_stepper();
		}
		
		// Check Welder State Command
		else if(command.type == WELDER_STATE){
		}
		
		// Check Feedback
		else if(command.type == FEEDBACK){
		}
		
		// Check None Command
		else if(command.type == NONE){
			for(int i=0; i<DOF_NUM; i++){
				step_start[i] = false;
			}
			command.move_value = 0.0;
		}
		
		// Send Current Position
		tx_current_pos[0] = 100.55;
		tx_current_pos[1] = 20.67;
		tx_current_pos[2] = 45.22;
		
		tx_current_angle[0] = 70.28;
		tx_current_angle[1] = 90.55;
		tx_current_angle[2] = 45.78;
		tx_current_angle[3] = 100.11;
		tx_current_angle[4] = 30.17;
		tx_current_angle[5] = 15.99;
		
		transmit_req_data();
		show_menu((Oled_Menu_t)selected_menu);
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
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  htim1.Init.Prescaler = 0;
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
  sSlaveConfig.TriggerFilter = 4;
  if (HAL_TIM_SlaveConfigSynchro(&htim1, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 4;
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
  htim2.Init.Prescaler = 0;
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
  sSlaveConfig.TriggerFilter = 4;
  if (HAL_TIM_SlaveConfigSynchro(&htim2, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 4;
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
  htim3.Init.Prescaler = 0;
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
  sSlaveConfig.TriggerFilter = 4;
  if (HAL_TIM_SlaveConfigSynchro(&htim3, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 4;
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
  htim4.Init.Prescaler = 0;
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
  sSlaveConfig.TriggerFilter = 4;
  if (HAL_TIM_SlaveConfigSynchro(&htim4, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 4;
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
  htim5.Init.Prescaler = 0;
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
  sSlaveConfig.TriggerFilter = 4;
  if (HAL_TIM_SlaveConfigSynchro(&htim5, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 4;
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 0;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 99;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 0;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 99;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

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
  sSlaveConfig.TriggerFilter = 4;
  if (HAL_TIM_SlaveConfigSynchro(&htim8, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 4;
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
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : LIMIT6_Pin */
  GPIO_InitStruct.Pin = LIMIT6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
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
  HAL_NVIC_SetPriority(EXTI2_IRQn, 13, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 13, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 13, 0);
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
void save_weldingpos_world(uint16_t welding_point, uint8_t start_addr, double* pos_data, size_t size){
	for(int i=0; i<24; i++) saved_pos[i] = 0;
	for(size_t i=0; i<size; i++) memcpy(&saved_pos[i*8], &pos_data[i], sizeof(double));
	
	EEPROM_PageWrite(&eeprom1, welding_point, start_addr, saved_pos, sizeof(saved_pos));
	HAL_Delay(10);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- READ WELDING POINT POSITION VALUE ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void read_weldingpos_world(uint16_t welding_point, uint8_t start_addr, double* stored_data, size_t size){
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
void save_weldingpos_angle(uint16_t welding_point, uint8_t start_addr, double* angle_data, size_t size){
	for(int i=0; i<48; i++) saved_angle[i] = 0;
	for(size_t i=0; i<size; i++) memcpy(&saved_angle[i*8], &angle_data[i], sizeof(double));
	
	EEPROM_PageWrite(&eeprom1, welding_point, start_addr, saved_angle, sizeof(saved_angle));
	HAL_Delay(10);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*--- SAVE WELDING POINT ANGLE VALUE ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void read_weldingpos_angle(uint16_t welding_point, uint8_t start_addr, double* stored_data, size_t size){
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
void move_stepper(Joint_Num_t select_joint, uint16_t step, Stepper_Dir_t dir, uint16_t input_freq){		
	// Joint 1 Stepper Control
	if(select_joint == J1){
		// Duty Cycle Calculation
		step_freq[0] = input_freq;
		step_period[0] = 1000000/step_freq[0];
		t_on[0] = step_period[0] * SET_DUTY_CYCLE;
		t_off[0] = step_period[0] - t_on[0];
		
		// Set Stepper Direction
		if(dir == DIR_CW && step_state[0] == false){
			step_dir[0] = 0;
			HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_SET);
		}
		else if(dir == DIR_CCW && step_state[0] == false){
			step_dir[0] = 1;
			HAL_GPIO_WritePin(DIR1_GPIO_Port, DIR1_Pin, GPIO_PIN_RESET);
		}
		
		// Pulse Start
		step_input[0] = step;
		if(step_counter[0] == 0 && step_counter[0] != step && step_state[0] == false){
			step_start[0] = true;
			HAL_TIM_Base_Start_IT(&htim6);
		}
		else if(step_start[0] == true && step_state[0] == false){
			HAL_TIM_Base_Start_IT(&htim6);
		}
	}
	
	// Joint 2 Stepper Control
	else if(select_joint == J2){
		// Duty Cycle Calculation
		step_freq[1] = input_freq;
		step_period[1] = 1000000/step_freq[1];
		t_on[1] = step_period[1] * SET_DUTY_CYCLE;
		t_off[1] = step_period[1] - t_on[1];
		
		// Set Stepper Direction
		if(dir == DIR_CW && step_state[1] == false){
			step_dir[1] = 0;
			HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_SET);
		}
		else if(dir == DIR_CCW && step_state[1] == false){
			step_dir[1] = 1;
			HAL_GPIO_WritePin(DIR2_GPIO_Port, DIR2_Pin, GPIO_PIN_RESET);
		}
		
		// Pulse Start
		step_input[1] = step;
		if(step_counter[1] == 0 && step_counter[1] != step && step_state[1] == false){
			step_start[1] = true;
			HAL_TIM_Base_Start_IT(&htim7);
		}
		else if(step_start[1] == true && step_state[1] == false){
			HAL_TIM_Base_Start_IT(&htim7);
		}
	}
	
	// Joint 3 Stepper Control
	else if(select_joint == J3){
		// Duty Cycle Calculation
		step_freq[2] = input_freq;
		step_period[2] = 1000000/step_freq[2];
		t_on[2] = step_period[2] * SET_DUTY_CYCLE;
		t_off[2] = step_period[2] - t_on[2];
		
		// Set Stepper Direction
		if(dir == DIR_CW && step_state[2] == false){
			step_dir[2] = 0;
			HAL_GPIO_WritePin(DIR3_GPIO_Port, DIR3_Pin, GPIO_PIN_SET);
		}
		else if(dir == DIR_CCW && step_state[2] == false){
			step_dir[2] = 1;
			HAL_GPIO_WritePin(DIR3_GPIO_Port, DIR3_Pin, GPIO_PIN_RESET);
		}
		
		// Pulse Start
		step_input[2] = step;
		if(step_counter[2] == 0 && step_counter[2] != step && step_state[2] == false){
			step_start[2] = true;
			HAL_TIM_Base_Start_IT(&htim12);
		}
		else if(step_start[2] == true && step_state[2] == false){
			HAL_TIM_Base_Start_IT(&htim12);
		}
	}
	
	// Joint 4 Stepper Control
	else if(select_joint == J4){
		// Duty Cycle Calculation
		step_freq[3] = input_freq;
		step_period[3] = 1000000/step_freq[3];
		t_on[3] = step_period[3] * SET_DUTY_CYCLE;
		t_off[3] = step_period[3] - t_on[3];
		
		// Set Stepper Direction
		if(dir == DIR_CW && step_state[3] == false){
			step_dir[3] = 0;
			HAL_GPIO_WritePin(DIR4_GPIO_Port, DIR4_Pin, GPIO_PIN_SET);
		}
		else if(dir == DIR_CCW && step_state[3] == false){
			step_dir[3] = 1;
			HAL_GPIO_WritePin(DIR4_GPIO_Port, DIR4_Pin, GPIO_PIN_RESET);
		}
		
		// Pulse Start
		step_input[3] = step;
		if(step_counter[3] == 0 && step_counter[3] != step && step_state[3] == false){
			step_start[3] = true;
			HAL_TIM_Base_Start_IT(&htim13);
		}
		else if(step_start[3] == true && step_state[3] == false){
			HAL_TIM_Base_Start_IT(&htim13);
		}
	}
	
	// Joint 5 Stepper Control
	else if(select_joint == J5){
		// Duty Cycle Calculation
		step_freq[4] = input_freq;
		step_period[4] = 1000000/step_freq[4];
		t_on[4] = step_period[4] * SET_DUTY_CYCLE;
		t_off[4] = step_period[4] - t_on[4];
		
		// Set Stepper Direction
		if(dir == DIR_CW && step_state[4] == false){
			step_dir[4] = 0;
			HAL_GPIO_WritePin(DIR5_GPIO_Port, DIR5_Pin, GPIO_PIN_SET);
		}
		else if(dir == DIR_CCW && step_state[4] == false){
			step_dir[4] = 1;
			HAL_GPIO_WritePin(DIR5_GPIO_Port, DIR5_Pin, GPIO_PIN_RESET);
		}
		
		// Pulse Start
		step_input[4] = step;
		if(step_counter[4] == 0 && step_counter[4] != step && step_state[4] == false){
			step_start[4] = true;
			HAL_TIM_Base_Start_IT(&htim14);
		}
		else if(step_start[4] == true && step_state[4] == false){
			HAL_TIM_Base_Start_IT(&htim14);
		}
	}
	
	// Joint 6 Stepper Control
	else if(select_joint == J6){
		// Duty Cycle Calculation
		step_freq[5] = input_freq;
		step_period[5] = 1000000/step_freq[5];
		t_on[5] = step_period[5] * SET_DUTY_CYCLE;
		t_off[5] = step_period[5] - t_on[5];
		
		// Set Stepper Direction
		if(dir == DIR_CW && step_state[5] == false){
			step_dir[5] = 0;
			HAL_GPIO_WritePin(DIR6_GPIO_Port, DIR6_Pin, GPIO_PIN_SET);
		}
		else if(dir == DIR_CCW && step_state[5] == false){
			step_dir[5] = 1;
			HAL_GPIO_WritePin(DIR6_GPIO_Port, DIR6_Pin, GPIO_PIN_RESET);
		}
		
		// Pulse Start
		step_input[5] = step;
		if(step_counter[5] == 0 && step_counter[5] != step && step_state[5] == false){
			step_start[5] = true;
			HAL_TIM_Base_Start_IT(&htim15);
		}
		else if(step_start[5] == true && step_state[5] == false){
			HAL_TIM_Base_Start_IT(&htim15);
		}
	}
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- JOINT BASE MOVE FUNCTION ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void move_joint(Joint_Num_t select_joint, double input_angle, Speed_t set_speed){
	// Joint RPM Set
	if(set_speed == LOW) joint_rpm = 2;
	else if(set_speed == MED) joint_rpm = 3;
	else if(set_speed == HIGH) joint_rpm = 4;
	
	// Joint 1 Move Control
	if(select_joint == J1){
		// Synchronize Joint 1 RPM
		stepper_rpm[0] = joint_rpm * stepper_ratio[0];
		stepper_freq[0] = stepper_rpm[0] * stepper_microstep[0] / 60;
		
		// Check For One Time Move
		if(prev_input_angle[0] != input_angle){
			if(prev_angle[0] < input_angle) move_stepper(select_joint, step_output[0], DIR_CW, stepper_freq[0]);
			else if(prev_angle[0] > input_angle) move_stepper(select_joint, step_output[0], DIR_CCW, stepper_freq[0]);
		}
		
		if(step_start[0] == true){
			step_output[0] = 0;
			if(command.move_sign == UNSIGNED_VAR) move_stepper(select_joint, step_output[0], DIR_CW, stepper_freq[0]);
			else move_stepper(select_joint, step_output[0], DIR_CCW, stepper_freq[0]);
		}
	}
	
	// Joint 2 Move Control
	else if(select_joint == J2){
		// Synchronize Joint 2 RPM
		stepper_rpm[1] = joint_rpm * stepper_ratio[1];
		stepper_freq[1] = stepper_rpm[1] * stepper_microstep[1] / 60;
		
		// Check For One Time Move
		if(prev_input_angle[1] != input_angle){
			if(prev_angle[1] < input_angle) move_stepper(select_joint, step_output[1], DIR_CW, stepper_freq[1]);
			else if(prev_angle[1] > input_angle) move_stepper(select_joint, step_output[1], DIR_CCW, stepper_freq[1]);
		}
		
		if(step_start[1] == true){
			step_output[1] = 0;
			if(command.move_sign == UNSIGNED_VAR) move_stepper(select_joint, step_output[1], DIR_CW, stepper_freq[1]);
			else move_stepper(select_joint, step_output[1], DIR_CCW, stepper_freq[1]);
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
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- JOINT ANGLE HOMING FUNCTION ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool homing(void){
	return false;
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- ENCODER INITIALIZE FUNCTION ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void encoder_init(void){
	// Encoder Joint 1
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_2);
	
	// Encoder Joint 2
	HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim8, TIM_CHANNEL_2);
	
	// Encoder Joint 3
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
	
	// Encoder Joint 4
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);
	
	// Encoder Joint 5
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
	
	// Encoder Joint 6
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- ENCODER COUNTER RESET FUNCTION ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void encoder_reset(void){
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- ENCODER READ FUNCTION ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void encoder_read(void){
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/* --- CUSTON RS232 TRANSMIT FUNCTION ---*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void transmit_req_data(void){
	Send_requested_data(&command, tx_current_pos, tx_current_angle, current_point, LINEAR, 100);
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
	
	if(HAL_GetTick() - prev_data_get > 300 && command.msg_get == false){
		command.type = NONE;
		prev_data_get = HAL_GetTick();
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
		else if(command.type == STANDBY) SSD1306_Puts("Standby     ", &Font_7x10, SSD1306_COLOR_WHITE);
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
