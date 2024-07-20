/*
Author	: Alfonsus Giovanni Mahendra Putra
Date		: 10 Juli 2024
*/

#ifndef RS3232_DRIVER_H
#define RS3232_DRIVER_H

#include "main.h"
#include "string.h"

#define HEADER1 0xFF
#define HEADER2 0xFF

#define BUFF_SIZE 64
#define RS232_TIMEOUT 100

#define AUTO_HOME_CMD				0x01
#define MAPPING_MODE_CMD		0x02
#define MOVE_POSITION_CMD		0x03
#define RUNNING_CMD					0x04
#define REQ_POS_CMD					0x05
#define REQ_ANG_CMD					0x06
#define SEND_REQ_CMD				0x07
#define MOTOR_STATE_CMD			0x08
#define WELDER_STATE_CMD		0x09
#define FEEDBACK_CMD				0x0A


//--- COMMAND TYPE TYPEDEF ---//
typedef enum{
	AUTO_HOME = 0x01,
	MAPPING,
	MOVE,
	RUN,
	REQ_POS,
	REQ_ANG,
	SEND_REQ,
	MOTOR_STATE,
	WELDER_STATE,
	FEEDBACK,
}Command_t;


//--- MANUAL CONTROL MODE TYPEDEF ---//
///////////////////////////////////////
typedef enum{
	POSITION_CTRL = 0x01,
	ANGLE_CTRL,
}Ctrl_Mode_t;
///////////////////////////////////////


//--- RUNNING MODE TYPEDEF ---//
////////////////////////////////
typedef enum{
	PATTERN_POS_MODE = 0x01,
	PATTERN_ANG_MODE,
}Run_Mode_t;
////////////////////////////////


//--- RUNNING STATE TYPEDEF ---//
////////////////////////////////
typedef enum{
	RUNNING_START = 0x01,
	RUNNING_PAUSE,
	RUNNING_STOP,
	RUNNING_CONTINUE
}Run_State_t;
////////////////////////////////


//--- MEMORY TYPEDEF ---//
////////////////////////////////
typedef enum{
	PATTERN1 = 0x01,
	PATTERN2,
	PATTERN3,
}Memory_Pattern_t;
////////////////////////////////


//--- POINT COORDINATE TYPDEF ---//
///////////////////////////////////
typedef enum{
	POINT1 = 0x01,
	POINT2,
	POINT3,
	POINT4,
	POINT5,
	POINT6,
	POINT7,
	POINT8
}Point_Pattern_t;
///////////////////////////////////


//--- REQUESTED TYPEDEF ---//
////////////////////////////////
typedef enum{
	SEND_POSITION = 0x01,
	SEND_ANGLE,
}Req_t;
////////////////////////////////


//--- MOTOR STATE TYPEDEF ---//
////////////////////////////////
typedef enum{
	MOTOR_OFF = 0x01,
	MOTOR_ON,
}Motor_State_t;
////////////////////////////////


//--- WELDER STATE TYPEDEF ---//
////////////////////////////////
typedef enum{
	WELDER_OFF = 0x01,
	WELDER_ON,
}Welder_State_t;
////////////////////////////////


//--- SPEED TYPEDEF ---//
/////////////////////////
typedef enum{
	LOW_SPEED = 0x01, 
	MEDIUM_SPEED,
	HIGH_SPEED,
}Speed_t;
/////////////////////////


//--- COMMAND FEEDBACK TYPEDEF ---//
////////////////////////////////////
typedef enum{
	AUTO_HOME_DONE = 0x01,
	DISTANCE_MOVE_DONE,
}Feedback_t;
////////////////////////////////////


//--- DATA COMMUNICATION TYPEDEF ---//
//////////////////////////////////////
typedef struct{
	double
	X_position, 
	Y_position,
	Z_position,
	
	motor1_angle,
	motor2_angle,
	motor3_angle,
	motor4_angle,
	motor5_angle,
	motor6_angle;

	Command_t type;
	Ctrl_Mode_t control_mode;
	Run_State_t state;
	Run_Mode_t running_mode;
	Point_Pattern_t point;
	Req_t requested_variable;
	Motor_State_t motor_state;
	Welder_State_t welder_state;
	Speed_t running_speed;
	Feedback_t feedback;
}Data_Get_t;
//////////////////////////////////////


/*RS232 INITIALIZE*/
void RS232_Init(UART_HandleTypeDef* huart_handler);

/*CHECKSUM FUNCTION*/
static uint16_t checksum_generator(uint8_t* arr, uint16_t size);

/*TRANSMITING COMMAND*/
//----------------------------------------------------------------------------------------------------
void Send_auto_home(void);
void Send_mapping_mode(Memory_Pattern_t pattern_select);
void Send_mapping_data(Point_Pattern_t point_num);

void Send_move(Ctrl_Mode_t control_mode, double* value, size_t size);
void Send_running(Run_State_t state, Run_Mode_t mode, Memory_Pattern_t pattern_select, Speed_t speed);

void Req_position(void);
void Req_angle(void);

void Send_req(Req_t req_var, double* value, size_t size);

void Send_motor_state(Motor_State_t state);
void Send_welder_state(Welder_State_t state);

void Send_feedback(Feedback_t fdbck);
//----------------------------------------------------------------------------------------------------

/*RECIEVING COMMAND*/
//---------------------------------
void Start_get_command(void);
void Get_command(Data_Get_t* get);
//---------------------------------

/*TRANSMIT RECIEVE MONITOR*/
//---------------------------
uint32_t check_state(void);
uint32_t check_error(void);
//---------------------------

#endif
