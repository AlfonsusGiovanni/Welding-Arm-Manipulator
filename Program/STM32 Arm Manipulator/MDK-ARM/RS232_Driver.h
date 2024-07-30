/*
Author	: Alfonsus Giovanni Mahendra Putra
Date		: 10 Juli 2024
*/

#ifndef RS3232_DRIVER_H
#define RS3232_DRIVER_H

#include "main.h"
#include "string.h"

/*RS232 COM HEADER*/
#define HEADER1 0xFF
#define HEADER2 0xFF

/*RS232 COM BUFFER SIZE*/
#define BUFF_SIZE 64

/*RS232 COM TIMEOUT*/
#define RS232_TIMEOUT 100

/*RS232 COM COMMAND*/
#define AUTO_HOME_CMD				0x01
#define MAPPING_MODE_CMD		0x02
#define MOVE_POSITION_CMD		0x03
#define RUNNING_CMD					0x04
#define REQ_POS_CMD					0x05
#define REQ_ANG_CMD					0x06
#define REQ_MAP_CMD					0x07
#define SEND_REQ_CMD				0x08
#define MOTOR_STATE_CMD			0x09
#define WELDER_STATE_CMD		0x0A
#define FEEDBACK_CMD				0x0B


//--- COMMAND TYPE TYPEDEF ---//
////////////////////////////////
typedef enum{
	AUTO_HOME = 0x01,
	MAPPING,
	MOVE,
	RUN,
	REQ_POS,
	REQ_ANG,
	REQ_MAP,
	SEND_REQ,
	MOTOR_STATE,
	WELDER_STATE,
	FEEDBACK,
	NONE,
}Command_t;
////////////////////////////////


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
	LINEAR = 0x01,
	CIRCULAR,
	WAVE,
}Welding_Pattern_t;
////////////////////////////////


//--- POINT COORDINATE TYPDEF ---//
///////////////////////////////////
typedef enum{
	START_POINT =  0x01,
	END_POINT
}Welding_Point_t;
///////////////////////////////////


//--- REQUESTED TYPEDEF ---//
////////////////////////////////
typedef enum{
	SEND_POSITION = 0x01,
	SEND_ANGLE,
	SEND_MAPPED_POINT,
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
	X_position, X_position_req,
	Y_position, Y_position_req,
	Z_position, Z_position_req,
	
	motor1_angle, motor1_angle_req,
	motor2_angle, motor2_angle_req,
	motor3_angle, motor3_angle_req,
	motor4_angle, motor4_angle_req,
	motor5_angle, motor5_angle_req,
	motor6_angle, motor6_angle_req;
	
	uint8_t
	point_num,
	move_speed;

	Command_t type;
	Ctrl_Mode_t control_mode;
	Run_State_t running_state;
	Run_Mode_t running_mode;
	Welding_Pattern_t pattern_type;
	Welding_Point_t point_type;
	Req_t requested_variable;
	Motor_State_t motor_state;
	Welder_State_t welder_state;
	Feedback_t feedback;
}Data_Get_t;
//////////////////////////////////////


/*RS232 INITIALIZE*/
void RS232_Init(UART_HandleTypeDef* huart_handler);

/*CHECKSUM FUNCTION*/
static uint16_t checksum_generator(uint8_t* arr, size_t size);

/*TRANSMITING COMMAND*/
//----------------------------------------------------------------------------------------------------
void Send_auto_home(void);
void Send_mapping(uint16_t point_num, Welding_Point_t point_type, Welding_Pattern_t pattern_type, uint8_t speed);
void Send_preview(uint16_t point_num);

void Send_move(Ctrl_Mode_t control_mode, double* value, size_t size);
void Send_running(Run_State_t state, Run_Mode_t mode);

void Req_position(void);
void Req_angle(void);
void Req_mapped(void);

void Send_req(Req_t req_var, double* value, uint16_t *mapped_point, size_t size);

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
