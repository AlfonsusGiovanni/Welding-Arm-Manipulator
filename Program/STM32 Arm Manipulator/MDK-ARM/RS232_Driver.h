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
#define HEADER3 0xFF

/*RS232 COM BUFFER SIZE*/
#define BUFF_SIZE 100

/*RS232 COM TIMEOUT*/
#define RS232_TIMEOUT 100

/*RS232 COM COMMAND*/
#define AUTO_HOME_CMD				0x01
#define MAPPING_MODE_CMD		0x02
#define PREVIEW_MODE_CMD		0x03
#define MOVE_POSITION_CMD		0x04
#define RUNNING_CMD					0x05
#define REQ_DATA_CMD				0x06
#define SEND_REQ_CMD				0x07
#define MOTOR_STATE_CMD			0x08
#define WELDER_STATE_CMD		0x09
#define FEEDBACK_CMD				0x0A


//--- COMMAND TYPE TYPEDEF ---//
////////////////////////////////
typedef enum{
	AUTO_HOME = 0x01,
	MAPPING,
	PREVIEW,
	MOVE,
	RUN,
	REQ_DATA,
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
	CARTESIAN_CTRL = 0x01,
	JOINT_CTRL,
}Ctrl_Mode_t;
///////////////////////////////////////


//--- MOVE VARIABLE TYPEDEF ---//
/////////////////////////////////
typedef enum{
	CARTESIAN_X = 0x01,
	CARTESIAN_Y,
	CARTESIAN_Z,
	
	JOINT_1,
	JOINT_2,
	JOINT_3,
	JOINT_4,
	JOINT_5,
	JOINT_6,
}Move_Var_t;
/////////////////////////////////


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
	NOT_SET,
}Welding_Pattern_t;
////////////////////////////////


//--- POINT COORDINATE TYPDEF ---//
///////////////////////////////////
typedef enum{
	START_POINT =  0x01,
	END_POINT,
	PATTERN,
}Data_type_t;
///////////////////////////////////


//--- MAPPING COMMAND TYPEDF ---//
//////////////////////////////////
typedef enum{
	SAVE_VALUE = 0x01,
	DELETE_VALUE,
}Mapping_State_t;
//////////////////////////////////


//--- REQUESTED TYPEDEF ---//
////////////////////////////////
typedef enum{
	SEND_POSITION = 0x01,
	SEND_ANGLE,
	SEND_WELDING_DATA,
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


//--- GLOBAL SPEED TYPEDEF ---//
////////////////////////////////
typedef enum{
	LOW = 0x01,
	MED,
	HIGH,
}Speed_t;
////////////////////////////////


//--- COMMAND FEEDBACK TYPEDEF ---//
////////////////////////////////////
typedef enum{
	AUTO_HOME_DONE = 0x01,
	DISTANCE_MOVE_DONE,
	CURRENT_POINT_DONE,
}Feedback_t;
////////////////////////////////////


//--- DATA COMMUNICATION TYPEDEF ---//
//////////////////////////////////////
typedef struct{
	double
	move_value,
	Cartesian_pos[3],
	Cartesian_pos_req[3],
	
	Joint_angle[6],
	Joint_angle_req[6];
	
	uint8_t
	welding_point,
	welding_speed;

	Command_t type;
	Ctrl_Mode_t control_mode;
	Move_Var_t move_variable;
	Run_Mode_t running_mode;
	Run_State_t running_state;
	Welding_Pattern_t pattern_type;
	Data_type_t data_type;
	Mapping_State_t mapping_state;
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
static uint16_t checksum_generator(uint8_t* arr, size_t size);

/*TRANSMITING COMMAND*/
//---------------------------------------------------------------------------------------------------------------------------------------------------
void Send_auto_home(void);
void Send_mapping(uint8_t point_num, Data_type_t point_type, Welding_Pattern_t pattern_type, uint8_t welding_speed, Mapping_State_t map_state);
void Send_preview(uint16_t point_num);

void Send_move(Ctrl_Mode_t control_mode, Move_Var_t var_type, double value);
void Send_running(Run_State_t state);

void Req_data(void);
 
void Send_requested_data(double* cartesian_value, double* joint_value, uint8_t welding_point, Welding_Pattern_t pattern_type, uint8_t welding_speed);

void Send_motor_state(Motor_State_t state);
void Send_welder_state(Welder_State_t state);

void Send_feedback(Feedback_t fdbck);
//---------------------------------------------------------------------------------------------------------------------------------------------------

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
