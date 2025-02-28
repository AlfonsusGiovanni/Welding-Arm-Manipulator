/*
Author	: Alfonsus Giovanni Mahendra Putra
Date		: 10 Juli 2024
*/

#ifndef RS3232_DRIVER_H
#define RS3232_DRIVER_H

#include "main.h"
#include "string.h"
#include "stdbool.h"

#define ALIGNED_8 __attribute__((aligned(8)))
#define IS_ALIGNED(ptr, align) (((uintptr_t)(ptr) % (align)) == 0)

/*RS232 COM HEADER*/
#define HEADER1 0xFF
#define HEADER2 0x5A
#define HEADER3 0xFF

/*RS232 COM BUFFER SIZE*/
#define BUFF_SIZE 80

/*RS232 COM TIMEOUT*/
#define RS232_TIMEOUT 100

/*RS232 COM COMMAND*/
#define AUTO_HOME_CMD				0x01	// Robot Homing Command
#define MAPPING_MODE_CMD		0x02	// Robot Welding Point Mapping Command
#define PREVIEW_MODE_CMD		0x03	// Robot Welding Point Preview Command
#define MOVE_POSITION_CMD		0x04	// Move Robot Position Command
#define RUNNING_CMD					0x05	// Robot Run Command
#define REQ_DATA_CMD				0x06	// Request Data Command
#define SEND_REQ_CMD				0x07	// Send Requested Data Command
#define MOTOR_STATE_CMD			0x08	// Motor State Command
#define WELDER_STATE_CMD		0x09	// Welder State Command
#define FEEDBACK_CMD				0x0A	// Feedback Command
#define STANDBY_CMD					0x0B	// Standby Command

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
	STANDBY,
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


//--- MOVE MODE TYPEDEF ---//
/////////////////////////////
typedef enum{
	CONTINUOUS = 0x01,
	DISTANCE,
	STEP,
}Move_Mode_t;
/////////////////////////////


//--- MOVE VARIABLE TYPEDEF ---//
/////////////////////////////////
typedef enum{
	AXIS_X = 0x01,
	AXIS_Y,
	AXIS_Z,
	
	JOINT_1,
	JOINT_2,
	JOINT_3,
	JOINT_4,
	JOINT_5,
	JOINT_6,
}Move_Var_t;
/////////////////////////////////


//--- VARIABLE MOVE SIGN TYPEDEF ---//
//////////////////////////////////////
typedef enum{
	SIGNED_VAR = 0x01,
	UNSIGNED_VAR,
}Move_Sign_t;
//////////////////////////////////////


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
	NOT_SET,
	DOT,
	LINEAR,
	CIRCULAR,
	WAVE,
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
	ANGLE_LIMIT_WARNING,
	MAIN_ONLINE,
	PENDANT_ONLINE
}Feedback_t;
////////////////////////////////////


//--- RS232 BUFF STATUS ---//
/////////////////////////////
typedef enum{
	BUFF_ALIGNED = 0x01,
	BUFF_MISSALIGNED,
}Buff_Status_t;
/////////////////////////////


//--- RS232 DATA STATUS ---//
/////////////////////////////
typedef enum{
	POSDATA_ALIGNED = 0x01,
	POSDATA_MISSALIGNED,
	JOINTDATA_ALIGNED,
	JOINTDATA_MISSALIGNED,
}Data_Status_t;
/////////////////////////////


//--- DATA COMMUNICATION TYPEDEF ---//
//////////////////////////////////////
typedef struct ALIGNED_8{	
	bool msg_sent;
	bool msg_get;
	
	uint8_t data_buff[BUFF_SIZE];
	uint8_t check_data_buff[BUFF_SIZE];
	uint8_t welding_point;
	uint8_t welding_speed;

	uint8_t padding2[7];
	
	double move_value;
	double Cartesian_pos[3];
	double Cartesian_pos_req[3];
	double Joint_angle[6];
	double Joint_angle_req[6];

	double Cartesian_send[3];
	double Joint_send[6];
	
	UART_HandleTypeDef* huart;

	Command_t type;
	Ctrl_Mode_t control_mode;
	Move_Mode_t move_mode;
	Move_Var_t move_variable;
	Move_Sign_t move_sign;
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
	Buff_Status_t buff_status;
	Data_Status_t data_status;
}Data_Get_t;
//////////////////////////////////////


/*RS232 INITIALIZE*/
//------------------------------------------------------------------
void RS232_Init(Data_Get_t* get, UART_HandleTypeDef* huart_handler);
//------------------------------------------------------------------

/*TRANSMITING COMMAND*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Send_auto_home(Data_Get_t* get);
void Send_mapping(Data_Get_t* get, uint8_t point_num, Data_type_t point_type, Welding_Pattern_t pattern_type, uint8_t welding_speed, Mapping_State_t map_state);
void Send_preview(Data_Get_t* get, uint16_t point_num);

void Send_move(Data_Get_t* get, Ctrl_Mode_t control_mode, Move_Mode_t move_mode, Move_Var_t var_type, Move_Sign_t move_sign, double value);
void Send_running(Data_Get_t* get, Run_State_t state);

void Req_data(Data_Get_t* get);
 
void Send_requested_data(Data_Get_t* get, double* cartesian_value, double* joint_value, uint8_t welding_point, Welding_Pattern_t pattern_type, uint8_t welding_speed);

void Send_motor_state(Data_Get_t* get, Motor_State_t state);
void Send_welder_state(Data_Get_t* get, Welder_State_t state);

void Send_feedback(Data_Get_t* get, Feedback_t fdbck);

void Send_standby(Data_Get_t* get);
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*RECIEVING COMMAND*/
//--------------------------------------
void Start_get_command(Data_Get_t* get, uint8_t *rx_buff);
void Get_command(Data_Get_t* get, uint8_t *rx_buff);
//--------------------------------------

/*COMMAND RESET*/
//----------------------------------
void Reset_command(Data_Get_t* get);
//----------------------------------

/*TRANSMIT RECIEVE MONITOR*/
//---------------------------
uint32_t check_state(void);
uint32_t check_error(void);
//---------------------------

#endif
