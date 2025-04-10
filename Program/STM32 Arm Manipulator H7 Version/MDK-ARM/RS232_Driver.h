/*
	6-DOF ARM Manipulator RS-232 Communication Library
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
#define BUFF_SIZE 60

/*RS232 COM TIMEOUT*/
#define RS232_TIMEOUT 100

/*RS232 COM COMMAND*/
#define AUTO_HOME_CMD				0x01	// Robot Homing Command
#define MAPPING_CMD					0x02	// Robot Welding Point Mapping Command
#define MOVE_POSITION_CMD		0x03	// Move Robot Position Command
#define RUNNING_CMD					0x04	// Robot Run Command
#define REQ_DATA_CMD				0x05	// Request Data Command
#define SEND_REQ_CMD				0x06	// Send Requested Data Command
#define MOTOR_STATE_CMD			0x07	// Motor State Command
#define WELDER_STATE_CMD		0x08	// Welder State Command
#define FEEDBACK_CMD				0x09	// Feedback Command
#define RESET_CMD						0x0A	// Reset Command

//--- COMMAND TYPE TYPEDEF ---//
////////////////////////////////
typedef enum{
	AUTO_HOME = 0x01,
	MAPPING,
	MOVE,
	RUN,
	REQ_DATA,
	SEND_REQ,
	MOTOR_STATE,
	WELDER_STATE,
	FEEDBACK,
	RESET_STATE,
	NONE,
}Command_t;
////////////////////////////////


//--- MANUAL CONTROL MODE TYPEDEF ---//
///////////////////////////////////////
typedef enum{
	WORLD_CTRL = 0x01,
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
	AXIS_RX,
	AXIS_RY,
	AXIS_RZ,
	
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
	WELDING_MODE = 0x01,
	PREVIEW_MODE,
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
	MAPPED_START_POINT = 0x01,
	MAPPED_END_POINT
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
	NO_FEEDBACK,
	AUTO_HOME_DONE,
	SAVE_DONE,
	DISTANCE_MOVE_DONE,
	CURRENT_POINT_DONE,
	MAIN_ONLINE,
	PENDANT_ONLINE,
	ANGLE_SOFT_LIMIT,
	ANGLE_HARD_LIMIT,
	POINT_INVALID,
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
	uint8_t preview_point_num;
	uint8_t welding_point_num;
	uint8_t welding_point_type;
	
	uint16_t invalid_point;
	
	float move_value;
	float World_pos_req[3];
	float World_rot_req[3];
	float Joint_angle_req[6];

	float Cartesian_send[3];
	float Joint_send[6];

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
	Req_t requested_data;
	Motor_State_t motor_state;
	Welder_State_t welder_state;
	Speed_t running_speed;
	Feedback_t feedback;
	Buff_Status_t buff_status;
	Data_Status_t data_status;
}Data_Get_t;
//////////////////////////////////////


/*RS232 INITIALIZE*/
//-------------------------------------------------
void RS232_Init(UART_HandleTypeDef* huart_handler);
//-------------------------------------------------

/*TRANSMITING COMMAND*/
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Send_auto_home(Data_Get_t* get);
void Send_mapping(Data_Get_t* get, uint8_t point_num, Data_type_t point_type, Welding_Pattern_t pattern_type, Speed_t welding_speed, Mapping_State_t map_state);

void Send_move(Data_Get_t* get, Ctrl_Mode_t control_mode, Move_Mode_t move_mode, Move_Var_t var_type, Move_Sign_t move_sign, float value);
void Send_running(Data_Get_t* get, Run_State_t state, Run_Mode_t mode, uint16_t point_num);

void Req_data(Data_Get_t* get, Req_t data);
 
void Send_requested_data(Data_Get_t* get, float* world_pos, float* world_rot, float* joint_value, uint8_t welding_point, Welding_Pattern_t pattern_type, Speed_t welding_speed);

void Send_motor_state(Data_Get_t* get, Motor_State_t state);
void Send_welder_state(Data_Get_t* get, Welder_State_t state);

void Send_feedback(Data_Get_t* get, Feedback_t fdbck, uint16_t invalid_point);

void Send_state_reset(Data_Get_t* get);
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*RECIEVING COMMAND*/
//--------------------------------------
void Start_get_command(Data_Get_t* get);
void Get_command(Data_Get_t* get);
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
