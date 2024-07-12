/*
Author	: Alfonsus Giovanni Mahendra Putra
Date		: 10 Juli 2024
*/

#ifndef RS3232_DRIVER_H
#define RS3232_DRIVER_H

#include "main.h"
#include "string.h"

#define HEADER1 0xFF
#define HEADER2 0xEE

#define SEND_CTRL_MODE			0x01
#define SEND_RUN_MODE				0x02
#define SEND_MOTOR_STATE		0x03
#define SEND_WELDER_STATE 	0x04
#define SEND_POSITION				0x05
#define SEND_ANGLE					0x06
#define SEND_REQ_POSITION		0x07
#define SEND_REQ_ANGLE			0x08


//--- CONTROL MODE TYPEDEF ---//
////////////////////////////////
typedef enum{
	POSITION_CTRL,
	ANGLE_CTRL,
}Ctrl_Mode_t;
////////////////////////////////


//--- RUNNING MODE TYPEDEF ---//
////////////////////////////////
typedef enum{
	HOMING_MODE,
	PATTERN_MODE,
	REPEAT_MODE,
}Run_Mode_t;
////////////////////////////////


//--- MOTOR STATE TYPEDEF ---//
////////////////////////////////
typedef enum{
	MOTOR_OFF,
	MOTOR_ON,
}Motor_State_t;
////////////////////////////////


//--- WELDER STATE TYPEDEF ---//
////////////////////////////////
typedef enum{
	WELDER_OFF,
	WELDER_ON,
}Welder_State_t;
////////////////////////////////


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
	
	Ctrl_Mode_t control_mode;
	Run_Mode_t running_mode;
	Motor_State_t motor_state;
}Data_Get_t;
//////////////////////////////////////


/*RS232 INITIALIZE*/
void RS232_Init(UART_HandleTypeDef* huart_handler);

/*CHECKSUM FUNCTION*/
static uint8_t checksum_generator(uint8_t* arr, uint8_t size);

/*TRANSMITING COMMAND*/
void Send_ctrl_mode(Ctrl_Mode_t mode);
void Send_run_mode(Run_Mode_t mode);
void Send_motor_state(uint8_t state);
void Send_welder_state(uint8_t state);
void Send_position(double* position);
void Send_angle(double* angle);
void Req_position(void);
void Req_angle(void);

/*RECIEVING COMMAND*/
void Start_get_command(void);
void Get_command(Data_Get_t* get);

#endif
