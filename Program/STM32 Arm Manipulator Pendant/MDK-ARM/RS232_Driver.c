/*
Author	: Alfonsus Giovanni Mahendra Putra
Date		: 10 Juli 2024
*/

#include "RS232_Driver.h"

static UART_HandleTypeDef* huart;
static uint8_t get_buff[64];

/*RS232 INITIALIZE*/
void RS232_Init(UART_HandleTypeDef* huart_handler){
	huart = huart_handler;
}

/*CHECKSUM FUNCTION*/
static uint16_t checksum_generator(uint8_t* arr, uint16_t size){
	uint16_t chksm = 0;
	for(uint16_t i = 0; i < size; i++) chksm += arr[i];
	return (chksm & 0xFFFF);
}

/*TRANSMITING COMMAND*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*SEND AUTO HOME COMMAND*/
void Send_auto_home(void){
	uint8_t tx_buff[BUFF_SIZE] = {HEADER1, HEADER2, AUTO_HOME_CMD};
	uint16_t chcksum = checksum_generator(tx_buff, sizeof(tx_buff));
	
	uint8_t
	chcksum_L = chcksum & 0xFF,
	chcksum_H = (chcksum >> 8) & 0xFF;
	
	tx_buff[62] = chcksum_L;
	tx_buff[63] = chcksum_H;
	
	HAL_UART_Transmit(huart, tx_buff, sizeof(tx_buff), RS232_TIMEOUT);
}

/*SEND MAPPING MODE COMMAND*/
void Send_mapping_mode(Memory_Pattern_t pattern_select){
	uint8_t mapping[BUFF_SIZE] = {HEADER1, HEADER2, MAPPING_MODE_CMD, pattern_select};
	uint16_t chcksum = checksum_generator(mapping, sizeof(mapping));
	
	uint8_t
	chcksum_L = chcksum & 0xFF,
	chcksum_H = (chcksum >> 8) & 0xFF;
	
	mapping[62] = chcksum_L;
	mapping[63] = chcksum_H;
	
	HAL_UART_Transmit(huart, mapping, sizeof(mapping), RS232_TIMEOUT);
}

/*SEND MAPPING DATA COMMAND*/
void Send_mapping_data(Point_Pattern_t point_num){
}

/*SEND MOVE COMMAND*/
void Send_move(Ctrl_Mode_t control_mode, double* value, size_t size){
}

/*SEND RUNNING COMMAND*/
void Send_running(Run_State_t state, Run_Mode_t mode, Memory_Pattern_t pattern_select, Speed_t speed){
}

/*SEND POSISITON REQ COMMAND*/
void Req_position(void){
	uint8_t req[BUFF_SIZE] = {HEADER1, HEADER2, REQ_POS_CMD};
	uint16_t chcksum = checksum_generator(req, sizeof(req));
	
	uint8_t
	chcksum_L = chcksum & 0xFF,
	chcksum_H = (chcksum >> 8) & 0xFF;
	
	req[62] = chcksum_L;
	req[63] = chcksum_H;
	
	HAL_UART_Transmit(huart, req, sizeof(req), RS232_TIMEOUT);
}

/*SEND ANGLE REQ COMMAND*/
void Req_angle(void){
	uint8_t req[BUFF_SIZE] = {HEADER1, HEADER2, REQ_ANG_CMD};
	uint16_t chcksum = checksum_generator(req, sizeof(req));
	
	uint8_t
	chcksum_L = chcksum & 0xFF,
	chcksum_H = (chcksum >> 8) & 0xFF;
	
	req[62] = chcksum_L;
	req[63] = chcksum_H;
	
	HAL_UART_Transmit(huart, req, sizeof(req), RS232_TIMEOUT);
}

/*SEND REQUESTED VALUE COMMAND*/
void Send_req(Req_t req_var, double* value, size_t size){
	uint8_t new_value;
	
	uint8_t send_req[BUFF_SIZE] = {HEADER1, HEADER2, SEND_REQ_CMD, req_var};
}

/*SEND MOTOR STATE COMMAND*/
void Send_motor_state(Motor_State_t state){
}

/*SEND WELDER STATE COMMAND*/
void Send_welder_state(Welder_State_t state){
}

/*SEND FEDDBACK COMMAND*/
void Send_feedback(Feedback_t fdbck){
	uint8_t feed_buff[BUFF_SIZE] = {HEADER1, HEADER2, FEEDBACK_CMD, fdbck};
	uint16_t chcksum = checksum_generator(feed_buff, sizeof(feed_buff));
	
	uint8_t
	chcksum_L = chcksum & 0xFF,
	chcksum_H = (chcksum >> 8) & 0xFF;
	
	feed_buff[62] = chcksum_L;
	feed_buff[63] = chcksum_H;
	
	HAL_UART_Transmit(huart, feed_buff, sizeof(feed_buff), RS232_TIMEOUT);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*RECIEVING COMMAND*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*START GET COM DATA*/
void Start_get_command(void){
	HAL_UART_Receive_DMA(huart, get_buff, sizeof(get_buff));
}

/*GET COM DATA*/
void Get_command(Data_Get_t* get){
	for(int i=0; i<sizeof(get_buff); i++){
		if(get_buff[i] == HEADER1 && get_buff[i+1] == HEADER2){
			//CHECK AUTO HOME COMMAND
			if(get_buff[i+2] == AUTO_HOME_CMD){
				get->type = AUTO_HOME;
			}
			
			//CHECK MAPPING MODE COMMAND
			else if(get_buff[i+2] == MAPPING_MODE_CMD){
				get->type = MAPPING;
			}
			
			//CHECK MOVE COMMAND
			else if(get_buff[i+2] == MOVE_POSITION_CMD){
				get->type = MOVE;
			}
			
			//CHECK START RUNNING COMMAND
			else if(get_buff[i+2] == RUNNING_CMD){
				get->type = RUN;
			}
			
			//CHECK REQ POSITION COMMAND
			else if(get_buff[i+2] == REQ_POS_CMD){
				get->type = REQ_POS;
			}
			
			//CHECK REQ ANGLE COMMAND
			else if(get_buff[i+2] == REQ_ANG_CMD){
				get->type = REQ_ANG;
			}
			
			//CHECK SEND REQ COMMAND
			else if(get_buff[i+2] == SEND_REQ_CMD){
				get->type = SEND_REQ;
			}
			
			//CEHCK MOTOR STATE COMMAND
			else if(get_buff[i+2] == MOTOR_STATE_CMD){
				get->type = MOTOR_STATE;
			}
			
			//CHECK WELDER STATE COMMAND
			else if(get_buff[i+2] == WELDER_STATE_CMD){
				get->type = WELDER_STATE;
				get->welder_state = get_buff[i+3];
			}
			
			//CHECK FEEDBACK COMMAND
			else if(get_buff[i+2] == FEEDBACK_CMD){
				get->type = FEEDBACK;
				get->feedback = get_buff[i+3];
			}
		}
	}
	HAL_UART_Receive_DMA(huart, get_buff, sizeof(get_buff));
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*TRANSMIT RECIEVE MONITOR*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint32_t check_state(void){
	return HAL_UART_GetState(huart);
}

uint32_t check_error(void){
	return HAL_UART_GetError(huart);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
