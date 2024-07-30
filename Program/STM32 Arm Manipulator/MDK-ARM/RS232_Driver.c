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
static uint16_t checksum_generator(uint8_t* arr, size_t size){
	uint16_t chksm = 0;
	for(uint16_t i=0; i<size; i++) chksm += arr[i];
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
	
	tx_buff[62] = chcksum_H;
	tx_buff[63] = chcksum_L;
	
	HAL_UART_Transmit(huart, tx_buff, sizeof(tx_buff), RS232_TIMEOUT);
}

/*SEND MAPPING MODE DATA COMMAND*/
void Send_mapping(uint16_t point_num, Welding_Point_t point_type, Welding_Pattern_t pattern_type, uint8_t speed){
	uint8_t mapping_data[BUFF_SIZE] = {HEADER1, HEADER2, MAPPING_MODE_CMD, point_type, pattern_type, speed};
	uint16_t chcksum = checksum_generator(mapping_data, sizeof(mapping_data));
	
	uint8_t
	chcksum_L = chcksum & 0xFF,
	chcksum_H = (chcksum >> 8) & 0xFF;
	
	mapping_data[62] = chcksum_H;
	mapping_data[63] = chcksum_L;
	
	HAL_UART_Transmit(huart, mapping_data, sizeof(mapping_data), RS232_TIMEOUT);
}

/*SEND MOVE COMMAND*/
void Send_move(Ctrl_Mode_t control_mode, double* value, size_t size){
}

/*SEND RUNNING COMMAND*/
void Send_running(Run_State_t state, Run_Mode_t mode){
}

/*SEND POSISITON REQ COMMAND*/
void Req_position(void){
	uint8_t req[BUFF_SIZE] = {HEADER1, HEADER2, REQ_POS_CMD};
	uint16_t chcksum = checksum_generator(req, sizeof(req));
	
	uint8_t
	chcksum_L = chcksum & 0xFF,
	chcksum_H = (chcksum >> 8) & 0xFF;
	
	req[62] = chcksum_H;
	req[63] = chcksum_L;
	
	HAL_UART_Transmit(huart, req, sizeof(req), RS232_TIMEOUT);
}

/*SEND ANGLE REQ COMMAND*/
void Req_angle(void){
	uint8_t req[BUFF_SIZE] = {HEADER1, HEADER2, REQ_ANG_CMD};
	uint16_t chcksum = checksum_generator(req, sizeof(req));
	
	uint8_t
	chcksum_L = chcksum & 0xFF,
	chcksum_H = (chcksum >> 8) & 0xFF;
	
	req[62] = chcksum_H;
	req[63] = chcksum_L;
	
	HAL_UART_Transmit(huart, req, sizeof(req), RS232_TIMEOUT);
}

/*SEND REQUESTED VALUE COMMAND*/
void Send_req(Req_t req_var, double* value, uint16_t *mapped_point, size_t size){
	uint8_t 
	new_valueX[8],
	new_valueY[8],
	new_valueZ[8];
	
	uint8_t
	new_valueA1[8],
	new_valueA2[8],
	new_valueA3[8],
	new_valueA4[8],
	new_valueA5[8],
	new_valueA6[8];
	
	uint8_t send_req[BUFF_SIZE] = {HEADER1, HEADER2, SEND_REQ_CMD, req_var};
	
	if(req_var == SEND_POSITION){		
		memcpy(new_valueX, &value[0], sizeof(double));
		memcpy(new_valueY, &value[1], sizeof(double));
		memcpy(new_valueZ, &value[2], sizeof(double));
		
		for(int i=0; i<8; i++){
			send_req[i+4] 	= new_valueX[i]; // 4th bit - 11th bit
			send_req[i+12] 	= new_valueY[i]; // 12th bit - 19th bit
			send_req[i+20] 	= new_valueZ[i]; // 20th bit - 27 bit
		}
	}
	
	else if(req_var == SEND_ANGLE){
		memcpy(new_valueA1, &value[0], sizeof(double));
		memcpy(new_valueA2, &value[1], sizeof(double));
		memcpy(new_valueA3, &value[2], sizeof(double));
		memcpy(new_valueA4, &value[3], sizeof(double));
		memcpy(new_valueA5, &value[4], sizeof(double));
		memcpy(new_valueA6, &value[5], sizeof(double));
		
		for(int i=0; i<8; i++){
			send_req[i+4] 	= new_valueA1[i]; // 4th bit - 11th bit
			send_req[i+12] 	= new_valueA2[i]; // 12th bit - 19th bit
			send_req[i+20] 	= new_valueA3[i]; // 20th bit - 27th bit
			send_req[i+28] 	= new_valueA4[i]; // 28th bit - 35th bit
			send_req[i+36] 	= new_valueA5[i]; // 36th bit - 43th bit
			send_req[i+44] 	= new_valueA6[i]; // 44th bit - 51th bit
		}
	}
	
	else if(req_var == SEND_MAPPED_POINT){
		for(size_t i=0; i<size; i++) send_req[i+4] = mapped_point[i];
	}
	
	uint16_t chcksum = checksum_generator(send_req, sizeof(send_req));
	
	uint8_t
	chcksum_L = chcksum & 0xFF,
	chcksum_H = (chcksum >> 8) & 0xFF;
	
	send_req[62] = chcksum_H;
	send_req[63] = chcksum_L;
	
	HAL_UART_Transmit(huart, send_req, sizeof(send_req), RS232_TIMEOUT);
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
	
	feed_buff[62] = chcksum_H;
	feed_buff[63] = chcksum_L;
	
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
			
			//CHECK MOTOR STATE COMMAND
			else if(get_buff[i+2] == MOTOR_STATE_CMD){
				get->type = MOTOR_STATE;
			}
			
			//CHECK WELDER STATE COMMAND
			else if(get_buff[i+2] == WELDER_STATE_CMD){
				get->type = WELDER_STATE;
				if(get_buff[i+3] == WELDER_ON) get->welder_state = WELDER_ON;
				if(get_buff[i+3] == WELDER_OFF) get->welder_state = WELDER_OFF;
			}
			
			//CHECK FEEDBACK COMMAND
			else if(get_buff[i+2] == FEEDBACK_CMD){
				get->type = FEEDBACK;
				if(get_buff[i+3] == AUTO_HOME_DONE) get->feedback = AUTO_HOME_DONE;
				if(get_buff[i+3] == DISTANCE_MOVE_DONE) get->feedback = DISTANCE_MOVE_DONE;
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
