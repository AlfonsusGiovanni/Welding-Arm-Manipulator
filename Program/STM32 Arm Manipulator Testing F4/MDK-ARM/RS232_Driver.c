/*
Author	: Alfonsus Giovanni Mahendra Putra
Date		: 10 Juli 2024
*/

#include "RS232_Driver.h"

/*RS232 INITIALIZE*/
void RS232_Init(Data_Get_t* get, UART_HandleTypeDef* huart_handler){
	get->huart = huart_handler;
}

/*TRANSMITING COMMAND*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*SEND AUTO HOME COMMAND*/
void Send_auto_home(Data_Get_t* get){
	uint8_t tx_buff[BUFF_SIZE] = {HEADER1, HEADER2, HEADER3, AUTO_HOME_CMD};	
	HAL_UART_Transmit_IT(get->huart, tx_buff, sizeof(tx_buff));
}

/*SEND MAPPING MODE DATA COMMAND*/
void Send_mapping(Data_Get_t* get, uint8_t point_num, Data_type_t point_type, Welding_Pattern_t pattern_type, uint8_t welding_speed, Mapping_State_t map_state){
	uint8_t mapping_data[BUFF_SIZE] = {HEADER1, HEADER2, HEADER3, MAPPING_MODE_CMD, point_num, point_type, pattern_type, welding_speed, map_state};	
	HAL_UART_Transmit_IT(get->huart, mapping_data, sizeof(mapping_data));
}

/*SEND PREVIEW MODE DATA COMMAND*/
void Send_preview(Data_Get_t* get, uint16_t point_num){
	uint8_t preview[BUFF_SIZE] = {HEADER1, HEADER2, HEADER3, RUNNING_CMD, point_num};	
	HAL_UART_Transmit_IT(get->huart, preview, sizeof(preview));
}

/*SEND MOVE COMMAND*/
void Send_move(Data_Get_t* get, Ctrl_Mode_t control_mode, Move_Mode_t move_mode, Move_Var_t var_type, Move_Sign_t move_sign, double value){
	uint8_t move_pos[BUFF_SIZE] = {HEADER1, HEADER2, HEADER3, MOVE_POSITION_CMD, control_mode, move_mode, var_type, move_sign};
	
	uint8_t new_value[8];
	memcpy(new_value, &value, sizeof(double));
	for(int i=0; i<8; i++){
		move_pos[i+8] = new_value[i];
	}
	HAL_UART_Transmit_IT(get->huart, move_pos, sizeof(move_pos));
}

/*SEND RUNNING COMMAND*/
void Send_running(Data_Get_t* get, Run_State_t state){
	uint8_t run[BUFF_SIZE] = {HEADER1, HEADER2, HEADER3, RUNNING_CMD, state};	
	HAL_UART_Transmit_IT(get->huart, run, sizeof(run));
}

/*SEND DATA REQUEST COMMAND*/
void Req_data(Data_Get_t* get){
	uint8_t req[BUFF_SIZE] = {HEADER1, HEADER2, HEADER3, REQ_DATA_CMD};	
	HAL_UART_Transmit_IT(get->huart, req, sizeof(req));
}

/*SEND REQUESTED DATA COMMAND*/
void Send_requested_data(Data_Get_t* get, double* cartesian_value, double* joint_value, uint8_t welding_point, Welding_Pattern_t pattern_type, uint8_t welding_speed){	
	uint8_t 
	new_valueX[8],
	new_valueY[8],
	new_valueZ[8],
	
	new_valueA1[8],
	new_valueA2[8],
	new_valueA3[8],
	new_valueA4[8],
	new_valueA5[8],
	new_valueA6[8];
	
	uint8_t send_req[BUFF_SIZE] = {HEADER1, HEADER2, HEADER3, SEND_REQ_CMD};
	
	memcpy(new_valueX, &cartesian_value[0], sizeof(double));
	memcpy(new_valueY, &cartesian_value[1], sizeof(double));
	memcpy(new_valueZ, &cartesian_value[2], sizeof(double));
	
	memcpy(new_valueA1, &joint_value[0], sizeof(double));
	memcpy(new_valueA2, &joint_value[1], sizeof(double));
	memcpy(new_valueA3, &joint_value[2], sizeof(double));
	memcpy(new_valueA4, &joint_value[3], sizeof(double));
	memcpy(new_valueA5, &joint_value[4], sizeof(double));
	memcpy(new_valueA6, &joint_value[5], sizeof(double));
	
	for(int i=0; i<8; i++){
		send_req[i+4] 	= new_valueX[i]; 	// 4th byte	 - 11th byte
		send_req[i+12] 	= new_valueY[i]; 	// 12th byte - 19th byte
		send_req[i+20] 	= new_valueZ[i]; 	// 20th byte - 27th byte
		send_req[i+28] 	= new_valueA1[i]; // 28th byte - 35th byte
		send_req[i+36] 	= new_valueA2[i]; // 36th byte - 43rd byte
		send_req[i+44] 	= new_valueA3[i]; // 44th byte - 51st byte 
		send_req[i+52] 	= new_valueA4[i]; // 52nd byte - 59th byte
		send_req[i+60] 	= new_valueA5[i]; // 60th byte - 67th byte
		send_req[i+68] 	= new_valueA6[i]; // 68th byte - 76nd byte
	}
				
	memcpy(&get->Cartesian_send[0], &cartesian_value[0], sizeof(double));
	memcpy(&get->Cartesian_send[1], &cartesian_value[1], sizeof(double));
	memcpy(&get->Cartesian_send[2], &cartesian_value[2], sizeof(double));
	
	memcpy(&get->Joint_send[0], &joint_value[0], sizeof(double));
	memcpy(&get->Joint_send[1], &joint_value[1], sizeof(double));
	memcpy(&get->Joint_send[2], &joint_value[2], sizeof(double));
	memcpy(&get->Joint_send[3], &joint_value[3], sizeof(double));
	memcpy(&get->Joint_send[4], &joint_value[4], sizeof(double));
	memcpy(&get->Joint_send[5], &joint_value[5], sizeof(double));
	
	send_req[77] = welding_point;
	send_req[78] = pattern_type;
	send_req[79] = welding_speed;	
	
	if(!IS_ALIGNED(send_req, 8)){
		get->buff_status = BUFF_MISSALIGNED;
	}
	else get->buff_status = BUFF_ALIGNED;
	
	if(get->buff_status == BUFF_ALIGNED){
		HAL_UART_Transmit(get->huart, send_req, sizeof(send_req), RS232_TIMEOUT);
	}
}

/*SEND MOTOR STATE COMMAND*/
void Send_motor_state(Data_Get_t* get, Motor_State_t state){
	uint8_t motor_state[BUFF_SIZE] = {HEADER1, HEADER2, HEADER3, MOTOR_STATE_CMD, state};		
	HAL_UART_Transmit_IT(get->huart, motor_state, sizeof(motor_state));
}

/*SEND WELDER STATE COMMAND*/
void Send_welder_state(Data_Get_t* get, Welder_State_t state){
	uint8_t welder_state[BUFF_SIZE] = {HEADER1, HEADER2, HEADER3, WELDER_STATE_CMD, state};	
	HAL_UART_Transmit_IT(get->huart, welder_state, sizeof(welder_state));
}

/*SEND FEDDBACK COMMAND*/
void Send_feedback(Data_Get_t* get, Feedback_t fdbck){
	uint8_t tx_buff[80] = {0xFF, 0x5A, 0xFF, 0x0A, 0x01};
		HAL_UART_Transmit_IT(get->huart, tx_buff, 80);
}

/*SEND STANDBY COMMAND*/
void Send_standby(Data_Get_t* get){
	uint8_t standby_buff[BUFF_SIZE] = {HEADER1, HEADER2, HEADER3, STANDBY_CMD};		
	HAL_UART_Transmit_IT(get->huart, standby_buff, sizeof(standby_buff));
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*RECIEVING COMMAND*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*START GET COM DATA*/
void Start_get_command(Data_Get_t* get, uint8_t *rx_buff){
	HAL_UART_Receive_IT(get->huart, rx_buff, BUFF_SIZE);
}

/*GET COM DATA*/
void Get_command(Data_Get_t* get, uint8_t *rx_buff){
	if(!IS_ALIGNED(rx_buff, 8)){
		get->buff_status = BUFF_MISSALIGNED;
	}
	else get->buff_status = BUFF_ALIGNED;
	
	for(int i=0; i<BUFF_SIZE; i++){
		if(rx_buff[i] == HEADER1 && rx_buff[i+1] == HEADER2 && rx_buff[i+2] == HEADER3 && get->buff_status == BUFF_ALIGNED){
			//CHECK AUTO HOME COMMAND 
			if(rx_buff[i+3] == AUTO_HOME_CMD){
				get->type = AUTO_HOME;
			}
			
			//CHECK MAPPING MODE COMMAND
			else if(rx_buff[i+3] == MAPPING_MODE_CMD){
				get->type = MAPPING;
			}
			
			//CHECK MOVE COMMAND
			else if(rx_buff[i+3] == MOVE_POSITION_CMD){
				get->type = MOVE;
				
				get->control_mode = (Ctrl_Mode_t) rx_buff[i+4];
				get->move_mode = (Move_Mode_t) rx_buff[i+5];
				get->move_variable = (Move_Var_t) rx_buff[i+6];
				get->move_sign = (Move_Sign_t) rx_buff[i+7];
				
				uint8_t move_variable[8];
				
				for(int j=0; j<8; j++) move_variable[j] = rx_buff[i+8+j];
				memcpy(&get->move_value, move_variable, sizeof(double));
			}
			
			//CHECK START RUNNING COMMAND
			else if(rx_buff[i+3] == RUNNING_CMD){
				get->type = RUN;
			}
			
			//CHECK REQ POSITION COMMAND
			else if(rx_buff[i+3] == REQ_DATA_CMD){
				get->type = REQ_DATA;
			}
			
			//CHECK SEND REQ COMMAND
			else if(rx_buff[i+3] == SEND_REQ_CMD){
				get->type = SEND_REQ;
				
				uint8_t 
				converted_Xpos[8],
				converted_Ypos[8],
				converted_Zpos[8],
				
				converted_Joint1[8],
				converted_Joint2[8],
				converted_Joint3[8],
				converted_Joint4[8],
				converted_Joint5[8],
				converted_Joint6[8];
				
				for(int j=0; j<8; j++){
					converted_Xpos[j] 	= rx_buff[j+4]; 	// 4th byte	 - 11th byte
					converted_Ypos[j] 	= rx_buff[j+12]; // 12th byte - 19th byte
					converted_Zpos[j] 	= rx_buff[j+20]; // 20th byte - 27th byte
					converted_Joint1[j] = rx_buff[j+28]; // 28th byte - 35th byte
					converted_Joint2[j] = rx_buff[j+36]; // 36th byte - 43rd byte
					converted_Joint3[j] = rx_buff[j+44]; // 44th byte - 51st byte 
					converted_Joint4[j] = rx_buff[j+52]; // 52nd byte - 59th byte
					converted_Joint5[j] = rx_buff[j+60]; // 60th byte - 67th byte
					converted_Joint6[j] = rx_buff[j+68]; // 68th byte - 76nd byte
				}
			
				if(IS_ALIGNED(converted_Xpos, 8) && IS_ALIGNED(converted_Ypos, 8) && IS_ALIGNED(converted_Zpos, 8)){
					get->data_status = POSDATA_MISSALIGNED;
				}
				else{
					get->data_status = POSDATA_ALIGNED;
					memcpy(&get->Cartesian_pos_req[0], converted_Xpos, sizeof(double));
					memcpy(&get->Cartesian_pos_req[1], converted_Ypos, sizeof(double));
					memcpy(&get->Cartesian_pos_req[2], converted_Zpos, sizeof(double));
				}
				
				if(IS_ALIGNED(converted_Joint1, 8) && IS_ALIGNED(converted_Joint2, 8) && IS_ALIGNED(converted_Joint3, 8) && 
					IS_ALIGNED(converted_Joint4, 8) && IS_ALIGNED(converted_Joint5, 8) && IS_ALIGNED(converted_Joint6, 8)){
					get->data_status = JOINTDATA_MISSALIGNED;
				}
				else{
					get->data_status = JOINTDATA_ALIGNED;
					memcpy(&get->Joint_angle_req[0], converted_Joint1, sizeof(double));
					memcpy(&get->Joint_angle_req[1], converted_Joint2, sizeof(double));
					memcpy(&get->Joint_angle_req[2], converted_Joint3, sizeof(double));
					memcpy(&get->Joint_angle_req[3], converted_Joint4, sizeof(double));
					memcpy(&get->Joint_angle_req[4], converted_Joint5, sizeof(double));
					memcpy(&get->Joint_angle_req[5], converted_Joint6, sizeof(double));
				}
				
				get->welding_point = rx_buff[i+77];
				get->pattern_type = (Welding_Pattern_t) rx_buff[i+78];
				get->welding_speed = rx_buff[i+79];	
			}
			
			//CHECK MOTOR STATE COMMAND
			else if(rx_buff[i+3] == MOTOR_STATE_CMD){
				get->type = MOTOR_STATE;
				get->motor_state = (Motor_State_t) rx_buff[i+4];
			}
			
			//CHECK WELDER STATE COMMAND
			else if(rx_buff[i+3] == WELDER_STATE_CMD){
				get->type = WELDER_STATE;
				get->welder_state = (Welder_State_t) rx_buff[i+4];
			}
			
			//CHECK FEEDBACK COMMAND
			else if(rx_buff[i+3] == FEEDBACK_CMD){
				get->type = FEEDBACK;
				get->feedback = (Feedback_t) rx_buff[i+4];
			}
			
			//CHECK STANDBY COMMAND
			else if(rx_buff[i+3] == STANDBY_CMD){
				get->type = STANDBY;
			}
			memcpy(&get->data_buff, rx_buff, sizeof(rx_buff));
		}
	}
	memset(rx_buff, 0x00, BUFF_SIZE);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
