/*
Author	: Alfonsus Giovanni Mahendra Putra
Date		: 10 Juli 2024
*/

#include "RS232_Driver.h"

static UART_HandleTypeDef* huart;
static uint8_t rx_buff[BUFF_SIZE];

/*RS232 INITIALIZE*/
void RS232_Init(UART_HandleTypeDef* huart_handler){
	huart = huart_handler;
}

/*TRANSMITING COMMAND*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*SEND AUTO HOME COMMAND*/
void Send_auto_home(Data_Get_t* get){
	uint8_t tx_buff[BUFF_SIZE] = {HEADER1, HEADER2, HEADER3, AUTO_HOME_CMD};	
	HAL_UART_Transmit_IT(huart, tx_buff, sizeof(tx_buff));
}

/*SEND MAPPING MODE DATA COMMAND*/
void Send_mapping(Data_Get_t* get, uint8_t point_num, Data_type_t point_type, Welding_Pattern_t pattern_type, uint8_t welding_speed, Mapping_State_t map_state){
	uint8_t mapping_data[BUFF_SIZE] = {HEADER1, HEADER2, HEADER3, MAPPING_MODE_CMD, point_num, point_type, pattern_type, welding_speed, map_state};	
	HAL_UART_Transmit_IT(huart, mapping_data, sizeof(mapping_data));
}

/*SEND PREVIEW MODE DATA COMMAND*/
void Send_preview(Data_Get_t* get, uint16_t point_num){
	uint8_t preview[BUFF_SIZE] = {HEADER1, HEADER2, HEADER3, RUNNING_CMD, point_num};	
	HAL_UART_Transmit_IT(huart, preview, sizeof(preview));
}

/*SEND MOVE COMMAND*/
void Send_move(Data_Get_t* get, Ctrl_Mode_t control_mode, Move_Mode_t move_mode, Move_Var_t var_type, Move_Sign_t move_sign, float value){
	uint8_t move_pos[BUFF_SIZE] = {HEADER1, HEADER2, HEADER3, MOVE_POSITION_CMD, control_mode, move_mode, var_type, move_sign};
	
	uint8_t new_value[8];
	memcpy(new_value, &value, sizeof(float));
	for(int i=0; i<4; i++){
		move_pos[i+8] = new_value[i];
	}
	HAL_UART_Transmit_IT(huart, move_pos, sizeof(move_pos));
}

/*SEND RUNNING COMMAND*/
void Send_running(Data_Get_t* get, Run_State_t state){
	uint8_t run[BUFF_SIZE] = {HEADER1, HEADER2, HEADER3, RUNNING_CMD, state};	
	HAL_UART_Transmit_IT(huart, run, sizeof(run));
}

/*SEND DATA REQUEST COMMAND*/
void Req_data(Data_Get_t* get){
	uint8_t req[BUFF_SIZE] = {HEADER1, HEADER2, HEADER3, REQ_DATA_CMD};	
	HAL_UART_Transmit_IT(huart, req, sizeof(req));
}

/*SEND REQUESTED DATA COMMAND*/
void Send_requested_data(Data_Get_t* get, float* cartesian_value, float* joint_value, uint8_t welding_point, Welding_Pattern_t pattern_type, uint8_t welding_speed){		
	uint8_t send_req[BUFF_SIZE] = {HEADER1, HEADER2, HEADER3, SEND_REQ_CMD};
	
	memcpy(&send_req[4], &cartesian_value[0], sizeof(float));
	memcpy(&send_req[8], &cartesian_value[1], sizeof(float));
	memcpy(&send_req[12], &cartesian_value[2], sizeof(float));
	
	memcpy(&send_req[16], &joint_value[0], sizeof(float));
	memcpy(&send_req[20], &joint_value[1], sizeof(float));
	memcpy(&send_req[24], &joint_value[2], sizeof(float));
	memcpy(&send_req[28], &joint_value[3], sizeof(float));
	memcpy(&send_req[32], &joint_value[4], sizeof(float));
	memcpy(&send_req[36], &joint_value[5], sizeof(float));
				
//	memcpy(&get->Cartesian_send[0], &cartesian_value[0], sizeof(float));
//	memcpy(&get->Cartesian_send[1], &cartesian_value[1], sizeof(float));
//	memcpy(&get->Cartesian_send[2], &cartesian_value[2], sizeof(float));
//	
//	memcpy(&get->Joint_send[0], &joint_value[0], sizeof(float));
//	memcpy(&get->Joint_send[1], &joint_value[1], sizeof(float));
//	memcpy(&get->Joint_send[2], &joint_value[2], sizeof(float));
//	memcpy(&get->Joint_send[3], &joint_value[3], sizeof(float));
//	memcpy(&get->Joint_send[4], &joint_value[4], sizeof(float));
//	memcpy(&get->Joint_send[5], &joint_value[5], sizeof(float));
	
	send_req[37] = welding_point;
	send_req[38] = pattern_type;
	send_req[39] = welding_speed;	
	
	HAL_UART_Transmit(huart, send_req, sizeof(send_req), RS232_TIMEOUT);
}

/*SEND MOTOR STATE COMMAND*/
void Send_motor_state(Data_Get_t* get, Motor_State_t state){
	uint8_t motor_state[BUFF_SIZE] = {HEADER1, HEADER2, HEADER3, MOTOR_STATE_CMD, state};		
	HAL_UART_Transmit_IT(huart, motor_state, sizeof(motor_state));
}

/*SEND WELDER STATE COMMAND*/
void Send_welder_state(Data_Get_t* get, Welder_State_t state){
	uint8_t welder_state[BUFF_SIZE] = {HEADER1, HEADER2, HEADER3, WELDER_STATE_CMD, state};	
	HAL_UART_Transmit_IT(huart, welder_state, sizeof(welder_state));
}

/*SEND FEDDBACK COMMAND*/
void Send_feedback(Data_Get_t* get, Feedback_t fdbck){
	uint8_t feed_buff[BUFF_SIZE] = {HEADER1, HEADER2, HEADER3, FEEDBACK_CMD, fdbck};		
	HAL_UART_Transmit_IT(huart, feed_buff, sizeof(feed_buff));
}

/*SEND STANDBY COMMAND*/
void Send_standby(Data_Get_t* get){
	uint8_t standby_buff[BUFF_SIZE] = {HEADER1, HEADER2, HEADER3, STANDBY_CMD};		
	HAL_UART_Transmit_IT(huart, standby_buff, sizeof(standby_buff));
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*RECIEVING COMMAND*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*START GET COM DATA*/
void Start_get_command(Data_Get_t* get){
	HAL_UART_Receive_IT(huart, rx_buff, BUFF_SIZE);
}

/*GET COM DATA*/
void Get_command(Data_Get_t* get){
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
				
				uint8_t move_variable[4];
				
				for(int j=0; j<4; j++) move_variable[j] = rx_buff[i+8+j];
				memcpy(&get->move_value, move_variable, sizeof(float));
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
				converted_Xpos[4],
				converted_Ypos[4],
				converted_Zpos[4],
				
				converted_Joint1[4],
				converted_Joint2[4],
				converted_Joint3[4],
				converted_Joint4[4],
				converted_Joint5[4],
				converted_Joint6[4];
				
				for(int j=0; j<4; j++){
					converted_Xpos[j] 	= rx_buff[j+4]; 
					converted_Ypos[j] 	= rx_buff[j+8];
					converted_Zpos[j] 	= rx_buff[j+12]; 
					converted_Joint1[j] = rx_buff[j+16]; 
					converted_Joint2[j] = rx_buff[j+20]; 
					converted_Joint3[j] = rx_buff[j+24]; 
					converted_Joint4[j] = rx_buff[j+28]; 
					converted_Joint5[j] = rx_buff[j+32]; 
					converted_Joint6[j] = rx_buff[j+36]; 
				}
			
				if(IS_ALIGNED(converted_Xpos, 8) && IS_ALIGNED(converted_Ypos, 8) && IS_ALIGNED(converted_Zpos, 8)){
					get->data_status = POSDATA_MISSALIGNED;
				}
				else{
					get->data_status = POSDATA_ALIGNED;
					memcpy(&get->Cartesian_pos_req[0], converted_Xpos, sizeof(float));
					memcpy(&get->Cartesian_pos_req[1], converted_Ypos, sizeof(float));
					memcpy(&get->Cartesian_pos_req[2], converted_Zpos, sizeof(float));
				}
				
				if(IS_ALIGNED(converted_Joint1, 8) && IS_ALIGNED(converted_Joint2, 8) && IS_ALIGNED(converted_Joint3, 8) && 
					IS_ALIGNED(converted_Joint4, 8) && IS_ALIGNED(converted_Joint5, 8) && IS_ALIGNED(converted_Joint6, 8)){
					get->data_status = JOINTDATA_MISSALIGNED;
				}
				else{
					get->data_status = JOINTDATA_ALIGNED;
					memcpy(&get->Joint_angle_req[0], converted_Joint1, sizeof(float));
					memcpy(&get->Joint_angle_req[1], converted_Joint2, sizeof(float));
					memcpy(&get->Joint_angle_req[2], converted_Joint3, sizeof(float));
					memcpy(&get->Joint_angle_req[3], converted_Joint4, sizeof(float));
					memcpy(&get->Joint_angle_req[4], converted_Joint5, sizeof(float));
					memcpy(&get->Joint_angle_req[5], converted_Joint6, sizeof(float));
				}
				
				get->welding_point = rx_buff[i+37];
				get->pattern_type = (Welding_Pattern_t) rx_buff[i+38];
				get->welding_speed = rx_buff[i+39];	
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


/*TRANSMIT RECIEVE MONITOR*/
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint32_t check_state(void){
	return HAL_UART_GetState(huart);
}

uint32_t check_error(void){
	return HAL_UART_GetError(huart);
}
//---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
