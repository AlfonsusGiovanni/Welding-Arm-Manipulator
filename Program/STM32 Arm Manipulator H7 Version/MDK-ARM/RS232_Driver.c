/*
	6-DOF ARM Manipulator RS-232 Communication Library
	Author	: Alfonsus Giovanni Mahendra Putra
	Date		: Juli 2024
*/

#include "RS232_Driver.h"

static UART_HandleTypeDef* huart;

/*RS232 INITIALIZE*/
void RS232_Init(UART_HandleTypeDef* huart_handler){
	huart = huart_handler;
}

/*TRANSMITING COMMAND*/
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*SEND SETTING COMMAND*/
void Send_setting_data(Data_Get_t* get, Setting_Mode_t set_mode,  Cal_Mode_t cal_mode, Zeroing_Select_t sel_joint, Speed_t set_speed){
	uint8_t tx_buff[BUFF_SIZE] = {HEADER1, HEADER2, HEADER3, SETTING_CMD, set_mode, cal_mode, sel_joint, set_speed};	
	HAL_UART_Transmit_IT(huart, tx_buff, sizeof(tx_buff));
}

/*SEND MAPPING MODE DATA COMMAND*/
void Send_mapping(Data_Get_t* get, uint16_t pt_num, Data_Point_t pt_type, Welding_Pattern_t pattern, Speed_t speed, Axis_Offset_t axis, Mapping_State_t map_state){
	uint8_t mapping_data[BUFF_SIZE] = {HEADER1, HEADER2, HEADER3, MAPPING_CMD, (pt_num >> 8) & 0xFF, pt_num & 0xFF, pt_type, pattern, speed, axis, map_state};	
	HAL_UART_Transmit_IT(huart, mapping_data, sizeof(mapping_data));
}

/*SEND PREVIEW MODE DATA COMMAND*/
void Send_preview(Data_Get_t* get, uint16_t point_num){
	uint8_t preview[BUFF_SIZE] = {HEADER1, HEADER2, HEADER3, RUNNING_CMD, point_num};	
	HAL_UART_Transmit_IT(huart, preview, sizeof(preview));
}

/*SEND MOVE COMMAND*/
void Send_move(Data_Get_t* get, Ctrl_Mode_t control_mode, Move_Mode_t move_mode, Move_Var_t var_type, Move_Sign_t move_sign, float value){
	uint8_t move_pos[BUFF_SIZE];
	
	move_pos[0] = HEADER1;
	move_pos[1] = HEADER2;
	move_pos[2] = HEADER3;
	move_pos[3] = MOVE_POSITION_CMD;
	move_pos[4] = control_mode;
	move_pos[5] = move_mode;
	move_pos[6] = var_type;
	move_pos[7] = move_sign;
	memcpy(&move_pos[8], &value, sizeof(float));
	
	HAL_UART_Transmit_IT(huart, move_pos, sizeof(move_pos));
}

/*SEND RUNNING COMMAND*/
void Send_running(Data_Get_t* get, Run_State_t state, Run_Mode_t mode, uint16_t point_num){
	uint8_t run[BUFF_SIZE] = {HEADER1, HEADER2, HEADER3, RUNNING_CMD, state, mode, (point_num >> 8) & 0xFF, point_num & 0xFF};	
	HAL_UART_Transmit_IT(huart, run, sizeof(run));
}

/*SEND DATA REQUEST COMMAND*/
void Req_data(Data_Get_t* get, Req_t data){
	uint8_t req[BUFF_SIZE] = {HEADER1, HEADER2, HEADER3, REQ_DATA_CMD, data};	
	HAL_UART_Transmit_IT(huart, req, sizeof(req));
}

/*SEND REQUESTED DATA COMMAND*/
void Send_requested_data(Data_Get_t* get, float* world_pos, float* world_rot, float* joint_value, uint8_t welding_point, Welding_Pattern_t pattern_type, Speed_t welding_speed){	
	uint8_t send_req[BUFF_SIZE];
	
	send_req[0] = HEADER1;
	send_req[1] = HEADER2;
	send_req[2] = HEADER3;
	
	send_req[3] = SEND_REQ_CMD;
	
	memcpy(&send_req[4], &world_pos[0], sizeof(float));
	memcpy(&send_req[8], &world_pos[1], sizeof(float));
	memcpy(&send_req[12], &world_pos[2], sizeof(float));
	
	memcpy(&send_req[16], &world_rot[0], sizeof(float));
	memcpy(&send_req[20], &world_rot[1], sizeof(float));
	memcpy(&send_req[24], &world_rot[2], sizeof(float));
	
	memcpy(&send_req[28], &joint_value[0], sizeof(float));
	memcpy(&send_req[32], &joint_value[1], sizeof(float));
	memcpy(&send_req[36], &joint_value[2], sizeof(float));
	memcpy(&send_req[40], &joint_value[3], sizeof(float));
	memcpy(&send_req[44], &joint_value[4], sizeof(float));
	memcpy(&send_req[48], &joint_value[5], sizeof(float));
	
	send_req[53] = welding_point;
	send_req[54] = pattern_type;
	send_req[55] = welding_speed;	
	
	if(!IS_ALIGNED(send_req, 8)){
		get->tx_buff_status = BUFF_MISSALIGNED;
	}
	else{
		get->tx_buff_status = BUFF_ALIGNED;
		HAL_UART_Transmit(huart, send_req, sizeof(send_req), RS232_TIMEOUT);
	}
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
void Send_feedback(Data_Get_t* get, Feedback_t fdbck, uint16_t num){
	uint8_t feed_buff[BUFF_SIZE] = {HEADER1, HEADER2, HEADER3, FEEDBACK_CMD, fdbck, ((num >> 8) & 0xFF), (num & 0xFF)};		
	HAL_UART_Transmit(huart, feed_buff, sizeof(feed_buff), RS232_TIMEOUT);
}

/*SEND STANDBY COMMAND*/
void Send_state_reset(Data_Get_t* get){
	uint8_t standby_buff[BUFF_SIZE] = {HEADER1, HEADER2, HEADER3, RESET_CMD};		
	HAL_UART_Transmit_IT(huart, standby_buff, sizeof(standby_buff));
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*RECIEVING COMMAND*/
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

/*START GET COM DATA*/
void Start_get_command(Data_Get_t* get){
	HAL_UART_Receive_IT(huart, get->data_buff, BUFF_SIZE);
}

/*GET COM DATA*/
void Get_command(Data_Get_t* get){
	if(!IS_ALIGNED(get->data_buff, 8)){
		get->rx_buff_status = BUFF_MISSALIGNED;
	}
	else get->rx_buff_status = BUFF_ALIGNED;
	
	for(int i=0; i<BUFF_SIZE; i++){
		if(get->data_buff[i] == HEADER1 && get->data_buff[i+1] == HEADER2 && get->data_buff[i+2] == HEADER3){
			//CHECK AUTO HOME COMMAND 
			if(get->data_buff[i+3] == SETTING_CMD){
				get->type = SETTING;
				get->setting_mode = (Setting_Mode_t) get->data_buff[i+4];
				get->calibration_mode = (Cal_Mode_t) get->data_buff[i+5];
				get->joint_zeroing = (Zeroing_Select_t) get->data_buff[i+6];
				get->running_speed = (Speed_t) get->data_buff[i+7];
			}
			
			//CHECK MAPPING MODE COMMAND
			else if(get->data_buff[i+3] == MAPPING_CMD){
				get->type = MAPPING;
				
				get->welding_point_num = (uint16_t)(get->data_buff[i+4] << 8 | get->data_buff[i+5]);
				get->data_type = (Data_Point_t) get->data_buff[i+6];
				get->pattern_type = (Welding_Pattern_t) get->data_buff[i+7];
				get->running_speed = (Speed_t) get->data_buff[i+8];
				get->axis_offset = (Axis_Offset_t) get->data_buff[i+9];
				get->mapping_state = (Mapping_State_t) get->data_buff[i+10];
			}
			
			//CHECK MOVE COMMAND
			else if(get->data_buff[i+3] == MOVE_POSITION_CMD){
				get->type = MOVE;
				
				get->control_mode = (Ctrl_Mode_t) get->data_buff[i+4];
				get->move_mode = (Move_Mode_t) get->data_buff[i+5];
				get->move_variable = (Move_Var_t) get->data_buff[i+6];
				get->move_sign = (Move_Sign_t) get->data_buff[i+7];
				memcpy(&get->move_value, &get->data_buff[i+8], sizeof(float));
			}
			
			//CHECK START RUNNING COMMAND
			else if(get->data_buff[i+3] == RUNNING_CMD){
				get->type = RUN;
				get->running_state = (Run_State_t) get->data_buff[i+4];
				get->running_mode = (Run_Mode_t) get->data_buff[i+5];
				get->preview_point_num = (uint16_t)((get->data_buff[i+6] << 8) | get->data_buff[i+7]);
			}
			
			//CHECK REQ POSITION COMMAND
			else if(get->data_buff[i+3] == REQ_DATA_CMD){
				get->type = REQ_DATA;
				get->requested_data = (Req_t) get->data_buff[i+4];
			}
			
			//CHECK SEND REQ COMMAND
			else if(get->data_buff[i+3] == SEND_REQ_CMD){
				get->type = SEND_REQ;
				
				memcpy(&get->World_pos_req[0], &get->data_buff[i+4], sizeof(float));
				memcpy(&get->World_pos_req[1], &get->data_buff[i+8], sizeof(float));
				memcpy(&get->World_pos_req[2], &get->data_buff[i+12], sizeof(float));
				
				memcpy(&get->World_rot_req[0], &get->data_buff[i+16], sizeof(float));
				memcpy(&get->World_rot_req[1], &get->data_buff[i+20], sizeof(float));
				memcpy(&get->World_rot_req[2], &get->data_buff[i+24], sizeof(float));
				
				memcpy(&get->Joint_angle_req[0], &get->data_buff[i+28], sizeof(float));
				memcpy(&get->Joint_angle_req[1], &get->data_buff[i+32], sizeof(float));
				memcpy(&get->Joint_angle_req[2], &get->data_buff[i+36], sizeof(float));
				memcpy(&get->Joint_angle_req[3], &get->data_buff[i+40], sizeof(float));
				memcpy(&get->Joint_angle_req[4], &get->data_buff[i+44], sizeof(float));
				memcpy(&get->Joint_angle_req[5], &get->data_buff[i+48], sizeof(float));
				
				get->welding_point_num = get->data_buff[i+53];
				get->pattern_type = (Welding_Pattern_t) get->data_buff[i+54];
				get->running_speed = (Speed_t)get->data_buff[i+55];	
			}
			
			//CHECK MOTOR STATE COMMAND
			else if(get->data_buff[i+3] == MOTOR_STATE_CMD){
				get->type = MOTOR_STATE;
				get->motor_state = (Motor_State_t) get->data_buff[i+4];
			}
			
			//CHECK WELDER STATE COMMAND
			else if(get->data_buff[i+3] == WELDER_STATE_CMD){
				get->type = WELDER_STATE;
				get->welder_state = (Welder_State_t) get->data_buff[i+4];
			}
			
			//CHECK FEEDBACK COMMAND
			else if(get->data_buff[i+3] == FEEDBACK_CMD){
				get->type = FEEDBACK;
				get->feedback = (Feedback_t) get->data_buff[i+4];
				get->feedback_num = get->data_buff[i+5] << 8 | get->data_buff[i+6];
			}
			
			//CHECK RESET STATE COMMAND
			else if(get->data_buff[i+3] == RESET_CMD){
				get->type = RESET_STATE;
			}
			memcpy(&get->data_buff, get->check_data_buff, sizeof(get->data_buff));
		}
	}
	memset(get->data_buff, 0x00, BUFF_SIZE);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*RESET COMMAND*/
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Reset_command(Data_Get_t* get){
	get->type = NO_COMMAND;
	get->setting_mode = NO_SETTING;
	get->joint_zeroing = NO_SELECTION;
	get->calibration_mode = NO_CALIBRATION;
	get->control_mode = NO_CTRL;
	get->move_mode = NO_MOVE;
	get->move_variable = NO_VAR;
	get->move_sign = NO_SIGN;
	get->running_mode = NO_RUN_MODE;
	get->running_state = NO_RUN_STATE;
	get->pattern_type = NO_PATTERN;
	get->data_type = NO_DATA_TYPE;
	get->mapping_state = NO_MAP_STATE;
	get->axis_offset = NO_AXIS_OFFSET;
	get->requested_data = NO_REQ;
	get->motor_state = NO_MOTOR_STATE;
	get->welder_state = NO_WELDER_STATE;
	get->running_speed = NO_SPEED;
	get->feedback = NO_FEEDBACK;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


/*TRANSMIT RECIEVE MONITOR*/
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint32_t check_state(void){
	return HAL_UART_GetState(huart);
}

uint32_t check_error(void){
	return HAL_UART_GetError(huart);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
