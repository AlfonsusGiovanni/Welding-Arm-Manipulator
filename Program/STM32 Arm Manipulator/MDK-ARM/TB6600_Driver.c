#include "TB6600_Driver.h"

// ENABLE DRIVER
void Driver_set_power(Driver_t* driver, Enable_t power_set){
	if(power_set == DRIVER_DISABLE) HAL_GPIO_WritePin(driver->ENA_PORT, driver->ENA_PIN, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(driver->ENA_PORT, driver->ENA_PIN, GPIO_PIN_RESET);
}

// MOVE STEPPER BY STEPPER STEP
void Move_stepper_step(Driver_t* driver, Move_direction_t direction, uint16_t value, uint16_t speed){
	if(direction == DIRECTION_CW) HAL_GPIO_WritePin(driver->DIR_PORT, driver->DIR_PIN, GPIO_PIN_SET);
	else if(direction == DIRECTION_CCW) HAL_GPIO_WritePin(driver->DIR_PORT, driver->DIR_PIN, GPIO_PIN_RESET);
	
	if(speed <= MAX_SPEED) speed = MAX_SPEED;
	
	HAL_GPIO_WritePin(driver->ENA_PORT, driver->ENA_PIN, GPIO_PIN_RESET);
	
	for(int i=0; i<value; i++){
		HAL_GPIO_WritePin(driver->PULSE_PORT, driver->PULSE_PIN, GPIO_PIN_SET);
		DELAY_US(speed);
		HAL_GPIO_WritePin(driver->PULSE_PORT, driver->PULSE_PIN, GPIO_PIN_RESET);
		DELAY_US(speed);
	}
}

// MOVE STEPPER BY STEPPER ANGLE
void Move_stepper_angle(Driver_t* driver, uint16_t value, uint16_t speed){
	driver->now_pos = value;
	driver->delta_pos = value - driver->prev_pos;
	
	if(driver->now_pos > driver->prev_pos) HAL_GPIO_WritePin(driver->DIR_PORT, driver->DIR_PIN, GPIO_PIN_SET);
	else if(driver->now_pos < driver->prev_pos) HAL_GPIO_WritePin(driver->DIR_PORT, driver->DIR_PIN, GPIO_PIN_RESET);
	
	if(driver->delta_pos < 0) driver->running_step = (-1)*driver->delta_pos / (360/driver->driver_step);
	else driver->running_step = driver->delta_pos / (360/driver->driver_step);

	if(speed <= MAX_SPEED) speed = MAX_SPEED;
	
	HAL_GPIO_WritePin(driver->ENA_PORT, driver->ENA_PIN, GPIO_PIN_RESET);
	
	if(value != driver->prev_pos){
		for(int i=0; i<driver->running_step; i++){
			HAL_GPIO_WritePin(driver->PULSE_PORT, driver->PULSE_PIN, GPIO_PIN_SET);
			DELAY_US(speed);
			HAL_GPIO_WritePin(driver->PULSE_PORT, driver->PULSE_PIN, GPIO_PIN_RESET);
			DELAY_US(speed);
		}
	}
	
	driver->prev_pos = value;
}

// MOVE STEPPER ANGLE BY GEAR RATIO
void Move_stepper_angle_by_gear_ratio(Driver_t* driver, uint16_t value, uint16_t speed){
}
