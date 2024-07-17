/*
Author	: Alfonsus Giovanni Mahendra Putra
Date		: 10 Juli 2024
*/

#include "Keypad_Driver.h"

static uint8_t
row_number, col_number;

static const unsigned long debounce_time = 10;
static unsigned long prev_tick;

// KEYPAD INITIALIZE
void Keypad_Init(Keypad_t *keypad, char *userKeymap, uint8_t row_num, uint8_t col_num){
	row_number = row_num;
	col_number = col_num;
	
	//SET USER KEYMAP
	for(int i=0; i<row_num*col_num; i++){
		keypad->listed_key[i] = userKeymap[i];
	}
	
	//ACTIVATE ROW GPIO CLOCK
	for(int i=0; i<row_num; i++){
		if(keypad->row_port[i] == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
		else if(keypad->row_port[i] == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
		else if(keypad->row_port[i] == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
		else if(keypad->row_port[i] == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
	}
	
	//ACTIVATE COLUMN GPIO CLOCK
	for(int i=0; i<col_num; i++){
		if(keypad->col_port[i] == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
		else if(keypad->col_port[i] == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
		else if(keypad->col_port[i] == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();
		else if(keypad->col_port[i] == GPIOD) __HAL_RCC_GPIOD_CLK_ENABLE();
	}
}

// SCAN KEYPAD BUTTON
void Keypad_Scan(Keypad_t *keypad){
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	
	//SET ROW PIN AS INPUT
	for(int i=0; i<row_number; i++){
		GPIO_InitStruct.Pin = keypad->row_pin[i];
		GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
		GPIO_InitStruct.Pull = GPIO_PULLUP;
		HAL_GPIO_Init(keypad->row_port[i], &GPIO_InitStruct);
	}
	
	//SET COLUMN PIN AS OUTPUT
	for(int c=0; c<col_number; c++){
		GPIO_InitStruct.Pin = keypad->col_pin[c];
		GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
		HAL_GPIO_Init(keypad->col_port[c], &GPIO_InitStruct);
		
		//SET COLUMN PIN TO LOW
		HAL_GPIO_WritePin(keypad->col_port[c], keypad->col_pin[c], GPIO_PIN_RESET);
		
		for(int r=0; r<row_number; r++){
			//READ EACH ROW & COLUMN INPUT
			keypad->pressed[r][c] = !HAL_GPIO_ReadPin(keypad->row_port[r], keypad->row_pin[r]);
			if(keypad->pressed[r][c] == true) keypad->stored_key = keypad->listed_key[(r*col_number)+c];
		}
		//RESET COLUMN PIN TO HIGH
		HAL_GPIO_WritePin(keypad->col_port[c], keypad->col_pin[c], GPIO_PIN_SET);
	}
}

// READ KEYPAD CHARACTER
char Keypad_Read(Keypad_t *keypad){
	if((HAL_GetTick() - prev_tick) > debounce_time){
		keypad->stored_key = 0x00;
		Keypad_Scan(keypad);
		prev_tick = HAL_GetTick();
	}
	return keypad->stored_key;
}
