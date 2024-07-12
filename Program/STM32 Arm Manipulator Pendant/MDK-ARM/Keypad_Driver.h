/*
Author	: Alfonsus Giovanni Mahendra Putra
Date		: 10 Juli 2024
*/

#ifndef KEYPAD_DRIVER_H
#define KEYPAD_DRIVER_H

#include "main.h"
#include "stdbool.h"

#define MAX_ROW 					5
#define MAX_COLUMN 				4
#define MAX_KEY 					MAX_ROW*MAX_COLUMN

#define makeKeymap(x) ((char*)x)

typedef struct{
	char 
	listed_key[MAX_KEY],
	stored_key;
	bool pressed[MAX_ROW][MAX_COLUMN];

	GPIO_TypeDef *row_port[MAX_ROW];
	uint16_t row_pin[MAX_ROW];
	
	GPIO_TypeDef *col_port[MAX_COLUMN];
	uint16_t col_pin[MAX_COLUMN];
}Keypad_t;

typedef enum{
	PRESSED,
	NOT_PRESSED
}State_t;

// KEYPAD INITIALIZE
void Keypad_Init(Keypad_t *keypad, char *userKeymap, uint8_t row_num, uint8_t col_num);

// SCAN KEYPAD BUTTON
void Keypad_Scan(Keypad_t *keypad);

// READ KEYPAD CHARACTER
char Keypad_Read(Keypad_t *keypad);

#endif
