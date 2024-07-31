/*
Author	: Alfonsus Giovanni Mahendra Putra
Date		: 22 October 2023
*/

#ifndef LCD_I2C_H
#define LCD_I2C_H

#include "main.h"
#include "stdbool.h"
#include "string.h"
#include "stdio.h"
#include "stdlib.h"
#include "math.h"


/*---ADDRESS---*/
#define I2C_EXPANDER_ADDR		0x27
#define LCD_WRITE_ADDR 			0x4E
#define LCD_READ_ADDR 			0x4F
#define ROW_ADDR_1 					0x00
#define ROW_ADDR_2 					0x40
#define ROW_ADDR_3 					0x14
#define ROW_ADDR_4 					0x54
#define ENA_HIGH_RST_LOW_HB 0x0C
#define ENA_LOW_RST_LOW_HB 	0x08
#define ENA_HIGH_RST_LOW_LB	0x0D
#define ENA_LOW_RST_LOW_LB 	0x09


/*---COMMANDS---*/
#define LCD_CLEAR 				0x01
#define LCD_HOME 					0x02
#define LCD_ENTRYMODE 		0x04
#define LCD_DISPLAYCTRL 	0x08
#define LCD_CURSORSHIFT 	0x10
#define LCD_FUNCTIONSET 	0x20
#define LCD_SETCGRAM_ADDR 0x40
#define LCD_SETDDRAM_ADDR	0x80


/*---ENTRY MODE SET---*/
#define LCD_ENTRYINCREMENT 		0x02
#define LCD_ENTRYDECREMENT 		0x00
#define LCD_ENTRYSHIFTENABLE 	0x01 
#define LCD_ENTRYSHIFTDISABLE	0x00


/*---ON/OFF SET---*/
#define LCD_ON 					0x04
#define LCD_OFF 				0x00
#define LCD_CURSORON 		0x02
#define LCD_CURSOROFF 	0x00
#define LCD_BLINKON 		0x01
#define LCD_BLINKOFF 		0x00


/*---DISPLAY CURSOR SHIFT SET*/
#define LCD_MOVEDISPLAY	0x08
#define LCD_MOVECURSOR 	0x00
#define LCD_MOVERIGHT 	0x04
#define LCD_MOVELEFT 		0x00


/*---FUNCTION SET---*/
#define LCD_4BITMODE 		0x00
#define LCD_8BITMODE 		0x10
#define LCD_1LINE 			0x00
#define LCD_2LINE			 	0x08
#define LCD_FTYPE_5X8 	0x00
#define LCD_FTYPE_5X10 	0x04


/*---BACKLIGHT SET---*/
#define BACKLIGHT_ON		0x08
#define BACKLIGHT_OFF		0x00


/*---LCD STATUS TYPEDEF---*/
typedef enum{
	LCD_OK = 0x01U,
	LCD_ERR = 0x02U
}lcd_status;

// LCD COMMAND SEND
lcd_status send_cmd(char data);

// LCD DATA SEND
lcd_status send_data(char data);

// LCD INITIALIZE
void lcd_init(I2C_HandleTypeDef *set_i2c);

// LCD SET CURSOR LOCATION
void lcd_set_cursor(int8_t col, int8_t row);

// LCD PRINT STRING VARIABLE
void lcd_printstr(char *string);

// LCD PRINT INTEGER VARIABLE
void lcd_printint(int input);

// LCD PRINT FLOAT VARIABLE
void lcd_printfloat(float input, uint8_t precision);

// LCD CLEAR
void lcd_clear(void);

// LCD BACKLIGHT SET
void lcd_backlight(uint8_t state);

#endif
