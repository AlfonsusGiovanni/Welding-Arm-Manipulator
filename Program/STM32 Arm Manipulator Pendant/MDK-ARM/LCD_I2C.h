/*
Author	: Alfonsus Giovanni
Date		: 22 October 2023
*/

#ifndef LCD_I2C_H
#define LCD_I2C_H

#include "main.h"
#include "stdbool.h"
#include "string.h"
#include "stdio.h"
#include "math.h"

/*---ADDRESS---*/
#define LCD_WRITE_ADDR 0x4E
#define LCD_READ_ADDR 0x4F
#define ROW_ADDR_TOP 0x80
#define ROW_ADDR_BOTTOM 0xC0
#define ENA_HIGH_RST_LOW_HB 0x0C
#define ENA_LOW_RST_LOW_HB 0x08
#define ENA_HIGH_RST_LOW_LB 0x0D
#define ENA_LOW_RST_LOW_LB 0x09


/*---COMMANDS---*/
#define LCD_CLEAR 0x01
#define LCD_HOME 0x02
#define LCD_ENTRYMODE 0x04
#define LCD_DISPLAYCTRL 0x08
#define LCD_CURSORSHIFT 0x10
#define LCD_FUNCTIONSET 0x20
#define LCD_SETCGRAM_ADDR 0x40
#define LCD_SETDDRAM_ADDR 0x80


/*---ENTRY MODE SET---*/
#define LCD_ENTRYINCREMENT 0x02
#define LCD_ENTRYDECREMENT 0x00
#define LCD_ENTRYSHIFTENABLE 0x01 
#define LCD_ENTRYSHIFTDISABLE 0x00


/*---ON/OFF SET---*/
#define LCD_ON 0x04
#define LCD_OFF 0x00
#define LCD_CURSORON 0x02
#define LCD_CURSOROFF 0x00
#define LCD_BLINKON 0x01
#define LCD_BLINKOFF 0x00


/*---DISPLAY CURSOR SHIFT SET*/
#define LCD_MOVEDISPLAY 0x08
#define LCD_MOVECURSOR 0x00
#define LCD_MOVERIGHT 0x04
#define LCD_MOVELEFT 0x00

/*---FUNCTION SET---*/
#define LCD_4BITMODE 0x00
#define LCD_8BITMODE 0x10
#define LCD_1LINE 0x00
#define LCD_2LINE 0x08
#define LCD_FTYPE_5X8 0x00
#define LCD_FTYPE_5X10 0x04

typedef enum{
	LCD_OK = 0x01U,
	LCD_ERR = 0x02U
}lcd_status;

lcd_status send_string(char data);
lcd_status send_cmd(char data);

void lcd_init(I2C_HandleTypeDef *set_i2c);
void lcd_set_cursor(int8_t col, int8_t row);
void lcd_printstr(char *string);
void lcd_printint(int input);
void lcd_printfloat(float input);
void lcd_clear(void);

#endif