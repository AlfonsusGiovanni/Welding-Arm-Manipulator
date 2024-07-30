/*
Author	: Alfonsus Giovanni Mahendra Putra
Date		: 22 October 2023
*/

#include "LCD_I2C.h"

#define I2C_TIMEOUT 100

static I2C_HandleTypeDef *hi2c;
static uint8_t buff_tx[4];

// LCD COMMAND SEND
lcd_status send_cmd(char data){
	buff_tx[0] = (data & 0xF0) | ENA_HIGH_RST_LOW_HB;
	buff_tx[1] = (data & 0xF0) | ENA_LOW_RST_LOW_HB;
	buff_tx[2] = ((data << 4) & 0xF0) | ENA_HIGH_RST_LOW_HB;
	buff_tx[3] = ((data << 4) & 0xF0) | ENA_LOW_RST_LOW_HB;
	
	if(HAL_I2C_Master_Transmit(hi2c, LCD_WRITE_ADDR, (uint8_t*)buff_tx, 4, I2C_TIMEOUT) == HAL_OK) return LCD_OK;
	else return LCD_ERR;
}

// LCD DATA SEND
lcd_status send_data(char data){
	buff_tx[0] = (data & 0xF0) | ENA_HIGH_RST_LOW_LB;
	buff_tx[1] = (data & 0xF0) | ENA_LOW_RST_LOW_LB;
	buff_tx[2] = ((data << 4) & 0xF0) | ENA_HIGH_RST_LOW_LB;
	buff_tx[3] = ((data << 4) & 0xF0) | ENA_LOW_RST_LOW_LB;
	
	if(HAL_I2C_Master_Transmit(hi2c, LCD_WRITE_ADDR, (uint8_t*)buff_tx, 4, I2C_TIMEOUT) == HAL_OK) return LCD_OK;
	else return LCD_ERR;
}

// LCD INITIALIZE
void lcd_init(I2C_HandleTypeDef *set_i2c){
	hi2c = set_i2c;
	uint8_t cmd;
	
	cmd = LCD_FUNCTIONSET | LCD_4BITMODE;
	send_cmd(cmd);
	HAL_Delay(10);
	
	cmd = LCD_FUNCTIONSET | LCD_4BITMODE | LCD_2LINE | LCD_FTYPE_5X8;
	send_cmd(cmd);
	
	HAL_Delay(1);
	cmd = LCD_DISPLAYCTRL | LCD_OFF;
	send_cmd(cmd);
	
	HAL_Delay(1);
	cmd = LCD_CLEAR;
	send_cmd(cmd);
	
	HAL_Delay(2);
	cmd = LCD_ENTRYMODE | LCD_ENTRYINCREMENT | LCD_ENTRYSHIFTDISABLE;
	send_cmd(cmd);
	
	HAL_Delay(1);
	cmd = LCD_DISPLAYCTRL | LCD_ON;
	send_cmd(cmd);
}

// LCD SET CURSOR LOCATION
void lcd_set_cursor(int8_t col, int8_t row){
	uint8_t cmd;
	uint16_t row_offsets[4] = {ROW_ADDR_1, ROW_ADDR_2, ROW_ADDR_3, ROW_ADDR_4};
	
	cmd = LCD_SETDDRAM_ADDR | (col + row_offsets[row]);
	
	send_cmd(cmd);
}

// LCD PRINT STRING VARIABLE
void lcd_printstr(char *string){
	while(*string) send_data(*string++);
}

// LCD PRINT INTEGER VARIABLE
void lcd_printint(int input){
	char strbuffer[50];
	snprintf(strbuffer, sizeof(strbuffer), "%d", input);
	
	lcd_printstr(strbuffer);
}

// LCD PRINT FLOAT VARIABLE
void lcd_printfloat(float input, uint8_t precision){
	char strbuffer[20];
	if(precision == 1) sprintf(strbuffer, "%.1f", input);
	if(precision == 2) sprintf(strbuffer, "%.2f", input);
	if(precision == 3) sprintf(strbuffer, "%.3f", input);
	
	lcd_printstr(strbuffer);
}

// LCD CLEAR
void lcd_clear(){
	send_cmd(LCD_CLEAR);
	HAL_Delay(2);
}

// LCD BACKLIGHT SET
void lcd_backlight(uint8_t state){
	
}
