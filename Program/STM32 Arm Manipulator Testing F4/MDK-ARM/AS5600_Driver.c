#include "AS5600_Driver.h"
#include "main.h"

static I2C_HandleTypeDef *hi2c;

void AS5600_Init(I2C_HandleTypeDef *set_i2c, uint8_t output_type){
	hi2c = set_i2c;
	
	uint8_t Data;
	Data = PWR_NORM;
	Data = Data|(HYST_OFF<<2);
	Data = Data|(output_type<<4);
	Data = Data|(FREQ_920HZ<<6);
	HAL_I2C_Mem_Write(hi2c, AS5600_ADDRESS , CONF_L_REG, 1, &Data, 1, 100);
	
	Data = 0;
	Data = Data|(FTH_10LSBs);
	Data = Data|(FILTER_16x<<2);
	Data = Data|(WATCHDOG_ON<<4);
	HAL_I2C_Mem_Write(hi2c, AS5600_ADDRESS , CONF_H_REG, 1, &Data, 1, 100);
}

void AS5600_Burn(void){
	uint8_t burn_cmd = 0x40;
  HAL_I2C_Mem_Write(hi2c, AS5600_ADDRESS, BURNS_REG, 1, &burn_cmd, 1, 100);
}

uint8_t Get_Magnet_Status(void){
	uint8_t Data[1];
	HAL_I2C_Mem_Read(hi2c, AS5600_ADDRESS , STATUS_REG, 1, &Data[1], 1, 100);
	return (Data[0]>>3)&(7);
}
