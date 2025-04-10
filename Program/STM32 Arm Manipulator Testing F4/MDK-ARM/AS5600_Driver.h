#ifndef AS5600_DRIVER_H
#define AS5600_DRIVER_H

#include "stm32f4xx_hal.h"
#include "stdbool.h"

/*--- AS5600 I2C ADDRESS ---*/
#define AS5600_ADDRESS  0x36 << 1

/*--- CONFIGURATION REGISTERS ADDRESS ---*/
#define ZMCO_REG				0x00
#define ZPOS_H_REG			0x01
#define ZPOS_L_REG			0x02
#define MPOS_H_REG			0x03
#define MPOS_L_REG			0x04
#define MANG_H_REG			0x05
#define MANG_L_REG			0x06
#define CONF_H_REG  		0x07 
#define CONF_L_REG			0x08

/*--- OUTPUT REGISTERS ADDRESS ---*/
#define RAW_ANG_H_REG		0x0C
#define RAW_ANG_L_REG		0x0D
#define ANGLE_H_REG			0x0E
#define ANGLE_L_REG			0x0F

/*--- STATUS REGISTERS ADDRESS*/
#define STATUS_REG			0x0B
#define AGC_REG					0x1A
#define MAG_H_REG				0x1B
#define MAG_L_REG				0x1C

/*--- BURNS REGISTER ADDRESS COMMAND ----*/
#define BURNS_REG				0xFF

/*--- REGISTER SETTING ---*/
#define PWR_NORM				0x00	
#define PWR_LPM1				0x01
#define PWR_LPM2				0x02
#define PWR_LPM3				0x03

#define HYST_OFF				0x00
#define HYST_LSB				0x01
#define HYST_2LSBs			0x02
#define HYST_3LSBs			0x03

#define ANALOG_FULL			0x00
#define ANALOG_REDUCE		0x01
#define DIGITAL_PWM			0x02

#define FREQ_115HZ			0x00
#define FREQ_230HZ			0x01
#define FREQ_460HZ			0x02
#define FREQ_920HZ			0x03

#define FILTER_16x			0x00
#define FILTER_8x				0x01
#define FILTER_4x				0x02
#define FILTER_2x				0x03

#define FTH_SLOW				0x00
#define FTH_6LSBs				0x01
#define FTH_7LSBs				0x02
#define FTH_9LSBs				0x03
#define FTH_18LSBs			0x04
#define FTH_21LSBs			0x05
#define FTH_24LSBs			0x06
#define FTH_10LSBs			0x07

#define WATCHDOG_OFF		0x00
#define WATCHDOG_ON			0x01

void AS5600_Init(I2C_HandleTypeDef *set_i2c, uint8_t output_type);
void AS5600_Burn(void);
uint8_t Get_Magnet_Status(void);

#endif
