/*
Author	: Alfonsus Giovanni Mahendra Putra
Date		: 27 November 2023
*/

#include "Arduino.h"
#include "Wire.h"

#ifndef EEPROM_LIB_H
#define EEPROM_LIB_H

#define EEPROM_8Kb_PAGE 		64
#define EEPROM_8Kb_PAGE_SIZE 	16
#define EEPROM_64Kb_PAGE 		256
#define EEPROM_64Kb_PAGE_SIZE 	32
#define EEPROM_128Kb_PAGE 		256
#define EEPROM_128Kb_PAGE_SIZE 	64
#define EEPROM_256Kb_PAGE 		512
#define EEPROM_256Kb_PAGE_SIZE 	64

#define WRITE_MEM		0x00
#define READ_MEM		0x01

#define I2C_Timeout 100

typedef enum{
	MEM_SIZE_8Kb,
	MEM_SIZE_64Kb,
	MEM_SIZE_128Kb,
	MEM_SIZE_256Kb,
}Memory_Size_t;

typedef enum{
	EEPROM_OK = 0x01U,
	EEPROM_TIMEOUT = 0x02U,
}EEPROM_Status_t;

typedef struct{
	uint16_t 
	page;
	
	uint8_t
	page_size,
	data_write,
	data_read,
	status,
	address;

	TwoWire* I2C_handler;
}EEPROM_t;

/*EEPROM INITIALIZE*/
void EEPROM_Init(EEPROM_t* mem, Memory_Size_t mem_model, uint8_t addr);

/*EEPROM PAGE RESET*/
void EEPROM_PageReset(EEPROM_t* mem, uint16_t page);

/*EEPROM BYTE WRITE & READ*/
void EEPROM_ByteWrite(EEPROM_t* mem, uint16_t page, uint8_t start_addr, uint8_t data, size_t size);
void EEPROM_ByteRead(EEPROM_t* mem, uint16_t page, uint8_t start_addr, uint8_t data, size_t data_count);

/*EEPROM PAGE WRITE & READ*/
void EEPROM_PageWrite(EEPROM_t* mem, uint16_t page, uint8_t start_addr, uint8_t* data, size_t size);
void EEPROM_PageRead(EEPROM_t* mem, uint16_t page, uint8_t start_addr, uint8_t* data, size_t data_count);
#endif
