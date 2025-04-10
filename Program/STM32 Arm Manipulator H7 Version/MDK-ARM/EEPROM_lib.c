/*
	AT-24 Series EEPROM Library
	Author	: Alfonsus Giovanni Mahendra Putra
	Date		: 27 November 2023
*/

#include "EEPROM_lib.h"

/*EEPROM INITIALIZE*/
void EEPROM_Init(I2C_HandleTypeDef* I2C_handler, EEPROM_t* mem, Memory_Size_t mem_model, uint8_t addr){
	mem->hi2c = I2C_handler;
	mem->address = addr;
	
	if(mem_model == MEM_SIZE_8Kb){
		mem->page = EEPROM_8Kb_PAGE;
		mem->page_size = EEPROM_8Kb_PAGE_SIZE;
	}
	
	else if(mem_model == MEM_SIZE_64Kb){
		mem->page = EEPROM_64Kb_PAGE;
		mem->page_size = EEPROM_64Kb_PAGE_SIZE;
	}
	
	else if(mem_model == MEM_SIZE_128Kb){
		mem->page = EEPROM_128Kb_PAGE;
		mem->page_size = EEPROM_128Kb_PAGE_SIZE;
	}
	
	else if(mem_model == MEM_SIZE_256Kb){
		mem->page = EEPROM_256Kb_PAGE;
		mem->page_size = EEPROM_256Kb_PAGE_SIZE;
	}
	
	else if(mem_model == MEM_SIZE_512Kb){
		mem->page = EEPROM_512Kb_PAGE;
		mem->page_size = EEPROM_512Kb_PAGE_SIZE;
	}
}

/*EEPROM PAGE RESET*/
void EEPROM_PageReset(EEPROM_t* mem, uint16_t page, uint8_t start_addr){
	uint16_t 
	dev_addr = mem->address | WRITE_MEM,
	page_addr = (page << 6) | start_addr;
	
	uint8_t data_reset[mem->page_size];
	memset(data_reset, 0x00, mem->page_size);
	
	if(HAL_I2C_Mem_Write(mem->hi2c, dev_addr, page_addr, I2C_MEMADD_SIZE_16BIT, data_reset, mem->page_size, I2C_Timeout) == HAL_OK) mem->status = EEPROM_OK;
	else mem->status = EEPROM_TIMEOUT;
}

/*EEPROM BYTE WRITE*/
void EEPROM_ByteWrite(EEPROM_t* mem, uint16_t page, uint8_t start_addr, uint8_t data, uint16_t size){
	uint16_t 
	dev_addr = mem->address | WRITE_MEM,
	page_addr = (page << 6) | start_addr;
	
	if(HAL_I2C_Mem_Write(mem->hi2c, dev_addr, page_addr, I2C_MEMADD_SIZE_16BIT, &data, size, I2C_Timeout) == HAL_OK) mem->status = EEPROM_OK;
	else mem->status = EEPROM_TIMEOUT;
}

/*EEPROM BYTE READ*/
void EEPROM_ByteRead(EEPROM_t* mem, uint16_t page, uint8_t start_addr, uint8_t* data, uint16_t data_count){
	uint16_t 
	dev_addr = mem->address | READ_MEM,
	page_addr = (page << 6) | start_addr;
	
	if(HAL_I2C_Mem_Read(mem->hi2c, dev_addr, page_addr, I2C_MEMADD_SIZE_16BIT, data, data_count, I2C_Timeout) == HAL_OK) mem->status = EEPROM_OK;
	else mem->status = EEPROM_TIMEOUT;
}

/*EEPROM PAGE WRITE*/
void EEPROM_PageWrite(EEPROM_t* mem, uint16_t page, uint8_t start_addr, uint8_t* data, uint16_t size){
	uint16_t 
	dev_addr = mem->address | WRITE_MEM,
	page_addr = (page << 6) | start_addr;
	
	if(HAL_I2C_Mem_Write(mem->hi2c, dev_addr, page_addr, I2C_MEMADD_SIZE_16BIT, data, size, I2C_Timeout) == HAL_OK) mem->status = EEPROM_OK;
	else mem->status = EEPROM_TIMEOUT;
	
	mem->status = EEPROM_OK;
}

/*EEPROM PAGE READ*/
void EEPROM_PageRead(EEPROM_t* mem, uint16_t page, uint8_t start_addr, uint8_t* data, uint16_t data_count){
	uint16_t 
	dev_addr = mem->address | READ_MEM,
	page_addr = (page << 6) | start_addr;

	if(HAL_I2C_Mem_Read(mem->hi2c, dev_addr, page_addr, I2C_MEMADD_SIZE_16BIT, data, data_count, I2C_Timeout) == HAL_OK) mem->status = EEPROM_OK;
	else mem->status = EEPROM_TIMEOUT;
}
