/*
Author		: Alfonsus Giovanni Mahendra Putra
Date		: 27 November 2023
*/

#include "EEPROM_lib.h"

/*EEPROM INITIALIZE*/
void EEPROM_Init(EEPROM_t* mem, Memory_Size_t mem_model, uint8_t addr){
	mem->address = addr;
	Wire.begin();
	 
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
}

/*EEPROM PAGE RESET*/
void EEPROM_PageReset(EEPROM_t* mem, uint16_t page){
	uint16_t 
	dev_addr = mem->address | READ_MEM,
	page_addr = (page << 6) | 0x00;
	
	uint8_t data_reset[mem->page_size];
	memset(data_reset, 0xFF, mem->page_size);

	Wire.beginTransmission(dev_addr);
	Wire.write((page_addr >> 8));
	Wire.write((page_addr & 0xFF));
	for(int i=0; i<mem->page_size; i+=7){
		Wire.write(data_reset[i]);
	}
	Wire.endTransmission();
}

void EEPROM_ByteWrite(EEPROM_t* mem, uint16_t page, uint8_t start_addr, uint8_t data, size_t size){
	uint16_t 
	dev_addr = mem->address | READ_MEM,
	page_addr = (page << 6) | start_addr;

	Wire.beginTransmission(dev_addr);
	Wire.write((page_addr >> 8));
	Wire.write((page_addr & 0xFF));
	Wire.write(data);
	Wire.endTransmission();
}

void EEPROM_ByteRead(EEPROM_t* mem, uint16_t page, uint8_t start_addr, uint8_t data, size_t data_count){
	uint16_t 
	dev_addr = mem->address | READ_MEM,
	page_addr = (page << 6) | start_addr;

	Wire.beginTransmission(dev_addr);
	Wire.write((page_addr >> 8));
	Wire.write((page_addr & 0xFF));
	Wire.endTransmission();

	Wire.requestFrom(dev_addr, data_count);
	if (Wire.available()) data = Wire.read();
}

void EEPROM_PageWrite(EEPROM_t* mem, uint16_t page, uint8_t start_addr, uint8_t* data, size_t size){
	uint16_t 
	dev_addr = mem->address | READ_MEM,
	page_addr = (page << 6) | start_addr;

	Wire.beginTransmission(dev_addr);
	Wire.write((page_addr >> 8));
	Wire.write((page_addr & 0xFF));
	for(size_t i=0; i<size; i++) Wire.write(data[i]);
	Wire.endTransmission();
}

void EEPROM_PageRead(EEPROM_t* mem, uint16_t page, uint8_t start_addr, uint8_t* data, size_t data_count){
	uint16_t 
	dev_addr = mem->address | READ_MEM,
	page_addr = (page << 6) | start_addr;

	Wire.beginTransmission(dev_addr);
	Wire.write((page_addr >> 8));
	Wire.write((page_addr & 0xFF));
	Wire.endTransmission();
}