#include "Arduino.h"
#include "DMAChannel.h"
#include "at24c256.h"

/*EEPROM VARIABLE*/
//---------------------------------------
double 
array_pos_start[3],
array_pos_end[3],
array_angle[6];

uint8_t
saved_pos[24], read_saved_pos[24],
read_saved_posX[8],
read_saved_posY[8],
read_saved_posZ[8],

saved_angle[24], read_saved_angle[24],
read_saved_angle1[8],
read_saved_angle2[8],
read_saved_angle3[8],
read_saved_angle4[8],
read_saved_angle5[8],
read_saved_angle6[8];
//---------------------------------------


/*EEPROM MEMORY CONFIGURATION*/
//------------------------------------------------------
#define EEPROM_ADDR     0xA0
#define EEPROM_TIMEOUT  10
AT24C256 eeprom(AT24C_ADDRESS_0, Wire, EEPROM_TIMEOUT);
//------------------------------------------------------


/*PROTOTYPE FUNCTION*/
//----------------------------------------------------------------------------------------------------
void Save_WeldingPoint_Position(uint16_t page, uint8_t start_addr, double* pos_data, size_t size);
void Read_weldingPoint_Position(uint16_t page, uint8_t start_addr, double* stored_data, size_t size);
//----------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  Wire.begin();
}

void loop() {
  
}

// FUNCTION CODE BEGIN ------------------------------------------------------------------------------------------------------------------------

/*SAVE WELDING POINT POSITION*/
void Save_WeldingPoint_Position(uint16_t page, uint8_t start_addr, double* pos_data, size_t size){
  uint16_t
  page_addr = (page << 6) | start_addr; 

  for(int i=0; i<24; i++) saved_pos[i] = 0;
	for(size_t i=0; i<size; i++) memcpy(&saved_pos[i*8], &pos_data[i], sizeof(double));

  eeprom.writeBuffer(page_addr, saved_pos, sizeof(saved_pos));
}

/*READ WELDING POINT POSITION*/
void Read_weldingPoint_Position(uint16_t page, uint8_t start_addr, double* stored_data, size_t size){
  uint16_t
  page_addr = (page << 6) | start_addr; 
 
  for(int i=0; i<24; i++) read_saved_pos[i] = 0;
	for(int i=0; i<3; i++) stored_data[i] = 0;
	
	eeprom.readBuffer(page_addr, read_saved_pos, sizeof(read_saved_pos));
	
	for(int i=0; i<8; i++){
		read_saved_posX[i] = read_saved_pos[i];
		read_saved_posY[i] = read_saved_pos[i+8];
		read_saved_posZ[i] = read_saved_pos[i+16];
	}
	
	memcpy(&stored_data[0], read_saved_posX, sizeof(double));
	memcpy(&stored_data[1], read_saved_posY, sizeof(double));
	memcpy(&stored_data[2], read_saved_posZ, sizeof(double));
} 
