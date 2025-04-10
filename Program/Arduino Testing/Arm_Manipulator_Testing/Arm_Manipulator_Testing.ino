#include "at24c256.h"

/*EEPROM ADDRESS SET*/
//----------------------------------
#define EEPROM_ADDR             0xA0
#define EEPROM_TIMEOUT          10

#define START_POINT_BYTE_ADDR		0x00
#define END_POINT_BYTE_ADDR			0x18
#define PATTERN_BYTE_ADDR				0x30
#define ROTATE_MODE_BYTE_ADDR		0x31
#define ROTATE_VALUE_BYTE_ADDR	0x32
//----------------------------------


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
AT24C256 eeprom(AT24C_ADDRESS_0, Wire, EEPROM_TIMEOUT);
//------------------------------------------------------


/*TESTING VARIABLE*/
//------------------------------------------
double myDouble = -40.96;
uint8_t test_char[8] = {0x7b, 0x14, 0xae, 0x47, 0xe1, 0x7a, 0x44, 0xc0};

double 
test_pos_start[3] = {-40.96, 12.15, -37.42},
test_pos_end[3] = {-20.96, 2.15, -37.42};
//------------------------------------------


/*PROTOTYPE FUNCTION*/
//----------------------------------------------------------------------------------------------------
void Save_WeldingPoint_Position(uint16_t page, uint8_t start_addr, double* pos_data, size_t size);
void Read_WeldingPoint_Position(uint16_t page, uint8_t start_addr, double* stored_data, size_t size);
//----------------------------------------------------------------------------------------------------

void setup() {
  Serial.begin(115200);
  Wire.begin();

  //Save_WeldingPoint_Position(0X00, START_POINT_BYTE_ADDR, test_pos_start, sizeof(test_pos_start));
  //delay(10);
  //Read_WeldingPoint_Position(0X00, START_POINT_BYTE_ADDR, array_pos_start, sizeof(array_pos_start));
  //delay(10);
  
  /*
  Serial.print(array_pos_start[0]);
  Serial.print("\t");
  Serial.print(array_pos_start[1]);
  Serial.print("\t");
  Serial.println(array_pos_start[2]);
  */

  memcpy(read_saved_posX, &myDouble, sizeof(double));
  Serial.println(read_saved_posX[0], HEX);
  /*
  memcpy(read_saved_posX, &myDouble, 8);
  Serial.print("Double value as bytes: ");
  for (size_t i = 0; i < 8; i++) {
    Serial.print(read_saved_posX[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  */
}

void loop() {
  
}

// FUNCTION CODE BEGIN ------------------------------------------------------------------------------------------------------------------------

/*SAVE WELDING POINT POSITION*/
void Save_WeldingPoint_Position(uint16_t page, uint8_t start_addr, double* pos_data, size_t size){
  uint16_t
  page_addr = (page << 6) | start_addr; 

	for(size_t i=0; i<size; i++) memcpy(&saved_pos[i*8], &pos_data[i], sizeof(double));

  eeprom.writeBuffer(page_addr, saved_pos, sizeof(saved_pos));
}

/*READ WELDING POINT POSITION*/
void Read_WeldingPoint_Position(uint16_t page, uint8_t start_addr, double* stored_data, size_t size){
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
