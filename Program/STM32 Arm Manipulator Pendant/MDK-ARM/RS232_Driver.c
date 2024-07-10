#include "RS232_Driver.h"

static UART_HandleTypeDef* huart;

/*RS232 INITIALIZE*/
void RS232_Init(UART_HandleTypeDef* huart_handler){
	huart = huart_handler;
}

/*CHECKSUM FUNCTION*/
static uint8_t checksum_generator(uint8_t* arr, uint8_t size){
	uint8_t chksm = 0;
	for(uint8_t i = 0; i < size; i++) chksm += arr[i];
	return (chksm & 0xFF);
}

/*SEND CONTROL MODE COMMAND*/
void Send_ctrl_mode(Ctrl_Mode_t mode);
