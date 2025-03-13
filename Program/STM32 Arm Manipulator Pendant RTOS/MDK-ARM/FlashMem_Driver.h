#ifndef FLASHMEM_DRIVER_H
#define FLASHMEM_DRIVER_H

#include "stm32f1xx_hal.h"

uint32_t Flash_Write_Data (uint32_t StartPageAddress, uint32_t *Data, uint16_t numberofwords);
void Flash_Read_Data (uint32_t StartPageAddress, uint32_t *RxBuf, uint16_t numberofwords);

#endif