#ifndef TB6600_DRIVER_H
#define TB6600_DRIVER_H

#include "stm32h7xx_hal.h"
#include "stdbool.h"

#define SYSTICK_LOAD (SystemCoreClock/1000000U)
#define SYSTICK_DELAY_CALIB (SYSTICK_LOAD >> 1)
#define DELAY_US(us) \
do { \
		 uint32_t start = SysTick->VAL; \
		 uint32_t ticks = (us * SYSTICK_LOAD)-SYSTICK_DELAY_CALIB;  \
		 while((start - SysTick->VAL) < ticks); \
} while (0)

#define MAX_SPEED 500

typedef struct{
	uint16_t
	gear_ratio,
	driver_step,
	running_step,
	prev_pos,
	now_pos;
	
	int16_t
	delta_pos;
	
	GPIO_TypeDef *PULSE_PORT;
	uint16_t PULSE_PIN;
	GPIO_TypeDef *DIR_PORT;
	uint16_t DIR_PIN;
	GPIO_TypeDef *ENA_PORT;
	uint16_t ENA_PIN;
}Driver_t;

typedef enum{
	DIRECTION_CW,
	DIRECTION_CCW
}Move_direction_t;

typedef enum{
	DRIVER_ENABLE,
	DRIVER_DISABLE
}Enable_t;

// ENABLE DRIVER
void Driver_set_power(Driver_t* driver, Enable_t power_set);
// MOVE STEPPER BY STEPPER STEP
void Move_stepper_step(Driver_t* driver, Move_direction_t direction, uint16_t value, uint16_t speed);
// MOVE STEPPER BY STEPPER ANGLE
void Move_stepper_angle(Driver_t* driver, uint16_t value, uint16_t speed);
// MOVE STEPPER ANGLE BY GEAR RATIO
void Move_stepper_angle_by_gear_ratio(Driver_t* driver, uint16_t value, uint16_t speed);

#endif
