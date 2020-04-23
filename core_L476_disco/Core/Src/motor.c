/*
 * motor.c
 *
 *  Created on: Feb 23, 2020
 *      Author: Adam
 */

#include "motor.h"
#include "main.h"
#include "stm32l4xx_hal.h"

uint16_t set_motor_steps = 0; // will be used externally

void Move(uint8_t dir, uint16_t steps)
{
	// set steps for motor
	set_motor_steps = steps;

	// set direction
	HAL_GPIO_WritePin(DIR_GPIO_Port, DIR_Pin, (dir == DIR_FORWARD) ? GPIO_PIN_RESET : GPIO_PIN_SET);

	// start timer to move motor
	TIM2->CR1 |= 0x0001;
}

void Move_Eighth(uint8_t dir)
{
	Move(dir, STEPS_PER_DIV);
}
