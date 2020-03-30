/*
 * motor.h
 *
 *  Created on: Feb 23, 2020
 *      Author: Adam
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "stm32f0xx_hal.h"

#define DIR_FORWARD 0
#define DIR_BACKWARD 1

#define STEPS_PER_DIV 25

void Move(uint8_t dir, uint16_t steps);
void Move_Eighth(uint8_t dir);

#endif /* INC_MOTOR_H_ */
