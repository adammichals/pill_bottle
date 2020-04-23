/*
 * timing.h
 *
 *  Created on: Apr 21, 2020
 *      Author: Adam
 */

#ifndef SRC_TIMING_H_
#define SRC_TIMING_H_

#include "stm32l4xx_hal.h"

void SetTimeSettings(uint8_t doses_per_day, uint8_t* dose_times);
void LoadTimeSettings();
void SetAlarm();


#endif /* SRC_TIMING_H_ */
