/*
 * timing.c
 *
 *  Created on: Apr 21, 2020
 *      Author: Adam
 */

#include "timing.h"
#include "flash.h"
#include "main.h"
#include "stm32l4xx_hal.h"

extern RTC_HandleTypeDef hrtc;

uint8_t _doses_per_day = 0;
uint8_t _dose_times[4] = {0x20, 0x24, 0x28, 0x32}; 	// TODO: sort this at input from USB
								// corresponds to seconds, for the sake of the demo

void SetTimeSettings(uint8_t doses_per_day, uint8_t* dose_times)
{
	_doses_per_day = doses_per_day;
	for (uint8_t i = 0; i < doses_per_day; i++)
	{
		_dose_times[i] = dose_times[i];
	}

	WriteTiming(doses_per_day, dose_times);
}

void LoadTimeSettings()
{
	ReadTiming(&_doses_per_day, _dose_times);
}

void SetAlarm()
{
	uint8_t next_dose_time;

	// get current time
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD); // not used, but required for GetTime to work

	// determine next dosage time
	if ( (sTime.Seconds >= _dose_times[0]) && (sTime.Seconds < _dose_times[1]) )
	{
		next_dose_time = _dose_times[1];
	}
	else if ( (sTime.Seconds >= _dose_times[1]) && (sTime.Seconds < _dose_times[2]) )
	{
		next_dose_time = _dose_times[2];
	}
	else if ( (sTime.Seconds >= _dose_times[2]) && (sTime.Seconds < _dose_times[3]) )
	{
		next_dose_time = _dose_times[3];
	}
	else
	{
		next_dose_time = _dose_times[0];
	}

	// set Alarm A
	HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);
	RTC_AlarmTypeDef sAlarm = {0};
	sAlarm.AlarmTime.Hours = 0x00;
	sAlarm.AlarmTime.Minutes = 0x0;
	sAlarm.AlarmTime.Seconds = next_dose_time;
	sAlarm.AlarmTime.SubSeconds = 0x0;
	sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
	sAlarm.AlarmMask = RTC_ALARMMASK_DATEWEEKDAY;
	sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
	sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
	sAlarm.AlarmDateWeekDay = 0x1;
	sAlarm.Alarm = RTC_ALARM_A;
	if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
	{
	  Error_Handler();
	}
}


