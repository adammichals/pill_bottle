/*
 * flash.c
 *
 *  Created on: Feb 29, 2020
 *      Author: Adam
 */

#include "flash.h"
#include "main.h"
#include "stm32l4xx_hal.h"

extern SPI_HandleTypeDef hspi1;
extern RTC_HandleTypeDef hrtc;
uint16_t log_addr = ADDRESS_START_LOG;

/*
 * Name:	ReadFlash
 * Use:		Read data stored in the flash
 * Inputs:	-address:	the address in memory to start reading from
 * 			-dataRx:	the array to read data into
 * 			-dataSize:	the size of the data to read, in bytes
 * Outputs:	none
 */
void ReadFlash(uint16_t address, uint8_t* dataRx, uint16_t dataSize)
{
	_WaitForReady();
	uint8_t header[4] = {OPCODE_READ, 0x00, (address & 0xFF00) >> 8, (address & 0x00FF)};

	HAL_GPIO_WritePin(flash_nSS_GPIO_Port, flash_nSS_Pin, GPIO_PIN_RESET);	// set nSS low to begin
	HAL_SPI_Transmit(&hspi1, header, 4, 10000);								// transmit header
	HAL_SPI_Receive(&hspi1, dataRx, dataSize, 10000);							// receive data
	HAL_GPIO_WritePin(flash_nSS_GPIO_Port, flash_nSS_Pin, GPIO_PIN_SET);	// set nSS high to end
}

/*
 * Name:	WriteFlash
 * Use:		Write 1-256 bytes to one page of the flash
 * Inputs:	-address:	the address in memory to start writing at
 * 			-dataTx:	the array of data to be written
 * 			-dataSize:	the size of the data to written, in bytes
 * Outputs:	none
 */
void WriteFlash(uint16_t address, uint8_t* dataTx, uint16_t dataSize)
{
	// send WRITE ENABLE command
	_WaitForReady();
	uint8_t opcode = OPCODE_WRITE_ENABLE;

	HAL_GPIO_WritePin(flash_nSS_GPIO_Port, flash_nSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &opcode, 1, 10000);
	HAL_GPIO_WritePin(flash_nSS_GPIO_Port, flash_nSS_Pin, GPIO_PIN_SET);

	// send WRITE command
	_WaitForReady();
	uint8_t header[4] = {OPCODE_WRITE, 0x00, (address & 0xFF00) >> 8, (address & 0x00FF)};

	HAL_GPIO_WritePin(flash_nSS_GPIO_Port, flash_nSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, header, 4, 10000);
	HAL_SPI_Transmit(&hspi1, dataTx, dataSize, 10000);
	HAL_GPIO_WritePin(flash_nSS_GPIO_Port, flash_nSS_Pin, GPIO_PIN_SET);
}

/*
 * Name:	ErasePage
 * Use:		Erase a single page of the flash
 * Inputs:	-pageNum:	page number (0-255) to be erased
 * Outputs:	none
 */
void ErasePage(uint8_t pageNum)
{
	// send WRITE ENABLE command
	_WaitForReady();
	uint8_t opcode = OPCODE_WRITE_ENABLE;

	HAL_GPIO_WritePin(flash_nSS_GPIO_Port, flash_nSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, &opcode, 1, 10000);
	HAL_GPIO_WritePin(flash_nSS_GPIO_Port, flash_nSS_Pin, GPIO_PIN_SET);

	// send PAGE ERASE command
	_WaitForReady();
	uint8_t header[4] = {OPCODE_PAGE_ERASE, 0x00, pageNum, 0x00};

	HAL_GPIO_WritePin(flash_nSS_GPIO_Port, flash_nSS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, header, 4, 10000);
	HAL_GPIO_WritePin(flash_nSS_GPIO_Port, flash_nSS_Pin, GPIO_PIN_SET);
}

/*
 * Name:	WriteID
 * Use:		Write master or patient ID number to flash
 * Inputs:	-numID: 	the ID number to be written
 * 			-typeID: 	USER_MASTER or USER_PATIENT
 * Outputs:	0 if successful, 1 if error
 */
uint8_t WriteID(uint8_t* numID, uint8_t typeID)
{
	uint16_t addr;
	uint8_t page;
	if (typeID == USER_MASTER)
	{
		addr = ADDRESS_MASTER_ID;
		page = PAGE_MASTER_ID;
	}
	else if (typeID == USER_PATIENT)
	{
		addr = ADDRESS_PATIENT_ID;
		page = PAGE_PATIENT_ID;
	}
	else
	{
		return 1; // FIXME: error handling
	}

	ErasePage(page);
	WriteFlash(addr, numID, SIZE_ID);

	return 0;
}


/*
 * Name:	WriteTiming
 * Use:		Write timing data to flash
 * Inputs:	-doses_per_day:	number of doses per day, 1-4
 * 			-times:			array of times of doses
 * Outputs:	0 if successful, 1 if error
 */
uint8_t WriteTiming(uint8_t doses_per_day, uint8_t* dose_times)
{
	if ( (doses_per_day < 1) || (doses_per_day > 4) )
	{
		return 1;
	}

	ErasePage(PAGE_TIMING);
	WriteFlash(ADDRESS_DOSES_PER_DAY, &doses_per_day, 1);
	WriteFlash(ADDRESS_TIME0, dose_times, doses_per_day*SIZE_TIME);

	return 0;
}


/*
 * Name:	ReadSettings
 * Use:		Read settings data from flash
 * Inputs:	-masterID:		array of SIZE_ID bytes with the master ID number
 * 			-patientID:		array of SIZE_ID bytes with the patient ID number
 * 			-doses_per_day:	byte with the number of doses per day
 * 			-times:			array of doses_per_day times, each of SIZE_TIME bytes
 * Outputs:	none
 */
void ReadSettings(uint8_t* masterID, uint8_t* patientID, uint8_t* doses_per_day, uint8_t* dose_times)
{
	ReadFlash(ADDRESS_MASTER_ID, masterID, SIZE_ID);
	ReadFlash(ADDRESS_PATIENT_ID, patientID, SIZE_ID);
	ReadFlash(ADDRESS_DOSES_PER_DAY, doses_per_day, SIZE_DOSES_PER_DAY);
	ReadFlash(ADDRESS_TIME0, dose_times, *doses_per_day * SIZE_TIME);
}

/*
 * Name:	ReadTiming
 * Use:		Read timing data from flash
 * Inputs:	-doses_per_day:	byte with the number of doses per day
 * 			-times:			array of doses_per_day times, each of SIZE_TIME bytes
 * Outputs:	none
 */
void ReadTiming(uint8_t* doses_per_day, uint8_t* dose_times)
{
	ReadFlash(ADDRESS_DOSES_PER_DAY, doses_per_day, SIZE_DOSES_PER_DAY);
	ReadFlash(ADDRESS_TIME0, dose_times, *doses_per_day * SIZE_TIME);
}

/*
 * Name:	WriteLog
 * Use:		Write a log entry to flash
 * Inputs:	-code:	code identifying the type of log
 * 			-data:	any data to accompany log
 * Outputs:	none
 */
void WriteLog(uint8_t code, uint8_t data)
{
	// get current time
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD);

	Log log;
	log.code = code;
	log.data = data;
	log.year = sDate.Year;
	log.month = sDate.Month;
	log.day = sDate.Date;
	log.hour = sTime.Hours;
	log.minute = sTime.Minutes;
	log.second = sTime.Seconds;

	WriteFlash(log_addr, (uint8_t*)&log, SIZE_LOG);
}

void _WaitForReady()
{
	uint8_t opcode = OPCODE_READ_STATUS_REG;
	uint8_t statusReg[2] = {1, 1};

	while ( (statusReg[0] & 0x01) == 1 )
	{
		HAL_GPIO_WritePin(flash_nSS_GPIO_Port, flash_nSS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, &opcode, 1, 10000);
		HAL_SPI_Receive(&hspi1, statusReg, 2, 10000);
		HAL_GPIO_WritePin(flash_nSS_GPIO_Port, flash_nSS_Pin, GPIO_PIN_SET);
	}
}

void _SeparateBytes(uint8_t* array, uint64_t data, uint8_t size)
{
	for (uint8_t i = 0; i < size; i++)
	{
		uint8_t calc = (data >> ((size - i - 1) * 8) ) & 0xFF;
		array[i] = calc;
	}
}
