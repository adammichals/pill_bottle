/*
 * flash.h
 *
 *  Created on: Feb 29, 2020
 *      Author: Adam
 */

#ifndef SRC_FLASH_H_
#define SRC_FLASH_H_

#include "stm32l4xx_hal.h"

#define MAX_BUFFER_SIZE 260 // FIXME
#define PAGE_SIZE 256

#define OPCODE_READ 0x03
#define OPCODE_WRITE_ENABLE 0x06
#define OPCODE_WRITE 0x02
#define OPCODE_PAGE_ERASE 0x81
#define OPCODE_READ_STATUS_REG 0x05

/*******	SETTINGS PAGES	*******/
#define PAGE_MASTER_ID 	0
#define PAGE_PATIENT_ID 1
#define PAGE_TIMING		2

#define USER_MASTER 	0
#define USER_PATIENT 	1

#define SIZE_ID 			4 // bytes
#define SIZE_DOSES_PER_DAY 	1 // bytes
#define SIZE_TIME			1 // bytes

#define ADDRESS_MASTER_ID 		PAGE_MASTER_ID * PAGE_SIZE
#define ADDRESS_PATIENT_ID		PAGE_PATIENT_ID * PAGE_SIZE
#define ADDRESS_DOSES_PER_DAY 	PAGE_TIMING * PAGE_SIZE
#define ADDRESS_TIME0			(ADDRESS_DOSES_PER_DAY + SIZE_DOSES_PER_DAY)
#define ADDRESS_TIME1			(ADDRESS_TIME0 + SIZE_TIME)
#define ADDRESS_TIME2			(ADDRESS_TIME1 + SIZE_TIME)
#define ADDRESS_TIME3			(ADDRESS_TIME2 + SIZE_TIME)


/*******	log pages	*******/
typedef struct Log
{
	uint8_t code;
	uint8_t data;

	uint8_t year;
	uint8_t month;
	uint8_t day;
	uint8_t hour;
	uint8_t minute;
	uint8_t second;
} Log;

#define PAGE_START_LOG 		3
#define ADDRESS_START_LOG	PAGE_START_LOG * PAGE_SIZE;
#define SIZE_LOG			sizeof(Log)


void ReadFlash(uint16_t address, uint8_t* data, uint16_t dataSize);
void WriteFlash(uint16_t address, uint8_t* data, uint16_t dataSize);
void ErasePage(uint8_t pageNum);
void _WaitForReady();

uint8_t WriteID(uint8_t* numID, uint8_t typeID);
uint8_t WriteTiming(uint8_t doses_per_day, uint8_t* dose_times);
void ReadSettings(uint8_t* masterID, uint8_t* patientID, uint8_t* doses_per_day, uint8_t* times);
void ReadTiming(uint8_t* doses_per_day, uint8_t* dose_times);
void WriteLog(uint8_t code, uint8_t data);
void _SeparateBytes(uint8_t* array, uint64_t data, uint8_t size);
//void SetUser(uint8_t userType, uint32_t userID);
//void SetDosagePeriod(uint32_t period);
//uint16_t WriteLog(uint8_t logType, uint32_t time, uint32_t date);

#endif /* SRC_FLASH_H_ */
