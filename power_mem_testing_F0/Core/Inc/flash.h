/*
 * flash.h
 *
 *  Created on: Feb 29, 2020
 *      Author: Adam
 */

#ifndef SRC_FLASH_H_
#define SRC_FLASH_H_

#include "stm32f0xx_hal.h"

#define MAX_BUFFER_SIZE 260 // FIXME
#define PAGE_SIZE 256

#define OPCODE_READ 0x03
#define OPCODE_WRITE_ENABLE 0x06
#define OPCODE_WRITE 0x02
#define OPCODE_PAGE_ERASE 0x81
#define OPCODE_READ_STATUS_REG 0x05

#define USER_MASTER 	0
#define USER_PATIENT 	1

#define SIZE_ID 			8 // bytes
//#define SIZE_PERIOD 	4 // bytes
#define SIZE_TIMES_PER_DAY 	1 // bytes
#define SIZE_TIME			1 // bytes

#define ADDRESS_MASTER 			(0*PAGE_SIZE + 0)
#define ADDRESS_PATIENT			(ADDRESS_MASTER + SIZE_ID)
//#define ADDRESS_PERIOD	(ADDRESS_PATIENT + SIZE_ID)
#define ADDRESS_TIMES_PER_DAY 	(ADDRESS_PATIENT + SIZE_ID)
#define ADDRESS_TIME0			(ADRESS_TIMES_PER_DAY + SIZE_TIMES_PER_DAY)
#define ADDRESS_TIME1			(ADDRESS_TIME0 + SIZE_TIME)
#define ADDRESS_TIME2			(ADDRESS_TIME1 + SIZE_TIME)
#define ADDRESS_TIME3			(ADDRESS_TIME2 + SIZE_TIME)

#define PAGE_SETTINGS 0

void ReadFlash(uint16_t address, uint8_t* data, uint16_t dataSize);
void WriteFlash(uint16_t address, uint8_t* data, uint16_t dataSize);
void WaitForReady();
void ErasePage(uint8_t pageNum);

void WriteSettings(uint8_t* masterID, uint8_t* patientID, uint8_t* dosagePeriod);
void ReadSettings(uint8_t* masterID, uint8_t* patientID, uint8_t* dosagePeriod);
void _UpdateSettings();
void _WaitForReady();
void _SeparateBytes(uint8_t* array, uint64_t data, uint8_t size);
//void SetUser(uint8_t userType, uint32_t userID);
//void SetDosagePeriod(uint32_t period);
//uint16_t WriteLog(uint8_t logType, uint32_t time, uint32_t date);

#endif /* SRC_FLASH_H_ */
