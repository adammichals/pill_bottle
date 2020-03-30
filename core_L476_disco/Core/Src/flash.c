/*
 * flash.c
 *
 *  Created on: Feb 29, 2020
 *      Author: Adam
 */

#include "flash.h"
#include "main.h"
#include "stm32f0xx_hal.h"

extern SPI_HandleTypeDef hspi1;

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
 * Name:	WriteSettings
 * Use:		Write settings data to flash
 * Inputs:	-masterID:		array of SIZE_ID bytes with the master ID number
 * 			-patientID:		array of SIZE_ID bytes with the patient ID number
 * 			-dosagePeriod:	array of SIZE_PERIOD bytes with the time between doses in seconds (FIXME)
 * Outputs:	none
 */
void WriteSettings(uint8_t* masterID, uint8_t* patientID, uint8_t* dosagePeriod)
{
	ErasePage(PAGE_SETTINGS);

	WriteFlash(ADDRESS_MASTER, masterID, SIZE_ID);
	WriteFlash(ADDRESS_PATIENT, patientID, SIZE_ID);
	//WriteFlash(ADDRESS_PERIOD, dosagePeriod, SIZE_PERIOD);
}

/*
 * Name:	ReadSettings
 * Use:		Read settings data from flash
 * Inputs:	-masterID:		array of SIZE_ID bytes with the master ID number
 * 			-patientID:		array of SIZE_ID bytes with the patient ID number
 * 			-dosagePeriod:	array of SIZE_PERIOD bytes with the time between doses in seconds (FIXME)
 * Outputs:	none
 */
void ReadSettings(uint8_t* masterID, uint8_t* patientID, uint8_t* dosagePeriod)
{
	ReadFlash(ADDRESS_MASTER, masterID, SIZE_ID);
	ReadFlash(ADDRESS_PATIENT, patientID, SIZE_ID);
	//ReadFlash(ADDRESS_PERIOD, dosagePeriod, SIZE_PERIOD);
}

void _UpdateSettings()
{
	//  TODO
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
