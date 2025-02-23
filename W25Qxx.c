/*
 * W25Qxx.c
 *
 *  Created on: Jan 1, 2025
 *      Author: vardh
 */

#include "main.h"
#include "W25Qxx.h"

extern SPI_HandleTypeDef hspi1;
#define W25Q_SPI hspi1

#define csLOW() HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)
#define csHIGH() HAL_GPIO_WritePin (GPIOB, GPIO_PIN_14, GPIO_PIN_RESET)

#define numBlock 64

void  W25Q_Reset (void)
{
	uint8_t tData[2];
	tData[0] = 0x66;
	tData[1] = 0x99;
	csLOW();
	HAL_SPI_Transmit(&W25Q_SPI, tData, 2, 100);
	csHIGH();
	HAL_Delay(100);
}

uint32_t W25Q_ReadID (void) {
	uint8_t tData = 0x9F;
	uint8_t rData[3];
	csLOW();
	HAL_SPI_Transmit(&W25Q_SPI, &tData, 1, 1000);
	HAL_SPI_Receive(&W25Q_SPI, rData, 3, 3000);
	csHIGH();
	return ((rData[0]<<16)| (rData[1]<<8)|rData[2]);
}

