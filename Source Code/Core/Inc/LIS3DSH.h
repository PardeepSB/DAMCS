/*
 * LIS3DSH.h
 *
 *  Created on: Jul 15, 2023
 *      Author: Pardeep
 */

#ifndef INC_LIS3DSH_H_
#define INC_LIS3DSH_H_


#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
#include <math.h>
#include <stdlib.h>

/* Global Variables ------------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;


/* Global functions prototypes ---------------------------------------------*/
extern void ACCELERO_IO_Read(uint8_t* pRxData, uint8_t u8Addr, uint16_t dataSize);
extern void ACCELERO_IO_Write(uint8_t *pTxData, uint8_t u8Addr, uint16_t dataSize);
extern void ACCELERO_Init();
extern void ACCELERO_GetXYZ(int16_t *pData);
extern void calcAngles(int16_t *pData, int8_t* pAngles);


#ifdef __cplusplus
}
#endif

#endif /* INC_LIS3DSH_H_ */
