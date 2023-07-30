/*
 * LIS3DSH.c
 *
 *  Created on: Jul 15, 2023
 *      Author: Pardeep
 */

/* Private includes ----------------------------------------------------------*/

#include "LIS3DSH.h"

/* Local defines ------------------------------------------------------------*/

#define WHO_AM_I 			0x0F
#define CTRL_REG4			0x20
#define READ_COMMAND 		0x80
#define FIRST_8_SDO			0x00
#define OUT_X_L				0x28
#define OUT_X_H				0x29
#define OUT_Y_L				0x2A
#define OUT_Y_H				0x2B
#define OUT_Z_L				0x2C
#define OUT_Z_H				0x2D
#define sens_0_06			0.06


/* Local Variables --------------------------------------------------------*/

static int16_t offsetX = 0;
static int16_t offsetY = 0;
static int16_t offsetZ = 0;
static int16_t pTest[3] = {0};

/* Local Function Prototypes --------------------------------------------------------*/

static uint8_t SPI1_WriteRead(uint8_t pTxData);

/* Global function definitions -----------------------------------------------------------*/

static uint8_t SPI1_WriteRead(uint8_t pTxData) {

	// Initalize variables
	uint8_t pRxData = 0;
	uint16_t txSize = 1;

	// Transmit and receive data from LIS3DSH accelerometer
	if (HAL_SPI_TransmitReceive(&hspi1, (uint8_t *)&pTxData, (uint8_t *)&pRxData, txSize, HAL_MAX_DELAY) != HAL_OK) {
		HAL_SPI_DeInit(&hspi1);
		HAL_SPI_Init(&hspi1);
	}

	// return received data
	return pRxData;

}


void ACCELERO_IO_Read(uint8_t* pRxData, uint8_t u8Addr, uint16_t dataSize){

	// Transmit Read bit, MS bit, and Address
	uint8_t pTxData = READ_COMMAND | u8Addr;

	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET);	//set CS low

	// Transmit read command and address of read location
	SPI1_WriteRead(pTxData);

	/* Receive the data that will be read from the device (MSB First) */
	*pRxData = SPI1_WriteRead(FIRST_8_SDO);
	dataSize--;
	pRxData++;

	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET); 		//set CS high

}


void ACCELERO_IO_Write(uint8_t *pTxData, uint8_t u8Addr, uint16_t dataSize){

	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_RESET); 	//set CS low

	// Transmit write command and address of write location
	SPI1_WriteRead(u8Addr);

	/* Receive the data that will be read from the device (MSB First) */
	SPI1_WriteRead(*pTxData);
	dataSize--;
	pTxData++;

	HAL_GPIO_WritePin(CS_I2C_SPI_GPIO_Port, CS_I2C_SPI_Pin, GPIO_PIN_SET); 		//set CS high

}


void ACCELERO_Init(){

	// Initialize Variables
	uint8_t regWHO = 0x00;
	uint8_t ctrl = 0x67;

	// Read WHO_AM_I register
	ACCELERO_IO_Read(&regWHO, WHO_AM_I, 1);

	// Configure MEMS: power mode(ODR) and axes enable
	ACCELERO_IO_Write(&ctrl, CTRL_REG4, 1);

	// Calibration of values read
	ACCELERO_GetXYZ(pTest);
	offsetX = pTest[0] * -1;
	offsetY = pTest[1] * -1;
	offsetZ = (pTest[2] - 1000) * -1;

}


void ACCELERO_GetXYZ(int16_t *pData){

	// Initialize Variables
	int8_t pRxData[6];
	float sensitivity = sens_0_06;
	float value = 0;

	// Read from all high and low output registers
	ACCELERO_IO_Read((uint8_t *)&pRxData[0], OUT_X_L, 1);
	ACCELERO_IO_Read((uint8_t *)&pRxData[1], OUT_X_H, 1);
	ACCELERO_IO_Read((uint8_t *)&pRxData[2], OUT_Y_L, 1);
	ACCELERO_IO_Read((uint8_t *)&pRxData[3], OUT_Y_H, 1);
	ACCELERO_IO_Read((uint8_t *)&pRxData[4], OUT_Z_L, 1);
	ACCELERO_IO_Read((uint8_t *)&pRxData[5], OUT_Z_H, 1);

	// Scale output accelerometer values base don sensitivity of system
	for(int i = 0; i < 3; i++){
		int16_t offset = (i == 0) ? offsetX : (i == 1) ? offsetY : (i == 2) ? offsetZ : 0;
		value = ((pRxData[2*i+1] << 8) + pRxData[2*i]) * sensitivity;
		pData[i] = (int16_t)value + offset;
	}

}


void calcAngles(int16_t *pData, int8_t* pAngles) {

	// Initialize Variables
	int8_t pitch;
	int8_t roll;

	// Scale accelerometer XYZ values
	double accelX = (double)pData[0]/1000;
	double accelY = (double)pData[1]/1000;
	double accelZ = (double)pData[2]/1000;

	// Calculate pitch and roll with incoming accelerometer data
	pitch = 180 * atan2(accelX, sqrt(accelY*accelY + accelZ*accelZ))/M_PI;
	roll = 180 * atan2(accelY, sqrt(accelX*accelX + accelZ*accelZ))/M_PI;

	pAngles[0] = pitch;
	pAngles[1] = roll;

}
