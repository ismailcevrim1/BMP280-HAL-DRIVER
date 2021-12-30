/*
 * BMP280.h
 *
 *  Created on: 8 Kas 2021
 *      Author: ismail
 */

#ifndef SRC_BMP280_H_
#define SRC_BMP280_H_

#include <math.h>
#include "stm32f4xx_hal.h"
#include <stdio.h>

#define BMP280_WRITE_REGISTER_ADDRESS 0xEE
#define BMP280_READ_REGISTER_ADDRESS 0xEF

extern I2C_HandleTypeDef hi2c1;
extern float temperature, pressure, altitude;

void BMP280_Init(void);
void BMP280_Read_Calibration_Values(void);
void BMP280_Get_Value(void);
void Low_Pass_Filter_26Hz(void);

#endif /* SRC_BMP280_H_ */
