	/*
	 * TCS.h
	 *
	 *  Created on: Jan 3, 2024
	 *      Author: Oski
	 */

	#ifndef INC_TCS_H_
	#define INC_TCS_H_

	#include "stm32f4xx.h"

	#define TCS34725_ADDRESS (0x29 << 1)
	#define TCS34725_COMMAND_BIT 0x80
	#define TCS34725_ID 0x12

	#define ENABLE_REG 0
	#define ATIME_REG 0x01
	#define CONTROL_REG 0x0f

	#define CLEAR 0x14
	#define RED 0x16
	#define GREEN 0x18
	#define BLUE 0x1A

	void I2C_Write8BIT (uint8_t reg, uint32_t value);
	uint8_t I2C_Read8BIT(uint8_t reg);
	uint16_t I2C_Read16BIT(uint8_t reg);
	uint16_t I2C_GetColor(uint8_t reg);
	void I2C_RGBCTime(uint16_t value);


	#endif /* INC_TCS_H_ */
