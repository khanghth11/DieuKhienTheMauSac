/*
 * TCS.c
 *
 *  Created on: Jan 3, 2024
 *      Author: Oski
 */

#include "TCS34725.h"


extern I2C_HandleTypeDef hi2c1;


//Funkcja ustawiająca wartość rejestru 8-bitowego
void I2C_Write8BIT (uint8_t reg, uint32_t value)
{
  uint8_t pkt[2];
  pkt[0] = (TCS34725_COMMAND_BIT | reg);
  pkt[1] = (value & 0xFF);
  HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS, pkt, 2, HAL_MAX_DELAY);
}


//Funkcja odczytująca rejestr 8-bitowy
uint8_t I2C_Read8BIT(uint8_t reg)
{
	uint8_t pkt;
	pkt = (TCS34725_COMMAND_BIT | reg);
	HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS, &pkt, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, TCS34725_ADDRESS, &pkt, 1, HAL_MAX_DELAY);

	return pkt;
}


//Funkcja odczytująca dwa rejestry 8-bitowe do jednej zmiennej 16-bitowej
uint16_t I2C_Read16BIT(uint8_t reg) {
	uint16_t ret;
	uint8_t data[2];

	data[0] = (TCS34725_COMMAND_BIT | reg);
	HAL_I2C_Master_Transmit(&hi2c1, TCS34725_ADDRESS, data, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, TCS34725_ADDRESS, data, 2, HAL_MAX_DELAY);

	ret = (uint16_t) data[1];
	ret <<= 8;
	ret |= (uint16_t) data[0];
	return ret;
}


//Funkcja służąca do pobrania koloru o danym adresie
uint16_t I2C_GetColor(uint8_t reg){
	uint16_t ret=-1;
	uint32_t clear = I2C_Read16BIT(0x14);
	switch (reg){
		case 0x16:
		case 0x18:
		case 0x1A:
			ret=I2C_Read16BIT(reg);
			ret = (float)ret / clear * 255.0;
		break;

		case 0x14:
			ret=clear;
		break;

	}
	return ret;
}


//Funkcja ustawiająca gain czujnika
void I2C_SetGain(uint8_t value){
	I2C_Write8BIT(CONTROL_REG, value);
}


//Automatyczne dostosowywanie czasu zbierania światła
void I2C_RGBCTime(uint16_t value){
	if(value<24) 				I2C_Write8BIT(ATIME_REG, 0xff);
	if(value>24  && value<101) 	I2C_Write8BIT(ATIME_REG, 0xf6);
	if(value>101 && value<154) 	I2C_Write8BIT(ATIME_REG, 0xd5);
	if(value>154 && value<700) 	I2C_Write8BIT(ATIME_REG, 0x00);
}

