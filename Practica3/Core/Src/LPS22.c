/*
 * LPS22.c
 *
 *  Created on: Feb 27, 2024
 *      Author: jesus
 */
#include "stm32l4xx_hal.h"
extern I2C_HandleTypeDef hi2c2;

void LPS22_Init(){
// inicializamos de tal forma que la frecuenca de muestrea sea 50 hz
	// atcualizacíon post lectida  mSB y LSB
	// odr2 = 1 y bd1 = 1 resto a 0
	uint8_t buffer[1];
	buffer[0] = 0x42;

	HAL_I2C_Mem_Write(&hi2c2,0xBA,0x10, I2C_MEMADD_SIZE_8BIT,buffer,1,1000);
	// escribmos en la direccion 0x10, que el esclabvo 0xBa en un buffer de 8 bits,la orden
}
float LPS22_ReadPress(){
	float press;
	// 1 - Declarar buffer de lectura de 3 bytes.
	uint8_t buffer[3];
	// 2 - Leer los 3 bytes de la presion
	// que empiezan en la direccion ram dn 28 en hex
	HAL_I2C_Mem_Read(&hi2c2,0xBA,0x28, I2C_MEMADD_SIZE_8BIT,buffer,3,1000);
	// 3 ensamblar 3 bytes como palabra de 24 bits.
	// bitwise or |
	uint32_t press_raw = (buffer[2]<<16) |(buffer[1]<<8) |buffer [0];
	// escalar la presion
	// .0F indica float ?
	press = press_raw / 4096.0f;
	return press;
}





