/*
 * HTS221.h
 *
 *  Created on: Feb 27, 2024
 *      Author: jesus
 */

#ifndef INC_HTS221_H_
#define INC_HTS221_H_
typedef struct{
	float temp;
	float hum;

}THSample;
void HTS221_Init();
THSample HTS221_Read();

#endif /* INC_HTS221_H_ */
