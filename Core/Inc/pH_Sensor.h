/*
 * pH_Sensor.h
 *
 *  Created on: Aug 29, 2025
 *      Author: Sridhar A
 */

#ifndef INC_PH_SENSOR_H_
#define INC_PH_SENSOR_H_
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
float get_PH();
void sortSamples(uint16_t arr[], int size);
uint16_t getMedian(uint16_t arr[], int size);
float calculateConfidence(uint16_t arr[], int size, uint16_t referenceValue,uint8_t threshold);
float convertPHfromAnalog(uint16_t PHrawValue);

#endif /* INC_PH_SENSOR_H_ */
