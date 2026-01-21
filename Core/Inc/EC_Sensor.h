/*
 * EC_Sensor.h
 *
 *  Created on: Sep 1, 2025
 *      Author: Sridhar A
 */

#ifndef INC_EC_SENSOR_H_
#define INC_EC_SENSOR_H_
#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
float get_EC();
float convertECfromAnalog(uint16_t ECrawValue);


#endif /* INC_EC_SENSOR_H_ */
