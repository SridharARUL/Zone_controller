/*
 * VEML7700.h
 *
 *  Created on: Sep 10, 2025
 *      Author: Sridhar A
 */

#ifndef INC_VEML7700_H_
#define INC_VEML7700_H_
#include <stdint.h>

uint32_t VEML7700_Power_On (void);
uint32_t VEML7700_Shutdown (void);
uint32_t VEML7700_SetALSIntegrationTime (uint16_t it);
uint16_t VEML7700_GetALSIntegrationTime (void);
uint32_t VEML7700_SetALSGain (uint16_t gain);
uint16_t VEML7700_GetALSGain (void);
uint16_t VEML7700_ReadALSData (void);
uint16_t VEML7700_ReadWhiteData (void);
float VEML7700_Convert2Lx (uint16_t rawalsdata, uint8_t alsgain, uint8_t alsIT);
float VEML7700_GetLx (void);
HAL_StatusTypeDef PowerOnVEML7700 (void);


#endif /* INC_VEML7700_H_ */
