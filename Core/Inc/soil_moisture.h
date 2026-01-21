/*
 * soil_moisture.h
 *
 *  Created on: Sep 2, 2025
 *      Author: Sridhar A
 */

#ifndef INC_SOIL_MOISTURE_H_
#define INC_SOIL_MOISTURE_H_

int16_t Get_Consumption (uint8_t sensor_id, uint8_t startadress , uint8_t endadress,uint8_t startlength , uint8_t endlength,uint8_t CONSUMPTION_RESPONSE_BYTES,uint8_t flag,uint8_t READ_HOLDING_REGISTERS);
HAL_StatusTypeDef Write_Single_Register(uint8_t sensor_id, uint16_t reg_address, uint16_t value);
#endif /* INC_SOIL_MOISTURE_H_ */
