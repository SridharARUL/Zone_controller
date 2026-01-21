/*
 * Error_Handling.h
 *
 *  Created on: Sep 19, 2025
 *      Author: Sridhar A
 */

#ifndef INC_ERROR_HANDLING_H_
#define INC_ERROR_HANDLING_H_
uint8_t TP4_Modbus_error(int16_t TP4_TEMP_Value,int8_t TP4_Set,uint8_t TP4_WStatus,int16_t TP4_Status,uint8_t Tp4_Fan_WStatus);
uint8_t Light_Error(float Lux);
//uint8_t water_Temp_Error(float Wtemp);
uint8_t HT_Error(float hum_Value,float Temp_value);
uint8_t TP4_Terror(uint8_t TP4_TEMP_Value);
uint8_t Ec_Error(float ECvalue);
uint8_t pH_error(float phVal,uint8_t pHTError);
#endif /* INC_ERROR_HANDLING_H_ */
