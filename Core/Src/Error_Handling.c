/*
 * Error_Handling.c
 *
 *  Created on: Sep 19, 2025
 *      Author: Sridhar A
 */

#include "main.h"
#include "VEML7700.h"


#include <stdint.h>

uint8_t TP4_Modbus_error(int16_t TP4_TEMP_Value,int8_t TP4_Set,uint8_t TP4_WStatus,int16_t TP4_Status,uint8_t Tp4_Fan_WStatus){

	if((TP4_TEMP_Value==-5)||(TP4_Set==-5)||(TP4_WStatus!=0)||(TP4_Status==-5)||(Tp4_Fan_WStatus==!0)){

		return 1;
	}
	else{
		return 0;
	}
}


uint8_t Light_Error(float Lux){

	if(Lux<0){
		return 1;
		PowerOnVEML7700();

	}
	else{
		return 0;

	}


}

//uint8_t water_Temp_Error(float Wtemp){
//
//	if(Wtemp<=0){
//			return 1;
//
//		}
//		else{
//			return 0;
//
//		}
//
//
//}

uint8_t HT_Error(float hum_Value,float Temp_value){

	if(Temp_value<=15){
				return 1;

			}
			else{
				return 0;

			}



}

uint8_t TP4_Terror(uint8_t TP4_TEMP_Value) {

	if (TP4_TEMP_Value < 0) {
		return 1;
	} else {
		return 0;
	}

}

uint8_t Ec_Error(float ECvalue){

	if(ECvalue < 0.1f){
		return 1;
	}
	else{
		return 0;
	}

}
uint8_t pH_error(float phVal, uint8_t pHTError)
{


    if (phVal <= 3.0f || phVal >= 10.0f || pHTError >= 10)
        return 1;
    else
        return 0;
}

