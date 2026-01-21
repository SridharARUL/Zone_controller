/*
 * EC_Sensor.c
 *
 *  Created on: Sep 1, 2025
 *      Author: Sridhar A
 */

#include "EC_Sensor.h"
#include "pH_Sensor.h"
#include<main.h>
#include <stdint.h>
#include "RelayInit.h"
#include <stdbool.h>
#include "Error_Handling.h"
#define ecsampleSize 20
#define RES2 (7500.0/0.66)
#define ECREF 200.0

float ECvalueRaw,ecValue;

uint16_t analogEc[20], ECrawValue, ecconfidence, ecanalog;
uint8_t Ecthreshold = 10,EC_Error;

bool pHflag=false;
extern bool ECflag;
extern volatile uint8_t PH_EC_cnt;
extern float Wtemp,kvalue;



extern uint8_t Wtemp_Error;

extern ADC_HandleTypeDef hadc1;

float ECvalue,Ec_Voltage;





float convertECfromAnalog(uint16_t ECrawValue) {

     Ec_Voltage = ((float)ECrawValue / 4096.0f) * 3300.0f;



    ECvalueRaw = 1000.0f * Ec_Voltage / RES2 / ECREF * kvalue * 10.0f;



    if (!Wtemp_Error)
        ecValue = ECvalueRaw / (1.0f + 0.0185f * (Wtemp - 25.0f));
    else
        ecValue = ECvalueRaw;


    return ecValue;
}





float get_EC() {

	if (ECflag) {

		Relay_Ecpwr(1);
		Relay_EcADCpwr(1);

		if (PH_EC_cnt >= 30) {

			EC_ADC1_Init();

			for (int i = 0; i < ecsampleSize; i++) {

				HAL_ADC_Start(&hadc1);

				HAL_ADC_PollForConversion(&hadc1, 100);

				analogEc[i] = HAL_ADC_GetValue(&hadc1);
//				sprintf(buffer, " analogEc[%d] = %u\r\n", i, analogEc[i]);
//				 HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

				HAL_Delay(10);

			}

			sortSamples(analogEc, ecsampleSize);

			ECrawValue = getMedian(analogEc, ecsampleSize);

//			sprintf(buffer, "ECrawValue: %d\r\n", ECrawValue);
//
//			HAL_UART_Transmit(&huart2, (uint8_t*) buffer, strlen(buffer),
//										HAL_MAX_DELAY);
		ecconfidence = calculateConfidence(analogEc, ecsampleSize,
												ECrawValue, Ecthreshold);
			ECvalue = convertECfromAnalog(ECrawValue);

			EC_Error= Ec_Error(ECvalue);


			HAL_ADC_Stop(&hadc1);

			pHflag = true;

			ECflag = false;
			Relay_Ecpwr(0);
			Relay_EcADCpwr(0);
			PH_EC_cnt = 0;

//			sprintf(buffer, "ECvalue: %.2f \r\n", ECvalue);
//
//			HAL_UART_Transmit(&huart2, (uint8_t*) buffer, strlen(buffer),
//							HAL_MAX_DELAY);
		}
	}

	return ECvalue;
}
