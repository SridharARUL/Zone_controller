/*
 * PH_Sensor.c
 *
 *  Created on: Aug 29, 2025
 *      Author: Sridhar A
 */
#include <stdint.h>
#include "main.h"
#include "pH_Sensor.h"
#include "RelayInit.h"
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "FA_RS485.h"
#include "Error_Handling.h"

#define sampleSize 20
char uart_msg1[64];

float pHvalue=0,alpha,phVal,PHValue ;
extern bool Sen_read3vflag,pHflag;

uint16_t analogpH[20], PHrawValue, confidence, analog;
uint8_t threshold = 10,pHTError,pH_Error;
extern volatile uint8_t PH_EC_cnt;
bool Phvalueflag = true;

extern uint32_t Acidic_voltage,Neutral_Voltage;
extern ADC_HandleTypeDef hadc1;


void sortSamples(uint16_t arr[], int size) {
	for (int i = 0; i < size - 1; i++) {
		for (int j = 0; j < size - i - 1; j++) {
			if (arr[j] > arr[j + 1]) {
				int temp = arr[j];
				arr[j] = arr[j + 1];
				arr[j + 1] = temp;
			}
		}
	}
}

uint16_t getMedian(uint16_t arr[], int size) {
	return arr[size / 2];
}

float calculateConfidence(uint16_t arr[], int size, uint16_t referenceValue,
		uint8_t threshold) {
	int inRange = 0;
	for (int i = 0; i < size; i++) {
		if (arr[i] >= (referenceValue - threshold)
				&& arr[i] <= (referenceValue + threshold)) {
			inRange++;
		}
	}
	return (float) inRange / size * 100.0f;
}


float convertPHfromAnalog(uint16_t PHrawValue)
{

    float voltage = (PHrawValue / 4095.0f) * 3300.0f;


    float acidic  = (float)Acidic_voltage;   // pH 4
    float neutral = (float)Neutral_Voltage;  // pH 7


    float slope = (7.0f - 4.0f) / (neutral - acidic);   // slope = ΔpH / ΔVoltage
    float intercept = 7.0f - (slope * neutral);         // using pH = slope*V + intercept


     PHValue = (slope * voltage) + intercept;


//    sprintf(buffer,
//            "PHraw:%u  Voltage:%.1fmV  pH:%.2f  Slope:%.5f  Intercept:%.2f  Neutral:%d  Acidic:%d  \r\n",
//            PHrawValue, voltage, PHValue, slope, intercept, Neutral_Voltage, Acidic_voltage);
//    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

    return PHValue;
}






float get_PH() {

	if (pHflag) {
		Relay_pHADCpwr(1);
		Relay_pHpwr(1);

		if (PH_EC_cnt >= 30) {


			PH_ADC1_Init();

			for (int i = 0; i < sampleSize; i++) {

				HAL_ADC_Start(&hadc1);
				HAL_ADC_PollForConversion(&hadc1, 100);

				analogpH[i] = HAL_ADC_GetValue(&hadc1);

				HAL_Delay(10);
			}





			sortSamples(analogpH, sampleSize);

			PHrawValue = getMedian(analogpH, sampleSize);

			confidence = calculateConfidence(analogpH, sampleSize, PHrawValue,
					threshold);



			phVal  = convertPHfromAnalog(PHrawValue);

			HAL_ADC_Stop(&hadc1);
			pHflag = false;
			Sen_read3vflag = true;
			Relay_pHADCpwr(0);
			Relay_pHpwr(0);
			PH_EC_cnt = 0;

			pH_Error=pH_error(phVal,pHTError);
			Print_All_Variables();

			 if (Phvalueflag) {
				pHvalue = phVal;
				Phvalueflag = false;
			}

			if (confidence < 50) {
				pHTError++;

				return pHvalue;
			}


			if (confidence <= 80)
				alpha = 0.1f;
			else
				alpha = 0.25f;

			pHTError = 0;

			// Exponential Moving Average
			pHvalue = alpha * phVal + (1 - alpha) * pHvalue;




		}
	}

	return pHvalue;

}



