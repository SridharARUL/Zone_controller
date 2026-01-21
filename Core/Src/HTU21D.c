/*
 * HTU21D.c
 *
 *  Created on: Sep 12, 2025
 *      Author: Sridhar A
 */

#include "HTU21D.h"
#include "pH_Sensor.h"
#include "main.h"
#include <stdint.h>
#include "stdbool.h"
extern ADC_HandleTypeDef hadc1;

#define sampleSize 20
extern char buffer[100];
uint16_t ADC_SAMPLE[10],ADCrawvalue;
float Humvalue,growTempvalue;


float gethumidity(){

	Hum_ADC1_Init();

	uint16_t humidityadc = adc_sample();
	float voltage_humidity = humidityadc / 4096.0 * 3.30;

	Humvalue = -12.5 + 125 * (voltage_humidity / 3.30);



	return Humvalue;

}

float get_Temp() {

	Temp_ADC1_Init();

	uint16_t Tempadc = adc_sample();

	float voltage_temp = Tempadc / 4096.0 * 3.30;

	growTempvalue = -60.625 + 206.25 * (voltage_temp / 3.30);


	return growTempvalue;

}

uint16_t adc_sample() {




	for (int i = 0; i < sampleSize; i++) {
		HAL_ADC_Start(&hadc1);

		HAL_ADC_PollForConversion(&hadc1, 100);
		ADC_SAMPLE[i] = HAL_ADC_GetValue(&hadc1);
		HAL_Delay(10);


	}

	HAL_ADC_Stop(&hadc1);    // Stop after loop

	sortSamples(ADC_SAMPLE, sampleSize);

	ADCrawvalue = getMedian(ADC_SAMPLE, sampleSize);

	return ADCrawvalue;
}
