
#include "WatertempSensor.h"
#include <stdio.h>
#include <string.h>
#include<main.h>
#include <stdint.h>


extern TIM_HandleTypeDef htim2;
float tempC,prevtemp;
uint8_t temperrorcnt=0,Wtemp_Error;

void TEMP_delay(uint16_t time) {


 	/* change your code here for the delay in microseconds */
 	__HAL_TIM_SET_COUNTER(&htim2, 0);
 	while ((__HAL_TIM_GET_COUNTER(&htim2)) < time);
 }


uint8_t ds18b20_init() {
	MX_GPIO_Init_Output();
 	HAL_GPIO_WritePin(Wtemp_Output_GPIO_Port, Wtemp_Output_Pin, SET);
 	TEMP_delay(10);
 	HAL_GPIO_WritePin(Wtemp_Output_GPIO_Port, Wtemp_Output_Pin, RESET);
 	TEMP_delay(1500);
 	HAL_GPIO_WritePin(Wtemp_Output_GPIO_Port, Wtemp_Output_Pin, SET);
 	MX_GPIO_Init_Input();
 	uint8_t t = 0;
 	while (HAL_GPIO_ReadPin(Wtemp_Output_GPIO_Port, Wtemp_Output_Pin)) {
 		t++;
 		if (t > 60) {
 			return 0;
 		}
 		TEMP_delay(2);
 	}
 	t = 480 - t;
 	MX_GPIO_Init_Output();
 	TEMP_delay(t*2);
 	HAL_GPIO_WritePin(Wtemp_Output_GPIO_Port, Wtemp_Output_Pin, SET);

 	return 1;
 }

void ds18b20_write(uint8_t data) {
	MX_GPIO_Init_Output();
	for (uint8_t i = 0; i < 8; i++) {
		HAL_GPIO_WritePin(Wtemp_Output_GPIO_Port, Wtemp_Output_Pin, RESET);
		TEMP_delay(20);
		if (data & 1) {
			HAL_GPIO_WritePin(Wtemp_Output_GPIO_Port, Wtemp_Output_Pin, SET);
		} else
			HAL_GPIO_WritePin(Wtemp_Output_GPIO_Port, Wtemp_Output_Pin, RESET);
		data >>= 1;
		TEMP_delay(100);
		HAL_GPIO_WritePin(Wtemp_Output_GPIO_Port, Wtemp_Output_Pin, SET);
	}
}

uint8_t ds18b20_read() {
	MX_GPIO_Init_Output();
	HAL_GPIO_WritePin(Wtemp_Output_GPIO_Port, Wtemp_Output_Pin, SET);
	TEMP_delay(1);

	uint8_t data = 0;
	for (uint8_t i = 0; i < 8; i++) {
		HAL_GPIO_WritePin(Wtemp_Output_GPIO_Port, Wtemp_Output_Pin, RESET);
		TEMP_delay(2);
		HAL_GPIO_WritePin(Wtemp_Output_GPIO_Port, Wtemp_Output_Pin, SET);
		MX_GPIO_Init_Input();
		TEMP_delay(10);
		data >>= 1;
		if (HAL_GPIO_ReadPin(Wtemp_Output_GPIO_Port, Wtemp_Output_Pin)) {
			data |= 0x80;
		}
		TEMP_delay(110);
		MX_GPIO_Init_Output();
		HAL_GPIO_WritePin(Wtemp_Output_GPIO_Port, Wtemp_Output_Pin, SET);
	}

	return data;
}

float Temperature_Read() {



	if (!ds18b20_init())
		return 0;
	ds18b20_write(0xCC);
	ds18b20_write(0x44);
	if (!ds18b20_init())
		return 0;
	ds18b20_write(0xCC);
	ds18b20_write(0xBE);

	int temp = ds18b20_read();
	temp |= ds18b20_read() << 8;

 tempC = temp * 0.0625f;

 if(tempC<=0 || tempC>=100 ){

	 temperrorcnt++;
	 if(temperrorcnt>=100){
		 tempC= prevtemp;
		 Wtemp_Error=1;
		 return tempC;
	 }

	 return prevtemp;
 }
 temperrorcnt=0;
 Wtemp_Error=0;
 tempC= prevtemp;
	return tempC;
}










