/*
 * RelayInit.c
 *
 *  Created on: Sep 1, 2025
 *      Author: Sridhar A
 */

#include"Relayinit.h"

#include<main.h>

void Relay_pHpwr(int state){

HAL_GPIO_WritePin(GPIOD, GPIO_PIN_0, (state ? GPIO_PIN_SET : GPIO_PIN_RESET));

}

void Relay_pHADCpwr(int state)
{
	 HAL_GPIO_WritePin(GPIOD, GPIO_PIN_1, (state ? GPIO_PIN_SET : GPIO_PIN_RESET));
}

void Relay_Ecpwr(int state) {
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_4, (state ? GPIO_PIN_SET : GPIO_PIN_RESET));
}

void Relay_EcADCpwr(int state){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, (state ? GPIO_PIN_SET : GPIO_PIN_RESET));

}

void RS484_MOIENABLE_TX(int state) {
	HAL_GPIO_WritePin(GPIOD, moisture_EN_Pin, (state ? GPIO_PIN_SET : GPIO_PIN_RESET));
}

void TP4_ENABLE_TX(int state) {
	HAL_GPIO_WritePin(GPIOD, T4_EN_Pin, (state ? GPIO_PIN_SET : GPIO_PIN_RESET));
}




void Rly_valve_F(int state) {
	HAL_GPIO_WritePin(GPIOA, VALVE_1_Pin, (state ? GPIO_PIN_SET : GPIO_PIN_RESET));
}

void Rly_valve_R(int state) {
	HAL_GPIO_WritePin(GPIOC, valve_2_Pin, (state ? GPIO_PIN_SET : GPIO_PIN_RESET));
}

void EcA_Pump_trigger(int state){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, (state ? GPIO_PIN_SET : GPIO_PIN_RESET));

}

void EcB_Pump_trigger(int state){
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, (state ? GPIO_PIN_SET : GPIO_PIN_RESET));

}

void pHDown_Pump_trigger(int state){

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, (state ? GPIO_PIN_SET : GPIO_PIN_RESET));

}

void pHup_Pump_trigger(int state){

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, (state ? GPIO_PIN_SET : GPIO_PIN_RESET));

}

void Waterpump_trigger(int state){

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, (state ? GPIO_PIN_SET : GPIO_PIN_RESET));

}


void sensor_pwr_5v(int state){

	HAL_GPIO_WritePin(GPIOC, Sen_pwr_5V_Pin, (state ? GPIO_PIN_SET : GPIO_PIN_RESET));

}

void sensor_pwr_3v(int state){

	HAL_GPIO_WritePin(GPIOC, Sen_pwr_3v_Pin, (state ? GPIO_PIN_SET : GPIO_PIN_RESET));

}

void FA_ENABLE_TX(int state){

	HAL_GPIO_WritePin(GPIOD, FA_RS485_Pin, (state ? GPIO_PIN_SET : GPIO_PIN_RESET));

}


void light(int state){

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, (state ? GPIO_PIN_SET : GPIO_PIN_RESET));

}


void PH_4(int state){

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_9, (state ? GPIO_PIN_SET : GPIO_PIN_RESET));

}


void PH_7(int state){

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, (state ? GPIO_PIN_SET : GPIO_PIN_RESET));

}


void EC_LED(int state){

	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, (state ? GPIO_PIN_SET : GPIO_PIN_RESET));

}


void calibration_fail(int state){

	HAL_GPIO_WritePin(GPIOB, calibration_fail_Pin, (state ? GPIO_PIN_SET : GPIO_PIN_RESET));

}
void macro_nutrient(int state){

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_2, (state ? GPIO_PIN_SET : GPIO_PIN_RESET));

}

void micro_nutrient(int state){

	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, (state ? GPIO_PIN_SET : GPIO_PIN_RESET));

}
