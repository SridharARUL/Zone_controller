/*
 * sys_control.h
 *
 *  Created on: Sep 5, 2025
 *      Author: Sridhar A
 */

#ifndef INC_SYS_CONTROL_H_
#define INC_SYS_CONTROL_H_
#include <main.h>
#include <stdint.h>

uint8_t waterlevel(uint8_t W_lvl1,uint8_t W_lvl2);
void valvecont(uint8_t W_lvlstatus);
void nutrientdosage(uint8_t Ec_A,uint8_t Ec_B,uint8_t pH_Down,uint8_t pH_Up);

void waterpump(int16_t soilMoisture[], int Total_no);
void ECpumperror();
void pHdownpumperror();
void pHuppumperror();
uint8_t TP4_fogger_O_P(float hum_Value,uint8_t Envir_HT_Error);
uint8_t TP4_curtain_O_P(float lux,uint8_t Lux_Error);
void TP4_cirulation_Fan(uint16_t circulationfan_setpoint);
void TP4_Decode_Outputs(int16_t status);
#endif /* INC_SYS_CONTROL_H_ */
