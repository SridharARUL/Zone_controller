/*
 * RelayInit.h
 *
 *  Created on: Sep 1, 2025
 *      Author: Sridhar A
 */

#ifndef INC_RELAYINIT_H_
#define INC_RELAYINIT_H_



void Relay_pHpwr(int state);
void Relay_pHADCpwr(int state);
void Relay_Ecpwr(int state);
void Relay_EcADCpwr(int state);
void RS484_MOIENABLE_TX(int state);
void TP4_ENABLE_TX(int state);
void Rly_valve_F(int state);
void Rly_valve_R(int state);
void EcA_Pump_trigger(int state);
void EcB_Pump_trigger(int state);
void pHDown_Pump_trigger(int state);
void pHup_Pump_trigger(int state);
void Waterpump_trigger(int state);
void sensor_pwr_3v(int state);
void sensor_pwr_5v(int state);
void FA_ENABLE_TX(int state);
void light(int state);
void PH_4(int state);
void PH_7(int state);
void EC_LED(int state);
void calibration_fail(int state);
void macro_nutrient(int state);
void micro_nutrient(int state);



#endif /* INC_RELAYINIT_H_ */
