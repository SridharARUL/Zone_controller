/*
 * sys_controller.c
 *
 *  Created on: Sep 5, 2025
 *      Author: Sridhar A
 */

#include"sys_control.h"
#include "main.h"
#include <stdint.h>
#include"Relayinit.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "EEPROM.h"

uint16_t Doseinit =360; //3 mins
#define pumpcheckwait 240

char uart_msg[100];
#define TP4_Write_Address 1

#define TP4_Write_TEMP 0x0005
#define TP4_Write_curtain 0x0006
#define TP4_Write_curtain_2 0x0007
#define TP4_Write_fogger 0x0008

#define TP4_OUTPUT_OFF 900

#define TP4_OUTPUT_ON 90
#define WATER_REFILL_DELAY_MS  1800000UL

//extern UART_HandleTypeDef huart2;
bool isEcDosing=true,ECDose_Activeflag=false,ECcycle_Tflag=false,Ecflag=true,pH_pump_Activeflag=false,pHcycle_Tflag=false,pH_flag=true,ECmixflag=false,pHdmixflag=false,pHupmixflag=false,pH_UPpump_Activeflag=false;
uint8_t  Ec_Pump_Error=0,pHd_Pump_Error=0,pHup_Pump_Error=0,valvestatus=0,EcA_status,EcB_status,pHDown_status,pHUp_status,waterpump_status,EcPumpEr,pHdPumpEr,pHupPumpEr,FAN_Status=0,Curtain_Status,Fogger_Status,Tp4_Fan_WStatus;
extern uint8_t W_lvlstatus,fogger_delay_active,W_lvl1,EC_Error,pH_Error,PH_EC_cnt,macro_status;
extern volatile uint16_t Dosestart_cnt1,Ec_PumpON_cnt,pH_pumpON_cnt,Ecmixcnt,pHdmixcnt,pHupmixcnt;
extern volatile uint32_t Eccycle_cnt,pHcycle_cnt,fogger_delay_counter;
extern uint32_t Ecdose_cycle,Ecdose_sec,pHdose_sec,pHdose_cycle,PumpON_setpoint,PumpOff_setpoint,LUX_MINsetpoint,LUX_MAXsetpoint,HUM_FOG_MINsetpoint,HUM_FOG_MAXsetpoint,pump_auto_Man,pump_on_Cnt;
extern bool Dosestartflag;
extern float EC_Setpoint,pH_min,pH_max,Ec,pH,pH_PumpEr_setpoint,Ec_PumpEr_setpoint;
uint16_t soil_errorSensors = 0;
float Prevec,PrevpH;
uint8_t level;
extern bool Sen_read5vflag,Sen_read3vflag;
uint32_t water_fill_count=0,ECA_ON_count,PHD_ON_count,PHUP_ON_count;
uint8_t waterlevel(uint8_t W_lvl1,uint8_t W_lvl2){

	static uint8_t W_empty_cycle_active = 0;
	static uint32_t W_empty_start_time = 0;

	uint32_t now = HAL_GetTick();

	if (W_lvl1 && W_lvl2) {

		level = 2;

	} else if (W_lvl1 && !W_lvl2) {
		level = 1;

	}

	else if ((!W_lvl1) && (!W_lvl2)) {

		level = 0;
	}

	else {

		level = 3;

	}

	if ((level == 0) && (W_empty_cycle_active == 0)) {
		W_empty_cycle_active = 1;
		W_empty_start_time = now;

	}

	/* After delay, check refill ONCE */
	if (W_empty_cycle_active) {
		if (now - W_empty_start_time >= WATER_REFILL_DELAY_MS)   // 30 minutes
		{
			if (level > 0)   // refilled
					{
				water_fill_count++;   // <-- increment your counter
				W_empty_cycle_active = 0;
			}
		}
	}

	return level;


}

void valvecont(uint8_t W_lvlstatus){


	if ((!W_lvlstatus)&&(!valvestatus)){

		Rly_valve_F(1);
		Rly_valve_R(0);
		HAL_Delay(300);


		Rly_valve_F(0);
		Rly_valve_R(0);

		valvestatus=1;
		Waterpump_trigger(0);
	   waterpump_status=0;
	   Dosestartflag=false;
		sensor_pwr_5v(0);
		sensor_pwr_3v(1);


		Sen_read5vflag=false;


		}
	else if((W_lvlstatus==2)&&(valvestatus)){

		Rly_valve_R(1);
		Rly_valve_F(0);
		HAL_Delay(300);


		Rly_valve_R(0);
		Rly_valve_F(0);
		PH_EC_cnt=0;
		Sen_read3vflag=true;
		valvestatus=0;
		Ecflag=true;
		Eccycle_cnt = 0;
		Dosestartflag=true;
		pH_flag=true;
		pHcycle_cnt=0;
		Dosestart_cnt1=0;

	}


}

void nutrientdosage(uint8_t Ec_A, uint8_t Ec_B, uint8_t pH_Down, uint8_t pH_Up) {
	static uint8_t Dosingstage = 0, pHDowndosestage = 0, pHupdosestage = 0;

	if (Doseinit <= Dosestart_cnt1) {

		Dosestartflag = false; //timer

		if ((Ec > EC_Setpoint) || (Ec_Pump_Error) || (!Ec_A) || (!Ec_B)) { //EC_Setpoint eeprom

			isEcDosing = false;

		} else {

			isEcDosing = true;

		}
		if (Ecflag) {
			if ((Ec_A && Ec_B && !valvestatus && W_lvlstatus && isEcDosing
					&& !EC_Error) && (Ec < EC_Setpoint)) {

				switch (Dosingstage) {
				case 0:

					Prevec = Ec;
					if (!pump_auto_Man) {
						Waterpump_trigger(0);
						waterpump_status = 0;
					}
					EcA_Pump_trigger(1);
					EcB_Pump_trigger(1);
					ECA_ON_count++;
					EcA_status = 1;
					EcB_status = 1;
					Dosingstage = 1;
					ECDose_Activeflag = true;   // for sec cnt flag in timer
					Ec_PumpON_cnt = 0;
					break;

				case 1:
					if ((Ec_PumpON_cnt >= (Ecdose_sec * 2))
							&& (ECDose_Activeflag)) { //40 sec eeprom
						Dosingstage = 0;
						EcA_Pump_trigger(0);
						EcB_Pump_trigger(0);
//					EcA_status = 0;
//					EcB_status = 0;
						Ec_PumpON_cnt = 0;
						ECDose_Activeflag = false;
						ECmixflag = true;
						Ecmixcnt = 0;
						ECcycle_Tflag = true;         //cycle timer enable flag
						Ecflag = false;
						Eccycle_cnt = 0;

					}
					break;
				}

			}

			else {
				EcA_Pump_trigger(0);
				EcB_Pump_trigger(0);
//			    EcA_status = 0;
//			    EcB_status = 0;
				Dosingstage = 0;
				ECDose_Activeflag = false;

			}
		}

		if ((ECcycle_Tflag) && (Eccycle_cnt >= (Ecdose_cycle * 120))) { //eeoprom

			Ecflag = true;
			Eccycle_cnt = 0;
			ECcycle_Tflag = false;

		}

		if (pH_flag) {

			if ((pH > pH_max) && pH_Down && !isEcDosing && W_lvlstatus
					&& !valvestatus && pH_flag && !pH_Error) {

				switch (pHDowndosestage) {
				case 0:
					PrevpH = pH;
					if (!pump_auto_Man) {
						Waterpump_trigger(0);
						waterpump_status = 0;
					}
					pHDown_Pump_trigger(1);

					pHDown_status = 1;
					PHD_ON_count++;
					pH_pump_Activeflag = true;
					pHDowndosestage = 1;
					pH_pumpON_cnt = 0;
					break;

				case 1:

					if ((pH_pumpON_cnt >= (pHdose_sec * 2))
							&& (pH_pump_Activeflag)) {
						pHDowndosestage = 0;
						pH_pumpON_cnt = 0;
						pHcycle_Tflag = true;
						pHcycle_cnt = 0;
						pHDown_Pump_trigger(0);

						//pHDown_status = 0;
						pH_flag = false;
						pH_pump_Activeflag = false;
						pHdmixflag = true;
						pHdmixcnt = 0;
					}

					break;

				}

			} else {
				pHDowndosestage = 0;
				pH_pump_Activeflag = false;
				pHDown_Pump_trigger(0);

			}

		}

		if (pH_flag) {

			if ((pH < pH_min) && pH_Up && !isEcDosing && W_lvlstatus
					&& !valvestatus && !pH_Error) {

				switch (pHupdosestage) {
				case 0:
					PrevpH = pH;
					if (!pump_auto_Man) {
						Waterpump_trigger(0);
						waterpump_status = 0;
					}
					pHup_Pump_trigger(1);
					pHUp_status = 1;
					PHUP_ON_count++;
					pH_UPpump_Activeflag = true;
					pHupdosestage = 1;
					pH_pumpON_cnt = 0;
					break;

				case 1:

					if ((pH_pumpON_cnt >= (pHdose_sec * 2))
							&& (pH_UPpump_Activeflag)) {
						pHup_Pump_trigger(0);

						//pHUp_status = 0;
						pHupdosestage = 0;
						pH_pumpON_cnt = 0;
						pHcycle_Tflag = true;
						pHcycle_cnt = 0;
						pH_flag = false;
						pH_UPpump_Activeflag = false;

						pHupmixflag = true;
						pHupmixcnt = 0;
					}

					break;

				}

			} else {
				pHupdosestage = 0;
				pHup_Pump_trigger(0);
				pH_UPpump_Activeflag = false;

			}

		}

		if ((pHcycle_Tflag) && (pHcycle_cnt >= (pHdose_cycle * 120))) {
			pH_flag = true;

			pHcycle_cnt = 0;
			pHcycle_Tflag = false;

		}

		if ((ECmixflag) && (Ecmixcnt >= pumpcheckwait)) {
			//mix off
			ECpumperror();
			Ecmixcnt = 0;
			ECmixflag = false;
		}

		if ((pHdmixflag) && (pHdmixcnt >= pumpcheckwait)) {
			//mix off
			pHdownpumperror();
			pHdmixcnt = 0;
			pHdmixflag = false;
		}

		if ((pHupmixflag) && (pHupmixcnt >= pumpcheckwait)) {
			//mix off
			pHuppumperror();
			pHupmixcnt = 0;
			pHupmixflag = false;
		}

	}

}


void waterpump(int16_t soilMoisture[], int Total_no){

    int validSensors = 0;
    int belowLow = 0;
    int aboveHigh = 0;


    for(int i = 0; i < Total_no; i++){
        if(soilMoisture[i] == -5){
        	soil_errorSensors |= (1 << i);
            continue;
        }
        else{
        	 soil_errorSensors &= ~(1 << i);
        }

        validSensors++;

        if((soilMoisture[i]/10) <= PumpON_setpoint) belowLow++;
        if((soilMoisture[i]/10) >= PumpOff_setpoint) aboveHigh++;
    }


    if (validSensors == 0 && !pump_auto_Man) {
           Waterpump_trigger(0);
           waterpump_status = 0;
//           sprintf(uart_msg, " Off1...\r\n");
//                                  HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);
           return;
       }

    //char uart_msg[100];
//
//        sprintf(uart_msg,
//                "aboveHigh=%d valid=%d Wlvl=%d macro=%d auto=%d\r\n",
//                aboveHigh,
//                validSensors,
//                W_lvlstatus,
//                macro_status,
//                pump_auto_Man);
//
//        HAL_UART_Transmit(&huart2,
//                          (uint8_t *)uart_msg,
//                          strlen(uart_msg),
//                          HAL_MAX_DELAY);

    if((belowLow >= (validSensors + 1) / 2) && !waterpump_status && !pHUp_status && !pHDown_status && !EcA_status && !EcB_status && !valvestatus && W_lvlstatus && !pump_auto_Man && !macro_status)
    {

        Waterpump_trigger(1);
        waterpump_status = 1;
        pump_on_Cnt++;
//        sprintf(uart_msg, " on....\r\n");
//               HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);

//		write_EEPROM();
//		read_EEPROM();
//		backup_write_EEPROM();
//		backup_read_EEPROM();

    }


    else if (  ((aboveHigh > (validSensors / 2)) && !pump_auto_Man) || (!W_lvlstatus) ||(macro_status))
    {

        Waterpump_trigger(0);
        waterpump_status = 0;
//        sprintf(uart_msg, " OFF....\r\n");
//          HAL_UART_Transmit(&huart2, (uint8_t*)uart_msg, strlen(uart_msg), HAL_MAX_DELAY);

    }


}




void ECpumperror(){

		if ((Ec-Prevec ) <= Ec_PumpEr_setpoint) {


			EcPumpEr++;

		} else {
			EcPumpEr = 0;

		}

		if (EcPumpEr >= 4) {

			Ec_Pump_Error=1;
		} else {

			Ec_Pump_Error=0;
		}


}



void pHdownpumperror() {

	if ((pH - PrevpH) <= pH_PumpEr_setpoint) {
		pHdPumpEr++;
	} else {
		pHdPumpEr = 0;
	}

	if (pHdPumpEr >= 4) {
		pHd_Pump_Error = 1;
	} else {
		pHd_Pump_Error = 0;
	}

}



void pHuppumperror() {


	if ((PrevpH - pH) <= pH_PumpEr_setpoint) {
		pHupPumpEr++;

	} else {
		pHupPumpEr = 0;
	}

	if (pHupPumpEr >= 4) {

		pHup_Pump_Error = 1;
	} else {

		pHup_Pump_Error = 0;
	}

}



uint8_t TP4_curtain_O_P(float lux,uint8_t Lux_Error){

	uint8_t status = 0;
	    static uint16_t last_state = 0;

	    uint16_t new_state;

	    if ((lux <= LUX_MINsetpoint)&&(!Lux_Error)){//
	        new_state = TP4_OUTPUT_ON;

	    }
	    else if ((lux >= LUX_MAXsetpoint)||(Lux_Error)){


	        new_state = TP4_OUTPUT_OFF;
	    }
	    else{
	        return status;
	    }

	    /* Write only if the required state is different */
	    if (new_state != last_state)
	    {
	        status = Write_Single_Register(TP4_Write_Address,TP4_Write_curtain,new_state);

	        if (status == 0)
	            last_state = new_state;

	    }

	    return status;

}



uint8_t TP4_fogger_O_P(float hum_Value, uint8_t Envir_HT_Error)
{
    uint8_t status = 0;
    static uint16_t last_state = 0;
    uint16_t new_state;


    if ((hum_Value <= HUM_FOG_MINsetpoint) && (!Envir_HT_Error) && (!fogger_delay_active)) {
        new_state = TP4_OUTPUT_ON;
    }

    else if ((hum_Value >= HUM_FOG_MAXsetpoint) || (Envir_HT_Error)) {
        new_state = TP4_OUTPUT_OFF;


        if (last_state == TP4_OUTPUT_ON && new_state == TP4_OUTPUT_OFF) {
            fogger_delay_active = 1;
            fogger_delay_counter = 0;
        }
    }
    else {
        return status;
    }

    if (new_state != last_state) {
        status = Write_Single_Register(TP4_Write_Address,
                                       TP4_Write_fogger,
                                       new_state);
        if (status == 0)
            last_state = new_state;
    }

    return status;
}





void TP4_Decode_Outputs(int16_t status){

	 if (status & 1) {
	        FAN_Status=1;
	    } else {
	    	 FAN_Status=0;
	    }

	    if (status & 2) {
	        Curtain_Status=1;
	    } else {
	    	Curtain_Status=0;
	    }

	    if (status & 4) {
	    	//Curtain_Status=1;
	    } else {
	    	//  Curtain_Status=0;
	    }

	    if (status & 8) {
	    	Fogger_Status=1;
	    } else {
	    	Fogger_Status=0;
	    }

}

void TP4_cirulation_Fan(uint16_t circulationfan_setpoint){
	 Tp4_Fan_WStatus = Write_Single_Register(TP4_Write_Address,TP4_Write_TEMP,circulationfan_setpoint);

}

