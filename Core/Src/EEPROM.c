/*
 * EEPROM.c
 *
 *  Created on: Sep 3, 2025
 *      Author: Sridhar A
 */
#include <stdint.h>
#include "main.h"
#include "EEPROM.h"
#include "FA_RS485.h"
//extern UART_HandleTypeDef huart2; // Modbus Slave UART

uint32_t R_Ecsetpoint,R_Ecdose_sec,R_Ecdose_cycle,R_pH_min,R_pH_max,R_pHdose_sec,R_pHdose_cyle,R_Pump_ONsetpoint,R_Pump_OFFsetpoint,R_Ec_pumpsetpoint,R_pH_pumpsetpoint;
uint32_t R_kvalue,R_LUX_MINsetpoint,R_LUX_MAXsetpoint,R_HUM_FOG_MINsetpoint,R_HUM_FOG_MAXsetpoint,R_HUM_FOG_WaitTime;
extern uint32_t Acidic_voltage ,Neutral_Voltage,Ecdose_sec,Ecdose_cycle,pHdose_sec,pHdose_cycle,PumpON_setpoint,PumpOff_setpoint,LUX_MINsetpoint,LUX_MAXsetpoint,HUM_FOG_MINsetpoint,HUM_FOG_MAXsetpoint,HUM_FOG_WaitTime,R_Total_no_mois,Total_no_mois,nutrient_setpoint;


uint32_t R_Neutral_Voltage,R_Acidic_voltage;
char buffe[250];


extern float kvalue,EC_Setpoint,pH_min,pH_max,Ec_PumpEr_setpoint,pH_PumpEr_setpoint;

extern uint32_t pump_auto_Man,nutrient_ON_sec,pump_on_Cnt,EcA_fill_count ,pHDN_fill_count,EcB_fill_count,pHUP_fill_count,water_fill_count;
extern uint32_t ECA_ON_count,PHD_ON_count,PHUP_ON_count;

extern volatile uint32_t pump_on_hours;
 uint32_t Rpump_auto_Man,Rnutrient_ON_sec,R_pump_on_cnt,R_EcA_fill_count ,R_pHDN_fill_count,R_EcB_fill_count,R_pHUP_fill_count;
uint32_t R_water_fill_count,R_ECA_ON_count,R_PHD_ON_count,R_PHUP_ON_count,R_pump_on_hours;
 uint32_t Rnutrient_setpoint;

 typedef struct {
     uint32_t Ron_hr;
     uint32_t Ron_min;
     uint32_t Roff_hr;
     uint32_t Roff_min;
     uint32_t Renable;
 } Schedule_Rt;

Schedule_Rt R_schedules[5];


void write_EEPROM(void){
	 HAL_FLASH_Unlock();
	  FLASH_EraseInitTypeDef eraseInitStruct;
	  uint32_t pageError = 0;

	  // Set up the erase parameters
	  eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	  eraseInitStruct.Page = 62;
	  //(FLASH_USER_START_ADDR - FLASH_BASE) / FLASH_PAGE_SIZE;  // Adjust for your STM32 series
	  eraseInitStruct.NbPages = 1;

	  HAL_FLASHEx_Erase(&eraseInitStruct, &pageError);  // Erase the Flash Memory

	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F000, Acidic_voltage );
	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F008, Neutral_Voltage);

	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F010, EC_Setpoint*100 );
	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F018, Ecdose_sec);
	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F020, Ecdose_cycle);

	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F028, pH_min*100 );
	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F030, pH_max*100 );
	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F038, pHdose_sec);
	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F040, pHdose_cycle);

	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F048, PumpON_setpoint);
	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F050, PumpOff_setpoint);

	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F058, Ec_PumpEr_setpoint*100);
	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F060, pH_PumpEr_setpoint*100 );

	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F068, kvalue * 100);

	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F070, LUX_MINsetpoint);
	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F078, LUX_MAXsetpoint);

	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F080, HUM_FOG_MINsetpoint);
	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F088, HUM_FOG_MAXsetpoint);
	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F090, HUM_FOG_WaitTime);
	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F098, Total_no_mois);

//	  // -------------------- Schedule 1 --------------------
//	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F0A0, sch1_on_hr);
//	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F0A8, sch1_on_min);
//	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F0B0, sch1_off_hr);
//	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F0B8, sch1_off_min);
//
//	  // -------------------- Schedule 2 --------------------
//	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F0C0, sch2_on_hr);
//	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F0C8, sch2_on_min);
//	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F0D0, sch2_off_hr);
//	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F0D8, sch2_off_min);
//
//	  // -------------------- Schedule 3 --------------------
//	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F0E0, sch3_on_hr);
//	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F0E8, sch3_on_min);
//	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F0F0, sch3_off_hr);
//	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F0F8, sch3_off_min);

	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F100, pump_auto_Man);

	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F108, nutrient_ON_sec);

	  HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F110, nutrient_setpoint);
	  // -------------------- count --------------------
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F118, pump_on_Cnt);


	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F120, EcA_fill_count);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F128, EcB_fill_count);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F130, pHDN_fill_count);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F138, pHUP_fill_count);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F140, water_fill_count);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F148, ECA_ON_count);
       	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F150, PHD_ON_count);
       	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F158, PHUP_ON_count);
       	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F160, pump_on_hours);


        uint32_t addr = 0x0801F168;

           for(uint8_t s = 0; s < 5; s++)
           {
               HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, schedules[s].on_hr);
               addr += 8;
               HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, schedules[s].on_min);
               addr += 8;
               HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, schedules[s].off_hr);
               addr += 8;
               HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, schedules[s].off_min);
               addr += 8;
               HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, schedules[s].enable);
               addr += 8;
           }


	  HAL_FLASH_Lock();

}

void read_EEPROM(void){

	R_Acidic_voltage = *(__IO uint32_t*)0x0801F000;
	R_Neutral_Voltage = *(__IO uint32_t*)0x0801F008;

	R_Ecsetpoint = *(__IO uint32_t*)0x0801F010;
	R_Ecdose_sec = *(__IO uint32_t*)0x0801F018;
	R_Ecdose_cycle = *(__IO uint32_t*)0x0801F020;

	R_pH_min = *(__IO uint32_t*)0x0801F028;
	R_pH_max = *(__IO uint32_t*)0x0801F030;
	R_pHdose_sec = *(__IO uint32_t*)0x0801F038;
	R_pHdose_cyle = *(__IO uint32_t*)0x0801F040;

	R_Pump_ONsetpoint = *(__IO uint32_t*)0x0801F048;
	R_Pump_OFFsetpoint = *(__IO uint32_t*)0x0801F050;

	R_Ec_pumpsetpoint = *(__IO uint32_t*)0x0801F058;
	R_pH_pumpsetpoint = *(__IO uint32_t*)0x0801F060;


	R_kvalue = *(__IO uint32_t*)0x0801F068;

	R_LUX_MINsetpoint = *(__IO uint32_t*)0x0801F070;
	R_LUX_MAXsetpoint = *(__IO uint32_t*)0x0801F078;

	R_HUM_FOG_MINsetpoint = *(__IO uint32_t*)0x0801F080;
	R_HUM_FOG_MAXsetpoint = *(__IO uint32_t*)0x0801F088;
	R_HUM_FOG_WaitTime = *(__IO uint32_t*)0x0801F090;
	R_Total_no_mois = *(__IO uint32_t*)0x0801F098;

//	// -------------------- Schedule 1 --------------------
//	Rsch1_on_hr   = *(__IO uint32_t*)0x0801F0A0;
//	Rsch1_on_min  = *(__IO uint32_t*)0x0801F0A8;
//	Rsch1_off_hr  = *(__IO uint32_t*)0x0801F0B0;
//	Rsch1_off_min = *(__IO uint32_t*)0x0801F0B8;
//
//	// -------------------- Schedule 2 --------------------
//	Rsch2_on_hr   = *(__IO uint32_t*)0x0801F0C0;
//	Rsch2_on_min  = *(__IO uint32_t*)0x0801F0C8;
//	Rsch2_off_hr  = *(__IO uint32_t*)0x0801F0D0;
//	Rsch2_off_min = *(__IO uint32_t*)0x0801F0D8;
//
//	// -------------------- Schedule 3 --------------------
//	Rsch3_on_hr   = *(__IO uint32_t*)0x0801F0E0;
//	Rsch3_on_min  = *(__IO uint32_t*)0x0801F0E8;
//	Rsch3_off_hr  = *(__IO uint32_t*)0x0801F0F0;
//	Rsch3_off_min = *(__IO uint32_t*)0x0801F0F8;

	Rpump_auto_Man = *(__IO uint32_t*)0x0801F100;

	Rnutrient_ON_sec = *(__IO uint32_t*)0x0801F108;
	Rnutrient_setpoint = *(__IO uint32_t*)0x0801F110;
	R_pump_on_cnt=*(__IO uint32_t*)0x0801F118;



	R_EcA_fill_count     = *(__IO uint32_t*)0x0801F120;
	R_EcB_fill_count     = *(__IO uint32_t*)0x0801F128;
	R_pHDN_fill_count    = *(__IO uint32_t*)0x0801F130;
	R_pHUP_fill_count    = *(__IO uint32_t*)0x0801F138;
	R_water_fill_count   = *(__IO uint32_t*)0x0801F140;

	R_ECA_ON_count  = *(__IO uint32_t*)0x0801F148;
	R_PHD_ON_count  = *(__IO uint32_t*)0x0801F150;
	R_PHUP_ON_count   = *(__IO uint32_t*)0x0801F158;
	R_pump_on_hours   = *(__IO uint32_t*)0x0801F160;

	uint32_t addr = 0x0801F168;

	for(uint8_t i = 0; i < 5; i++)
	{
	    R_schedules[i].Ron_hr   = *(__IO uint32_t*)addr; addr += 8;
	    R_schedules[i].Ron_min  = *(__IO uint32_t*)addr; addr += 8;
	    R_schedules[i].Roff_hr  = *(__IO uint32_t*)addr; addr += 8;
	    R_schedules[i].Roff_min = *(__IO uint32_t*)addr; addr += 8;
	    R_schedules[i].Renable  = *(__IO uint32_t*)addr; addr += 8;
	}




}

void backup_write_EEPROM(void){

	  HAL_FLASH_Unlock();
	  FLASH_EraseInitTypeDef eraseInitStruct;
	  uint32_t pageError = 0;

	  // Set up the erase parameters
	  eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	  eraseInitStruct.Page = 63;
	  //(FLASH_USER_START_ADDR - FLASH_BASE) / FLASH_PAGE_SIZE;  // Adjust for your STM32 series
	  eraseInitStruct.NbPages = 1;

	  HAL_FLASHEx_Erase(&eraseInitStruct, &pageError);  // Erase the Flash Memory

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F800,
			R_Acidic_voltage);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F808,
			R_Neutral_Voltage);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F810, R_Ecsetpoint);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F818, R_Ecdose_sec);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F820, R_Ecdose_cycle);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F828, R_pH_min);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F830, R_pH_max);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F838, R_pHdose_sec);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F840, R_pHdose_cyle);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F848,
			R_Pump_ONsetpoint);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F850,
			R_Pump_OFFsetpoint);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F858,
			R_Ec_pumpsetpoint);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F860,
			R_pH_pumpsetpoint);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F868, R_kvalue);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F870,
			R_LUX_MINsetpoint);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F878,
			R_LUX_MAXsetpoint);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F880,
			R_HUM_FOG_MINsetpoint);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F888,
			R_HUM_FOG_MAXsetpoint);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F890,
			R_HUM_FOG_WaitTime);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F898,
			R_Total_no_mois);

//	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F8A0, Rsch1_on_hr);
//	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F8A8, Rsch1_on_min);
//	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F8B0, Rsch1_off_hr);
//	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F8B8, Rsch1_off_min);
//
//	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F8C0, Rsch2_on_hr);
//	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F8C8, Rsch2_on_min);
//	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F8D0, Rsch2_off_hr);
//	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F8D8, Rsch2_off_min);
//
//	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F8E0, Rsch3_on_hr);
//	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F8E8, Rsch3_on_min);
//	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F8F0, Rsch3_off_hr);
//	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F8F8, Rsch3_off_min);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F900, Rpump_auto_Man);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F908,Rnutrient_ON_sec);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F910,Rnutrient_setpoint);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F918,R_pump_on_cnt);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F920,  R_EcA_fill_count);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F928,R_EcB_fill_count);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F930, R_pHDN_fill_count);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F938, R_pHUP_fill_count);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F940, R_water_fill_count);

	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F948, R_ECA_ON_count);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F950, R_PHD_ON_count);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F958, R_PHUP_ON_count);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, 0x0801F960, R_pump_on_hours);



	// -------------------- Save 6 Schedules Backup --------------------
	uint32_t addr = 0x0801F968;   // next free address after R_pump_on_hours

	for(uint8_t i = 0; i < 5; i++)
	{
	    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, R_schedules[i].Ron_hr);
	    addr += 8;

	    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, R_schedules[i].Ron_min);
	    addr += 8;

	    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, R_schedules[i].Roff_hr);
	    addr += 8;

	    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, R_schedules[i].Roff_min);
	    addr += 8;

	    HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, addr, R_schedules[i].Renable);
	    addr += 8;
	}









      HAL_FLASH_Lock();
}


void backup_read_EEPROM(void){


	Acidic_voltage = *(__IO uint32_t*)0x0801F800;
	Neutral_Voltage = *(__IO uint32_t*)0x0801F808;



	uint32_t ec_set = *(__IO uint32_t*)0x0801F810;
	EC_Setpoint = ((float)ec_set / 100.0f);

	Ecdose_sec = *(__IO uint32_t*)0x0801F818;
	Ecdose_cycle = *(__IO uint32_t*)0x0801F820;
	uint32_t pH_min1 = *(__IO uint32_t*)0x0801F828;
	pH_min = ((float)pH_min1 / 100.0f);
	uint32_t pH_max1 = *(__IO uint32_t*)0x0801F830;
	pH_max = ((float)pH_max1 / 100.0f);
	pHdose_sec = *(__IO uint32_t*)0x0801F838;
	pHdose_cycle = *(__IO uint32_t*)0x0801F840;

	PumpON_setpoint = *(__IO uint32_t*)0x0801F848;
	PumpOff_setpoint = *(__IO uint32_t*)0x0801F850;


	uint32_t EC_PumpEr_setpoint = *(__IO uint32_t*)0x0801F858;
	Ec_PumpEr_setpoint = ((float)EC_PumpEr_setpoint / 100.0f);

	uint32_t PH_PumpEr_setpoint = *(__IO uint32_t*)0x0801F860;
	pH_PumpEr_setpoint = ((float)PH_PumpEr_setpoint / 100.0f);

	uint32_t EC_kvalue = *(__IO uint32_t*)0x0801F868;
	kvalue = ((float)EC_kvalue / 100.0f);

	LUX_MINsetpoint = *(__IO uint32_t*)0x0801F870;
	LUX_MAXsetpoint = *(__IO uint32_t*)0x0801F878;

	HUM_FOG_MINsetpoint = *(__IO uint32_t*)0x0801F880;
	HUM_FOG_MAXsetpoint = *(__IO uint32_t*)0x0801F888;
	HUM_FOG_WaitTime = *(__IO uint32_t*)0x0801F890;
	Total_no_mois = *(__IO uint32_t*)0x0801F898;


//	sch1_on_hr   = *(__IO uint32_t*)0x0801F8A0;
//	sch1_on_min  = *(__IO uint32_t*)0x0801F8A8;
//	sch1_off_hr  = *(__IO uint32_t*)0x0801F8B0;
//	sch1_off_min = *(__IO uint32_t*)0x0801F8B8;
//
//	sch2_on_hr   = *(__IO uint32_t*)0x0801F8C0;
//	sch2_on_min  = *(__IO uint32_t*)0x0801F8C8;
//	sch2_off_hr  = *(__IO uint32_t*)0x0801F8D0;
//	sch2_off_min = *(__IO uint32_t*)0x0801F8D8;
//
//	sch3_on_hr   = *(__IO uint32_t*)0x0801F8E0;
//	sch3_on_min  = *(__IO uint32_t*)0x0801F8E8;
//	sch3_off_hr  = *(__IO uint32_t*)0x0801F8F0;
//	sch3_off_min = *(__IO uint32_t*)0x0801F8F8;

	pump_auto_Man = *(__IO uint32_t*)0x0801F900;

	nutrient_ON_sec = *(__IO uint32_t*)0x0801F908;

	nutrient_setpoint = *(__IO uint32_t*)0x0801F910;
	pump_on_Cnt = *(__IO uint32_t*)0x0801F918;

	EcA_fill_count   = *(__IO uint32_t*)0x0801F920;
	EcB_fill_count   = *(__IO uint32_t*)0x0801F928;
	pHDN_fill_count  = *(__IO uint32_t*)0x0801F930;
	pHUP_fill_count  = *(__IO uint32_t*)0x0801F938;
	water_fill_count  = *(__IO uint32_t*)0x0801F940;

	ECA_ON_count  = *(__IO uint32_t*)0x0801F948;
	PHD_ON_count  = *(__IO uint32_t*)0x0801F950;
	PHUP_ON_count  = *(__IO uint32_t*)0x0801F958;
	pump_on_hours  = *(__IO uint32_t*)0x0801F960;


	uint32_t addr = 0x0801F968;
	    for(uint8_t i=0; i<5; i++)
	    {
	        schedules[i].on_hr   = *(__IO uint32_t*)(addr);        addr += 8;
	        schedules[i].on_min  = *(__IO uint32_t*)(addr);        addr += 8;
	        schedules[i].off_hr  = *(__IO uint32_t*)(addr);        addr += 8;
	        schedules[i].off_min = *(__IO uint32_t*)(addr);        addr += 8;
	        schedules[i].enable  = *(__IO uint32_t*)(addr);        addr += 8;
	    }


//
//	// First part
//	sprintf(buffe,
//	    "\r\n--- Flash Values ---\r\n"
//		"Acidic_voltage       : %lu\r\n"
//		"Neutral_Voltage      : %lu\r\n"
//		"Kvalue               : %.2f\r\n"
//	    "EC_Setpoint          : %.2f\r\n"
//	    "Ecdose_sec           : %lu\r\n"
//	    "Ecdose_cycle         : %lu\r\n"
//	    "pH_min               : %.2f\r\n"
//	    "pH_max               : %.2f\r\n"
//	    "pHdose_sec           : %lu\r\n"
//	    "pHdose_cycle         : %lu\r\n"
//	    "PumpON_setpoint      : %lu\r\n"
//	    "PumpOff_setpoint     : %lu\r\n",
//		Acidic_voltage,
//		Neutral_Voltage,
//		kvalue,
//	    EC_Setpoint,
//	    Ecdose_sec,
//	    Ecdose_cycle,
//	    pH_min,
//	    pH_max,
//	    pHdose_sec,
//	    pHdose_cycle,
//	    PumpON_setpoint,
//	    PumpOff_setpoint
//
//	);
//	HAL_UART_Transmit(&huart2, (uint8_t*)buffe, strlen(buffe), HAL_MAX_DELAY);
//
//	// Second part
//	sprintf(buffe,
//	    "Ec_PumpEr_setpoint   : %.2f\r\n"
//	    "pH_PumpEr_setpoint   : %.2f\r\n"
//	    "LUX_MINsetpoint      : %lu\r\n"
//	    "LUX_MAXsetpoint      : %lu\r\n"
//	    "HUM_FOG_MINsetpoint  : %lu\r\n"
//	    "HUM_FOG_MAXsetpoint  : %lu\r\n"
//	    "HUM_FOG_WaitTime     : %lu\r\n"
//		"Total_no_mois         : %lu\r\n"
//	    "---------------------\r\n",
//	    Ec_PumpEr_setpoint,
//	    pH_PumpEr_setpoint,
//	    LUX_MINsetpoint,
//	    LUX_MAXsetpoint,
//	    HUM_FOG_MINsetpoint,
//	    HUM_FOG_MAXsetpoint,
//	    HUM_FOG_WaitTime,
//		Total_no_mois
//	);
//	HAL_UART_Transmit(&huart2, (uint8_t*)buffe, strlen(buffe), HAL_MAX_DELAY);
//
//	 sprintf(buffe,
//
//	        "Pump Mode: %lu  nutrient_ON_sec: %lu nutrient_setpoint  %lu \r\n",
//
//	        pump_auto_Man,nutrient_ON_sec,nutrient_setpoint);
//
//	    HAL_UART_Transmit(&huart2, (uint8_t*)buffe, strlen(buffe), HAL_MAX_DELAY);
//
//	    sprintf(buffe,
//	        "\r\n--- COUNTERS ---\r\n"
//			"pump_on_Cnt       : %lu\r\n"
//			"EcA_fill_count    : %lu\r\n"
//			"EcB_fill_count    : %lu\r\n"
//			"pHDN_fill_count   : %lu\r\n"
//			"pHUP_fill_count   : %lu\r\n"
//			"water_fill_count  : %lu\r\n"
//			"ECA_ON_count    : %lu\r\n"
//			"PHD_ON_count   : %lu\r\n"
//			"PHUP_ON_count   : %lu\r\n"
//			"pump_on_hours  : %lu\r\n"
//
//
//	        "==============================\r\n",
//	        pump_on_Cnt,
//	        EcA_fill_count,
//	        EcB_fill_count,
//	        pHDN_fill_count,
//	        pHUP_fill_count,
//	        water_fill_count,
//			ECA_ON_count,
//			PHD_ON_count,
//			PHUP_ON_count,
//			pump_on_hours
//	    );
//	    HAL_UART_Transmit(&huart2,(uint8_t*)buffe,strlen(buffe),HAL_MAX_DELAY);
//
//	    sprintf(buffe, "\r\n--- Schedules ---\r\n");
//	    HAL_UART_Transmit(&huart2, (uint8_t*)buffe, strlen(buffe), HAL_MAX_DELAY);
//
//	    for(uint8_t i=0; i<5; i++)
//	    {
//	        sprintf(buffe,
//	            "Schedule %d: ON  %02lu:%02lu  |  OFF %02lu:%02lu  | Enable: %lu\r\n",
//	            i+1,
//	            schedules[i].on_hr,
//	            schedules[i].on_min,
//	            schedules[i].off_hr,
//	            schedules[i].off_min,
//	            schedules[i].enable
//	        );
//	        HAL_UART_Transmit(&huart2,(uint8_t*)buffe,strlen(buffe),HAL_MAX_DELAY);
//	    }



}
