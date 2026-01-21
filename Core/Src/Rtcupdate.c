/*
 * Time.c
 *
 *  Created on: Nov 5, 2025
 *      Author: Sridhar Denvik
 */

#include "rtcupdate.h"
#include "main.h"
#include <stdio.h>
#include <stdint.h>
#include <time.h>
#include"Relayinit.h"
#include "EEPROM.h"
#include "FA_RS485.h"
// for struct tm, time_t, gmtime()
extern  uint8_t waterpump_status;
extern uint8_t W_lvlstatus,valvestatus;

extern uint32_t nutrient_setpoint;
uint32_t pump_on_Cnt;
/*extern UART_HandleTypeDef huart2; /*/
extern RTC_HandleTypeDef hrtc;
uint8_t output_triggered ,macro_status,micro_status,checkflag,count_updateflag=1;
extern volatile uint16_t output_timer_count;
extern uint8_t pHup_Pump_Error;
 char msg2[256];
#define IST_OFFSET_SEC 19800  // 5 hours 30 minutes
 void Set_RTC_From_Epoch(uint32_t epoch)
 {
     epoch += IST_OFFSET_SEC;  // Convert UTC to IST

     time_t raw = epoch;
     struct tm *timeinfo = gmtime(&raw);

     RTC_TimeTypeDef sTime = {0};
     RTC_DateTypeDef sDate = {0};

     sTime.Hours   = timeinfo->tm_hour;
     sTime.Minutes = timeinfo->tm_min;
     sTime.Seconds = timeinfo->tm_sec;
     sDate.Date    = timeinfo->tm_mday;
     sDate.Month   = timeinfo->tm_mon + 1;
     sDate.Year    = timeinfo->tm_year - 100;

     HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
     HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    /* sprintf(msg2, "RTC (IST) set: %02d-%02d-%04d %02d:%02d:%02d (Epoch: %lu)\r\n",
                sDate.Date, sDate.Month, sDate.Year + 2000,
                sTime.Hours, sTime.Minutes, sTime.Seconds, epoch);

        HAL_UART_Transmit(&huart2, (uint8_t*)msg2, strlen(msg2), HAL_MAX_DELAY);*/
 }

 void Relay_Schedule_Control(void)
  {
      RTC_TimeTypeDef sTime;
      RTC_DateTypeDef sDate;

      // Read current RTC time
      HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
      HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

      // Convert current time to minutes since midnight
      uint16_t now = sTime.Hours * 60 + sTime.Minutes;
   static uint8_t relay_state=0;
      uint8_t new_state = 0; // default OFF

      // Loop through all schedules
      for(uint8_t i = 0; i < 6; i++)
      {
          if(schedules[i].enable) // only enabled schedules
          {
              uint16_t on_time  = schedules[i].on_hr  * 60 + schedules[i].on_min;
              uint16_t off_time = schedules[i].off_hr * 60 + schedules[i].off_min;

              // Check if current time is within schedule
              if(now >= on_time && now < off_time)
              {
                  new_state = 1; // Relay ON
                  break;         // No need to check further schedules
              }
          }
      }

      // Update relay if state changed
      if(new_state != relay_state)
      {
          relay_state = new_state;
          Waterpump_trigger(relay_state);

          if(relay_state)
          {
              waterpump_status = 1;
              pump_on_Cnt++;

              // Optional: save status to EEPROM
              write_EEPROM();
              read_EEPROM();
              backup_write_EEPROM();
              backup_read_EEPROM();
          }
          else
          {
              waterpump_status = 0;
          }

          // Optional: debug print
         /* char msg[100];
          sprintf(msg, "Relay %s at %02d:%02d\r\n",
                  relay_state ? "ON" : "OFF",
                  sTime.Hours, sTime.Minutes);
          HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);*/
      }
  }

void micro_Macro(void)
{
    RTC_TimeTypeDef sTime;
    RTC_DateTypeDef sDate;

    //static uint8_t done_today = 0;  // ensures single trigger per day

    HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    // Check 08:00 AM (24-hour = 8)
	if ((sTime.Hours == nutrient_setpoint|| sTime.Hours == (nutrient_setpoint + 4)) && sTime.Minutes == 0&& !macro_status && !micro_status && !output_triggered && !checkflag && W_lvlstatus)    {

    	macro_nutrient(1);
    	micro_nutrient(1);

		output_triggered = 1;
		output_timer_count = 0;
		macro_status = 1;
		micro_status = 1;
       checkflag=1;

     /*   sprintf(msg2, "Output Triggered at 08:00 AM\r\n");
        HAL_UART_Transmit(&huart2, (uint8_t*)msg2, strlen(msg2), HAL_MAX_DELAY);*/
    }


    else if(((sTime.Hours == nutrient_setpoint || (sTime.Hours == nutrient_setpoint + 4)) && sTime.Minutes == 10 && checkflag)){

    	checkflag=0;

//    	 sprintf(msg2, "flsg orff AM\r\n");
//    	 HAL_UART_Transmit(&huart2, (uint8_t*)msg2, strlen(msg2), HAL_MAX_DELAY);

    }


 if ((sTime.Hours == 18 && sTime.Minutes==0)&&( count_updateflag)){
	            write_EEPROM();
	         	read_EEPROM();
	         	backup_write_EEPROM();
	         	backup_read_EEPROM();

	         	count_updateflag=0;
 }



}


