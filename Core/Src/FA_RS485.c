/*
 * FA_RS485.c
 *
 *  Created on: Sep 26, 2025
 *      Author: Sridhar A
 */

#include "rtcupdate.h"
#include "main.h"
#include "soil_moisture.h"

#include"Relayinit.h"
#include "EEPROM.h"
#include <stdint.h>
#include "sys_control.h"
#include <time.h>
#include "FA_RS485.h"
#include<String.h>
#include "WatertempSensor.h"
#include <stdlib.h>

extern UART_HandleTypeDef huart1; //
extern UART_HandleTypeDef huart2; //
extern RTC_HandleTypeDef hrtc;

char buf[250];

#define FA_MODBUS_BUFFER_SIZE 100

#define SLAVE_ID 0x01

#define UART2_MAX_FRAME     256



uint8_t  uart2_rx_byte;
char    uart2_rx_buf[UART2_MAX_FRAME];
uint16_t uart2_rx_len = 0;


extern int16_t soilmoisuture[12];


extern uint16_t soil_errorSensors;
extern float Wtemp, pH, Ec, Lux, hum_Value, Temp_value;
extern int16_t TP4_TEMP_Value;
extern uint8_t Ec_A, Ec_B, pH_Up, pH_Down, Wtemp_Error, Ec_Pump_Error,
		pHd_Pump_Error, pHup_Pump_Error, Envir_HT_Error, Lux_Error, TP4_ERROR,
		W_lvlstatus, W_lvl1, W_lvl2, TP4_Temp_error, EC_Error, pH_Error;
extern uint8_t valvestatus, waterpump_status, EcA_status, EcB_status,
		pHUp_status, pHDown_status, FAN_Status, Curtain_Status, Fogger_Status,
		macro_status, micro_status;
uint8_t RxByte, light_status, RxByte_U2;
uint8_t Modbus_Slave_Buffer[FA_MODBUS_BUFFER_SIZE];

uint16_t Modbus_Buffer_Count1 = 0, circulationfan_setpoint;
volatile uint8_t FrameReceived = 0,uart2_frame_ready=0;
extern volatile uint32_t pump_on_hours;
extern uint32_t pump_on_Cnt, EcA_fill_count, pHDN_fill_count, EcB_fill_count,
		pHUP_fill_count, water_fill_count, ECA_ON_count, PHD_ON_count,
		PHUP_ON_count;
uint32_t Ecdose_sec, Ecdose_cycle, pHdose_sec, pHdose_cycle, PumpON_setpoint,
		PumpOff_setpoint, LUX_MINsetpoint, LUX_MAXsetpoint, HUM_FOG_MINsetpoint,
		HUM_FOG_MAXsetpoint, HUM_FOG_WaitTime, R_Total_no_mois, Total_no_mois,
		nutrient_setpoint;
uint32_t W_Ecsetpoint, W_pH_min, W_pH_max, W_Ec_pumpsetpoint, W_pH_pumpsetpoint,
		pump_auto_Man, nutrient_ON_sec;

Schedule_t schedules[MAX_SCHEDULES];

#define SCHEDULE_START_ADDR  0x0802
#define REG_PER_SCHEDULE     5

float EC_Setpoint, pH_min, pH_max, Ec_PumpEr_setpoint, pH_PumpEr_setpoint;

// ================= CRC Check =================
uint8_t Check_CRC_FA(uint8_t *frame, uint16_t len) {
	if (len < 4)
		return 0;
	uint16_t tmp_crc = ModbusCRC16(frame, len - 2);
	uint16_t received_crc = (frame[len - 1] | (frame[len - 2] << 8)); // swapped

	//  uint16_t received_crc = (frame[len - 2] | (frame[len - 1] << 8));

//sprintf(buf, "Calc CRC: %04X, Recv CRC: %04X\r\n", tmp_crc, received_crc);
//        HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	return (tmp_crc == received_crc);
}

uint16_t Get_Register_Value(uint16_t addr) {

	if (addr >= 0x0020 && addr < (0x0020 + Total_no_mois)) {
		uint16_t index = addr - 0x0020;

		return soilmoisuture[index];
	}

	switch (addr) {
	// Sensor value

	case 0x0000:
		return (uint16_t) (Wtemp * 10);
	case 0x0001:
		return (uint16_t) (Temp_value * 10);
	case 0x0002:
		return (uint16_t) (hum_Value * 10);
	case 0x0003:
		return (uint16_t) (Lux * 10);

	case 0x0004:
		return TP4_TEMP_Value;
	case 0x0005:
		return (uint16_t) (pH * 10);
	case 0x0006:
		return (uint16_t) (Ec * 10);
	case 0x0007:
		return Ec_A;
	case 0x0008:
		return Ec_B;
	case 0x0009:
		return pH_Up;
	case 0x000A:
		return pH_Down;
	case 0x000B:
		return W_lvlstatus;
	case 0x000C:
		return pump_on_Cnt;
	case 0x000D:
		return EcA_fill_count;
	case 0x000E:
		return EcB_fill_count;
	case 0x000F:
		return pHUP_fill_count;
	case 0x0010:
		return pHDN_fill_count;
	case 0x00011:
		return water_fill_count;
	case 0x00012:
		return ECA_ON_count;
	case 0x00013:
		return PHD_ON_count;
	case 0x00014:
		return PHUP_ON_count;
	case 0x00015:
		return pump_on_hours;

		// Error flags
	case 0x0040:
		return Wtemp_Error;
	case 0x0041:
		return Envir_HT_Error;
	case 0x0042:
		return Lux_Error;
	case 0x0043:
		return TP4_ERROR;
	case 0x0044:
		return Ec_Pump_Error;
	case 0x0045:
		return pHd_Pump_Error;
	case 0x0046:
		return pHup_Pump_Error;

	case 0x0047:
		return pH_Error;
	case 0x0048:
		return EC_Error;
	case 0x0049:
		return soil_errorSensors;
	case 0x004a:
		return TP4_Temp_error;

		//status

	case 0x1000:
		return valvestatus;
	case 0X1001:
		return waterpump_status;
	case 0x1002:
		return EcA_status;
	case 0x1003:
		return EcB_status;
	case 0x1004:
		return pHUp_status;
	case 0x1005:
		return pHDown_status;
	case 0x1006:
		return FAN_Status;
	case 0x1007:
		return Curtain_Status;
	case 0x1008:
		return Fogger_Status;
	case 0x1009:
		return light_status;

	case 0x101A:
		return macro_status;

	case 0x101b:
		return micro_status;

	default:
		return 0;
	}
}

// ================= UART Reception =================
void Modbus_StartReception(void) {
	Modbus_Buffer_Count1 = 0;
	FrameReceived = 0;
	HAL_UART_Receive_IT(&huart1, &RxByte, 1);

	uart2_rx_len = 0;
	    uart2_frame_ready = 0;
	    HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

	if (huart->Instance == USART1) {
//    	sprintf(buf, "interrupt ...\r\n");
//    			        HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
		if (Modbus_Buffer_Count1 < FA_MODBUS_BUFFER_SIZE) {
			Modbus_Slave_Buffer[Modbus_Buffer_Count1++] = RxByte;
		}
		if (Modbus_Buffer_Count1 >= 8) { // minimum frame

			FrameReceived = 1;
		}
		HAL_UART_Receive_IT(&huart1, &RxByte, 1); // continue reception
	}
	/* ================= NORMAL UART ================= */
	else if (huart->Instance == USART2)
	    {

	        if (uart2_rx_len < UART2_MAX_FRAME - 1)
	        {
	            uart2_rx_buf[uart2_rx_len++] = uart2_rx_byte;

	            if (uart2_rx_byte == '\n')
	            {
//	            	sprintf(buf, "enrter\r\n");
//	            			  HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	                uart2_rx_buf[uart2_rx_len] = '\0';
	                uart2_frame_ready = 1;
	            }
	        }
	        HAL_UART_Receive_IT(&huart2, &uart2_rx_byte, 1);
	    }

}

// ===================== Read Registers (0x03) =====================
void Handle_Read_Registers(uint16_t startAddr, uint16_t numRegs) {
	uint8_t response[FA_MODBUS_BUFFER_SIZE];
	uint8_t idx = 0;

	response[idx++] = SLAVE_ID;
	response[idx++] = 0x03;
	response[idx++] = numRegs * 2; // byte count

	for (uint16_t i = 0; i < numRegs; i++) {
		uint16_t val = Get_Register_Value(startAddr + i);
		response[idx++] = val >> 8;
		response[idx++] = val & 0xFF;
	}

	uint16_t crc = ModbusCRC16(response, idx);
	response[idx++] = crc >> 8;
	response[idx++] = crc & 0xFF;

//    // Debug
//    sprintf(buffer, "TX Read Frame: ");
//    for (uint8_t j = 0; j < idx; j++)
//        sprintf(buffer + strlen(buffer), "%02X ", response[j]);
//    sprintf(buffer + strlen(buffer), "\r\n");
//    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

	// Send Frame
	FA_ENABLE_TX(1);
	HAL_UART_Transmit(&huart1, response, idx, HAL_MAX_DELAY);
	HAL_Delay(2);
	FA_ENABLE_TX(0);
}

// ===================== Write Multiple Registers (0x10) =====================

void Handle_Write_Multiple_Registers(uint16_t startAddr, uint16_t numRegs,
		uint8_t *data) {
	uint8_t valueIndex = 0;
	uint8_t isEpochWrite = 0;
	for (uint16_t i = 0; i < numRegs; i++) {
		uint16_t val = (data[valueIndex] << 8) | data[valueIndex + 1];
		valueIndex += 2;

		switch (startAddr + i) {
		case 0x07CF:
			W_Ecsetpoint = val;
			EC_Setpoint = (float) W_Ecsetpoint / 100;
			break;
		case 0x07D0:
			Ecdose_sec = val;
			break;
		case 0x07D1:
			Ecdose_cycle = val;
			break;

		case 0x07D2:
			W_pH_min = val;
			pH_min = (float) W_pH_min / 100;
			break;
		case 0x07D3:
			W_pH_max = val;
			pH_max = (float) W_pH_max / 100;
			break;

		case 0x07D4:
			pHdose_sec = val;
			break;
		case 0x07D5:
			pHdose_cycle = val;
			break;
		case 0x07D6:
			PumpON_setpoint = val;
			break;
		case 0x07D7:
			PumpOff_setpoint = val;
			break;

		case 0x07D8:
			W_Ec_pumpsetpoint = val;
			Ec_PumpEr_setpoint = (float) W_Ec_pumpsetpoint / 100;
			break;
		case 0x07D9:
			W_pH_pumpsetpoint = val;
			pH_PumpEr_setpoint = (float) W_pH_pumpsetpoint / 100;
			break;
		case 0x07DA:
			LUX_MINsetpoint = val;
			break;

		case 0x07DB:
			LUX_MAXsetpoint = val;
			break;
		case 0x07DC:
			HUM_FOG_MINsetpoint = val;
			break;
		case 0x07DD:
			HUM_FOG_MAXsetpoint = val;
			break;
		case 0x07DE:
			HUM_FOG_WaitTime = val;
			break;
		case 0x07DF:
			Total_no_mois = val;

			break;
		case 0x07E0:
			circulationfan_setpoint = (val * 10);
			TP4_cirulation_Fan(circulationfan_setpoint);
			break;
		case 0x07E1:
			pump_auto_Man = val;

			break;
		case 0x07E2:
			nutrient_ON_sec = val;

			break;
		case 0x07E3:
			nutrient_setpoint = val;

			break;
		case 0x0800: {
			if ((i + 1) < numRegs) {
				// Read exactly 4 bytes in big-endian (Modbus standard)
				uint32_t epoch = ((uint32_t) data[valueIndex - 2] << 24)
						| ((uint32_t) data[valueIndex - 1] << 16)
						| ((uint32_t) data[valueIndex] << 8)
						| ((uint32_t) data[valueIndex + 1]);

//		        sprintf(buf, "Parsed Epoch: %lu (0x%08lX)\r\n", epoch, epoch);
//		        HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

				Set_RTC_From_Epoch(epoch);

				isEpochWrite = 1;
				i++;
				valueIndex += 2;
			}
			break;
		}

		case 0x0802 ... 0x081a: {
			uint16_t offset = (startAddr + i) - SCHEDULE_START_ADDR;
			uint8_t sch_idx = offset / REG_PER_SCHEDULE;   // 0..5
			uint8_t field = offset % REG_PER_SCHEDULE;   // 0..4

			if (sch_idx < MAX_SCHEDULES) {
				switch (field) {
				case 0:
					schedules[sch_idx].on_hr = val;
					break;
				case 1:
					schedules[sch_idx].on_min = val;
					break;
				case 2:
					schedules[sch_idx].off_hr = val;
					break;
				case 3:
					schedules[sch_idx].off_min = val;
					break;
				case 4:
					schedules[sch_idx].enable = val;
					break;
				}
			}
			break;
		}
		default:
			break;
		}
	}

	if (!isEpochWrite) {
		write_EEPROM();
		read_EEPROM();
		backup_write_EEPROM();
		backup_read_EEPROM();
	}
	// ACK response
	uint8_t response[8];
	uint8_t idx = 0;
	response[idx++] = SLAVE_ID;
	response[idx++] = 0x10;
	response[idx++] = startAddr >> 8;
	response[idx++] = startAddr & 0xFF;
	response[idx++] = numRegs >> 8;
	response[idx++] = numRegs & 0xFF;

	uint16_t crc = ModbusCRC16(response, idx);
	response[idx++] = crc >> 8;
	response[idx++] = crc & 0xFF;

	// Debug
//	sprintf(buf, "TX1 Write Ack: ");
//	for (uint8_t j = 0; j < idx; j++)
//		sprintf(buf + strlen(buf), "%02X ", response[j]);
//	sprintf(buf + strlen(buf), "\r\n");
//	HAL_UART_Transmit(&huart2, (uint8_t*) buf, strlen(buf),
//			HAL_MAX_DELAY);

	// Send ACK Frame
	FA_ENABLE_TX(1);
	HAL_UART_Transmit(&huart1, response, idx, HAL_MAX_DELAY);

	HAL_Delay(2);
	FA_ENABLE_TX(0);
}

void Handle_Write_Single_Register(uint16_t regAddr, uint16_t value) {
	// Handle your specific register (0x2000)
	if (regAddr == 0x2000) {
		if (value == 0) {
			light(0);
			light_status = 0;
		} else if (value == 1) {
			light(1);
			light_status = 1;
		}
	}

	if (regAddr == 0x2001) {
		EcA_status = 0;
		EcB_status = 0;
		pHUp_status = 0;
		pHDown_status = 0;

	}

	// Prepare response (Echo request)
	uint8_t response[8];
	uint8_t idx = 0;

	response[idx++] = SLAVE_ID;
	response[idx++] = 0x06;
	response[idx++] = regAddr >> 8;
	response[idx++] = regAddr & 0xFF;
	response[idx++] = value >> 8;
	response[idx++] = value & 0xFF;

	uint16_t crc = ModbusCRC16(response, idx);
	response[idx++] = crc >> 8;
	response[idx++] = crc & 0xFF;

	// Send back ACK frame
	FA_ENABLE_TX(1);
	HAL_UART_Transmit(&huart1, response, idx, HAL_MAX_DELAY);
	HAL_Delay(2);
	FA_ENABLE_TX(0);

}

void Process_Slave_Request(void) {
	if (!FrameReceived)
		return;
//        sprintf(buf, "RX Frame: ");
//        for (uint16_t i = 0; i < Modbus_Buffer_Count1; i++) {
//            sprintf(buf + strlen(buf), "%02X ", Modbus_Slave_Buffer[i]);
//        }
//        sprintf(buf + strlen(buf), "\r\n");
//        HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

	if (!Check_CRC_FA(Modbus_Slave_Buffer, Modbus_Buffer_Count1)) {
//        sprintf(buf, "CRC error\r\n");
//        HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
		Modbus_Buffer_Count1 = 0;
		FrameReceived = 0;
		return;
	}

	uint8_t slave = Modbus_Slave_Buffer[0];
	uint8_t func = Modbus_Slave_Buffer[1];
	uint16_t startAddr = (Modbus_Slave_Buffer[2] << 8) | Modbus_Slave_Buffer[3];
	uint16_t numRegs = (Modbus_Slave_Buffer[4] << 8) | Modbus_Slave_Buffer[5];

	if (slave != SLAVE_ID) {
		Modbus_Buffer_Count1 = 0;
		FrameReceived = 0;
		return;
	}

	if (func == 0x03) {
		Handle_Read_Registers(startAddr, numRegs);
	} else if (func == 0x10) {
		Handle_Write_Multiple_Registers(startAddr, numRegs,
				&Modbus_Slave_Buffer[7]);
	} else if (func == 0x06) {
		uint16_t value = (Modbus_Slave_Buffer[4] << 8) | Modbus_Slave_Buffer[5];
		Handle_Write_Single_Register(startAddr, value);
	}

	// Reset frame
	Modbus_Buffer_Count1 = 0;
	FrameReceived = 0;
}
void Process_UART2_Command(void)
{
    if (!uart2_frame_ready)
        return;

    uart2_frame_ready = 0;

    char frame[UART2_MAX_FRAME];
    strncpy(frame, (char *)uart2_rx_buf, UART2_MAX_FRAME - 1);
    frame[UART2_MAX_FRAME - 1] = '\0';

    memset(uart2_rx_buf, 0, sizeof(uart2_rx_buf));
    uart2_rx_len = 0;

    /* Remove newline */
    char *nl = strchr(frame, '\n');
    if (nl) *nl = '\0';

    /* Find CRC */
    char *last_comma = strrchr(frame, ',');
    if (!last_comma)
        return;

    uint16_t rx_crc = (uint16_t)strtol(last_comma + 1, NULL, 16);

    uint16_t calc_crc = ModbusCRC16(
        (uint8_t *)frame,
        (uint16_t)(last_comma - frame)
    );

    sprintf(buf, "Rx CRC: %04X, Calc CRC: %04X\r\n", rx_crc, calc_crc);
    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);

    if (rx_crc != calc_crc)
        return;

    *last_comma = '\0';

    /* Parse CMD safely */
    uint8_t cmd[32] = {0};
    uint8_t cmd_count = 0;

    char temp[UART2_MAX_FRAME];
    strcpy(temp, frame);

    char *token = strtok(temp, ",");
    while (token && cmd_count < 32)
    {
        cmd[cmd_count++] = atoi(token);
        token = strtok(NULL, ",");
    }

    /* Dispatch */
    if (cmd_count == 2 && cmd[0] == 0 && cmd[1] == 1)
        Send_Sensor_Data();

    else if (cmd_count == 2 && cmd[0] == 0 && cmd[1] == 2)
    	Send_SoilMoisture();


    else if (cmd_count == 2 && cmd[0] == 0 && cmd[1] == 3)
        Send_Error_Data();

    else if (cmd_count == 2 && cmd[0] == 0 && cmd[1] == 4)
    	 Send_Status_Data();

    else if (cmd[0] == 0 && cmd[1] == 7 && cmd_count > 2){
    	sprintf(buf, "Setpoint frame received\r\n");
               			  HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
        Handle_Setpoint_Frame(frame);
    }

    else if (cmd_count == (2 + (MAX_SCHEDULES * 5)) &&
             cmd[0] == 0 && cmd[1] == 6)
    {
        uint8_t idx = 2;

        for (uint8_t i = 0; i < MAX_SCHEDULES; i++)
        {
            schedules[i].on_hr   = cmd[idx++];
            schedules[i].on_min  = cmd[idx++];
            schedules[i].off_hr  = cmd[idx++];
            schedules[i].off_min = cmd[idx++];
            schedules[i].enable  = cmd[idx++];
        }

        char ack[64];
        int len = snprintf(ack, sizeof(ack), "0,7,1");
        uint16_t crc = ModbusCRC16((uint8_t*)ack, len);
        len += snprintf(ack + len, sizeof(ack) - len,
                        ",%04X\r\n", crc);

        HAL_UART_Transmit(&huart2,
                          (uint8_t*)ack,
                          len, HAL_MAX_DELAY);
    }
}









void Send_Sensor_Data(){


	char payload[256];
	int len = snprintf(payload, sizeof(payload),
	        "%d,%.1f,%.1f,%.1f,%.1f,%.1f,%.2f,%.2f,%.1f,%.1f,%.1f,%.1f,"
	        "%d,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%.1f",1,
	        20.5,   // Wtemp
	        30.2,   // Temp_value
	        65.8,   // hum_Value
	        120.0,  // Lux
	        28.6,   // TP4_TEMP_Value
	        6.20,   // pH
	        1.85,   // EC
	        0.5,    // Ec_A
	        0.6,    // Ec_B
	        0.0,    // pH_Up
	        0.0,    // pH_Down
	        1,      // W_lvlstatus
	        100UL,  // pump_on_Cnt
	        5UL,    // EcA_fill_count
	        4UL,    // EcB_fill_count
	        2UL,    // pHUP_fill_count
	        1UL,    // pHDN_fill_count
	        3UL,    // water_fill_count
	        15UL,   // ECA_ON_count
	        8UL,    // PHD_ON_count
	        9UL,    // PHUP_ON_count
	        12.5    // pump_on_hours
	    );



	    uint16_t crc = ModbusCRC16((uint8_t*)payload, len);

	    char tx[256];
	    int tx_len = snprintf(tx, sizeof(tx), "%s,%04X\n", payload, crc);

	    HAL_UART_Transmit(&huart2, (uint8_t*)tx, tx_len, HAL_MAX_DELAY);

}


void Send_Status_Data(){

	 char payload[128];

	    int len = snprintf(payload, sizeof(payload),
	        "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",4,
	        valvestatus,
	        waterpump_status,
	        EcA_status,
	        EcB_status,
	        pHUp_status,
	        pHDown_status,
	        FAN_Status,
	        Curtain_Status,
	        Fogger_Status,
	        light_status,
	        macro_status,
	        micro_status
	    );


	    uint16_t crc = ModbusCRC16((uint8_t*)payload, len);

	    	    char tx[140];
	    	    int tx_len = snprintf(tx, sizeof(tx), "%s,%04X\n", payload, crc);

	    	    HAL_UART_Transmit(&huart2, (uint8_t*)tx, tx_len, HAL_MAX_DELAY);



}

void Send_Error_Data(){

	char payload[128];

	    int len = snprintf(payload, sizeof(payload),
	        "%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",3,
	        Wtemp_Error,
	        Envir_HT_Error,
	        Lux_Error,
	        TP4_ERROR,
	        Ec_Pump_Error,
	        pHd_Pump_Error,
	        pHup_Pump_Error,
	        pH_Error,
	        EC_Error,
	        soil_errorSensors,
	        TP4_Temp_error
	    );

	    uint16_t crc = ModbusCRC16((uint8_t*)payload, len);

	   	    	    char tx[140];
	   	    	    int tx_len = snprintf(tx, sizeof(tx), "%s,%04X\n", payload, crc);

	   	    	    HAL_UART_Transmit(&huart2, (uint8_t*)tx, tx_len, HAL_MAX_DELAY);

}



void Send_SoilMoisture(void)
{
    char payload[256];
    int len = 0;

//    /* Safety check */
//    if (Total_no_mois == 0 || Total_no_mois > 12)
//        return;

    /* Add sensor count */
    len += snprintf(payload + len,
                    sizeof(payload) - len,
                    "%d,%lu",2,
                    Total_no_mois);

    /* Add moisture values */
    for (uint32_t i = 0; i < Total_no_mois; i++)
    {
        len += snprintf(payload + len,
                        sizeof(payload) - len,
                        ",%u",
                        soilmoisuture[i]);
    }

//    if (len <= 0 || len >= sizeof(payload))
//        return;

    /* Calculate CRC */
    uint16_t crc = ModbusCRC16((uint8_t *)payload, (uint16_t)len);

    /* Build TX frame */
    char tx[280];
    int tx_len = snprintf(tx, sizeof(tx),
                          "%s,%04X\n",
                          payload,
                          crc);

//    if (tx_len <= 0 || tx_len >= sizeof(tx))
//        return;

    HAL_UART_Transmit(&huart2,
                      (uint8_t *)tx,
                      tx_len,HAL_MAX_DELAY);
}


void Handle_Setpoint_Frame(char *frame)
{
    char *token;

    /* Skip CMD bytes (0,7) */
    token = strtok(frame, ","); // 0
    token = strtok(NULL, ",");  // 7

    /* Extract values in exact order */

    EC_Setpoint = atof(strtok(NULL, ","));
    Ecdose_sec = atol(strtok(NULL, ","));
    Ecdose_cycle = atol(strtok(NULL, ","));

    pH_min = atof(strtok(NULL, ","));
    pH_max = atof(strtok(NULL, ","));

    pHdose_sec = atol(strtok(NULL, ","));
    pHdose_cycle = atol(strtok(NULL, ","));

    PumpON_setpoint = atol(strtok(NULL, ","));
    PumpOff_setpoint = atol(strtok(NULL, ","));

    Ec_PumpEr_setpoint = atof(strtok(NULL, ","));
    pH_PumpEr_setpoint = atof(strtok(NULL, ","));

    LUX_MINsetpoint = atol(strtok(NULL, ","));
    LUX_MAXsetpoint = atol(strtok(NULL, ","));

    HUM_FOG_MINsetpoint = atol(strtok(NULL, ","));
    HUM_FOG_MAXsetpoint = atol(strtok(NULL, ","));

    HUM_FOG_WaitTime = atol(strtok(NULL, ","));

    Total_no_mois = atol(strtok(NULL, ","));
    circulationfan_setpoint = atol(strtok(NULL, ","));

    pump_auto_Man = atol(strtok(NULL, ","));

    nutrient_ON_sec = atol(strtok(NULL, ","));
    nutrient_setpoint = atol(strtok(NULL, ","));

    /* Send response with all received values: 7,EC,Ecdose_sec,Ecdose_cycle,...,nutrient_setpoint,CRC */
    char payload[256];
    int len = snprintf(payload, sizeof(payload),
        "7,%.2f,%lu,%lu,%.2f,%.2f,%lu,%lu,%lu,%lu,%.2f,%.2f,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu,%lu",
        EC_Setpoint, Ecdose_sec, Ecdose_cycle,
        pH_min, pH_max,
        pHdose_sec, pHdose_cycle,
        PumpON_setpoint, PumpOff_setpoint,
        Ec_PumpEr_setpoint, pH_PumpEr_setpoint,
        LUX_MINsetpoint, LUX_MAXsetpoint,
        HUM_FOG_MINsetpoint, HUM_FOG_MAXsetpoint, HUM_FOG_WaitTime,
        Total_no_mois, circulationfan_setpoint,
        pump_auto_Man, nutrient_ON_sec, nutrient_setpoint);

    uint16_t crc = ModbusCRC16((uint8_t*)payload, len);

    char tx[300];
    int tx_len = snprintf(tx, sizeof(tx), "%s,%04X\n", payload, crc);

    HAL_UART_Transmit(&huart2, (uint8_t*)tx, tx_len, HAL_MAX_DELAY);
}











void Print_All_Variables(void) {
	/* // char buf[300];

	 // ---------- 1️⃣ Sensor Values ----------
	 //    sprintf(buf,
	 //        "\r\n---- Sensor Values1 ----\r\n"
	 //        "Water Temp: %.1f C\r\n"
	 //        "Air Temp: %.1f C\r\n"
	 //        "Humidity: %.1f %%\r\n"
	 //        "Light: %.1f Lux\r\n"
	 //        "TP4 Temp: %u\r\n"
	 //        "pH: %.2f\r\n"
	 //        "EC: %.2f\r\n"
	 //        "EC A: %u\r\n"
	 //        "EC B: %u\r\n"
	 //        "pH Up: %u\r\n"
	 //        "pH Down: %u\r\n"
	 //        "Water Level Status: %u\r\n",
	 //        Wtemp, Temp_value, hum_Value, Lux, TP4_TEMP_Value,
	 //        pH, Ec, Ec_A, Ec_B, pH_Up, pH_Down, W_lvlstatus
	 //    );
	 //    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 200);
	 //
	 //    // ---------- 2️⃣ Error Flags ----------
	 //    sprintf(buf,
	 //        "\r\n---- Error Flags ----\r\n"
	 //        "Water Temp Error: %u\r\n"
	 //        "Env Temp/Humidity Error: %u\r\n"
	 //        "Light Error: %u\r\n"
	 //        "TP4 Error: %u\r\n"
	 //        "EC Pump Error: %u\r\n"
	 //        "pH Down Pump Error: %u\r\n"
	 //        "pH Up Pump Error: %u\r\n"
	 //        "pH Sensor Error: %u\r\n"
	 //        "EC Sensor Error: %u\r\n"
	 //        "Soil Error Sensors: %u\r\n",
	 //        Wtemp_Error, Envir_HT_Error, Lux_Error, TP4_ERROR,
	 //        Ec_Pump_Error, pHd_Pump_Error, pHup_Pump_Error,
	 //        pH_Error, EC_Error, soil_errorSensors
	 //    );
	 //    HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 200);

	 // ---------- 3️⃣ Status Flags ----------
	 sprintf(buf, "\r\n---- Status Flags ----\r\n"
	 //			"Valve: %u\r\n"
	 //			"Water Pump: %u\r\n"
	 //			"EC A Pump: %u\r\n"
	 //			"EC B Pump: %u\r\n"
	 //			"pH Up Pump: %u\r\n"
	 //			"pH Down Pump: %u\r\n"
	 //			"Fan: %u\r\n"
	 //			"Curtain: %u\r\n"
	 //			"Fogger: %u\r\n"
	 //			"Light: %u\r\n"
	 //			"macro_status: %u\r\n"
	 //			"micro_status: %u\r\n"

	 "ECA_ON_count: %u\r\n"
	 "PHD_ON_count: %u\r\n"
	 "PHUP_ON_count: %u\r\n"
	 "pump_on_hours: %u\r\n"
	 "pump_on_Cnt: %u\r\n"
	 "EcA_fill_count: %u\r\n"
	 "EcB_fill_count: %u\r\n"
	 "pHDN_fill_count: %u\r\n"
	 "pHUP_fill_count: %u\r\n"
	 "water_fill_count: %u\r\n"




	 "------------------------\r\n\r\n",
	 ECA_ON_count,PHD_ON_count,PHUP_ON_count,pump_on_hours,pump_on_Cnt,EcA_fill_count,EcB_fill_count,pHUP_fill_count,water_fill_count  );
	 HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), 200);*/

	 RTC_TimeTypeDef sTime;
	 RTC_DateTypeDef sDate;
	 HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	 HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	 // Print over UART
	 sprintf(buf, "RTC Time: %02d:%02d:%02d  Date: %02d-%02d-20%02d\r\n",
	 sTime.Hours, sTime.Minutes, sTime.Seconds,
	 sDate.Date, sDate.Month, sDate.Year);

	 HAL_UART_Transmit(&huart2, (uint8_t*)buf, strlen(buf), HAL_MAX_DELAY);
}

