/*
 * Soil_moisture.c
 *
 *  Created on: Sep 2, 2025
 *      Author: Sridhar A
 */
#include "stdio.h"
#include "stdlib.h"
#include <stdint.h>
#include "main.h"
#include "soil_moisture.h"
#include "RelayInit.h"
#include <string.h>

#define MODBUS_BUFFER_SIZE             100


extern UART_HandleTypeDef huart3;

extern UART_HandleTypeDef huart4;
int16_t raw_value,fan1,fan2,fan3,fan4;

uint8_t Modbus_Buffer_Count = 0;
uint8_t Modbus_Response_Buffer[MODBUS_BUFFER_SIZE];
uint8_t Modbus_Request_Buffer[MODBUS_BUFFER_SIZE];

uint16_t ModbusCRC16(uint8_t *modbusframe, uint8_t Length) {
	uint16_t crc_register = 0xFFFF, crc_temp;
	uint8_t ival = 0, right_shift_count = 0;

	while (ival < Length) {
		crc_register = (crc_register ^ modbusframe[ival]);
		do {
			crc_temp = crc_register;
			crc_register = crc_register >> 1;

			if ((crc_temp & 0x0001) == 0x0001) {
				crc_register = (crc_register ^ 0xA001);
			}
			right_shift_count++;
		} while (right_shift_count < 8);

		right_shift_count = 0;
		ival++;
	}

	crc_temp = crc_register;
	crc_register = crc_register >> 8;
	crc_temp = crc_temp << 8;
	crc_register = (crc_register | crc_temp);

	return crc_register;
}


uint8_t Check_CRC(void) {
	uint16_t tmp_crc, tmp_received_crc;

	tmp_received_crc = (Modbus_Response_Buffer[Modbus_Buffer_Count - 2] << 8) | Modbus_Response_Buffer[Modbus_Buffer_Count - 1];

	if ((tmp_received_crc == 0xFFFF) || (tmp_received_crc == 0x0000))
		return 0;

	tmp_crc = ModbusCRC16(&Modbus_Response_Buffer[0], (Modbus_Buffer_Count - 2));
	return (tmp_crc == tmp_received_crc) ? 1 : 0;
}



HAL_StatusTypeDef Send_MODBUS_Command(uint8_t sensor_id, uint8_t no_of_bytes,uint8_t flag) {
	uint16_t tmp_crc;
	HAL_StatusTypeDef status;
	 char msg[100];
	tmp_crc = ModbusCRC16(&Modbus_Request_Buffer[0], no_of_bytes);

	Modbus_Request_Buffer[no_of_bytes] = tmp_crc >> 8;
	Modbus_Request_Buffer[no_of_bytes + 1] = tmp_crc;

	if(flag){


		TP4_ENABLE_TX(1);

		status = HAL_UART_Transmit(&huart4, Modbus_Request_Buffer, no_of_bytes + 2, 100);
		 HAL_Delay(2);
		TP4_ENABLE_TX(0);

		HAL_UART_Abort(&huart4);
		return status;
	}
	else{
		RS484_MOIENABLE_TX(1);


			status = HAL_UART_Transmit(&huart3, Modbus_Request_Buffer, no_of_bytes + 2, 100);//+2 crc byte
			 HAL_Delay(2);
			RS484_MOIENABLE_TX(0);

			HAL_UART_Abort(&huart3);
			return status;



	}
}


HAL_StatusTypeDef Request_Consumption(uint8_t sensor_id, uint8_t startadress, uint8_t endadress, uint8_t startlength, uint8_t endlength,uint8_t flag,uint8_t READ_HOLDING_REGISTERS) {
	Modbus_Request_Buffer[0] = sensor_id;
	Modbus_Request_Buffer[1] = READ_HOLDING_REGISTERS;
	Modbus_Request_Buffer[2] = startadress;
	Modbus_Request_Buffer[3] = endadress;
	Modbus_Request_Buffer[4] = startlength;
	Modbus_Request_Buffer[5] = endlength;

	return Send_MODBUS_Command(sensor_id, 6,flag);
}


int16_t Get_Consumption(uint8_t sensor_id, uint8_t startadress, uint8_t endadress, uint8_t startlength, uint8_t endlength, uint8_t CONSUMPTION_RESPONSE_BYTES, uint8_t flag,uint8_t READ_HOLDING_REGISTERS) {

	HAL_StatusTypeDef return_status;

	return_status = Request_Consumption(sensor_id, startadress, endadress, startlength, endlength,flag,READ_HOLDING_REGISTERS);


	if (return_status != HAL_OK){


		return -5;
	}

	 memset(Modbus_Response_Buffer, 0, MODBUS_BUFFER_SIZE);

	Modbus_Buffer_Count = CONSUMPTION_RESPONSE_BYTES;

	if(flag){


		return_status = HAL_UART_Receive(&huart4, Modbus_Response_Buffer, CONSUMPTION_RESPONSE_BYTES, 100);

	}

	else{


	return_status = HAL_UART_Receive(&huart3, Modbus_Response_Buffer, CONSUMPTION_RESPONSE_BYTES, 200);




	}


	if (return_status != HAL_OK){


		return -5;
	}

	if (Check_CRC()) {

		if(flag==2){

			fan1= (Modbus_Response_Buffer[3] << 8) | Modbus_Response_Buffer[4];

			fan2=(Modbus_Response_Buffer[5] << 8) | Modbus_Response_Buffer[6];
			fan3=(Modbus_Response_Buffer[7] << 8) | Modbus_Response_Buffer[8];
			fan4=(Modbus_Response_Buffer[9] << 8) | Modbus_Response_Buffer[10];
			return 1;

		}


        raw_value = (Modbus_Response_Buffer[3] << 8) | Modbus_Response_Buffer[4];
			return raw_value;


	} else {


		return -5;
	}
}

HAL_StatusTypeDef Write_Single_Register(uint8_t sensor_id, uint16_t reg_address, uint16_t value)
{
    uint16_t tmp_crc;
    HAL_StatusTypeDef status;
    uint8_t index = 0;
    uint8_t response[8];

    // Build Write Frame
    Modbus_Request_Buffer[index++] = sensor_id;                  // Slave ID
    Modbus_Request_Buffer[index++] = 0x06;                       // Function code 0x06 = Write Single Register
    Modbus_Request_Buffer[index++] = (reg_address >> 8) & 0xFF;  // Register Hi
    Modbus_Request_Buffer[index++] = reg_address & 0xFF;         // Register Lo
    Modbus_Request_Buffer[index++] = (value >> 8) & 0xFF;        // Value Hi
    Modbus_Request_Buffer[index++] = value & 0xFF;               // Value Lo

    // CRC
    tmp_crc = ModbusCRC16(Modbus_Request_Buffer, index);
    Modbus_Request_Buffer[index++] = tmp_crc >> 8;
    Modbus_Request_Buffer[index++] = tmp_crc & 0xFF;

    // Transmit via UART4
    TP4_ENABLE_TX(1);
    status = HAL_UART_Transmit(&huart4, Modbus_Request_Buffer, index, 100);
    HAL_Delay(2);
    TP4_ENABLE_TX(0);
    HAL_UART_Abort(&huart4);

    if (status != HAL_OK) {
           return status;
       }

       // Receive response (same 8 bytes back)
       status = HAL_UART_Receive(&huart4, response, 8, 100);

       if (status == HAL_OK) {
           // Compare response with request

           if (memcmp(Modbus_Request_Buffer, response, 8) == 0) {
               return HAL_OK;  // Write confirmed
           } else {
               return HAL_ERROR;  // Response mismatch
           }
       }

       return status; // Could be timeout or error




}



