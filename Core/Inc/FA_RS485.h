/*
 * FA_RS485.h
 *
 *  Created on: Sep 26, 2025
 *      Author: Sridhar A
 */

#ifndef INC_FA_RS485_H_
#define INC_FA_RS485_H_

uint16_t Get_Register_Value(uint16_t addr);
uint8_t Check_CRC_FA(uint8_t *frame, uint16_t len);
void Modbus_StartReception(void);
void Process_Slave_Request(void);
void Handle_Read_Registers(uint16_t startAddr, uint16_t numRegs);
void Handle_Write_Multiple_Registers(uint16_t startAddr, uint16_t numRegs, uint8_t *data);
void Print_All_Variables(void);
void Process_UART2_Command(void);
void Send_Sensor_Data(void );

void Send_Status_Data(void);
void Handle_Setpoint_Frame(char *frame);

#define MAX_SCHEDULES        5
typedef struct {
    uint32_t on_hr;
    uint32_t on_min;
    uint32_t off_hr;
    uint32_t off_min;
    uint32_t enable;
} Schedule_t;

extern Schedule_t schedules[MAX_SCHEDULES];
#endif /* INC_FA_RS485_H_ */
