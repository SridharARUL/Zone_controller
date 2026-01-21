/*
 * VEML7700.c
 *
 *  Created on: Sep 10, 2025
 *      Author: Sridhar A
 */


#include "main.h"
#include <stdint.h>

extern I2C_HandleTypeDef hi2c2;

#define VEML7700_DevAdress (0x10 << 1)

// ALS gain constants
#define REG_ALS_CONF_GAIN_1     (0x00 << 11) // x1 (default)
#define REG_ALS_CONF_GAIN_2     (0x01 << 11) // x2
#define REG_ALS_CONF_GAIN_1_8   (0x02 << 11) // x(1/8)
#define REG_ALS_CONF_GAIN_1_4   (0x03 << 11) // x(1/4)

// ALS integration times (ms)
#define REG_ALS_CONF_IT25       (0x0C << 6)
#define REG_ALS_CONF_IT50       (0x08 << 6)
#define REG_ALS_CONF_IT100      (0x00 << 6)
#define REG_ALS_CONF_IT200      (0x01 << 6)
#define REG_ALS_CONF_IT400      (0x02 << 6)
#define REG_ALS_CONF_IT800      (0x03 << 6)

// VEML6030 registers //
#define REG_ALS_CONF            0x00
#define REG_ALS_WH              0x01
#define REG_ALS_WL              0x02
#define REG_POWER_SAVING        0x03
#define REG_ALS                 0x04
#define REG_WHITE               0x05
#define REG_ALS_INT             0x06

// Register 0x0: ALS_CONF //
// ALS integration times - all bits
#define REG_ALS_CONF_IT_CLEAR   (0x0f << 6)

// ALS persistent protect number
#define REG_ALS_CONF_PERS_1     (0x00 << 4)
#define REG_ALS_CONF_PERS_2     (0x01 << 4)
#define REG_ALS_CONF_PERS_4     (0x02 << 4)
#define REG_ALS_CONF_PERS_8     (0x03 << 4)

// ALS interrupt enable
#define REG_ALS_CONF_IT_ENABLE  (0x01 << 1)

// ALS shutdown setting
#define REG_ALS_CONF_SHUTDOWN   0x01

// Register 0x3: POWER SAVING
// Power saving modes
#define REG_POWER_SAVING_PSM_1  (0x00 << 1)
#define REG_POWER_SAVING_PSM_2  (0x01 << 1)
#define REG_POWER_SAVING_PSM_3  (0x02 << 1)
#define REG_POWER_SAVING_PSM_4  (0x03 << 1)

#define REG_POWER_SAVING_ENABLE  0x01

const float LUX_RES_TABLE[6][4] = { { 0.0576f, 0.0288f, 0.4608f, 0.2304f }, {
		0.0288f, 0.0144f, 0.2304f, 0.1152f }, { 0.0144f, 0.0072f, 0.1152f,
		0.0576f }, { 0.0072f, 0.0036f, 0.0576f, 0.0288f }, { 0.1152f, 0.0576f,
		0.9216f, 0.4608f }, { 0.2304f, 0.1152f, 1.8432f, 0.9216f } };


static uint32_t writeRegister(I2C_HandleTypeDef *handlei2c, uint8_t reg,uint16_t value) {
	uint8_t payload[3] = { reg, value & 0xff, value >> 8 };
	return HAL_I2C_Master_Transmit(handlei2c, VEML7700_DevAdress, &payload[0],3, 500);
}


static int16_t readRegister(I2C_HandleTypeDef *handlei2c, uint8_t reg) {
	uint8_t payload[2] = { 0 };
	int err = HAL_I2C_Mem_Read(handlei2c, VEML7700_DevAdress, reg, 1, &payload[0], 2, 500);
	if (err != HAL_OK) {
		return -2;
	}
	return ((payload[1] << 8) | payload[0]);
}


uint32_t VEML7700_Power_On(void) {
	return writeRegister(&hi2c2, REG_ALS_CONF, 0);
}


uint32_t VEML7700_Shutdown(void) {
	// Get current config and set shutdown bit
	uint16_t config = readRegister(&hi2c2, REG_ALS_CONF);
	config |= REG_ALS_CONF_SHUTDOWN;
	return writeRegister(&hi2c2, REG_ALS_CONF, config);
}


uint32_t VEML7700_SetALSIntegrationTime(uint16_t it) {
	uint16_t config = readRegister(&hi2c2, REG_ALS_CONF);
	config &= ~REG_ALS_CONF_IT_CLEAR;
	config |= it;
	return writeRegister(&hi2c2, REG_ALS_CONF, config);
}


uint16_t VEML7700_GetALSIntegrationTime(void) {
	uint16_t config = readRegister(&hi2c2, REG_ALS_CONF);
	return (config & REG_ALS_CONF_IT_CLEAR) >> 6;
}


uint32_t VEML7700_SetALSGain(uint16_t gain) {
	uint16_t config = readRegister(&hi2c2, REG_ALS_CONF);
	// Clear all gain bits
	config &= ~REG_ALS_CONF_GAIN_1_4;
	config |= gain;
	return writeRegister(&hi2c2, REG_ALS_CONF, gain);
}


uint16_t VEML7700_GetALSGain(void) {
	uint16_t config = readRegister(&hi2c2, REG_ALS_CONF);
	return (config & REG_ALS_CONF_GAIN_1_4) >> 11;
}


uint16_t VEML7700_ReadALSData(void) {
	return readRegister(&hi2c2, REG_ALS);
}


uint16_t VEML7700_ReadWhiteData(void) {
	return readRegister(&hi2c2, REG_WHITE);
}


float VEML7700_Convert2Lx(uint16_t rawalsdata, uint8_t alsgain, uint8_t alsIT) {
	return (LUX_RES_TABLE[alsIT][alsgain] * rawalsdata);
}

HAL_StatusTypeDef PowerOnVEML7700 (void)
{
	HAL_StatusTypeDef return_status;
	return_status = VEML7700_Power_On();
	HAL_Delay(10);
	return return_status;
}

float VEML7700_GetLx(void) {
	int16_t tmpals = VEML7700_ReadALSData();
	if(tmpals == -2){
		return -2;
	}
	return (VEML7700_Convert2Lx(tmpals, 0, 0));
}

