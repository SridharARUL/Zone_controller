################################################################################
# Automatically-generated file. Do not edit!
# Toolchain: GNU Tools for STM32 (13.3.rel1)
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Core/Src/Digital_input.c \
../Core/Src/EC_Sensor.c \
../Core/Src/EEPROM.c \
../Core/Src/Error_Handling.c \
../Core/Src/FA_RS485.c \
../Core/Src/HTU21D.c \
../Core/Src/PH_Sensor.c \
../Core/Src/RelayInit.c \
../Core/Src/Rtcupdate.c \
../Core/Src/Soil_moisture.c \
../Core/Src/VEML7700.c \
../Core/Src/WaterTempSensor.c \
../Core/Src/callibration.c \
../Core/Src/main.c \
../Core/Src/stm32g0xx_hal_msp.c \
../Core/Src/stm32g0xx_it.c \
../Core/Src/sys_control.c \
../Core/Src/syscalls.c \
../Core/Src/sysmem.c \
../Core/Src/system_stm32g0xx.c 

OBJS += \
./Core/Src/Digital_input.o \
./Core/Src/EC_Sensor.o \
./Core/Src/EEPROM.o \
./Core/Src/Error_Handling.o \
./Core/Src/FA_RS485.o \
./Core/Src/HTU21D.o \
./Core/Src/PH_Sensor.o \
./Core/Src/RelayInit.o \
./Core/Src/Rtcupdate.o \
./Core/Src/Soil_moisture.o \
./Core/Src/VEML7700.o \
./Core/Src/WaterTempSensor.o \
./Core/Src/callibration.o \
./Core/Src/main.o \
./Core/Src/stm32g0xx_hal_msp.o \
./Core/Src/stm32g0xx_it.o \
./Core/Src/sys_control.o \
./Core/Src/syscalls.o \
./Core/Src/sysmem.o \
./Core/Src/system_stm32g0xx.o 

C_DEPS += \
./Core/Src/Digital_input.d \
./Core/Src/EC_Sensor.d \
./Core/Src/EEPROM.d \
./Core/Src/Error_Handling.d \
./Core/Src/FA_RS485.d \
./Core/Src/HTU21D.d \
./Core/Src/PH_Sensor.d \
./Core/Src/RelayInit.d \
./Core/Src/Rtcupdate.d \
./Core/Src/Soil_moisture.d \
./Core/Src/VEML7700.d \
./Core/Src/WaterTempSensor.d \
./Core/Src/callibration.d \
./Core/Src/main.d \
./Core/Src/stm32g0xx_hal_msp.d \
./Core/Src/stm32g0xx_it.d \
./Core/Src/sys_control.d \
./Core/Src/syscalls.d \
./Core/Src/sysmem.d \
./Core/Src/system_stm32g0xx.d 


# Each subdirectory must supply rules for building sources it contributes
Core/Src/%.o Core/Src/%.su Core/Src/%.cyclo: ../Core/Src/%.c Core/Src/subdir.mk
	$(error unable to generate command line)

clean: clean-Core-2f-Src

clean-Core-2f-Src:
	-$(RM) ./Core/Src/Digital_input.cyclo ./Core/Src/Digital_input.d ./Core/Src/Digital_input.o ./Core/Src/Digital_input.su ./Core/Src/EC_Sensor.cyclo ./Core/Src/EC_Sensor.d ./Core/Src/EC_Sensor.o ./Core/Src/EC_Sensor.su ./Core/Src/EEPROM.cyclo ./Core/Src/EEPROM.d ./Core/Src/EEPROM.o ./Core/Src/EEPROM.su ./Core/Src/Error_Handling.cyclo ./Core/Src/Error_Handling.d ./Core/Src/Error_Handling.o ./Core/Src/Error_Handling.su ./Core/Src/FA_RS485.cyclo ./Core/Src/FA_RS485.d ./Core/Src/FA_RS485.o ./Core/Src/FA_RS485.su ./Core/Src/HTU21D.cyclo ./Core/Src/HTU21D.d ./Core/Src/HTU21D.o ./Core/Src/HTU21D.su ./Core/Src/PH_Sensor.cyclo ./Core/Src/PH_Sensor.d ./Core/Src/PH_Sensor.o ./Core/Src/PH_Sensor.su ./Core/Src/RelayInit.cyclo ./Core/Src/RelayInit.d ./Core/Src/RelayInit.o ./Core/Src/RelayInit.su ./Core/Src/Rtcupdate.cyclo ./Core/Src/Rtcupdate.d ./Core/Src/Rtcupdate.o ./Core/Src/Rtcupdate.su ./Core/Src/Soil_moisture.cyclo ./Core/Src/Soil_moisture.d ./Core/Src/Soil_moisture.o ./Core/Src/Soil_moisture.su ./Core/Src/VEML7700.cyclo ./Core/Src/VEML7700.d ./Core/Src/VEML7700.o ./Core/Src/VEML7700.su ./Core/Src/WaterTempSensor.cyclo ./Core/Src/WaterTempSensor.d ./Core/Src/WaterTempSensor.o ./Core/Src/WaterTempSensor.su ./Core/Src/callibration.cyclo ./Core/Src/callibration.d ./Core/Src/callibration.o ./Core/Src/callibration.su ./Core/Src/main.cyclo ./Core/Src/main.d ./Core/Src/main.o ./Core/Src/main.su ./Core/Src/stm32g0xx_hal_msp.cyclo ./Core/Src/stm32g0xx_hal_msp.d ./Core/Src/stm32g0xx_hal_msp.o ./Core/Src/stm32g0xx_hal_msp.su ./Core/Src/stm32g0xx_it.cyclo ./Core/Src/stm32g0xx_it.d ./Core/Src/stm32g0xx_it.o ./Core/Src/stm32g0xx_it.su ./Core/Src/sys_control.cyclo ./Core/Src/sys_control.d ./Core/Src/sys_control.o ./Core/Src/sys_control.su ./Core/Src/syscalls.cyclo ./Core/Src/syscalls.d ./Core/Src/syscalls.o ./Core/Src/syscalls.su ./Core/Src/sysmem.cyclo ./Core/Src/sysmem.d ./Core/Src/sysmem.o ./Core/Src/sysmem.su ./Core/Src/system_stm32g0xx.cyclo ./Core/Src/system_stm32g0xx.d ./Core/Src/system_stm32g0xx.o ./Core/Src/system_stm32g0xx.su

.PHONY: clean-Core-2f-Src

