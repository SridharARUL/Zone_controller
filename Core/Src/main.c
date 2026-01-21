/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @Author         : Sridhar
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "WaterTempSensor.h"
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include "pH_Sensor.h"
#include "EC_Sensor.h"
#include "soil_moisture.h"
#include "EEPROM.h"
#include"Digital_input.h"
#include"sys_control.h"
#include <stdbool.h>
#include"VEML7700.h"
#include "HTU21D.h"
#include "RelayInit.h"
#include "Error_Handling.h"
#include "FA_RS485.h"
#include "callibration.h"
#include "Rtcupdate.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define UFS_ADDRESS	  1
#define READ_HOLDING_REGISTERS 0x03
#define humidity_start_ADDRESS	0x00
#define humidity_end_ADDRESS	0x00
#define humidity_lengthS_ADDRESS	0x00
#define humidity_lengthE_ADDRESS	0x01
#define condumptionbyte 7
#define ERROR_THRESHOLD  20
#define TP4_ADDRESS	  1
#define TP4READ_INPUT_REGISTERS 0x04
#define TP4_start_ADDRESS	0x00
#define TP4_end_ADDRESS	0x00
#define TP4_Status_ADDRESS 0x0e
#define TP4_lengthS_ADDRESS	0x00
#define TP4_lengthE_ADDRESS	0x01
#define TP4_condumptionbyte 7


#define TP4_ADDRESS	  1
#define TP4READ_HOLDING_REGISTERS 0x03
#define TP4_SET_start_ADDRESS	0x00
#define TP4_SET_end_ADDRESS	0x05
#define TP4_SET_lengthS_ADDRESS	0x00
#define TP4_SET_lengthE_ADDRESS	0x04
#define TP4_SET_condumptionbyte 13




float Wtemp=0,pH,Ec,Lux,hum_Value,Temp_value;
char buffer[200];// String buffer

int16_t soilmoisuture[15],TP4_TEMP_Value,TP4_Status;
int16_t last_valid_value[15];
uint8_t error_count[15];
int8_t TP4_Set,TP4_Temp_error;
volatile uint8_t PH_EC_cnt=0,sensorRcnt=0,fogger_delay_active;
uint8_t W_lvl1 =0,W_lvl2=0,Ec_A,Ec_B,pH_Up,pH_Down,W_lvlstatus,TP4_WStatus,TP4_ERROR,Lux_Error,Envir_HT_Error;
volatile uint16_t Dosestart_cnt1=0,Ec_PumpON_cnt=0,pH_pumpON_cnt=0,Ecmixcnt=0,pHdmixcnt,pHupmixcnt,output_timer_count=0,count_update;
volatile uint32_t Eccycle_cnt=0,pHcycle_cnt=0,fogger_delay_counter,pump_on_seconds,pump_on_hours;
bool Dosestartflag=true,Sen_read3vflag=true,Sen_read5vflag=false,ECflag=false;
extern bool ECDose_Activeflag,ECcycle_Tflag,pH_pump_Activeflag,pHcycle_Tflag,ECmixflag,pHdmixflag,pHupmixflag,pH_UPpump_Activeflag;
extern uint8_t Tp4_Fan_WStatus,output_triggered,macro_status,micro_status,waterpump_status,count_updateflag;
extern uint32_t HUM_FOG_WaitTime,pump_auto_Man;
extern volatile uint8_t FrameReceived, buttonHandled,valvestatus,uart2_frame_ready;
extern uint32_t Total_no_mois,nutrient_ON_sec;
volatile bool soil_readflag =false;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_IWDG_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USART4_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void ThreeV_sensor() {

	if (Sen_read3vflag) {

		sensor_pwr_3v(1);

		if (PH_EC_cnt >= 10) {

			Wtemp = Temperature_Read();
			if(Wtemp==85){
				Wtemp=25;
			}

			//Wtemp_Error=water_Temp_Error(Wtemp);

			Lux = VEML7700_GetLx();
			Lux_Error=Light_Error(Lux);

			hum_Value = gethumidity();
            Temp_value = get_Temp();
            Envir_HT_Error=HT_Error(hum_Value,Temp_value);

          	W_lvl1 = waterlevel1(); //low
			W_lvl2 = waterlevel2(); //High

			W_lvlstatus = waterlevel(W_lvl1, W_lvl2);

			sensor_pwr_3v(0);
			valvecont(W_lvlstatus);
			Sen_read3vflag = false;
			Sen_read5vflag = true;
			PH_EC_cnt = 0;



		}

	}

}
void FiveV_sensor() {

	if (Sen_read5vflag) {

		sensor_pwr_5v(1);

		if (PH_EC_cnt >= 10) {


			TP4_TEMP_Value = Get_Consumption(TP4_ADDRESS, TP4_start_ADDRESS,
			TP4_end_ADDRESS, TP4_lengthS_ADDRESS, TP4_lengthE_ADDRESS,
			TP4_condumptionbyte, 1, TP4READ_INPUT_REGISTERS);


			TP4_Temp_error=	TP4_Terror(TP4_TEMP_Value);

//      		TP4_Set = Get_Consumption(TP4_ADDRESS, TP4_SET_start_ADDRESS,
//					TP4_SET_end_ADDRESS, TP4_SET_lengthS_ADDRESS,
//					TP4_SET_lengthE_ADDRESS, TP4_SET_condumptionbyte, 2,
//					TP4READ_HOLDING_REGISTERS);

			TP4_WStatus = TP4_fogger_O_P(hum_Value,Envir_HT_Error);
			TP4_WStatus = TP4_curtain_O_P(Lux,Lux_Error);

			TP4_Status = Get_Consumption(TP4_ADDRESS, TP4_start_ADDRESS,
						TP4_Status_ADDRESS, TP4_lengthS_ADDRESS, TP4_lengthE_ADDRESS,
						TP4_condumptionbyte, 1, TP4READ_INPUT_REGISTERS); //outputstatus


			TP4_Decode_Outputs(TP4_Status);
			TP4_ERROR = TP4_Modbus_error(TP4_TEMP_Value, TP4_Set, TP4_WStatus,TP4_Status,Tp4_Fan_WStatus);

			Ec_A = Ec_A_Level();
			Ec_B = Ec_B_Level();
			pH_Up = pH_Up_Level();
			pH_Down = pH_Down_Level();
			Sen_read5vflag = false;
		    sensor_pwr_5v(0);
			ECflag = true;
			PH_EC_cnt = 0;

		}

	}

}



void Read_Soil_Moisture_Step(void)
{

	static uint8_t current_sensor = 0;
	 int16_t value;
	 value = Get_Consumption(
        UFS_ADDRESS + current_sensor,       // sensor Modbus ID
        humidity_start_ADDRESS,
        humidity_end_ADDRESS,
        humidity_lengthS_ADDRESS,
        humidity_lengthE_ADDRESS,
        condumptionbyte,
        0,
        READ_HOLDING_REGISTERS);

   //  Print the value on UART2


    /* -------- Error filter -------- */
	 if (value == -5)
	     {
	         error_count[current_sensor]++;

	         if (error_count[current_sensor] >= ERROR_THRESHOLD)
	         {
	             // ❗ Same sensor failed 5 times continuously
	             soilmoisuture[current_sensor] = -5;
	         }
	         else
	         {
	             // Keep last valid value
	             soilmoisuture[current_sensor] = last_valid_value[current_sensor];
	         }
	     }
	     else
	     {
	         // Valid reading
	         soilmoisuture[current_sensor] = value;
	         last_valid_value[current_sensor] = value;
	         error_count[current_sensor] = 0;   // reset only this sensor
	     }
//	    sprintf(buffer, "Soil Moisture[%d]: %d\r\n", current_sensor + 1, soilmoisuture[current_sensor]);
//	    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
//

    // Move to next sensor
    current_sensor++;

    // ✅ If all sensors have been read once — call waterpump()
    if (current_sensor >= Total_no_mois)
    {
        current_sensor = 0; // reset index
        waterpump(soilmoisuture, Total_no_mois);
    }
}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_IWDG_Init();
  MX_USART3_UART_Init();
  MX_USART4_UART_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_RTC_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  PowerOnVEML7700();

  sprintf(buffer, "reset............\r\n");
  HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);


  read_EEPROM();
  backup_write_EEPROM();
  backup_read_EEPROM();
  Modbus_StartReception();
  HAL_TIM_Base_Start(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
  HAL_TIM_Base_Start_IT(&htim6);
  HAL_Delay(1000);
  PH_EC_cnt=0;

   Write_Single_Register(1,0x0008, 900);

   Write_Single_Register(1,0x0006,900);




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {

		HAL_IWDG_Refresh(&hiwdg); // Refresh before 5s expires
		if (!valvestatus) {
			ThreeV_sensor();
			FiveV_sensor();

			Ec = get_EC();
			pH = get_PH();
		}

		if (valvestatus) {

			W_lvl1 = waterlevel1(); //low
			W_lvl2 = waterlevel2(); //High
			W_lvlstatus = waterlevel(W_lvl1, W_lvl2);
			valvecont(W_lvlstatus);
			PH_EC_cnt = 0;
		}

		if (FrameReceived) {
			Process_Slave_Request();
		}

		//if(uart2_frame_ready){
			 Process_UART2_Command();
			//uart2_frame_ready = 0;
//		}

		if (soil_readflag) {
			Read_Soil_Moisture_Step();
			soil_readflag = false;

		}
		nutrientdosage(Ec_A, Ec_B, pH_Down, pH_Up);

		if (!buttonHandled) {
			CheckButtonPress();
		}

		if (AnyLEDActive()) {
			UpdateLEDTimers();
		}
		if (pump_auto_Man) {
			Relay_Schedule_Control();
		}

		micro_Macro();


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 8;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_39CYCLES_5;
  hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_39CYCLES_5;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */
  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
      Error_Handler();
    }

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10B17DB5;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 624;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */
	 __HAL_RCC_PWR_CLK_ENABLE();
	     HAL_PWR_EnableBkUpAccess();
  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  hrtc.Init.OutPutPullUp = RTC_OUTPUT_PULLUP_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 24-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 999-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 31999-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 63999;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART4_UART_Init(void)
{

  /* USER CODE BEGIN USART4_Init 0 */

  /* USER CODE END USART4_Init 0 */

  /* USER CODE BEGIN USART4_Init 1 */

  /* USER CODE END USART4_Init 1 */
  huart4.Instance = USART4;
  huart4.Init.BaudRate = 4800;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART4_Init 2 */

  /* USER CODE END USART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11|Sen_pwr_5V_Pin|Sen_pwr_3v_Pin|valve_2_Pin
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, VALVE_1_Pin|GPIO_PIN_8|GPIO_PIN_12, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, calibration_fail_Pin|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3
                          |GPIO_PIN_4|Wtemp_Output_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, FA_RS485_Pin|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2
                          |T4_EN_Pin|moisture_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC11 Sen_pwr_5V_Pin Sen_pwr_3v_Pin valve_2_Pin
                           PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|Sen_pwr_5V_Pin|Sen_pwr_3v_Pin|valve_2_Pin
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC2 PC3 PC8 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : VALVE_1_Pin PA8 PA12 */
  GPIO_InitStruct.Pin = VALVE_1_Pin|GPIO_PIN_8|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : calibration_fail_Pin PB1 PB2 PB12
                           PB13 PB14 PB15 PB3
                           PB4 Wtemp_Output_Pin */
  GPIO_InitStruct.Pin = calibration_fail_Pin|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_12
                          |GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_3
                          |GPIO_PIN_4|Wtemp_Output_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : FA_RS485_Pin PD0 PD1 PD2
                           T4_EN_Pin moisture_EN_Pin */
  GPIO_InitStruct.Pin = FA_RS485_Pin|GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2
                          |T4_EN_Pin|moisture_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PC10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void MX_GPIO_Init_Input(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin : PB5 */
	GPIO_InitStruct.Pin = Wtemp_Output_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(Wtemp_Output_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

void MX_GPIO_Init_Output(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(Wtemp_Output_GPIO_Port, Wtemp_Output_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : PB12 */
	GPIO_InitStruct.Pin = Wtemp_Output_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	// GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
	HAL_GPIO_Init(Wtemp_Output_GPIO_Port, &GPIO_InitStruct);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

void PH_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};

    hadc1.Instance = ADC1;
       hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
       hadc1.Init.Resolution = ADC_RESOLUTION_12B;
       hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
       hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
       hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
       hadc1.Init.LowPowerAutoWait = DISABLE;
       hadc1.Init.LowPowerAutoPowerOff = DISABLE;
       hadc1.Init.ContinuousConvMode = DISABLE;
       hadc1.Init.NbrOfConversion = 1;
       hadc1.Init.DiscontinuousConvMode = DISABLE;
       hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
       hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
       hadc1.Init.DMAContinuousRequests = DISABLE;
       hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
       hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_39CYCLES_5;
       hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_39CYCLES_5;
       hadc1.Init.OversamplingMode = DISABLE;
       hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
       if (HAL_ADC_Init(&hadc1) != HAL_OK)
       {
         Error_Handler();
       }



    sConfig.Channel      = ADC_CHANNEL_6;    // PA6
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}



void EC_ADC1_Init(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};


    hadc1.Instance = ADC1;
       hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
       hadc1.Init.Resolution = ADC_RESOLUTION_12B;
       hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
       hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
       hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
       hadc1.Init.LowPowerAutoWait = DISABLE;
       hadc1.Init.LowPowerAutoPowerOff = DISABLE;
       hadc1.Init.ContinuousConvMode = DISABLE;
       hadc1.Init.NbrOfConversion = 1;
       hadc1.Init.DiscontinuousConvMode = DISABLE;
       hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
       hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
       hadc1.Init.DMAContinuousRequests = DISABLE;
       hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
       hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_39CYCLES_5;
       hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_39CYCLES_5;
       hadc1.Init.OversamplingMode = DISABLE;
       hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
       if (HAL_ADC_Init(&hadc1) != HAL_OK)
       {
         Error_Handler();
       }

    sConfig.Channel      = ADC_CHANNEL_7;    // PA7
    sConfig.Rank         = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;

    if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
        Error_Handler();
    }
}






void Hum_ADC1_Init(void)
{



	  ADC_ChannelConfTypeDef sConfig = {0};


	  hadc1.Instance = ADC1;
	  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	  hadc1.Init.ScanConvMode = ADC_SCAN_SEQ_FIXED_BACKWARD;
	  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	  hadc1.Init.LowPowerAutoWait = DISABLE;
	  hadc1.Init.LowPowerAutoPowerOff = DISABLE;
	  hadc1.Init.ContinuousConvMode = DISABLE;
	  hadc1.Init.NbrOfConversion = 1;
	  hadc1.Init.DiscontinuousConvMode = DISABLE;
	  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	  hadc1.Init.DMAContinuousRequests = DISABLE;
	  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	  hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_39CYCLES_5;
	  hadc1.Init.OversamplingMode = DISABLE;
	  hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
	  if (HAL_ADC_Init(&hadc1) != HAL_OK)
	  {
	    Error_Handler();
	  }

  sConfig.Channel      = ADC_CHANNEL_18;   // PC5
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }
//  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
//        Error_Handler();
//      }

}

void Temp_ADC1_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
    hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
    hadc1.Init.Resolution = ADC_RESOLUTION_12B;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
    hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
    hadc1.Init.LowPowerAutoWait = DISABLE;
    hadc1.Init.LowPowerAutoPowerOff = DISABLE;
    hadc1.Init.ContinuousConvMode = DISABLE;
    hadc1.Init.NbrOfConversion = 1;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc1.Init.DMAContinuousRequests = DISABLE;
    hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
    hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_39CYCLES_5;
    hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_39CYCLES_5;
    hadc1.Init.OversamplingMode = DISABLE;
    hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
      Error_Handler();
    }



  sConfig.Channel      = ADC_CHANNEL_5;    // PC0
  sConfig.Rank         = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;

  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
    Error_Handler();
  }

//  if (HAL_ADCEx_Calibration_Start(&hadc1) != HAL_OK) {
//         Error_Handler();
//       }

}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM3) {
		// 500 ms interrupt here
		soil_readflag=true;

		PH_EC_cnt++;

		if (Dosestartflag) {
			Dosestart_cnt1++;
		}
		if (ECDose_Activeflag) {
			Ec_PumpON_cnt++;

		}

		if (ECcycle_Tflag) {
			Eccycle_cnt++;
		}

		if(ECmixflag){
			Ecmixcnt++;
		}

		if (pH_pump_Activeflag) {
			pH_pumpON_cnt++;
		}
		if(pH_UPpump_Activeflag){
			pH_pumpON_cnt++;
		}



		if (pHcycle_Tflag) {
			pHcycle_cnt++;

		}

		if(pHdmixflag){
			pHdmixcnt++;
		}

		if(pHupmixflag){
			pHupmixcnt++;
		}

		 if (fogger_delay_active) {
			fogger_delay_counter++;
			if (fogger_delay_counter >= (HUM_FOG_WaitTime*120)) {
				fogger_delay_active = 0;
				fogger_delay_counter = 0;
			}
		}


		 if(output_triggered == 1)
		         {
		             output_timer_count++;    // count every 500ms

		             if(output_timer_count >= nutrient_ON_sec*2)
		             {

		             	macro_nutrient(0);
		             	micro_nutrient(0);
		             	macro_status=0;
		             	 micro_status=0;
		             	output_timer_count=0;
		                output_triggered = 0;  // Ready for next day's trigger


		             }
		         }

	}
	else if(htim->Instance == TIM6){


		 if (waterpump_status == 1)
		        {
		            pump_on_seconds++;

		            if (pump_on_seconds >= 3600)
		            {
		                pump_on_seconds = 0;   // reset seconds
		                pump_on_hours++;       // ✅ 1 hour completed
		            }
		        }

		 if(!count_updateflag){

			 count_update++;

			 if(count_update>=300){
				 count_update=0;
				 count_updateflag=1;

			 }
		 }


	}



}




/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
