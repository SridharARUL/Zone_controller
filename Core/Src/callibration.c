

#include "main.h"
#include"Relayinit.h"
#include <stdbool.h>




uint8_t ledTestActive = 0;
uint32_t ledTestStart = 0;

uint8_t calibrationFailActive = 0;
uint32_t calibrationFailStart = 0;

uint8_t ph4Active = 0;
uint32_t ph4Start = 0;

uint8_t ph7Active = 0;
uint32_t ph7Start = 0;

uint8_t ecActive = 0;
uint32_t ecStart = 0;
char buffr[100];
#define PH_PRESS_TIME     3000
#define EC_PRESS_TIME     10000
#define LED_TIMEOUT_MS    3000
extern bool Phvalueflag;
extern uint16_t PHrawValue;
volatile uint32_t pressStartTime = 0;
volatile uint8_t buttonHandled = 1;
uint32_t  Acidic_voltage,Neutral_Voltage;
extern float ECvalueRaw,Wtemp,Ec_Voltage;
float kvalue;
extern uint8_t Wtemp_Error;
#define RES2 (7500.0/0.66)
#define ECREF 200.0

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == GPIO_PIN_10)
  {
	  pressStartTime = HAL_GetTick(); // record press start
	   buttonHandled = 0;              // reset flag

  }

}

// ---------- BUTTON HANDLER ----------
void CheckButtonPress(void)
{
    uint8_t buttonState = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_10);

    if (buttonState == GPIO_PIN_SET) // released
    {
        uint32_t heldTime = HAL_GetTick() - pressStartTime;

        // CASE 1: LED TEST (<3s)
        if (heldTime < PH_PRESS_TIME)
        {
            PH_4(1);
            PH_7(1);
            EC_LED(1);
            calibration_fail(1);

            ledTestActive = 1;
            ledTestStart = HAL_GetTick();
        }

        // CASE 2: pH CALIBRATION (3s - 10s)
        else if (heldTime >= PH_PRESS_TIME && heldTime < EC_PRESS_TIME)
        {
            uint16_t Voltage = (uint16_t)((PHrawValue / 4096.0) * 3300);



            // pH 4 calibration
            if ((Voltage >= 1150) && (Voltage <= 1250))
            {
            	Phvalueflag=true;
            	Acidic_voltage = Voltage;
                PH_4(1);
                ph4Active = 1;
                ph4Start = HAL_GetTick();




            }
            // pH 7 calibration
            else if ((Voltage >= 975) && (Voltage <= 1075))
            {
            	Phvalueflag=true;
                Neutral_Voltage = Voltage;
                PH_7(1);
                ph7Active = 1;


                ph7Start = HAL_GetTick();
            }
            else
            {

				Acidic_voltage = 1200;
                Neutral_Voltage = 1024;
                calibration_fail(1);
                calibrationFailActive = 1;
                calibrationFailStart = HAL_GetTick();
            }

            write_EEPROM();
            read_EEPROM();
            backup_write_EEPROM();
            backup_read_EEPROM();
        }

        // CASE 3: EC CALIBRATION (>10s)
        else
        {

            static float rawECsolution;
            float KValueTemp = 0;

            if ((ECvalueRaw >= 0.3f) && (ECvalueRaw <= 1.9f))
            {
                if (!Wtemp_Error)
                    rawECsolution = 1.413f * (1.0f + 0.0185f * (Wtemp - 25.0f));
                else
                	rawECsolution = 1.413f * (1.0f + 0.0185f * (25.0f - 25.0f));
            }
            else
            {
            	kvalue = 1;
                calibration_fail(1);
                calibrationFailActive = 1;
                calibrationFailStart = HAL_GetTick();

            }

            KValueTemp = (RES2 * ECREF * rawECsolution) / (1000.0f * Ec_Voltage * 10.0f);

            if ((KValueTemp >= 0.3f) && (KValueTemp <= 10.0f))
            {
                kvalue = KValueTemp;
                EC_LED(1);
                ecActive = 1;
                ecStart = HAL_GetTick();

            }
            else
            {
                kvalue = 1;
                calibration_fail(1);
                calibrationFailActive = 1;
                calibrationFailStart = HAL_GetTick();
            }

            write_EEPROM();
            read_EEPROM();
            backup_write_EEPROM();
            backup_read_EEPROM();
        }

        buttonHandled = 1;
    }
}

uint8_t AnyLEDActive(void)
{
    return (ledTestActive || ph4Active || ph7Active || ecActive || calibrationFailActive);
}

void UpdateLEDTimers(void)
{
    uint32_t now = HAL_GetTick();

    if (ledTestActive && (now - ledTestStart >= LED_TIMEOUT_MS))
    {
        PH_4(0);
        PH_7(0);
        EC_LED(0);
        calibration_fail(0);
        ledTestActive = 0;
    }

    if (ph4Active && (now - ph4Start >= LED_TIMEOUT_MS))
    {
        PH_4(0);
        ph4Active = 0;
    }

    if (ph7Active && (now - ph7Start >= LED_TIMEOUT_MS))
    {
        PH_7(0);
        ph7Active = 0;
    }

    if (ecActive && (now - ecStart >= LED_TIMEOUT_MS))
    {
        EC_LED(0);
        ecActive = 0;
    }

    if (calibrationFailActive && (now - calibrationFailStart >= LED_TIMEOUT_MS))
    {
        calibration_fail(0);
        calibrationFailActive = 0;
    }
}


