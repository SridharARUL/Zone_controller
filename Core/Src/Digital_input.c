/*
 * Digital_input.c
 *
 *  Created on: Sep 4, 2025
 *      Author: Sridhar A
 */
#include"Digital_input.h"
#include"main.h"
#include <stdint.h>
#define DELAY_MS   1800000UL   // 30 minutes
uint32_t EcA_fill_count = 0,pHDN_fill_count,EcB_fill_count,pHUP_fill_count;

uint8_t waterlevel1 (){//low

	uint8_t w_lvl1 =HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_2);

	return w_lvl1;
}

uint8_t waterlevel2 (){//high

	uint8_t w_lvl2 =HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_3);

	return w_lvl2;
}



uint8_t Ec_A_Level (){
	static uint8_t ECA_empty_cycle_active = 0;
	    static uint32_t ECA_empty_start_time = 0;


	uint8_t ECA =HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);
	uint32_t now = HAL_GetTick();


	if ((ECA == 0) && (ECA_empty_cycle_active == 0))
	    {
		ECA_empty_cycle_active = 1;
		ECA_empty_start_time = now;
	    }

	    // Step 2: After 30 minutes, check ONCE
	    if (ECA_empty_cycle_active)
	    {
	        if (now >= (ECA_empty_start_time + DELAY_MS)) // 30 minutes
	        {
	            if (ECA == 1)
	            {
	            	EcA_fill_count++;   // VALID COUNT

	            	ECA_empty_cycle_active = 0;
	            }


	        }
	    }

	return ECA;
}


uint8_t Ec_B_Level(void)
{
    static uint8_t  ECB_empty_cycle_active = 0;
    static uint32_t ECB_empty_start_time = 0;

    uint8_t  ECB = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15);
    uint32_t now = HAL_GetTick();

    if ((ECB == 0) && (ECB_empty_cycle_active == 0))
    {
        ECB_empty_cycle_active = 1;
        ECB_empty_start_time = now;
    }

    if (ECB_empty_cycle_active)
    {
        if (now - ECB_empty_start_time >= DELAY_MS)
        {
            if (ECB == 1)
            {
                EcB_fill_count++;
                ECB_empty_cycle_active = 0;
            }
        }
    }

    return ECB;
}


uint8_t pH_Up_Level(void)
{
    static uint8_t  pHUP_empty_cycle_active = 0;
    static uint32_t pHUP_empty_start_time = 0;

    uint8_t  pHUP = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_2);
    uint32_t now  = HAL_GetTick();

    if ((pHUP == 0) && (pHUP_empty_cycle_active == 0))
    {
        pHUP_empty_cycle_active = 1;
        pHUP_empty_start_time = now;
    }

    if (pHUP_empty_cycle_active)
    {
        if (now - pHUP_empty_start_time >= DELAY_MS)
        {
            if (pHUP == 1)
            {
                pHUP_fill_count++;
                pHUP_empty_cycle_active = 0;
            }
        }
    }

    return pHUP;
}


uint8_t pH_Down_Level(void)
{
    static uint8_t  pHDN_empty_cycle_active = 0;
    static uint32_t pHDN_empty_start_time = 0;

    uint8_t  pHDN = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_3);
    uint32_t now  = HAL_GetTick();

    if ((pHDN == 0) && (pHDN_empty_cycle_active == 0))
    {
        pHDN_empty_cycle_active = 1;
        pHDN_empty_start_time = now;
    }

    if (pHDN_empty_cycle_active)
    {
        if (now - pHDN_empty_start_time >= DELAY_MS)
        {
            if (pHDN == 1)
            {
                pHDN_fill_count++;
                pHDN_empty_cycle_active = 0;
            }
        }
    }

    return pHDN;
}

