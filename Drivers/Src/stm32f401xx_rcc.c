/*
 *
  ******************************************************************************
  * @file    stm32f401xx_rcc.c
  * @author  Rayen	Bouafif
  * @mail    bouafifrayen01@gmail.com
  * @Tel     (+216)29 049 373
  * @date    02-02-2024
  *****************************************************************************
* This File Also contain the APIs explanation in the comments
*/

#include "stm32f401xx_rcc.h"

/********************************************************************
 * @fn				- RCC_GetPCLK1Value
 *
 * @brief			- This returns the APB1 clock value
 *
 * @param[in]		- NONE
 *
 * @return 			- Return The APB1 Clock Frequency
 *
 * @note			- NONE
 */
uint32_t RCC_GetPCLK1Value(void)
{
	uint32_t pclk1,SystemClk;

	uint8_t clksrc,temp,ahbp,apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0) // HSI As a Clock source
	{
		SystemClk = 16000000; // 16Mhz
	}else if(clksrc == 1) //HSE As a Clock source
	{
		SystemClk = 8000000; // 80Mhz
	}else if (clksrc == 2) //PPL As a Clock Source
	{
		SystemClk = RCC_GetPLLOutputClock();
	}

	//for ahb
	temp = ((RCC->CFGR >> 4 ) & 0xF);

	if(temp < 8)
	{
		ahbp = 1;
	}else
	{
		ahbp = AHB_PreScaler[temp-8];
	}



	//apb1
	temp = ((RCC->CFGR >> 10 ) & 0x7);

	if(temp < 4)
	{
		apb1p = 1;
	}else
	{
		apb1p = APB1_PreScaler[temp-4];
	}

	pclk1 =  (SystemClk / ahbp) /apb1p;

	return pclk1;
}
