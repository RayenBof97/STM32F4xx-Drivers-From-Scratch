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

uint16_t AHB_PreScaler[8] = {2,4,8,16,64,128,256,512};
uint8_t APB_PreScaler[4] = { 2, 4 , 8, 16};

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
uint32_t RCC_GetPCLK1Value()
{
	uint32_t pclk1,SystemClk;

	uint8_t clksrc,temp,ahbp,apb1p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0) // HSI As a Clock source
	{
		SystemClk = HSI_VALUE; // 16Mhz
	}else if(clksrc == 1) //HSE As a Clock source
	{
		SystemClk = HSE_VALUE; // 80Mhz
	}else if (clksrc == 2) //PPL As a Clock Source
	{
		SystemClk = RCC_GetPLLOutputClock();
	}
	//Determine the Pre-scaler of AHB1
	temp = ((RCC->CFGR >> 4 ) & 0xF);

	if(temp < 8)
	{
		ahbp = 1; //The clock is not divided
	}else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	//Determine the Pr-escaler of APB1
	temp = ((RCC->CFGR >> 10 ) & 0x7);

	if(temp < 4)
	{
		apb1p = 1; //The APB1 Clock is not divided
	}else
	{
		apb1p = APB_PreScaler[temp-4];
	}

	pclk1 =  (SystemClk / ahbp) /apb1p;

	return pclk1;
}


/********************************************************************
 * @fn				- RCC_GetPCLK2Value
 *
 * @brief			- This returns the APB2 clock value
 *
 * @param[in]		- NONE
 *
 * @return 			- Return The APB2 Clock Frequency
 *
 * @note			- NONE
 */
uint32_t RCC_GetPCLK2Value()
{
	uint32_t pclk2,SystemClk;

	uint8_t clksrc,temp,ahbp,apb2p;

	clksrc = ((RCC->CFGR >> 2) & 0x3);

	if(clksrc == 0) // HSI As a Clock source
	{
		SystemClk = HSI_VALUE; // 16Mhz
	}else if(clksrc == 1) //HSE As a Clock source
	{
		SystemClk = HSE_VALUE; // 80Mhz
	}else if (clksrc == 2) //PPL As a Clock Source
	{
		SystemClk = RCC_GetPLLOutputClock();
	}
	//Determine the Pre-scaler of AHB1
	temp = ((RCC->CFGR >> 4 ) & 0xF);

	if(temp < 8)
	{
		ahbp = 1; //The clock is not divided
	}else
	{
		ahbp = AHB_PreScaler[temp-8];
	}

	//Determine the Pr-escaler of APB2
	temp = ((RCC->CFGR >> 13 ) & 0x7);

	if(temp < 4)
	{
		apb2p = 1; //The APB1 Clock is not divided
	}else
	{
		apb2p = APB_PreScaler[temp-4];
	}

	pclk2 =  (SystemClk / ahbp) /apb2p;

	return pclk2;
}
/********************************************************************
 * @fn				- RCC_GetPLLOutputClock
 *
 * @brief			- Determine the clock Frequency of the Phase-Locked Loop Clock source (PLL)
 *
 * @param[in]		- NONE
 *
 * @return 			- Return The PLL Clock Frequency
 *
 * @note			- NONE
 */
uint32_t RCC_GetPLLOutputClock(){

	uint32_t pll_source, pllm, plln, pllp;
	uint32_t vco_input, vco_output, pll_output;

	// Check PLL source: 0 = HSI (16 MHz), 1 = HSE (8 MHz)
	pll_source = (RCC->PLLCFGR & (1 << 22)) ? 8000000 : 16000000;

	// Extract PLL configuration parameters
	pllm = RCC->PLLCFGR & 0x3F;                       // Bits [5:0] for PLLM
	plln = (RCC->PLLCFGR >> 6) & 0x1FF;               // Bits [14:6] for PLLN
	pllp = (((RCC->PLLCFGR >> 16) & 0x3) + 1) * 2;    // Bits [17:16] for PLLP

	// Calculate VCO input frequency
	vco_input = pll_source / pllm;

	// Calculate VCO output frequency
	vco_output = vco_input * plln;

	// PLL output clock frequency
	pll_output = vco_output / pllp;

	return pll_output;
}
