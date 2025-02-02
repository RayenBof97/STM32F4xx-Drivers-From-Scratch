/*
 *
  ******************************************************************************
  * @file    stm32f401xx_rcc.h
  * @author  Rayen	Bouafif
  * @mail    bouafifrayen01@gmail.com
  * @Tel     (+216)29 049 373
  * @date    02-02-2024
  *****************************************************************************
*
*/

/*
 * NOTE : This Driver is specified to USART and I2C Drivers , It only contains APIs that we are going to implement in the other drivers
 */

#ifndef INC_STM32F401XX_RCC_H_
#define INC_STM32F401XX_RCC_H_
#include "stm32f401xx.h"


#define HSE_VALUE	 80000000
#define HSI_VALUE 	 16000000

/**************************************************************************************************
 * 									APIs Supported by this driver
 *					For further informations, please check the functions definition
 **************************************************************************************************/
//This returns the APB1 clock value
uint32_t RCC_GetPCLK1Value(void);

//This returns the APB2 clock value
uint32_t RCC_GetPCLK2Value(void);

uint32_t  RCC_GetPLLOutputClock(void);




#endif /* INC_STM32F401XX_RCC_H_ */
