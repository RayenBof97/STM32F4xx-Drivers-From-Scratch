/*
 *
  ******************************************************************************
  * @file    stm32f401xx_spi.h
  * @author  Rayen	Bouafif
  * @mail    bouafifrayen01@gmail.com
  * @Tel     (+216)29 049 373
  * @date    22-01-2025
  *****************************************************************************
*
*/

#ifndef INC_STM32F401XX_SPI_H_
#define INC_STM32F401XX_SPI_H_
#include "stm32f401xx.h"

typedef struct{
	uint8_t SPI_DeviceMode;							/*!<Value from @GPIO_Pins*/
	uint8_t SPI_BusConfig;							/*!<Value from @GPIO_Pins*/
	uint8_t SPI_SclkSpeed;							/*!<Value from @GPIO_Pins*/
	uint8_t SPI_DFF;								/*!<Value from @GPIO_Pins*/
	uint8_t SPI_CPOL;								/*!<Value from @GPIO_Pins*/
	uint8_t SPI_CPHA;								/*!<Value from @GPIO_Pins*/
	uint8_t SPI_SSM;								/*!<Value from @GPIO_Pins*/
}SPIx_Config_t;

/*
 * Handler Structure for SPIx Peripheral
 */

typedef struct{
	SPIx_t *pSPIx;
	SPIx_Config_t SPI_Config;
}SPIx_Handler_t;

/*
 * @Device_Mode
 */
#define MASTER 0
#define SLAVE 1

/*
 * @BusConfig
 */
#define S 0											/*Simplexe Mode*/
#define HD 1										/*Half Duplex Mode*/
#define FD 2 										/*Full Duplex Mode*/

/*
 * @Clock_Speed
 */


/*
 * @Data_Format
 */
#define _8B 0										/*8 Bit Data Format*/
#define _16B 1										/*16 Bits Data Format*/



/**************************************************************************************************
 * 									APIs Supported by this driver
 *					For further informations, please check the functions definition
 **************************************************************************************************/
/*
 * SPI Peripheral Clock Control
 */
void RB_SPI_PeriClockControl(SPIx_t *pSPIx, uint8_t State);

/*
 * Init and De-init SPIx
 */
void RB_SPI_Init(SPIx_Handler_t *pSPIHandle);
void RB_SPI_DeInit(SPIx_t *pSPIx);

/*
 * SPI Data TX and RX
 */
void RB_SPI_Data_TX(SPIx_t *pSPIx,uint8_t *pTxBuffer,uint32_t len);
void RB_SPI_Data_RX(SPIx_t *pSPIx,uint8_t *pRxBuffer,uint32_t len);


/*
 * SPI IRQ Configuration an ISR handling
 */
void RB_SPI_IRQITConfig(uint8_t IRQNumber, uint8_t state);
void RB_SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority);
void RB_SPI_IRQHandling(SPIx_Handler_t *pSPIHandle);




#endif /* INC_STM32F401XX_SPI_H_ */
