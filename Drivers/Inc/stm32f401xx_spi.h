/*
 *
  ******************************************************************************
  * @file    stm32f401xx.h
  * @author  Rayen	Bouafif
  * @mail    bouafifrayen01@gmail.com
  * @Tel     (+216)29 049 373
  * @date    09-11-2024
  *****************************************************************************
*
*/

#ifndef INC_STM32F401XX_GPIO_H_
#define INC_STM32F401XX_GPIO_H_
#include "stm32f401xx.h"

typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
}SPIx_Config_t;

/*
 * Handler Structure for SPIx Peripheral
 */

typedef struct{
	SPIx_t *pSPIx;
	SPIx_Config_t SPI_Config;
}SPIx_Handler_t;



/**************************************************************************************************
 * 									APIs Supported by this driver
 *					For further informations, please check the functions definition
 **************************************************************************************************/
/*
 * Peripheral Clock Control
 */
void RB_SPI_PeriClockControl(SPIx_t *pSPIx, uint8_t State);

/*
 * Init and De-init SPIx
 */
void RB_SPI_Init(SPIx_Handler_t *pSPIHandle);
void RB_SPI_DeInit(SPIx_t *pSPIx);

/*
 * Data Send and Receive
 */
void RB_SPI_Data_TX(SPIx_t *pSPIx,uint8_t *pTxBuffer,uint32_t len);
void RB_SPI_Data_RX(SPIx_t *pSPIx,uint8_t *pRxBuffer,uint32_t len);


/*
 * IRQ Configuration an ISR handling
 */
void RB_SPI_IRQITConfig(uint8_t IRQNumber, uint8_t state);
void RB_SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority);
void RB_SPI_IRQHandling(SPIx_Handler_t *pSPIHandle);




#endif /* INC_STM32F401XX_GPIO_H_ */
