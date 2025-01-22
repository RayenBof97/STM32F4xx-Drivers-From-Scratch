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
	uint8_t SPI_DeviceMode;							/*!<Value from @Device_Mode*/
	uint8_t SPI_BusConfig;							/*!<Value from @Bus_Config*/
	uint8_t SPI_SclkSpeed;							/*!<Value from @Clock_Speed*/
	uint8_t SPI_DFF;								/*!<Value from @Data_Format*/
	uint8_t SPI_CPOL;								/*!<Value from @CPOL_Mode*/
	uint8_t SPI_CPHA;								/*!<Value from @CPHA_Mode*/
	uint8_t SPI_SSM;								/*!<Value from @SM_Mode*/
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
#define SPI_MODE_SLAVE 0
#define SPI_MODE_MASTER 1


/*
 * @Bus_Config
 */
#define SPI_BUS_CONFIG_FD  1
#define SPI_BUS_CONFIG_HD  2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY 3

/*
 * @Clock_Speed
 */
#define SPI_SCLK_SPEED_DIV2 0
#define SPI_SCLK_SPEED_DIV4 1
#define SPI_SCLK_SPEED_DIV8 2
#define SPI_SCLK_SPEED_DIV16 3
#define SPI_SCLK_SPEED_DIV32 4
#define SPI_SCLK_SPEED_DIV64 5
#define SPI_SCLK_SPEED_DIV128 6
#define SPI_SCLK_SPEED_DIV256 7

/*
 * @Data_Format
 */
#define SPI_DFF_8BITS 0
#define SPI_DFF_16BITS 1

/*
 * @CPOL_Mode
 */
#define SPI_CPOL_LOW 0
#define SPI_CPOL_HIGH 1

/*
 * @CPHA_Mode
 */
#define SPI_CPHA_LOW 0
#define SPI_CPHA_HIGH 1
/*
 * @SM_Mode
 */
#define SPI_SM_HARDWARE 0
#define SPI_SM_SOFTWARE 1
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
