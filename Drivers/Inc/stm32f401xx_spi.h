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
	SPIx_t *pSPIx;									/*!<Base Address of the SPI Peripheral*/
	SPIx_Config_t SPI_Config;						/*!<Structure of SPI Configuration*/
	uint8_t *pTxBuffer;								/*!<Storing the Application Tx Buffer */
	uint8_t *pRxBuffer;								/*!<Storing the Application Rx Buffer*/
	uint32_t TxLen;									/*!<Length of the app Tx Buffer*/
	uint32_t RxLen;									/*!<Length of the app Rx Buffer*/
	uint8_t TxState;								/*!<Sotring the state of Tx*/
	uint8_t RxState;								/*!<Storing the state of Rx*/
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

/*
 * SPI Application states
 */
#define SPI_READY 		0
#define SPI_BUSY_IN_RX	1
#define SPI_BUSY_IN_TX	2

/*
 * SPI Application Events
 */
#define SPI_EVENT_TX_CMPLT	1
#define SPI_EVENT_RX_CMPLT	2
#define SPI_EVENT_OVR_ERR   3
#define SPI_EVENT_CRC_ERR 	4

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
 * SPI Data TX and RX (Poll mode)
 */
void RB_SPI_Data_TX(SPIx_t *pSPIx,uint8_t *pTxBuffer,uint32_t len);
void RB_SPI_Data_RX(SPIx_t *pSPIx,uint8_t *pRxBuffer,uint32_t len);

/*
 * SPI Data TX and RX (Interruption mode)
 */
uint8_t RB_SPI_Data_TXIT(SPIx_Handler_t *pSPIHandle,uint8_t *pTxBuffer,uint32_t len);
uint8_t RB_SPI_Data_RXIT(SPIx_Handler_t *pSPIHandle,uint8_t *pRxBuffer,uint32_t len);

/*
 * SPI IRQ Configuration an ISR handling
 */
void RB_SPI_IRQITConfig(uint8_t IRQNumber, uint8_t state);
void RB_SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority);
void RB_SPI_IRQHandling(SPIx_Handler_t *pSPIHandle);

/*
 * Application Callback Functions
 */
void SPI_ApplicationEventCallback(SPIx_Handler_t *pSPIHandle,uint8_t event);
/*
 * Others APIs
 */
void SPI_PeriphControl(SPIx_t *pSPIx,uint8_t state);
void SPI_SSI_Config(SPIx_t *pSPIx,uint8_t state);
uint8_t RB_SPI_GetFlagStatus(SPIx_t *pSPIx,uint8_t flag);
void RB_SPI_ClearOVRFlag(SPIx_t *pSPIx);
void RB_SPI_CloseTx(SPIx_Handler_t *pSPIHandle);
void RB_SPI_CloseRx(SPIx_Handler_t *pSPIHandle);

#endif /* INC_STM32F401XX_SPI_H_ */
