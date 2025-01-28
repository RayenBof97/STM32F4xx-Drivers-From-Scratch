/*
 *
  ******************************************************************************
  * @file    stm32f401xx_usart.h
  * @author  Rayen	Bouafif
  * @mail    bouafifrayen01@gmail.com
  * @Tel     (+216)29 049 373
  * @date    28-01-2025
  *****************************************************************************
*
*/

#ifndef INC_STM32F401XX_USART_H_
#define INC_STM32F401XX_USART_H_
#include "stm32f401xx.h"

typedef struct{
	uint8_t USART_Mode;							/*!<Value from @USART_Mode*/
	uint8_t USART_Baud;							/*!<Value from @USART_Baud*/
	uint8_t USART_NbOfStopBits;					/*!<Value from @USART_NbOfStopBits*/
	uint8_t USART_WordLength;					/*!<Value from @USART_WordLength*/
	uint8_t USART_ParityControl;				/*!<Value from @USART_ParityControl*/
	uint8_t USART_HWFlowControl;				/*!<Value from @USART_HWFlowControl*/
}USARTx_PinConfig_t;


/*
 * Handler Structure for SPIx Peripheral
 */

typedef struct{
	USARTx_t *pUSARTx;
	USARTx_PinConfig_t USART_PinConfig;
}USARTx_Handler_t;


/*
 * @USART_Mode
 */
#define USART_TX	0
#define USART_RX	1

/*
 * @USART_Baud
 */


/*
 * @USART_NbOfStopBits
 */
#define ONE_STOP_BIT	0
#define HALF_STOP_BIT	1
#define TWO_STOP_BIT	2
#define ONE_AND_HALF_STOP_BIT 3

/*
 * @USART_WordLength
 */
#define USART_8_DATA_BITS	0
#define USART_9_DATA_BITS	1

/*
 * @USART_ParityControl
 */
#define USART_PARITY_DISABLE	0
#define USART_EVEN_PARITY		1
#define USART_ODD_PARITY		2

/*
 * @USART_HWFlowControl
 */

/**************************************************************************************************
 * 									APIs Supported by this driver
 *					For further informations, please check the functions definition
 **************************************************************************************************/
/*
 * USART Peripheral Clock Control
 */
void RB_USART_PeriClockControl(USARTx_t *pUSARTx, uint8_t State);

/*
 * Init and De-init USARTx
 */
void RB_USART_Init(USARTx_Handler_t *pUSARTHandle);
void RB_USART_DeInit(USARTx_t *pUSARTx);

/*
 * USART Data TX and RX (Poll mode)
 */
void RB_USART_Data_TX(SPIx_t *pUSARTx,uint8_t *pTxBuffer,uint32_t len);
void RB_USART_Data_RX(SPIx_t *pUSARTx,uint8_t *pRxBuffer,uint32_t len);

/*
 * USART Data TX and RX (Interruption mode)
 */
uint8_t RB_USART_Data_TXIT(USARTx_Handler_t *pUSARTHandle,uint8_t *pTxBuffer,uint32_t len);
uint8_t RB_USART_Data_RXIT(USARTx_Handler_t *pUSARTHandle,uint8_t *pRxBuffer,uint32_t len);

/*
 * USART IRQ Configuration an ISR handling
 */
void RB_USART_IRQITConfig(uint8_t IRQNumber, uint8_t state);
void RB_USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority);
void RB_USART_IRQHandling(USARTx_Handler_t *pUSARTHandle);

#endif /* INC_STM32F401XX_USART_H_ */
