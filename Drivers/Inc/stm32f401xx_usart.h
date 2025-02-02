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
	uint32_t USART_Baud;						/*!<Value from @USART_Baud*/
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
#define USART_MODE_TX	0
#define USART_MODE_RX	1
#define USART_MODE_TXRX	2

/*
 * @USART_Baud
 */
#define USART_STD_BAUD_1200					1200
#define USART_STD_BAUD_2400					400
#define USART_STD_BAUD_9600					9600
#define USART_STD_BAUD_19200 				19200
#define USART_STD_BAUD_38400 				38400
#define USART_STD_BAUD_57600 				57600
#define USART_STD_BAUD_115200 				115200
#define USART_STD_BAUD_230400 				230400
#define USART_STD_BAUD_460800 				460800
#define USART_STD_BAUD_921600 				921600
#define USART_STD_BAUD_2M 					2000000
#define USART_STD_BAUD_3M 					3000000

/*
 * @USART_NbOfStopBits
 */
#define USART_STOPBIT_ONE		0
#define USART_STOPBIT_HALF		1
#define USART_STOPBIT_TWO		2
#define USART_STOPBIT_ONE_HALF 	3

/*
 * @USART_WordLength
 */
#define USART_WORDLEN_8BITS		0
#define USART_WORDLEN_9BITS		1

/*
 * @USART_ParityControl
 */
#define USART_PARITY_DISABLE	0
#define USART_PARITY_EVEN		1
#define USART_PARITY_ODD		2

/*
 * @USART_HWFlowControl
 */
#define USART_HWFLOW_CTRL_NONE    	0
#define USART_HWFLOW_CTRL_CTS    	1
#define USART_HWFLOW_CTRL_RTS    	2
#define USART_HWFLOW_CTRL_CTS_RTS	3

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
void RB_USART_Data_TX(USARTx_t *pUSARTx,uint8_t *pTxBuffer,uint32_t len);
void RB_USART_Data_RX(USARTx_t *pUSARTx,uint8_t *pRxBuffer,uint32_t len);

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

/*
 * Others APIs
 */
void USART_PeriphControl(USARTx_t *pUSARTx,uint8_t state);
uint8_t RB_USART_GetFlagStatus(USARTx_t *pUSARTx,uint8_t flag);
void RB_USART_ClearFlag(USARTx_t *pUSARTx,uint8_t flag);

/*
 * Application callback
 */
void USART_ApplicationEventCallback(USART_Handle_t *pUSARTHandle,uint8_t AppEv);

#endif /* INC_STM32F401XX_USART_H_ */
