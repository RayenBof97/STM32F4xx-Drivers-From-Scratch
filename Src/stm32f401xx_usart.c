/*
 *
  ******************************************************************************
  * @file    stm32f401xx_usart.c
  * @author  Rayen	Bouafif
  * @mail    bouafifrayen01@gmail.com
  * @Tel     (+216)29 049 373
  * @date    28-01-2025
  *****************************************************************************
* This File Also contain the APIs explanation in the comments
*/



#include "stm32f401xx_usart.h"



/*
 * USART Peripheral Clock Control
 */

/********************************************************************
 * @fn				- RB_USART_PeriClockControl
 *
 * @brief			- This function enable or disable the USART peripheral clock
 *
 * @param[in]		- Pointer to USARTx structure
 * @param[in]		- ENABLE DISABLE Macros (which are an integer 0 or 1)
 *
 * @return 			- NONE
 *
 * @note			- NONE
 */
void RB_USART_PeriClockControl(USARTx_t *pUSARTx, uint8_t State){
	if (State == ENABLE) {
			if (pUSARTx == USART1) {
				USART1_PCLK_EN();
			} else if (pUSARTx == USART2) {
				USART2_PCLK_EN();
			} else if (pUSARTx == USART6) {
				USART6_PCLK_EN();
			}
		} else {  // DISABLE state
			if (pUSARTx == USART1) {
				USART1_PCLK_DI();
			} else if (pUSARTx == USART2) {
				USART2_PCLK_DI();
			} else if (pUSARTx == USART6) {
				USART6_PCLK_DI();
			}
	}
}

/*
 * Init and De-init USARTx
 */

/********************************************************************
 * @fn				- RB_USART_Init
 *
 * @brief			- initialize an USART peripheral
 * @param[in]		- a pointer to a structure of type USARTx_Handler_t that holds configuration of the USART and USART Peripehral pointer
 *
 * @return 			- NONE
 *
 * @note			- NONE
 */
void RB_USART_Init(USARTx_Handler_t *pUSARTHandle){

	//Enable The Clock
	RB_USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);



}

/********************************************************************
 * @fn				- RB_USART_DeInit
 *
 * @brief			- Deinitialize a specific USART (Reset all USART registers)
 *
 * @param[in]		- a pointer to a SPI Structure
 *
 * @return 			- NONE
 *
 * @note			- NONE
 */
void RB_USART_DeInit(USARTx_t *pUSARTx){

	if (pUSARTx == USART1) {
	    USART1_REG_RESET();
	} else if (pUSARTx == USART2) {
	    USART2_REG_RESET();
	} else if (pUSARTx == USART6) {
	    USART6_REG_RESET();
	}
}

/*
 * USART Data TX and RX
 */

/********************************************************************
 * @fn				- RB_USART_Data_TX
 *
 * @brief			- Transfer Data (Poll mode)
 *
 * @param[in]		- a pointer on the USART Structure
 * @param[in]		- The Buffer which will contain the Transferred data
 * @param[in]       - The length of the message
 *
 * @return 			- NONE
 * @note			- This API is a blocking call
 */
void RB_USART_Data_TX(SPIx_t *pUSARTx,uint8_t *pTxBuffer,uint32_t len){

}

/********************************************************************
 * @fn				- RB_USART_Data_RX
 *
 * @brief			- Receive Data (Poll Mode)
 *
 * @param[in]		- a pointer on the USART Structure
 * @param[in]		- The Buffer which will contain the received data
 * @param[in]       - The length of the message
 *
 * @return 			- NONE
 * @note			- This API is also a blocking call
 */
void RB_USART_Data_RX(SPIx_t *pUSARTx,uint8_t *pRxBuffer,uint32_t len){

}

/*
 * USART Data TX and RX (Interruption mode)
 */
/********************************************************************
 * @fn				- RB_USART_Data_TXIT
 *
 * @brief			- Transfer Data (Interrupt mode)
 *
 * @param[in]		- a pointer on the USART Handle Structure
 * @param[in]		- The Buffer which will contain the Transferred data
 * @param[in]       - The length of the message
 *
 * @return 			- Return the state of Tx
 * @note			- This API is a non-blocking call
 */
uint8_t RB_USART_Data_TXIT(USARTx_Handler_t *pUSARTHandle,uint8_t *pTxBuffer,uint32_t len){
	return 0;
}

/********************************************************************
 * @fn				- RB_USART_Data_RXIT
 *
 * @brief			- Receive Data (Interrupt Mode)
 *
 * @param[in]		- a pointer on the USART Handle Structure
 * @param[in]		- The Buffer which will contain the received data
 * @param[in]       - The length of the message
 *
 * @return 			- Return the state of Rx
 * @note			- This API is also a non-blocking call
 */
uint8_t RB_USART_Data_RXIT(USARTx_Handler_t *pUSARTHandle,uint8_t *pRxBuffer,uint32_t len){
	return 0;
}

/*
 * USART IRQ Configuration an ISR handling
 */
/********************************************************************
 * @fn				- RB_USART_IRQITConfig
 *
 * @brief			- Enable IRQ
 *
 * @param[in]		- IRQ Number
 * @param[in]		- state (Disable = 0 or Enable = 1)
 *
 * @return 			- NONE
 *
 * @note			- NONE
 */
void RB_USART_IRQITConfig(uint8_t IRQNumber, uint8_t state){
	if (state == ENABLE) {
	    if (IRQNumber < 32) {
	        // Configure the NVIC_ISER0
	    	*NVIC_ISER0 |= ( 1 << IRQNumber );
	    } else if (IRQNumber < 64) {
	        // Configure the NVIC_ISER1
	    	*NVIC_ISER1 |= ( 1 << IRQNumber % 32 );
	    } else if (IRQNumber < 96) {
	        // Configure the NVIC_ISER2
	    	*NVIC_ISER2 |= ( 1 << IRQNumber % 64 );
	    }
	} else { // state == DISABLE
	    if (IRQNumber < 32) {
	        // Configure the NVIC_ICER0
	    	*NVIC_ICER0 |= ( 1 << IRQNumber );
	    } else if (IRQNumber < 64) {
	        // Configure the NVIC_ICER1
	    	*NVIC_ICER1 |= ( 1 << IRQNumber % 32 );
	    } else if (IRQNumber < 96) {
	        // Configure the NVIC_ICER2
	    	*NVIC_ICER2 |= ( 1 << IRQNumber % 64 );
	    }
	}
}


/********************************************************************
 * @fn				- RB_USART_IRQPriorityConfig
 *
 * @brief			- Configure the priority of the IRQ
 *
 * @param[in]		- IRQ Number
 * @param[in]		- Priority (Integer)
 *
 * @return 			- NONE
 *
 * @note			- NONE
 */
void RB_USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority){
	// Finding out the IPRx Register
	uint8_t iprx = IRQNumber / 4 ;
	uint8_t iprx_section = IRQNumber % 4 ;
	uint8_t shift = (8 * iprx_section) + (8 - IMPLEMENTED_PRIORITY_BITS);
	*( NVIC_IPR + iprx ) |= (Priority << shift) ;
}

/********************************************************************
 * @fn				- RB_USART_IRQHandling
 *
 * @brief			- Handling the IRQ on USART Peripherals
 *
 * @param[in]		- Pointer on a USART_handler Structure containing the pointer of USART peripheral and its configuration
 *
 * @return 			- NONE
 *
 * @note			- NONE
 */
void RB_USART_IRQHandling(USARTx_Handler_t *pUSARTHandle){

}



