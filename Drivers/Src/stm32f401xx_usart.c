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

	uint32_t tempreg=0;

	//Enable Clock for USART Peripheral
	RB_USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

/******************************** Configuration of CR1******************************************/

	//Enable USART Tx and Rx engines according to the USART_Mode configuration
	if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_RX)
	{
		//Enable the RX Mode
		tempreg|= (1 << USART_CR1_RE);
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TX)
	{
		//Enable the TX Mode
		tempreg |= (1 << USART_CR1_TE);

	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		//Enable Both TX and RX
		tempreg |= ( ( 1 << USART_CR1_RE) | ( 1 << USART_CR1_TE) );
	}

    //Configuration of the data word Length
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M ;


    //Configuration of parity control bit fields
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EVEN)
	{
		//Enable The Parity Control
		tempreg |= ( 1 << USART_CR1_PCE);

		//By default, the Parity Selection is 0 (Even)

	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_ODD)
	{
		//Enable The Parity Control
	    tempreg |= ( 1 << USART_CR1_PCE);

	    //Enable ODD parity
	    tempreg |= ( 1 << USART_CR1_PS);

	}

   //Program the CR1 register
	pUSARTHandle->pUSARTx->CR1 = tempreg;

/******************************** Configuration of CR2******************************************/

	tempreg=0;

	//Configuration of Number of Stop Bits
	tempreg |= pUSARTHandle->USART_Config.USART_NbOfStopBits << USART_CR2_STOP;

	//Program the CR2 register
	pUSARTHandle->pUSARTx->CR2 = tempreg;

/******************************** Configuration of CR3******************************************/

	tempreg=0;

	//Configuration of USART hardware flow control
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HWFLOW_CTRL_CTS)
	{
		//Enable CTS
		tempreg |= ( 1 << USART_CR3_CTSE);


	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HWFLOW_CTRL_RTS)
	{
		//Enable RTS
		tempreg |= (1 << USART_CR3_RTSE);

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HWFLOW_CTRL_CTS_RTS)
	{
		//Enable CTS and RTS
		tempreg |= ( ( 1 << USART_CR3_CTSE) | ( 1 << USART_CR3_RTSE) );
	}


	pUSARTHandle->pUSARTx->CR3 = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

	//Baud Rate Configuration
	USART_SetBaudRate(pUSARTHandle->pUSARTx,pUSARTHandle->USART_Config.USART_Baud);

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
void RB_USART_Data_TX(USARTx_Handler_t *pUSARTHandle,uint8_t *pTxBuffer,uint32_t len){

	uint16_t *pdata;

	for(uint32_t i = 0 ; i < len; i++)
	{
		//Wait until TXE flag is set in the SR (Until TXE is empty)
		while(! RB_USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_SR_TXE));

         //Check the Data word Length
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//if 9BIT, load the DR with 2bytes masking the bits other than first 9 bits
			pdata = (uint16_t*) pTxBuffer;
			pUSARTHandle->pUSARTx->DR = (*pdata & (uint16_t)0x01FF);

			//check for USART_ParityControl
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE) //We have 9 Bits of user data
			{
				//No parity is used in this transfer. so, 9bits of user data will be sent
				//Implement the code to increment pTxBuffer twice
				pTxBuffer++;
				pTxBuffer++;
			}
			else
			{
				//Parity bit is used in this transfer . so , 8bits of user data will be sent
				//The 9th bit will be replaced by parity bit by the hardware
				pTxBuffer++;
			}
		}
		else //In case of 8Bits word Length
		{
			//This is 8bit data transfer
			pUSARTHandle->pUSARTx->DR = (*pTxBuffer  & (uint8_t)0xFF);

			//Increment the buffer address
			pTxBuffer++;
		}
	}

	//Wait until TC flag is set in the SR
	while( ! RB_USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_SR_TC));
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
void RB_USART_Data_RX(USARTx_Handler_t *pUSARTHandle,uint8_t *pRxBuffer,uint32_t len){

	for(uint32_t i = 0 ; i < len; i++)
	{
		//Implement the code to wait until RXNE flag is set in the SR
		while(! RB_USART_GetFlagStatus(pUSARTHandle->pUSARTx,USART_SR_RXNE));

		//Check the USART_WordLength
		if(pUSARTHandle->USART_Config.USART_WordLength == USART_WORDLEN_9BITS)
		{
			//Check if we are using a Parity control bit
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used. so, all 9bits is a user data
				//read only first 9 bits. so, mask the DR with 0x01FF
				*((uint16_t*) pRxBuffer) = (pUSARTHandle->pUSARTx->DR  & (uint16_t)0x01FF);

				//Increment Buffer
				pRxBuffer++;
				pRxBuffer++;
			}
			else
			{
				//Parity is used, so, 8bits will be of user data and 1 bit is parity
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);

				//Increment the pRxBuffer
				pRxBuffer++;
			}
		}
		else //In case of 8Bits Word length
		{
			//check are we using USART_ParityControl control or not
			if(pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_DISABLE)
			{
				//No parity is used , so all 8bits will be of user data
				//read 8 bits from DR
				*pRxBuffer =(pUSARTHandle->pUSARTx->DR  & (uint8_t)0xFF);
			}
			else
			{
				//Parity is used, so , 7 bits will be of user data and 1 bit is parity
				//read only 7 bits
				*pRxBuffer = (pUSARTHandle->pUSARTx->DR  & (uint8_t)0x7F);
			}
			//increment the pRxBuffer
			pRxBuffer++;
		}
	}
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
	return 0 ;
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

/*
 * Others APIs
 */
/********************************************************************
 * @fn				- USART_PeriphControl
 *
 * @brief			- Enable/Disable the USART Peripheral
 *
 * @param[in]		- Pointer on USART Peripheral
 * @param[in]		- State (ENABLE or DISABLE)
 *
 * @return 			- NONE
 *
 * @note			- The USART Peripheral is Disabled by default to allow control configuration on the USART Peripheral
 */
void USART_PeriphControl(USARTx_t *pUSARTx,uint8_t state){
	if (state == ENABLE)
	{
		pUSARTx->CR1 |= (1 << USART_CR1_UE);
	}else
	{
		pUSARTx->CR1 &= ~(1 << USART_CR1_UE);
	}
}

/********************************************************************
 * @fn				- RB_USART_GetFlagStatus
 *
 * @brief			- Return the status of specific flag
 *
 * @param[in]		- Pointer on USART Peripheral
 * @param[in]		- Desired Flag (Use the Bit position in the SPI_SR Register)
 *
 * @return 			- Status of the flag
 *
 * @note			- NONE
 */
uint8_t RB_USART_GetFlagStatus(USARTx_t *pUSARTx,uint8_t flag)
{
	return ((pUSARTx->SR & flag) ? FLAG_SET : FLAG_RESET);
}

/********************************************************************
 * @fn				- RB_USART_ClearFlag
 *
 * @brief			- Clear a specific Flag
 *
 * @param[in]		- Pointer on USART Peripheral
 * @param[in]		- Desired Flag (Use the Bit position in the SPI_SR Register)
 *
 * @return 			- NONE
 *
 * @note			- NONE
 */
void RB_USART_ClearFlag(USARTx_t *pUSARTx,uint8_t flag){
	pUSARTx->SR &= ~(1 << flag);
}

/*********************************************************************
 * @fn      		  - USART_SetBaudRate
 *
 * @brief             -	Configure the Baud rate in the BRR Register
 *
 * @param[in]         - USART Peripheral
 * @param[in]         -	Baud rate (Values chosen from @USART_Baud)
 *
 * @return            - NONE
 *
 * @Note              - NONE

 */
void USART_SetBaudRate(USARTx_t *pUSARTx, uint32_t BaudRate)
{

	//Variable to hold the APB clock
	uint32_t PCLKx;

	uint32_t usartdiv;

	//variables to hold Mantissa and Fraction values
	uint32_t M_part,F_part;

	uint32_t tempreg=0;

  //Get the value of APB bus clock in to the variable PCLKx
  if(pUSARTx == USART1 || pUSARTx == USART6)
  {
	   //USART1 and USART6 are hanging on APB2 bus
	   PCLKx = RCC_GetPCLK2Value();
  }else
  {
	  //USART2 is hanging on the APB1 Bus
	   PCLKx = RCC_GetPCLK1Value();
  }

  //Check for OVER8 configuration bit
  if(pUSARTx->CR1 & (1 << USART_CR1_OVER8))
  {
	   //OVER8 = 1 : over sampling by 8
	   usartdiv = ((25*PCLKx) / (2 *BaudRate)); //We multiplied by 100 to keep the fractional part
  }else
  {
	   //OVER8 = 0 : over sampling by 16
	  usartdiv = ((25*PCLKx) / (4 *BaudRate)); //Same thing in here
  }

  //Calculate the Mantissa part
  M_part = usartdiv/100;

  //Place the Mantissa part in the BRR (at the Bit USART_BRR_MANTISSA = 4)
  tempreg |= M_part << USART_BRR_MANTISSA;

  //Extract the fraction part
  F_part = (usartdiv - (M_part * 100));

  //Calculate the final fractional
  if(pUSARTx->CR1 & ( 1 << USART_CR1_OVER8))
   {
	  //OVER8 = 1 : over sampling by 8
	  F_part = ((( F_part * 8)+ 50) / 100) & ((uint8_t)0x07);

   }else
   {
	   //OVER8 = 0 : over sampling by 16
	   F_part = ((( F_part * 16)+ 50) / 100) & ((uint8_t)0x0F);

   }

  //Place the fraction in the USART_BRR_Fraction
  tempreg |= F_part << USART_BRR_FRACTION;

  //Copy tempreg to Baud Rate Register
  pUSARTx->BRR = tempreg;
}
