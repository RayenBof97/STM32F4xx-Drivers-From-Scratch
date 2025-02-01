/*
 *
  ******************************************************************************
  * @file    stm32f401xx_spi.c
  * @author  Rayen	Bouafif
  * @mail    bouafifrayen01@gmail.com
  * @Tel     (+216)29 049 373
  * @date    09-11-2024
  *****************************************************************************
* This File Also contain the APIs explanation in the comments
*/

#include "stm32f401xx_spi.h"


/*PRIVATE FUNCTIONS*/
static void Spi_Txe_IT_Handler(SPIx_Handler_t *pSPIHandle);
static void Spi_Rxe_IT_Handler(SPIx_Handler_t *pSPIHandle);
static void Spi_Ovr_Err_IT_Handler(SPIx_Handler_t *pSPIHandle);

/*
 * SPI Peripheral Clock Control
 */

/********************************************************************
 * @fn				- RB_SPI_PeriClockControl
 *
 * @brief			- This function enable or disable the SPI peripheral clock
 *
 * @param[in]		- Pointer to SPIx structure
 * @param[in]		- ENABLE DISABLE Macros (which are an integer 0 or 1)
 *
 * @return 			- NONE
 *
 * @note			- NONE
 */
void RB_SPI_PeriClockControl(SPIx_t *pSPIx, uint8_t State){
	if (State == ENABLE) {
			if (pSPIx == SPI1) {
				SPI1_PCLK_EN();
			} else if (pSPIx == SPI2) {
				SPI2_PCLK_EN();
			} else if (pSPIx == SPI3) {
				SPI3_PCLK_EN();
			} else if (pSPIx == SPI4) {
				SPI4_PCLK_EN();
			}
		} else {  // DISABLE state
			if (pSPIx == SPI1) {
				SPI1_PCLK_DI();
			} else if (pSPIx == SPI2) {
				SPI2_PCLK_DI();
			} else if (pSPIx == SPI3) {
				SPI3_PCLK_DI();
			} else if (pSPIx == SPI4) {
				SPI4_PCLK_DI();
			}
	}
}


/*
 * Init and De-init SPIx
 */

/********************************************************************
 * @fn				- RB_SPI_Init
 *
 * @brief			- initialize an SPI peripheral
 * @param[in]		- a pointer to a structure of type SPIx_Handler_t that holds configuration of the SPI and SPI pointer
 *
 * @return 			- NONE
 *
 * @note			- NONE
 */
void RB_SPI_Init(SPIx_Handler_t *pSPIHandle){
	uint32_t temp = 0 ; //Temporary register

	//Enable the Clock
	RB_SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	/* Initialise the device mode */
	temp |= (pSPIHandle->SPI_Config.SPI_DeviceMode << SPI_CR1_MSTR);

	/* Initialise the Bus configuration*/
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
		{
			temp &= ~(1 << SPI_CR1_BIDIMODE);
		}
		else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
		{
			temp &= ~(1 << SPI_CR1_BIDIMODE);
			temp |=  (1 << SPI_CR1_RXONLY);
		}
		else if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
		{
			temp |= (1 << SPI_CR1_BIDIMODE);
		}
	 /* Initialise the Serial Clock speed */
	temp |= (pSPIHandle->SPI_Config.SPI_SclkSpeed << SPI_CR1_BR);

	/* Initialise the Data format*/
	temp |= (pSPIHandle->SPI_Config.SPI_DFF << SPI_CR1_DFF);

	/*Initialise CPOL and CPHA*/
	temp |= (pSPIHandle->SPI_Config.SPI_CPHA << SPI_CR1_CPHA);
	temp |= (pSPIHandle->SPI_Config.SPI_CPOL << SPI_CR1_CPOL);

	/*Initialise SSM */
	temp |= (pSPIHandle->SPI_Config.SPI_SSM << SPI_CR1_SSM);

	pSPIHandle->pSPIx->CR1 = temp ;
}


/********************************************************************
 * @fn				- RB_SPI_DeInit
 *
 * @brief			- Deinitialize a specific SPI (Reset all SPI registers)
 *
 * @param[in]		- a pointer to a SPI Structure
 *
 * @return 			- NONE
 *
 * @note			- NONE
 */
void RB_SPI_DeInit(SPIx_t *pSPIx){

	if (pSPIx == SPI1) {
	    SPI1_REG_RESET();
	} else if (pSPIx == SPI2) {
	    SPI2_REG_RESET();
	} else if (pSPIx == SPI3) {
	    SPI3_REG_RESET();
	} else if (pSPIx == SPI4) {
	    SPI4_REG_RESET();
	}
}

/*
 * SPI Data TX and RX
 */

/********************************************************************
 * @fn				- RB_SPI_Data_TX
 *
 * @brief			- Transfer Data (Poll mode)
 *
 * @param[in]		- a pointer on the SPI Structure
 * @param[in]		- The Buffer which will contain the Transferred data
 * @param[in]       - The length of the message
 *
 * @return 			- NONE
 * @note			- This API is a blocking call
 */
void RB_SPI_Data_TX(SPIx_t *pSPIx,uint8_t *pTxBuffer,uint32_t len){

	while (len > 0)
	{
		while(!RB_SPI_GetFlagStatus(pSPIx,SPI_SR_TXE)); //Wait until TX Buffer is empty

		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) //16 Bits
		{
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			len--;
			len--;
			(uint16_t*)pTxBuffer++;
		}else   								//8 Bits
		{
			pSPIx->DR = *pTxBuffer;
			(uint8_t*)pTxBuffer++;
			len--;
		}

	}
}


/********************************************************************
 * @fn				- RB_SPI_Data_RX
 *
 * @brief			- Receive Data (Poll Mode)
 *
 * @param[in]		- a pointer on the SPI Structure
 * @param[in]		- The Buffer which will contain the received data
 * @param[in]       - The length of the message
 *
 * @return 			- NONE
 * @note			- This API is also a blocking call
 */
void RB_SPI_Data_RX(SPIx_t *pSPIx,uint8_t *pRxBuffer,uint32_t len){

	while (len > 0)
	{
		while(!RB_SPI_GetFlagStatus(pSPIx,SPI_SR_RXNE)); //Wait until RX Buffer is non-empty

		if (pSPIx->CR1 & (1 << SPI_CR1_DFF)) //16 Bits
		{
			*((uint16_t*)pRxBuffer) = (uint16_t)pSPIx->DR;
			len--;
			(uint16_t*)pRxBuffer++;
		}else   							//8 Bits
		{
			*(pRxBuffer) = (uint8_t)pSPIx->DR;
			pRxBuffer++;
		}
		len--;
	}
}

/*
 * SPI Data TX and RX (Interruption mode)
 */
/********************************************************************
 * @fn				- RB_SPI_Data_TXIT
 *
 * @brief			- Transfer Data (Interrupt mode)
 *
 * @param[in]		- a pointer on the SPI Handle Structure
 * @param[in]		- The Buffer which will contain the Transferred data
 * @param[in]       - The length of the message
 *
 * @return 			- Return the state of Tx
 * @note			- This API is a non-blocking call
 */
uint8_t RB_SPI_Data_TXIT(SPIx_Handler_t *pSPIHandle,uint8_t *pTxBuffer,uint32_t len){
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
		//Save Tx Buffer Address and Length informations
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = len;
		//Mark the TX State as busy
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		//Enable the TXEIE Control Bit to enable the interrupt whenever TXE is 1 (TX Buffer is empty)
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE);
	}
	return state;
}
/********************************************************************
 * @fn				- RB_SPI_Data_RXIT
 *
 * @brief			- Receive Data (Interrupt Mode)
 *
 * @param[in]		- a pointer on the SPI Handle Structure
 * @param[in]		- The Buffer which will contain the received data
 * @param[in]       - The length of the message
 *
 * @return 			- Return the state of Rx
 * @note			- This API is also a non-blocking call
 */
uint8_t RB_SPI_Data_RXIT(SPIx_Handler_t *pSPIHandle,uint8_t *pRxBuffer,uint32_t len){
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX)
	{
		//Save Rx Buffer Address and Length informations
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = len;
		//Mark the RX State as busy
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		//Enable the RXNEIE Control Bit to enable the interrupt whenever RXNE is 1 (RX Buffer is full)
		pSPIHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE);
	}
	return state;
}

/*
 * SPI IRQ Configuration an ISR handling
 */
/********************************************************************
 * @fn				- RB_SPI_IRQITConfig
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
void RB_SPI_IRQITConfig(uint8_t IRQNumber, uint8_t state){
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
 * @fn				- RB_SPI_IRQPriorityConfig
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
void RB_SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority){
	// Finding out the IPRx Register
	uint8_t iprx = IRQNumber / 4 ;
	uint8_t iprx_section = IRQNumber % 4 ;
	uint8_t shift = (8 * iprx_section) + (8 - IMPLEMENTED_PRIORITY_BITS);
	*( NVIC_IPR + iprx ) |= (Priority << shift) ;
}

/********************************************************************
 * @fn				- RB_SPI_IRQHandling
 *
 * @brief			- Handling the IRQ on SPI Peripherals
 *
 * @param[in]		- Pointer on a SPI_handler Structure containing the pointer of SPI peripheral and its configuration
 *
 * @return 			- NONE
 *
 * @note			- NONE
 */

void RB_SPI_IRQHandling(SPIx_Handler_t *pSPIHandle){
	uint8_t temp1,temp2;

	//Check for the TXE Flag
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);
	if (temp1 && temp2)
	{
		Spi_Txe_IT_Handler(pSPIHandle);
	}

	//Check for the RXE Flag
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);
	if (temp1 && temp2)
	{
		Spi_Rxe_IT_Handler(pSPIHandle);
	}

	//Check for the OVR Flag
	temp1 = pSPIHandle->pSPIx->SR & (1 << SPI_SR_OVR );
	temp2 = pSPIHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);
	if (temp1 && temp2)
	{
		Spi_Ovr_Err_IT_Handler(pSPIHandle);
	}

}


/*
 * Others APIs
 */
/********************************************************************
 * @fn				- SPI_PeriphControl
 *
 * @brief			- Enable/Disable the SPI Peripheral
 *
 * @param[in]		- Pointer on SPI Peripheral
 * @param[in]		- State (ENABLE or DISABLE)
 *
 * @return 			- NONE
 *
 * @note			- The SPI Peripheral is Disabled by default to allow control configuration on the SPI Peripheral
 */
void SPI_PeriphControl(SPIx_t *pSPIx,uint8_t state){
	if (state == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}

}

/********************************************************************
 * @fn				- SPI_SSI_Config
 *
 * @brief			- Enable/Disable the SSI in NSS Software mode
 *
 * @param[in]		- Pointer on SPI Peripehral
 * @param[in]		- State (ENABLE or DISABLE)
 *
 * @return 			- NONE
 *
 * @note			- This is necessary when to do when you are using the SPI Peripheral in master mode (Set SSI to 1)
 */
void SPI_SSI_Config(SPIx_t *pSPIx,uint8_t state){
	if (state == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SSI);
	}else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI);
	}
}
/********************************************************************
 * @fn				- RB_SPI_GetFlag
 *
 * @brief			- Return the status of specific flag
 *
 * @param[in]		- Pointer on SPI Peripheral
 * @param[in]		- Desired Flag (Macros defined @SPI_Flags , or use the Bit position in the SPI_SR Register
 *
 * @return 			- Status of the flag (FLAG_SET or FLAG_RESET)
 *
 * @note			- NONE
 */
uint8_t RB_SPI_GetFlagStatus(SPIx_t *pSPIx,uint8_t flag){
	return (pSPIx->SR & (1 << flag));
}

/********************************************************************
 * @fn				- RB_SPI_ClearOVRFlag
 *
 * @brief			- Clear the OVR Flag in Application
 *
 * @param[in]		- Pointer on SPI Peripheral
 *
 * @return 			- NONE
 *
 * @note			- Used when OVR Error Happen
 */
void RB_SPI_ClearOVRFlag(SPIx_t *pSPIx){
	(void)pSPIx->DR;
	(void)pSPIx->SR;
}

/********************************************************************
 * @fn				- RB_SPI_CloseTx
 *
 * @brief			- Close the SPI Transmission
 *
 * @param[in]		- Pointer on SPI Handler
 *
 * @return 			- NONE
 *
 * @note			- NONE
 */
void RB_SPI_CloseTx(SPIx_Handler_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE); //Disable the interrupt from TXE
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

/********************************************************************
 * @fn				- RB_SPI_CloseRx
 *
 * @brief			- Close the SPI Reception
 *
 * @param[in]		- Pointer on SPI Handler
 *
 * @return 			- NONE
 *
 * @note			- NONE
 */
void RB_SPI_CloseRx(SPIx_Handler_t *pSPIHandle){
	pSPIHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_RXNEIE); //Disable the interrupt from TXE
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;
}



/* PRIVATE FUNCTIONS SOURCES CODE */
static void Spi_Txe_IT_Handler(SPIx_Handler_t *pSPIHandle)
{
	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) //16 Bits
			{
				pSPIHandle->pSPIx->DR = *((uint16_t*)pSPIHandle->pTxBuffer);
				pSPIHandle->TxLen-=2;
				(uint16_t*)pSPIHandle->pTxBuffer++;
			}else   								//8 Bits
			{
				pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);
				pSPIHandle->TxLen--;
				pSPIHandle->pTxBuffer++;
			}
	if(! pSPIHandle->TxLen) //If the Transmission is over , We have to close the transmission
	{
		RB_SPI_CloseTx(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT); //Inform the application that the transmission is done
	}
}


static void Spi_Rxe_IT_Handler(SPIx_Handler_t *pSPIHandle)
{

	if (pSPIHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF)) //16 Bits
			{
				*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t)pSPIHandle->pSPIx->DR;
				pSPIHandle->RxLen-=2;
				pSPIHandle->pRxBuffer+=2;
			}else   								//8 Bits
			{
				*(pSPIHandle->pRxBuffer) = (uint8_t)pSPIHandle->pSPIx->DR;
				pSPIHandle->RxLen--;
				pSPIHandle->pRxBuffer++;
			}
	if(! pSPIHandle->RxLen) //If the Transmission is over , We have to close the transmission
		{
			RB_SPI_CloseRx(pSPIHandle);
			SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
		}

}

static void Spi_Ovr_Err_IT_Handler(SPIx_Handler_t *pSPIHandle){
	//Clear the OVR Flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		(void)pSPIHandle->pSPIx->DR;
		(void)pSPIHandle->pSPIx->SR;
	}
	//Inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR); //Call The RB_SPI_ClearOVRFlag To clear the flag explicitly in the application
}

/*
 * Application CallBack Functions
 */
__weak void SPI_ApplicationEventCallback(SPIx_Handler_t *pSPIHandle,uint8_t event){
	//This an application callback function that user can override
}
