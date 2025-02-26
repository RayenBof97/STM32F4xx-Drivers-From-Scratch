/*
 *
  ******************************************************************************
  * @file    stm32f401xx_i2c.c
  * @author  Rayen	Bouafif
  * @mail    bouafifrayen01@gmail.com
  * @Tel     (+216)29 049 373
  * @date    03-02-2024
  *****************************************************************************
* This File Also contain the APIs explanation in the comments
*/

#include "stm32f401xx_i2c.h"
#include "stm32f401xx_rcc.h"
/* Static Functions prototypes*/
static void I2C_GenerateStartCondition(I2Cx_t *pI2Cx);
static void I2C_GenerateStopCondition(I2Cx_t *pI2Cx);
static void I2C_ExecuteAddressPhaseWrite(I2Cx_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ExecuteAddressPhaseRead(I2Cx_t *pI2Cx,uint8_t SlaveAddr);
static void I2C_ClearADDRStatus(I2Cx_Handler_t* pI2CHandle);
static void I2C_CloseTX(I2Cx_Handler_t* pI2CHandle);
static void I2C_CloseRX(I2Cx_Handler_t* pI2CHandle);
static void I2C_MasterHandleTXEInterrupt(I2Cx_Handler_t *pI2CHandle);
static void I2C_MasterHandleRXNEInterrupt(I2Cx_Handler_t *pI2CHandle);
/*
 * Static Functions Definitions
 */
static void I2C_GenerateStartCondition(I2Cx_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_GenerateStopCondition(I2Cx_t *pI2Cx){
	pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

static void I2C_ExecuteAddressPhaseWrite(I2Cx_t *pI2Cx,uint8_t SlaveAddr){
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr &= ~(0x1); //SlaveAddr = SlaveAddr (7Bits) + R/NW bit = 0
	pI2Cx->DR = SlaveAddr; // Here SB Flag is cleared
}

static void I2C_ExecuteAddressPhaseRead(I2Cx_t *pI2Cx,uint8_t SlaveAddr){
	SlaveAddr = SlaveAddr << 1;
	SlaveAddr |= 0x1; //SlaveAddr = SlaveAddr (7Bits) + R/NW bit = 1
	pI2Cx->DR = SlaveAddr; // Here SB Flag is cleared
}

static void I2C_ClearADDRStatus(I2Cx_Handler_t* pI2CHandle) {
    uint32_t dummy;

    // Check if the device is in MASTER mode
    if (RB_I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_MSL)) {
        // MASTER MODE
        if ( (pI2CHandle->TxRxState == I2C_BUSY_RX) && (pI2CHandle->RxSize == 1) ) {
            // Disable Acknowledgment in one byte transfer , So the slave won't expect to send another byte
            RB_I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
        }
    }

    // Clear ADDR flag by reading SR1 and SR2
    dummy = pI2CHandle->pI2Cx->SR1;
    dummy = pI2CHandle->pI2Cx->SR2;
    (void)dummy; // Suppress unused variable warning
}

static void I2C_CloseTX(I2Cx_Handler_t* pI2CHandle){
	//Disable Interrupts

	//Disable ITBUFEN
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	//Disable ITEVFEN
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	//Initialise Handler Fields
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->TxLen = 0;
	pI2CHandle->pTxBuffer = NULL;
}

static void I2C_CloseRX(I2Cx_Handler_t* pI2CHandle){
	//Disable Interrupts

	//Disable ITBUFEN
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
	//Disable ITEVFEN
	pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

	//Initialise Handler Fields
	pI2CHandle->TxRxState = I2C_READY;
	pI2CHandle->RxLen = 0;
	pI2CHandle->RxSize = 0;
	pI2CHandle->pRxBuffer = NULL;

	if(pI2CHandle->I2C_Config.I2C_ACKControl== I2C_ACK_ENABLE)
	{
		RB_I2C_ManageAcking(pI2CHandle->pI2Cx, ENABLE);
	}
}

static void I2C_MasterHandleTXEInterrupt(I2Cx_Handler_t *pI2CHandle){
	if (pI2CHandle->TxLen > 0)
	{
		pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);
		pI2CHandle->TxLen--;
		pI2CHandle->pTxBuffer++;
	}
}

static void I2C_MasterHandleRXNEInterrupt(I2Cx_Handler_t *pI2CHandle){
    if(pI2CHandle->RxLen == 0 )
    {
        //close I2C TX

        //Generate Stop Condition
        if(pI2CHandle->Sr == I2C_SR_DISABLE)
            I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

        //2 . Close the I2C rx
        I2C_CloseRX(pI2CHandle);

        //3. Notify the application
        I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_RX_CMPLT);
    }

	if (pI2CHandle->RxSize == 1)
	{
		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
	}

	if (pI2CHandle->RxSize > 1)
	{
		if(pI2CHandle->RxLen == 2)
		{
			//Clear ACK Bit
			RB_I2C_ManageAcking(pI2CHandle->pI2Cx, DISABLE);
		}

		*pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
		pI2CHandle->RxLen--;
		pI2CHandle->pRxBuffer++;
	}
}
/*
 * I2C Peripheral Clock Control
 */

/********************************************************************
 * @fn				- RB_I2C_PeriClockControl
 *
 * @brief			- This function enable or disable the I2C peripheral clock
 *
 * @param[in]		- Pointer to I2Cx structure
 * @param[in]		- ENABLE DISABLE Macros (which are an integer 0 or 1)
 *
 * @return 			- NONE
 *
 * @note			- NONE
 */

void RB_I2C_PeriClockControl(I2Cx_t *pI2Cx, uint8_t State){
	if (State == ENABLE) {
			if (pI2Cx == I2C1) {
				I2C1_PCLK_EN();
			} else if (pI2Cx == I2C2) {
				I2C2_PCLK_EN();
			} else if (pI2Cx == I2C3) {
				I2C3_PCLK_EN();
			}
		} else {  // DISABLE state
			if (pI2Cx == I2C1) {
				I2C1_PCLK_DI();
			} else if (pI2Cx == I2C2) {
				I2C2_PCLK_DI();
			} else if (pI2Cx == I2C3) {
				I2C3_PCLK_DI();
			}
	}
}


/*
 * Init and De-init I2Cx
 */

/********************************************************************
 * @fn				- RB_I2C_Init
 *
 * @brief			- initialize an I2C peripheral
 * @param[in]		- a pointer to a structure of type I2Cx_Handler_t that holds configuration of the I2C and I2C pointer
 *
 * @return 			- NONE
 *
 * @note			- NONE
 */
void RB_I2C_Init(I2Cx_Handler_t *pI2CHandle){

	//Enable the Clock for I2Cx
	RB_I2C_PeriClockControl(pI2CHandle->pI2Cx, ENABLE);

	//Configuring ACK Bit in CR1 Register
	pI2CHandle->pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	pI2CHandle->pI2Cx->CR1 |= pI2CHandle->I2C_Config.I2C_ACKControl << I2C_CR1_ACK;

	//Configure Peripheral Clock Frequency in the CR2
	pI2CHandle->pI2Cx->CR2 &= ~(0x3F); //Clearing
	pI2CHandle->pI2Cx->CR2 |= ((RCC_GetPCLK1Value() / 1000000U) & 0x3F);

	//Device Address (7Bits Adressing mode , we will not care about the 1st, 8th and 9th bits)
	pI2CHandle->pI2Cx->OAR1 &= ~(0xFE);
	pI2CHandle->pI2Cx->OAR1 |= ((uint16_t)(pI2CHandle->I2C_Config.I2C_DeviceAddress << I2C_OAR1_ADD7_1)) | (1 << I2C_OAR1_ADDMODE);

	//CCR (Clock Control Register) Calculation
	uint16_t ccr = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_STANDARD)
	{
		//Standard Mode
		pI2CHandle->pI2Cx->CCR &= ~(1 << I2C_CCR_FS); //Standard mode (Standard speed)
		ccr = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed) ) & 0xFFF; //Calculate CCR value for SM
		pI2CHandle->pI2Cx->CCR &= ~(0xFFF); //Clearing
		pI2CHandle->pI2Cx->CCR |= ccr;


	}else
	{
		//Fast Mode (Fast Frequency)
		pI2CHandle->pI2Cx->CCR |= 1 << I2C_CCR_FS; //Configure Fast Mode
		pI2CHandle->pI2Cx->CCR |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY); //Configure FM Duty cycle

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == 0) // T_low/T_high = 2
		{
			ccr = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed) ) & 0xFFF;
		}else // T_low/T_high = 16/9
		{
			ccr = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed) ) & 0xFFF;
		}
		pI2CHandle->pI2Cx->CCR &= ~(0xFFF); //Clearing
		pI2CHandle->pI2Cx->CCR |= ccr;
	}

	//Configuring TRISE
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_STANDARD)
		{
			//Standard Mode
			uint8_t trise ;
			trise = (RCC_GetPCLK1Value() / 1000000U) + 1;
			pI2CHandle->pI2Cx->TRISE |= (trise & 0x3F);

		}else
		{
			//Fast Mode
			uint8_t trise;
			trise = ((RCC_GetPCLK1Value()*300)/ 1000000000U ) + 1;
			pI2CHandle->pI2Cx->TRISE |= (trise & 0x3F);
		}

}

/********************************************************************
 * @fn				- RB_I2C_DeInit
 *
 * @brief			- Deinitialize a specific I2C (Reset all I2C registers)
 *
 * @param[in]		- a pointer to a I2C Structure
 *
 * @return 			- NONE
 *
 * @note			- NONE
 */
void RB_I2C_DeInit(I2Cx_t *pI2Cx){

	if (pI2Cx == I2C1) {
	    I2C1_REG_RESET();
	} else if (pI2Cx == I2C2) {
	    I2C2_REG_RESET();
	} else if (pI2Cx == I2C3) {
	    I2C3_REG_RESET();
	}

}

/*
 * I2C Data TX and RX (Poll mode)
 */

/********************************************************************
 * @fn				- RB_I2C_MasterTX
 *
 * @brief			- Master Transfer Data to slave (Poll mode)
 *
 * @param[in]		- a pointer on the I2C Handler
 * @param[in]		- The Buffer which will contain the Transferred data
 * @param[in]       - The length of the message
 * @param[in]		- Slave Address to send to
 *
 * @return 			- NONE
 * @note			- This API is a blocking call
 */
void RB_I2C_MasterTX(I2Cx_Handler_t *pI2CHandle,uint8_t* pTxBuffer, uint32_t length, uint8_t SlaveAddr,uint8_t Sr){

	//Generate START Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

	//Check SB Flag (Check if Start Generation is completed)
	// Note : SCL Will be stretched (pulled to low) until SB is cleared
	while(! RB_I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_SB) );

	//Send the Addr of the slave with R/NW
	I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, SlaveAddr);

	//Check if Addr is sent by chekcing the ADDR Flag
	// Note : SCL Will be stretched (pulled to low) until ADDR Bit is cleared
	while(! RB_I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_ADDR) );
	I2C_ClearADDRStatus(pI2CHandle); //Clear Addr Status

	//Send data until length = 0
	while (length > 0){
		while(! RB_I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_TxE) ); //Wait until The peripheral is ready to accept a new byte to TX
		pI2CHandle->pI2Cx->DR = *pTxBuffer;
		pTxBuffer++;
		length--;
	}

	//Wait for TXE = 1 and BTF = 1 Before generating the stop condition
	// NOTE : When BTF = 1 SCL will be stretched to low
	while(! RB_I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_TxE) ); //TXE = 1 if DR Empty
	while(! RB_I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_BTF) ); //BTF = 1 if both DR and Shift R are empty
	if(Sr == I2C_SR_DISABLE)
		 I2C_GenerateStopCondition(pI2CHandle->pI2Cx);//Generate Stop

}


/********************************************************************
 * @fn				- RB_I2C_Data_RX
 *
 * @brief			- Receive Data (Poll Mode)
 *
 * @param[in]		- a pointer on the I2C Structure
 * @param[in]		- The Buffer which will contain the received data
 * @param[in]       - The length of the message
 *
 * @return 			- NONE
 * @note			- This API is also a blocking call
 */
void RB_I2C_MasterRX(I2Cx_Handler_t *pI2CHandle,uint8_t* pRxBuffer, uint32_t length, uint8_t SlaveAddr,uint8_t Sr){
	//START Condition
	I2C_GenerateStartCondition(pI2CHandle->pI2Cx);
	//Check SB Flag (Check if Start Generation is completed)
	// Note : SCL Will be stretched (pulled to low) until SB is cleared
	while(! RB_I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_SB) );

	//Send the Addr of the slave with R/NW
	I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, SlaveAddr);

	//Check if Addr is sent by chekcing the ADDR Flag
	while(! RB_I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_ADDR) );

	if (length == 1)
	{
		//Disable Acking
		RB_I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);
		//Clear the ADDR Flag
		I2C_ClearADDRStatus(pI2CHandle);
		//Wait until RXNE = 1 (RX DR is Full with the sent byte)
		while(! RB_I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_RxNE) );
		//Generate Stop Condition
		if (Sr == I2C_SR_DISABLE)
			I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
		//Read data into buffer
		*pRxBuffer = pI2CHandle->pI2Cx->DR;
	}


	if (length > 1)
	{
		//Clear ADDR Flag
		I2C_ClearADDRStatus(pI2CHandle);
		//Read data until Length = 0
		for (uint32_t i = length; i > 0 ; i--)
		{
			//Wait until RXNE = 1
			while(! RB_I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_SR1_RxNE) );
			if (i == 2)
			{
				//Disable Acking
				RB_I2C_ManageAcking(pI2CHandle->pI2Cx,DISABLE);
				//Generate Stop Condition
				if (Sr == I2C_SR_DISABLE)
					I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
			}
			//Read Data from DR and increment Buffer
			*pRxBuffer = pI2CHandle->pI2Cx->DR;
			pRxBuffer++;
		}
	}
	//After Transmission , Renable the ACK for future operations
	if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
	{
		RB_I2C_ManageAcking(pI2CHandle->pI2Cx,ENABLE);
	}
}

/*
 * I2C Data TX and RX (Interruption mode)
 */

/********************************************************************
 * @fn				- RB_I2C_MasterTX_IT
 *
 * @brief			- Transfer Data (Interrupt mode)
 *
 * @param[in]		- a pointer on the I2C Handle Structure
 * @param[in]		- The Buffer which will contain the Transferred data
 * @param[in]       - The length of the message
 * @param[in]		- The Address of the slave
 *
 * @return 			- Return the state of Tx
 * @note			- This API is a non-blocking call
 */
uint8_t RB_I2C_MasterTX_IT(I2Cx_Handler_t *pI2CHandle,uint8_t* pTxBuffer, uint32_t length, uint8_t SlaveAddr, uint8_t Sr){

	uint8_t state = pI2CHandle->TxRxState;

	if( (state != I2C_BUSY_RX) && (state != I2C_BUSY_TX) ) //I2C is Half Duplex
	{
		pI2CHandle->pTxBuffer = pTxBuffer;
		pI2CHandle->TxLen = length;
		pI2CHandle->TxRxState = I2C_BUSY_TX;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Generate Start condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Enable ITBUFEN
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Enable ITEVFEN
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Enable ITERREN
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
	}
	return state;
}

/********************************************************************
 * @fn				- RB_I2C_MasterRX_IT
 *
 * @brief			- Receive Data (Interrupt Mode)
 *
 * @param[in]		- a pointer on the I2C Handle Structure
 * @param[in]		- The Buffer which will contain the received data
 * @param[in]       - The length of the message
 * @param[in]		- The Address of the slave
 *
 * @return 			- Return the state of Rx
 * @note			- This API is also a non-blocking call
 */
uint8_t RB_I2C_MasterRX_IT(I2Cx_Handler_t *pI2CHandle,uint8_t* pRxBuffer, uint32_t length, uint8_t SlaveAddr,uint8_t Sr){

	uint8_t busystate = pI2CHandle->TxRxState;

	if( (busystate != I2C_BUSY_TX) && (busystate != I2C_BUSY_RX))
	{
		pI2CHandle->pTxBuffer = pRxBuffer;
		pI2CHandle->RxLen = length;
		pI2CHandle->TxRxState = I2C_BUSY_RX;
		pI2CHandle->RxSize = length;
		pI2CHandle->DevAddr = SlaveAddr;
		pI2CHandle->Sr = Sr;

		//Generate Start Condition
		I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

		//Enable ITBUFEN
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

		//Enable ITEVFEN
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

		//Enable ITERREN
		pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);

	}
	return busystate;
}


/*
 * I2C IRQ Configuration an ISR handling
 */

/********************************************************************
 * @fn				- RB_I2C_IRQITConfig
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
void RB_I2C_IRQITConfig(uint8_t IRQNumber, uint8_t state){
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
 * @fn				- RB_I2C_IRQPriorityConfig
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
void RB_I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority){
	// Finding out the IPRx Register
	uint8_t iprx = IRQNumber / 4 ;
	uint8_t iprx_section = IRQNumber % 4 ;
	uint8_t shift = (8 * iprx_section) + (8 - IMPLEMENTED_PRIORITY_BITS);
	*( NVIC_IPR + iprx ) |= (Priority << shift) ;
}

/*********************************************************************
 * @fn      		  - RB_I2C_EV_IRQHandling
 *
 * @brief             -	IRQ Handling for I2C Events
 *
 * @param[in]         - I2C Peripheral

 * @return            -	NONE
 *
 * @Note              - Interrupt handling for I2C Events

 */
void RB_I2C_EV_IRQHandling(I2Cx_Handler_t *pI2CHandle)
{

	//Refer to RM0368 Page 491 for the interrupts requests
	uint32_t temp1,temp2,temp3;
	temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
	temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);
	temp3 = RB_I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB);

	//SB (Start bit) Event (Only applicable in Master Mode , It's always 0 in slave)

	if (temp3 && temp1) //Handle interrupt generated by SB(Start Bit) event (Only Applicable in Master Mode)
	{
		//SB Event is set
		if(pI2CHandle->TxRxState == I2C_BUSY_TX)
		{
			I2C_ExecuteAddressPhaseWrite(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);

		}else if(pI2CHandle->TxRxState == I2C_BUSY_RX)
		{
			I2C_ExecuteAddressPhaseRead(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
		}
	}

	temp3 = RB_I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR); //Check for ADDR Flag
	//ADDR Event (MM : When Address is sent , SM : When Address is matched)
	if (temp3 && temp1) //Interruption generated from ADDR EV
	{
		I2C_ClearADDRStatus(pI2CHandle);
	}

	temp3 = RB_I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF);
	//BTF(Byte Transfer Finished) event
	if (temp1 && temp3)
	{
	    // Check if the I2C is busy in transmission
	    if (pI2CHandle->TxRxState == I2C_BUSY_TX)
	    {
	        // Ensure TXE is set
	        if (RB_I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TxE))
	        {
	            // If BTF and TXE are set and no more data to send
	            if (pI2CHandle->TxLen == 0)
	            {
	                // Generate STOP condition if repeated start SR is disabled
	                if (pI2CHandle->Sr == I2C_SR_DISABLE)
	                {
	                    I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
	                }

	                // Reset the handle structure (Close for next Transmissions)
	                I2C_CloseTX(pI2CHandle);

	                // Notify the application about transmission completion
	                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_CMPLT);
	            }
	        }
	    }
	    else if (pI2CHandle->TxRxState == I2C_BUSY_RX)
	    {
	        // Add handling for receive state if needed in future
	    }
	}

	temp3 = RB_I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_STOPF);
	if (temp3 && temp1) // STOPF event (Applicable only for slave)
	{
		//STOPF is set
		//Need to be cleared
		pI2CHandle->pI2Cx->CR1 |= 0x0; // To clear the STOPF (A simple read to
		I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_STOP);
	}

	temp3 = RB_I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TxE);
	// TXE event (TX Buffer is empty) (Applicable for Master)
	if (temp1 && temp2 && temp3)
	{
		if(RB_I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_MSL)) //Check for Master Mode
		{
			//Data TX
			if(pI2CHandle->TxRxState == I2C_BUSY_TX)
			{
				I2C_MasterHandleTXEInterrupt(pI2CHandle);
			}
		}else
		{
			//Device in Slave Mode
			if(RB_I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TRA)) //Check if Slave in TX mode
			{
				I2C_ApplicationEventCallback(pI2CHandle,I2C_EV_DATA_REQ);
			}
		}
	}


    temp3 = RB_I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RxNE);

    // RXNE event (RX Buffer is Full)
    if (temp1 && temp2 && temp3)
    {
        if (RB_I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_MSL))  // Check if in Master Mode
        {
            if (pI2CHandle->TxRxState == I2C_BUSY_RX)
            {
                I2C_MasterHandleRXNEInterrupt(pI2CHandle);
            }
        }
        else  // Slave Mode
        {
            if (!RB_I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TRA)) // Check if receiving
            {
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
            }
        }
    }

}


/*********************************************************************
  * @fn      		   -RB_I2C_ER_IRQHandling
  *
  * @brief             -IRQ Handling for I2C Events
  *
  * @param[in]         -I2C Peripheral
  *
  * @return            -NONE
  *
  * @Note              -Interrupt handling for I2C Errors
  */

 void RB_I2C_ER_IRQHandling(I2Cx_Handler_t *pI2CHandle)
 {

     uint32_t temp1,temp2;

     //Know the status of  ITERREN control bit in the CR2
     temp2 = (pI2CHandle->pI2Cx->CR2) & ( 1 << I2C_CR2_ITERREN);



     temp1 = RB_I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BERR);
     if(temp1  && temp2 )
     {
         //BUS ERROR

         //CLEAR BUS ERROR
         pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_BERR);

         //Notify the application about a bus error
        I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_BERR);
     }

     temp1 = RB_I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ARLO);
     if(temp1  && temp2)
     {
         //ARBITRATION LOST ERROR

         //CLEAR ARLO ERROR
         pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_ARLO);

         //Notify the application about a arbitration lost error
         I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_ARLO);

     }

     temp1 = RB_I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_AF);
     if(temp1  && temp2)
     {
         //ACK FAILURE ERROR

    	 //CLEAR ACK ERROR
         pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_AF);

         //Implement the code to notify the application about the error
         I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_AF);
     }

     temp1 = (pI2CHandle->pI2Cx->SR1) & ( 1 << I2C_SR1_OVR);
     if(temp1  && temp2)
     {
         //OVR ERROR

         //CLEAR OVR ERROR
         pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_OVR);

         //Notify the application about the OVR Error
         I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_OVR);
     }

     temp1 = RB_I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TIMEOUT);
     if(temp1  && temp2)
     {
         //TIMEOUT ERROR

         //CLEAR TIMEOUT ERROR
         pI2CHandle->pI2Cx->SR1 &= ~( 1 << I2C_SR1_TIMEOUT);

         //Notify the application about the timeout error
         I2C_ApplicationEventCallback(pI2CHandle,I2C_ERROR_TIMEOUT);
     }

 }

/*
 * Others APIs
 */

/********************************************************************
 * @fn				- RB_I2C_PeriphControl
 *
 * @brief			- Enable/Disable the I2C Peripheral
 *
 * @param[in]		- Pointer on I2C Peripheral
 * @param[in]		- State (ENABLE or DISABLE)
 *
 * @return 			- NONE
 *
 * @note			- The I2C Peripheral is Disabled by default to allow control configuration on the I2C Peripheral
 */
void RB_I2C_PeriphControl(I2Cx_t *pI2Cx,uint8_t state){
	if (state == ENABLE)
	{
		pI2Cx->CR1 |= (1 << I2C_CR1_PE);
	}else
	{
		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
	}
}

/********************************************************************
 * @fn				- RB_I2C_GetFlagStatus
 *
 * @brief			- Return the status of specific flag
 *
 * @param[in]		- Pointer on I2C Peripheral
 * @param[in]		- Desired Flag (Uses Macros in @I2C_Flags)
 *
 * @return 			- Status of the flag (FLAG_SET or FLAG_RESET)
 *
 * @note			- NONE
 */
uint8_t RB_I2C_GetFlagStatus(I2Cx_t *pI2Cx, uint8_t flag) {
	if (flag < 20) {
		return (pI2Cx->SR1 & (1 << flag)) ? FLAG_SET : FLAG_RESET;
	} else
	{
		return (pI2Cx->SR2 & (1<<(flag - 20))) ? FLAG_SET : FLAG_RESET;
	}
}

/********************************************************************
 * @fn				- RB_I2C_ManageAcking
 *
 * @brief			- Enable or Disable Acking
 *
 * @param[in]		- Pointer on I2C Peripheral
 * @param[in]		- Desired Status (ENABLE or DISABLE)
 *
 * @return 			- NONE
 *
 * @note			- NONE
 */
void RB_I2C_ManageAcking(I2Cx_t *pI2Cx,uint8_t status){
	if (status == DISABLE)
	{
		//Disbale Acking
		pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
	}else
	{
		//Enable Acking
		pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
	}
}

/*
 * Call-Back Function
 */
__weak void I2C_ApplicationEventCallback(I2Cx_Handler_t *pI2CHandle,uint8_t AppEv){

}




