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

	//Configuring ACK Bit in CR1 Register
	pI2CHandle->pI2Cx->CR1 |= pI2CHandle->I2C_Config.I2C_ACKControl << 10;

	//Configure FREQ in the CR2
	pI2CHandle->pI2Cx->CR2 &= ~(0x3F);
	pI2CHandle->pI2Cx->CR2 |= (RCC_GetPCLK1Value() / 1000000) & 0x3F;

	//Own Address
	pI2CHandle->pI2Cx->OAR1 &= ~(0xFE);
	pI2CHandle->pI2Cx->OAR1 |= (uint16_t)(pI2CHandle->I2C_Config.I2C_DeviceAddress << 1) | (1 << 14);

	//CCR Calculation
	uint16_t ccr = 0;
	if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_STANDARD)
	{
		//Standard Mode
		pI2CHandle->pI2Cx->CCR &= ~(1 << I2C_CCR_FS);
		ccr = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed) ) & 0xFFF;
		pI2CHandle->pI2Cx->CCR &= ~(0xFFF);
		pI2CHandle->pI2Cx->CCR |= ccr;


	}else
	{
		//Fast Mode
		pI2CHandle->pI2Cx->CCR |= (1 << I2C_CCR_FS) | (pI2CHandle->I2C_Config.I2C_FMDutyCycle << I2C_CCR_DUTY);

		if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == 0)
		{
			ccr = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed) ) & 0xFFF;
		}else
		{
			ccr = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed) ) & 0xFFF;
		}
		pI2CHandle->pI2Cx->CCR &= ~(0xFFF);
		pI2CHandle->pI2Cx->CCR |= ccr;
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
void RB_I2C_MasterTX(I2Cx_Handler_t *pI2CHandle,uint8_t* pTxBuffer, uint32_t length, uint8_t SlaveAddr){

	//START Condition
	pI2CHandle->pI2Cx->CR1 |= (1 << I2C_CR1_START);
	//Check SB Flag (Check if Start Generation is completed)

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
void RB_I2C_Data_RX(I2Cx_t *pI2Cx,uint8_t *pRxBuffer,uint32_t len){

}

/*
 * I2C Data TX and RX (Interruption mode)
 */

/********************************************************************
 * @fn				- RB_I2C_Data_TXIT
 *
 * @brief			- Transfer Data (Interrupt mode)
 *
 * @param[in]		- a pointer on the I2C Handle Structure
 * @param[in]		- The Buffer which will contain the Transferred data
 * @param[in]       - The length of the message
 *
 * @return 			- Return the state of Tx
 * @note			- This API is a non-blocking call
 */
uint8_t RB_I2C_Data_TXIT(I2Cx_Handler_t *pI2CHandle,uint8_t *pTxBuffer,uint32_t len){
	return 0;
}

/********************************************************************
 * @fn				- RB_I2C_Data_RXIT
 *
 * @brief			- Receive Data (Interrupt Mode)
 *
 * @param[in]		- a pointer on the I2C Handle Structure
 * @param[in]		- The Buffer which will contain the received data
 * @param[in]       - The length of the message
 *
 * @return 			- Return the state of Rx
 * @note			- This API is also a non-blocking call
 */
uint8_t RB_I2C_Data_RXIT(I2Cx_Handler_t *pI2CHandle,uint8_t *pRxBuffer,uint32_t len){
	return 0;
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

/*
 * Others APIs
 */

/********************************************************************
 * @fn				- I2C_PeriphControl
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
void I2C_PeriphControl(I2Cx_t *pI2Cx,uint8_t state){
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
 * @param[in]		- Desired Flag (Use the Bit position in the I2C_SR Register)
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






