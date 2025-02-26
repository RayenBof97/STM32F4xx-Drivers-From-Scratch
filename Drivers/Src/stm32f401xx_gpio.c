/*
 *
  ******************************************************************************
  * @file    stm32f401xx_gpio.c
  * @author  Rayen	Bouafif
  * @mail    bouafifrayen01@gmail.com
  * @Tel     (+216)29 049 373
  * @date    09-11-2024
  *****************************************************************************
* This File Also contain the APIs explanation in the comments
*/



#include "stm32f401xx_gpio.h"


/*
 * GPIO Peripheral Clock Control
 */

/********************************************************************
 * @fn				- RB_GPIO_PeriClockControl
 *
 * @brief			- This function enable or disable the GPIO peripheral clock
 *
 * @param[in]		- Pointer to GPIO structure
 * @param[in]		- ENABLE DISABLE Macros (which are an integer 0 or 1)
 *
 * @return 			- NONE
 *
 * @note			- NONE
 */
void RB_GPIO_PeriClockControl(GPIOx_t *pGPIOx, uint8_t State){
	if (State == ENABLE) {
			if (pGPIOx == GPIOA) {
				GPIOA_PCLK_EN();
			} else if (pGPIOx == GPIOB) {
				GPIOB_PCLK_EN();
			} else if (pGPIOx == GPIOC) {
				GPIOC_PCLK_EN();
			} else if (pGPIOx == GPIOD) {
				GPIOD_PCLK_EN();
			} else if (pGPIOx == GPIOE) {
				GPIOE_PCLK_EN();
			} else if (pGPIOx == GPIOH) {
				GPIOH_PCLK_EN();
			}
		} else {  // DISABLE state
			if (pGPIOx == GPIOA) {
				GPIOA_PCLK_DI();
			} else if (pGPIOx == GPIOB) {
				GPIOB_PCLK_DI();
			} else if (pGPIOx == GPIOC) {
				GPIOC_PCLK_DI();
			} else if (pGPIOx == GPIOD) {
				GPIOD_PCLK_DI();
			} else if (pGPIOx == GPIOE) {
				GPIOE_PCLK_DI();
			} else if (pGPIOx == GPIOH) {
				GPIOH_PCLK_DI();
			}
		}
	}


/*
 * Init and De-init GPIO
 */

/********************************************************************
 * @fn				- RB_GPIO_Init
 *
 * @brief			- initialize a specific GPIO pin
 *
 * @param[in]		- a pointer to a structure of type GPIOx_Handler_t that holds configuration of the pin
 *
 * @return 			- NONE
 *
 * @note			- NONE
 */
void RB_GPIO_Init(GPIOx_Handler_t *pGPIOHandle){
	uint32_t temp = 0 ; //Temporary register

	//Enable the Clock
	RB_GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//Initialise Mode of GPIO
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Clearing Field (Setting the specific field to 0)
	pGPIOHandle->pGPIOx->MODER |= temp; //Setting Value
	}else{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// Configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clearing the RTSR
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// Configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clearing the FTSR
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			//Configure both FTSR and RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		//Select GPIO in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t code = GPIO_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = code << (4 * temp2);
		//Enable the Interrupt Mask
		EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	temp = 0;
	//Initialise the speed of GPIO
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;
	//Configure PUPD Settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
 	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp= 0;
	//Configure the Output type
	temp = (pGPIOHandle->GPIO_PinConfig.PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	temp = 0;
	//Configure the Alternate Function Settings
	if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTERNATE){
		//Configure the alternate function
		uint8_t temp1 = 0;
		uint8_t temp2 = 0;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= pGPIOHandle->GPIO_PinConfig.GPIO_pinAltFuncMode << (4 * temp2);

	}
}


/********************************************************************
 * @fn				- RB_GPIO_DeInit
 *
 * @brief			- Deinitialize a specific GPIO port (Reset all registers)
 *
 * @param[in]		- a pointer to a GPIO Structure
 *
 * @return 			- NONE
 *
 * @note			- NONE
 */

void RB_GPIO_DeInit(GPIOx_t *pGPIOx){

	if (pGPIOx == GPIOA) {
	    GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB) {
	    GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC) {
	    GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD) {
	    GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE) {
	    GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOH) {
	    GPIOH_REG_RESET();
	}
}


/*
 * Data Read and Write
 */

/********************************************************************
 * @fn				- RB_GPIO_ReadInputPin
 *
 * @brief			- Read the state of a single pin in GPIOx
 *
 * @param[in]		- a pointer to a GPIO Structure
 * @param[in]		- the Pin number in the port
 *
 * @return 			- the state of the pin (0 or 1)
 *
 * @note			- NONE
 */
uint8_t RB_GPIO_ReadInputPin(GPIOx_t *pGPIOx, uint8_t PinNumber){
	uint8_t temp;
	temp = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return temp;
}

/********************************************************************
 * @fn				- RB_GPIO_ReadInputPort
 *
 * @brief			- Read the state of the whole output
 *
 * @param[in]		- a pointer to a GPIO Structure
 *
 * @return 			- the state of all pins in portX
 *
 * @note			- NONE
 */
uint16_t RB_GPIO_ReadInputPort(GPIOx_t *pGPIOx){
	uint16_t temp;
	temp = (uint16_t)pGPIOx->IDR;
	return temp;
}

/********************************************************************
 * @fn				- RB_GPIO_WriteOutputPin
 *
 * @brief			- Write a state into a GPIO Pin
 *
 * @param[in]		- a pointer to a GPIO Structure
 * @param[in]		- Pin Number (Can use PIN Macros)
 * @param[in]		- value of the state ( 0 or 1)
 *
 * @return 			- NONE
 *
 * @note			- NONE
 */
void RB_GPIO_WriteOutputPin(GPIOx_t *pGPIOx, uint8_t PinNumber, uint8_t value){
	if(value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}

}

/********************************************************************
 * @fn				- RB_GPIO_WriteOutputPort
 *
 * @brief			- Write a value into a GPIO Port
 *
 * @param[in]		- a pointer to a GPIO Structure
 * @param[in]		- value of the state (Ranging from 0x0000 to 0xFFFF)
 *
 * @return 			- NONE
 *
 * @note			- NONE
 */
void RB_GPIO_WriteOutputPort(GPIOx_t *pGPIOx, uint16_t value) {

    pGPIOx->ODR = (pGPIOx->ODR & 0xFFFF0000) | (value & 0xFFFF);
}


/********************************************************************
 * @fn				- RB_GPIO_TogglePin
 *
 * @brief			- Switch the state of a GPIO Pin
 *
 * @param[in]		- a pointer to a GPIO Structure
 * @param[in]		- Pin Number (Can use PIN Macros)
 *
 * @return 			- NONE
 *
 * @note			- NONE
 */
void RB_GPIO_TogglePin(GPIOx_t *pGPIOx, uint8_t PinNumber){
	pGPIOx->ODR ^= (1 << PinNumber);
}


/*
 * Interrupt routine
 */
/********************************************************************
 * @fn				- RB_GPIO_IRQITConfig
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
void RB_GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t state){
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
 * @fn				- RB_GPIO_IRQPriorityConfig
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
void RB_GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority){
	// Finding out the IPRx Register
	uint8_t iprx = IRQNumber / 4 ;
	uint8_t iprx_section = IRQNumber % 4 ;
	uint8_t shift = (8 * iprx_section) + (8 - IMPLEMENTED_PRIORITY_BITS);
	*( NVIC_IPR + iprx ) |= (Priority << shift) ;
}



/********************************************************************
 * @fn				- RB_GPIO_IRQHandling
 *
 * @brief			- Handling the IRQ on GPIO Peripherals
 *
 * @param[in]		- Pin Number
 *
 * @return 			- NONE
 *
 * @note			- NONE
 */

void RB_GPIO_IRQHandling(uint8_t PinNumber){
	//Clearing the EXTI Pending register if it's SET
	if ( EXTI->PR & (1 << PinNumber) )
	{
		EXTI->PR |= (1 << PinNumber);
	}
}


