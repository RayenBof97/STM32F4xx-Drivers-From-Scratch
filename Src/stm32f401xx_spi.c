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
 * @brief			- Transfer Data
 *
 * @param[in]		- a pointer on the SPI Structure
 * @param[in]		- The Buffer which will contain the Transferred data
 * @param[in]       - The length of the message
 *
 * @return 			- NONE
 * @note			- NONE
 */
void RB_SPI_Data_TX(SPIx_t *pSPIx,uint8_t *pTxBuffer,uint32_t len){

}


/********************************************************************
 * @fn				- RB_SPI_Data_RX
 *
 * @brief			- Receive Data
 *
 * @param[in]		- a pointer on the SPI Structure
 * @param[in]		- The Buffer which will contain the received data
 * @param[in]       - The length of the message
 *
 * @return 			- NONE
 * @note			- NONE
 */
void RB_SPI_Data_RX(SPIx_t *pSPIx,uint8_t *pRxBuffer,uint32_t len){

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

}
