/*
 *
  ******************************************************************************
  * @file    stm32f401xx_i2c.h
  * @author  Rayen	Bouafif
  * @mail    bouafifrayen01@gmail.com
  * @Tel     (+216)29 049 373
  * @date    03-02-2025
  *****************************************************************************
*
*/

#ifndef INC_STM32F401XX_I2C_H_
#define INC_STM32F401XX_I2C_H_
#include "stm32f401xx.h"

typedef struct{

}I2Cx_Config_t;

typedef struct{
	I2Cx_t *pI2Cx;									/*!<Base Address of the SPI Peripheral*/
	I2Cx_Config_t I2C_Config;						/*!<Structure of SPI Configuration*/
	uint8_t *pTxBuffer;								/*!<Storing the Application Tx Buffer */
	uint8_t *pRxBuffer;								/*!<Storing the Application Rx Buffer*/
	uint32_t TxLen;									/*!<Length of the app Tx Buffer*/
	uint32_t RxLen;									/*!<Length of the app Rx Buffer*/
	uint8_t TxState;								/*!<Sotring the state of Tx*/
	uint8_t RxState;								/*!<Storing the state of Rx*/
}I2Cx_Handler_t;




/**************************************************************************************************
 * 									APIs Supported by this driver
 *					For further informations, please check the functions definition
 **************************************************************************************************/
/*
 * I2C Peripheral Clock Control
 */
void RB_I2C_PeriClockControl(I2Cx_t *pI2Cx, uint8_t State);

/*
 * Init and De-init I2Cx
 */
void RB_I2C_Init(I2Cx_Handler_t *pI2CHandle);
void RB_I2C_DeInit(I2Cx_t *pI2Cx);

/*
 * I2C Data TX and RX (Poll mode)
 */
void RB_I2C_Data_TX(I2Cx_t *pI2Cx,uint8_t *pTxBuffer,uint32_t len);
void RB_I2C_Data_RX(I2Cx_t *pI2Cx,uint8_t *pRxBuffer,uint32_t len);

/*
 * I2C Data TX and RX (Interruption mode)
 */
uint8_t RB_I2C_Data_TXIT(I2Cx_Handler_t *pI2CHandle,uint8_t *pTxBuffer,uint32_t len);
uint8_t RB_I2C_Data_RXIT(I2Cx_Handler_t *pI2CHandle,uint8_t *pRxBuffer,uint32_t len);

/*
 * I2C IRQ Configuration an ISR handling
 */
void RB_I2C_IRQITConfig(uint8_t IRQNumber, uint8_t state);
void RB_I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority);
void RB_I2C_IRQHandling(I2Cx_Handler_t *pSPIHandle);

/*
 * Application Callback Functions
 */
void I2C_ApplicationEventCallback(I2Cx_Handler_t *pI2CHandle,uint8_t event);
/*
 * Others APIs
 */
void I2C_PeriphControl(I2Cx_t *pI2Cx,uint8_t state);
uint8_t RB_I2C_GetFlagStatus(I2Cx_t *pI2Cx,uint8_t flag);
void RB_I2C_ClearOVRFlag(I2Cx_t *pI2Cx);
void RB_I2C_CloseTx(I2Cx_Handler_t *pI2CHandle);
void RB_I2C_CloseRx(I2Cx_Handler_t *pI2CHandle);





#endif /* INC_STM32F401XX_I2C_H_ */
