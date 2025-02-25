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
	uint32_t I2C_SCLSpeed;							/*!<Value from @SCL_Speed*/
	uint8_t I2C_DeviceAddress;						/*!<Stores Address of Device*/
	uint8_t I2C_ACKControl;							/*!<Value from @ACK_Control*/
	uint16_t I2C_FMDutyCycle;						/*!<Value from @Duty_Cycle*/
}I2Cx_Config_t;

typedef struct{
	I2Cx_t *pI2Cx;									/*!<Base Address of the SPI Peripheral*/
	I2Cx_Config_t I2C_Config;						/*!<Structure of SPI Configuration*/
	uint8_t *pTxBuffer;								/*!<Store the Adress of the Application TxBuffer*/
	uint8_t *pRxBuffer;								/*!<Store the Adress of the Application RxBuffer*/
	uint32_t TxLen;									/*!<Store the Tx Length*/
	uint32_t RxLen;									/*!<Store the Rx Length*/
	uint8_t TxRxState;								/*!<Store the communication state*/
	uint8_t DevAddr;								/*!<To store Slave device Address*/
	uint32_t RxSize;								/*!<Store the Rx Size*/
	uint8_t Sr;										/*!<Store the repeated start value, Values from @SR_Values*/
}I2Cx_Handler_t;

/*
 * I2C Application State
 */
#define I2C_READY 				0
#define I2C_BUSY_TX				1
#define I2C_BUSY_RX				2

/*@SCL_Speed*/
#define I2C_SCL_SPEED_STANDARD 100000
#define I2C_SCL_SPEED_FAST200K 200000
#define I2C_SCL_SPEED_FAST400K 400000

/*@ACK_Control*/
#define I2C_ACK_DISABLE 		0
#define I2C_ACK_ENABLE			1

/*@Duty_Cycle*/
#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1

/*@SR_Values*/
#define I2C_SR_DISABLE			0
#define I2C_SR_ENABLE			1


/*
 * I2C Event and Errors Macros
 */

#define I2C_EV_TX_CMPLT  	 	0
#define I2C_EV_RX_CMPLT  	 	1
#define I2C_EV_STOP       		2
#define I2C_EV_DATA_REQ         3
#define I2C_EV_DATA_RCV         4
#define I2C_ERROR_BERR 	 		5
#define I2C_ERROR_ARLO  		6
#define I2C_ERROR_AF    		7
#define I2C_ERROR_OVR   		8
#define I2C_ERROR_TIMEOUT 		9


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
 * I2C Data TX and RX
 */
void RB_I2C_MasterTX(I2Cx_Handler_t *pI2CHandle,uint8_t* pTxBuffer, uint32_t length, uint8_t SlaveAddr,uint8_t Sr);
void RB_I2C_MasterRX(I2Cx_Handler_t *pI2CHandle,uint8_t* pRxBuffer, uint32_t length, uint8_t SlaveAddr,uint8_t Sr);

uint8_t RB_I2C_MasterTX_IT(I2Cx_Handler_t *pI2CHandle,uint8_t* pTxBuffer, uint32_t length, uint8_t SlaveAddr,uint8_t Sr);
uint8_t RB_I2C_MasterRX_IT(I2Cx_Handler_t *pI2CHandle,uint8_t* pRxBuffer, uint32_t length, uint8_t SlaveAddr,uint8_t Sr);
/*
 * I2C IRQ Configuration an ISR handling
 */
void RB_I2C_IRQITConfig(uint8_t IRQNumber, uint8_t state);
void RB_I2C_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority);
void RB_I2C_EV_IRQHandling(I2Cx_Handler_t *pI2CHandle);
void RB_I2C_ER_IRQHandling(I2Cx_Handler_t *pI2CHandle);

/*
 * Others APIs
 */
void RB_I2C_PeriphControl(I2Cx_t *pI2Cx,uint8_t state);
uint8_t RB_I2C_GetFlagStatus(I2Cx_t *pI2Cx,uint8_t flag);
void RB_I2C_ManageAcking(I2Cx_t *pI2Cx,uint8_t status);

/*
 * Application Callback Functions
 */
void I2C_ApplicationEventCallback(I2Cx_Handler_t *pI2CHandle,uint8_t event);


#endif /* INC_STM32F401XX_I2C_H_ */
