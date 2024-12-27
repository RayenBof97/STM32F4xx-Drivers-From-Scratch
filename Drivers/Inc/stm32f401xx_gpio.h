/**
  ******************************************************************************
  * @file    stm32f401xx.h
  * @author  Rayen	Bouafif
  * @mail    bouafifrayen01@gmail.com
  * @Tel     (+216)29 049 373
  * @date    08-11-2024
  *****************************************************************************
*
*/

#ifndef INC_STM32F401XX_GPIO_H_
#define INC_STM32F401XX_GPIO_H_
#include "stm32f401xx.h"

typedef struct{
	uint8_t GPIO_PinNumber;					/*!<Value from @GPIO_Pins*/
	uint8_t GPIO_PinMode;					/*!<Value from @GPIO_Modes*/
	uint8_t GPIO_PinSpeed;					/*!<Value from @GPIO_Speed_Mode*/
	uint8_t GPIO_PinPuPdControl;			/*!<Value from @GPIO_PU_PD_Modes*/
	uint8_t PinOPType;						/*!<Value from @GPIO_Output_type*/
	uint8_t GPIO_pinAltFuncMode;			/*!<Value from @GPIO_AF*/
}GPIOx_PinConfig_t;

typedef struct{
	GPIOx_t *pGPIOx;
	GPIOx_PinConfig_t GPIO_PinConfig;
}GPIOx_Handler_t;

/*
 * @GPIO_Pins
 */
#define GPIO_PIN_0				0
#define GPIO_PIN_1				1
#define GPIO_PIN_2				2
#define GPIO_PIN_3				3
#define GPIO_PIN_4				4
#define GPIO_PIN_5				5
#define GPIO_PIN_6				6
#define GPIO_PIN_7				7
#define GPIO_PIN_8				8
#define GPIO_PIN_9				9
#define GPIO_PIN_10				10
#define GPIO_PIN_11				11
#define GPIO_PIN_12				12
#define GPIO_PIN_13				13
#define GPIO_PIN_14				14
#define GPIO_PIN_15				15

/*
 * @GPIO_Modes
 */
#define GPIO_MODE_INPUT			0
#define GPIO_MODE_OUTPUT		1
#define GPIO_MODE_ALTERNATE		2
#define GPIO_MODE_ANALOG		3
#define GPIO_MODE_IT_FT			4
#define GPIO_MODE_IT_RT			5
#define GPIO_MODE_IT_RFT		6

/*
 * @GPIO_Speed_Mode
 */
#define GPIO_SPEED_LOW			0
#define GPIO_SPEED_MEDIUM		1
#define GPIO_SPEED_FAST			2
#define GPIO_SPEED_HIGH			3

/*
 * @GPIO_PU_PD_Modes
 * GPIO Pullup and Pulldown configuration
 */
#define GPIO_NO_PUPD			0
#define GPIO_PIN_PU				1
#define GPIO_PIN_PD				2

/*
 * @GPIO_Output_type
 */
#define GPIO_OUT_TYPE_PP		0
#define GPIO_OUT_TYPE_OD		1

/*
 * @GPIO_AF
 * Selection of the Alternate function
 */
#define GPIO_AF_0				0
#define GPIO_AF_1				1
#define GPIO_AF_2				2
#define GPIO_AF_3				3
#define GPIO_AF_4				4
#define GPIO_AF_5				5
#define GPIO_AF_6				6
#define GPIO_AF_7				7
#define GPIO_AF_8				8
#define GPIO_AF_9				9
#define GPIO_AF_10				10
#define GPIO_AF_11				11
#define GPIO_AF_12				12
#define GPIO_AF_13				13
#define GPIO_AF_14				14
#define GPIO_AF_15				15

/**************************************************************************************************
 * 									APIs Supported by this driver
 *					For further informations, please check the functions definition
 **************************************************************************************************/
/*
 * Peripheral Clock Control
 */
void RB_GPIO_PeriClockControl(GPIOx_t *pGPIOx, uint8_t State);

/*
 * Init and De-init GPIO
 */
void RB_GPIO_Init(GPIOx_Handler_t *pGPIOHandle);
void RB_GPIO_DeInit(GPIOx_t *pGPIOx);

/*
 * Data Read and Write
 */
uint8_t RB_GPIO_ReadInputPin(GPIOx_t *pGPIOx, uint8_t PinNumber);
uint16_t RB_GPIO_ReadInputPort(GPIOx_t *pGPIOx);
void RB_GPIO_WriteOutputPin(GPIOx_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void RB_GPIO_WriteOutputPort(GPIOx_t *pGPIOx,uint16_t value);
void RB_GPIO_TogglePin(GPIOx_t *pGPIOx, uint8_t PinNumber);


/*
 * Interrupt routine
 */
void RB_GPIO_IRQITConfig(uint8_t IRQNumber, uint8_t state);
void RB_GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t Priority);
void RB_GPIO_IRQHandling(uint8_t PinNumber);










#endif /* INC_STM32F401XX_GPIO_H_ */
