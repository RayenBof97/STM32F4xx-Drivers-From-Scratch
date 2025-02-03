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
				I2C1_PCLK_EN();
			} else if (pI2Cx == I2C2) {
				I2C2_PCLK_EN();
			} else if (pI2Cx == I2C3) {
				I2C3_PCLK_EN();
			}
	}
}
