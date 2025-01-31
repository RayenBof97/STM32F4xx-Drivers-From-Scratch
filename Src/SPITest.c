/*
 * SPITest.c
 *
 *  Created on: Jan 30, 2025
 *      Author: Rayen Bouafif
 */

#include "stm32f401xx.h"
#include "string.h"

void SPI2_GPIOInit(void){

	GPIOx_Handler_t spi_gpio;
	memset(&spi_gpio,0,sizeof(spi_gpio));

	spi_gpio.pGPIOx = GPIOC;
	spi_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTERNATE;
	spi_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;
	spi_gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	spi_gpio.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_NO_PUPD;
	spi_gpio.GPIO_PinConfig.PinOPType =  GPIO_OUT_TYPE_PP;
	spi_gpio.GPIO_PinConfig.GPIO_pinAltFuncMode = GPIO_AF_5;
	//Init MOSI
	RB_GPIO_Init(&spi_gpio);

	//Init SCLK
	spi_gpio.pGPIOx = GPIOB;
	spi_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_10;

	RB_GPIO_Init(&spi_gpio);
/*   REMOVE THE COMMENT IF YOU ARE USING THESE PINS
	//Init MISO
	spi_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;

	RB_GPIO_Init(spi_gpio);

	//Init NSS
	spi_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_9;

	RB_GPIO_Init(spi_gpio);
*/
}

void SPI2_Init(void){
	SPIx_Handler_t	spi2;
	spi2.pSPIx = SPI2;
	spi2.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	spi2.SPI_Config.SPI_DeviceMode = SPI_MODE_MASTER;
	spi2.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	spi2.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
    spi2.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
    spi2.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
    spi2.SPI_Config.SPI_SSM = SPI_SM_SOFTWARE;

    RB_SPI_Init(&spi2);

}

int main(){
	char user_data[] = "Hello World";

	//Initialisation
	SPI2_GPIOInit();
	SPI2_Init();
	SPI_PeriphControl(SPI2,ENABLE);
	//Send Data
	RB_SPI_Data_TX(SPI2,(uint8_t*)user_data,strlen(user_data));


	while(1);


return 0;
}


