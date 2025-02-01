/*
 * ArduinoTx.c
 *
 *  Created on: Feb 1, 2025
 *      Author: Rayen
 */

#include "stm32f401xx.h"
#include "string.h"


void delay(){
    int i = 0;
    for (i=0;i<200000;i++);
}

void SPI2_GPIOInit(void){

	GPIOx_Handler_t spi_gpio;
	memset(&spi_gpio,0,sizeof(spi_gpio));

	spi_gpio.pGPIOx = GPIOC;
	spi_gpio.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTERNATE;
	spi_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_3;
	spi_gpio.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	spi_gpio.GPIO_PinConfig.GPIO_PinPuPdControl= GPIO_PIN_PU;
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
*/
	//Init NSS
	spi_gpio.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_9;

	RB_GPIO_Init(&spi_gpio);

}

void SPI2_Init(void){
	SPIx_Handler_t	spi2;
	spi2.pSPIx = SPI2;
	spi2.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	spi2.SPI_Config.SPI_DeviceMode = SPI_MODE_MASTER;
	spi2.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; //2Mhz
	spi2.SPI_Config.SPI_DFF = SPI_DFF_8BITS;
    spi2.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
    spi2.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
    spi2.SPI_Config.SPI_SSM = SPI_SM_HARDWARE;

    RB_SPI_Init(&spi2);

}

void Button_GPIOInit(){
	GPIOx_Handler_t GPIOButton;
	GPIOButton.pGPIOx = GPIOC;
	GPIOButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_INPUT;
	GPIOButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIOButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GPIOButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	GPIOButton.GPIO_PinConfig.PinOPType = GPIO_OUT_TYPE_PP;

	RB_GPIO_Init(&GPIOButton);
}
int main(){
	char user_data[] = "Hello world";

	//Initialisation
	Button_GPIOInit();
	SPI2_GPIOInit();
	SPI2_Init();
	//Enable the NSS Output
	SPI_SSOE_Config(SPI2,ENABLE);
	while(1)
	{
		while(RB_GPIO_ReadInputPin(GPIOC, GPIO_PIN_13));
		delay();
		SPI_PeriphControl(SPI2,ENABLE);
		//Sending Data Length First to The Arduino
		uint8_t dataLength = strlen(user_data);
		RB_SPI_Data_TX(SPI2,&dataLength,1);
		//Send Data
		RB_SPI_Data_TX(SPI2,(uint8_t*)user_data,dataLength);

		//Disbale the peripheral
		while(RB_SPI_GetFlagStatus(SPI2,SPI_SR_BSY));
		SPI_PeriphControl(SPI2,DISABLE);
	}

return 0;
}


