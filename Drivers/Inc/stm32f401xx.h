/*
 * stm32f401xx.h
 *
 *  Created on: Sep 26, 2024
 *      Author: Rayen Bouafif
 *
 *  WARNING : I included the peripherals that I use most of the time and didn't include all of the existing periph
 *  I focused on GPIO , USART ,I2C , SPI Peripherals that i'll be testing after
 *
 */

#ifndef INC_STM32F401XX_H_
#define INC_STM32F401XX_H_

#include <stdint.h>


//General Define
#define __vo volatile
#define ENABLE 1
#define DISABLE 0
#define	SET ENABLE
#define RESET DISABLE
#define GPIO_PIN_SET SET
#define GPIO_PIN_RESET RESET

/*
 * IRQ (Interrupt Request) Numbers of STM32F401x MCU
 */
#define IRQ_NO_EXTI0								6
#define IRQ_NO_EXTI1								7
#define IRQ_NO_EXTI2								8
#define IRQ_NO_EXTI3								9
#define IRQ_NO_EXTI4								10
#define IRQ_NO_EXTI9_5								23
#define IRQ_NO_EXTI15_10							40

/*
 * IRQ Priority Levels
 */
#define IRQ_PRIO_NO0								0
#define IRQ_PRIO_NO1								1
#define IRQ_PRIO_NO2								2
#define IRQ_PRIO_NO3								3
#define IRQ_PRIO_NO4								4
#define IRQ_PRIO_NO5								5
#define IRQ_PRIO_NO6								6
#define IRQ_PRIO_NO7								7
#define IRQ_PRIO_NO8								8
#define IRQ_PRIO_NO9								9
#define IRQ_PRIO_NO10								10
#define IRQ_PRIO_NO11								11
#define IRQ_PRIO_NO12								12
#define IRQ_PRIO_NO13								13
#define IRQ_PRIO_NO14								14
#define IRQ_PRIO_NO15								15

/************************************** PROCESSOR SPECIFIC DETAILS ******************************************************/
/*
 * ARM Cortex M4 NVIC ISERx Register Addresses
 */
#define NVIC_ISER0								( (__vo uint32_t*)0xE000E100 )
#define NVIC_ISER1								( (__vo uint32_t*)0xE000E104 )
#define NVIC_ISER2								( (__vo uint32_t*)0xE000E108 )
#define NVIC_ISER3								( (__vo uint32_t*)0xE000E10C )

/*
 * ARM Cortex M4 NVIC ICERx Register Addresses
 */
#define NVIC_ICER0								( (__vo uint32_t*)0xE000E180 )
#define NVIC_ICER1								( (__vo uint32_t*)0xE000E184 )
#define NVIC_ICER2								( (__vo uint32_t*)0xE000E188 )
#define NVIC_ICER3								( (__vo uint32_t*)0xE000E18C )

/*
 * ARM Cortex M4 NVIC Priority Register Addresses
 */
#define NVIC_IPR								( (__vo uint32_t*)0xE000E400 )
/*
 * Implemented Priority bits
 */
#define IMPLEMENTED_PRIORITY_BITS				4


/*
 * Base addresses of SRAM , Flash ,ROM and OTP
 */
#define SRAM_BASEADDR							0x20000000UL     	    /*!		<Base Address of main SRAM*/
#define Flash_BASEADDR							0x08000000UL			/*!		<Base Address of Flash Memory*/
#define ROM_BASEADDR							0x1FFF0000UL			/*!		<Base Address of System Memory*/
#define OTP_BASEADDR							0x1FFF7800UL			/*!		<Base Address of One Time Programmable Memory*/
#define PERIPH_BASEADDR							0x40000000UL			/*!		<Base address of Peripherals*/


/*
 * Base addresses of AHBx and APBx Bus Peripheral
 */
#define APB1PERIPH_BASEADDR						PERIPH_BASEADDR 						/*!		<Base address of APB1 Peripherals*/
#define APB2PERIPH_BASEADDR						(PERIPH_BASEADDR + 0x00010000UL)		/*!		<Base address of APB2 Peripherals*/
#define AHB1PERIPH_BASEADDR						(PERIPH_BASEADDR + 0x00020000UL) 		/*!		<Base address of AHB1 Peripherals*/
#define AHB2PERIPH_BASEADDR						(PERIPH_BASEADDR + 0x10000000UL)		/*!		<Base address of AHB2 Peripherals*/

/*
 * Base addresses of APB1 Bus Peripherals
 */
#define TIM2_BASEADDR							(APB1PERIPH_BASEADDR + 0x0000)			/*!<	Base Address of Timer2 Peripheral*/
#define TIM3_BASEADDR							(APB1PERIPH_BASEADDR + 0x0400) 			/*!<	Base Address of Timer3 Peripheral*/
#define TIM4_BASEADDR							(APB1PERIPH_BASEADDR + 0x0800)			/*!<	Base Address of Timer4 Peripheral*/
#define TIM5_BASEADDR							(APB1PERIPH_BASEADDR + 0x0C00)			/*!<	Base Address of Timer5 Peripheral*/
#define RTCBKP_BASEADDR							(APB1PERIPH_BASEADDR + 0x2800)			/*!<	Base Address of RTC&BKP Registers*/
#define WWDG_BASEADDR							(APB1PERIPH_BASEADDR + 0x2C00)			/*!<	Base Address of Window Watchdog Peripheral*/
#define IWDG_BASEADDR							(APB1PERIPH_BASEADDR + 0x3000)			/*!<	Base Address of Independent Watchdog Peripheral*/
#define I2S2ext_BASEADDR                        (APB1PERIPH_BASEADDR + 0x3400)        	/*!<	Base Address of I2S2ext Peripheral */
#define SPI2_BASEADDR                           (APB1PERIPH_BASEADDR + 0x3800)       	/*!< 	Base Address of SPI2/I2S2 Peripheral */
#define SPI3_BASEADDR                           (APB1PERIPH_BASEADDR + 0x3C00)        	/*!< 	Base Address of SPI3/I2S3 Peripheral */
#define I2S3ext_BASEADDR                        (APB1PERIPH_BASEADDR + 0x4000)       	/*!< 	Base Address of I2S3ext Peripheral */
#define USART2_BASEADDR                         (APB1PERIPH_BASEADDR + 0x4400)       	/*!< 	Base Address of USART2 Peripheral */
#define I2C1_BASEADDR                           (APB1PERIPH_BASEADDR + 0x5400)        	/*!< 	Base Address of I2C1 Peripheral */
#define I2C2_BASEADDR                           (APB1PERIPH_BASEADDR + 0x5800)        	/*!< 	Base Address of I2C2 Peripheral */
#define I2C3_BASEADDR                           (APB1PERIPH_BASEADDR + 0x5C00)      	/*!< 	Base Address of I2C3 Peripheral */
#define PWR_BASEADDR                            (APB1PERIPH_BASEADDR + 0x7000)       	/*!< 	Base Address of Power Control Peripheral */

/*
 * Base addresses of APB2 Bus Peripherals
 */
#define TIM1_BASEADDR                           (APB2PERIPH_BASEADDR + 0x0000)         /*!< 	Base Address of Timer1 Peripheral */
#define USART1_BASEADDR                         (APB2PERIPH_BASEADDR + 0x1000)         /*!< 	Base Address of USART1 Peripheral */
#define USART6_BASEADDR                         (APB2PERIPH_BASEADDR + 0x1400)         /*!< 	Base Address of USART6 Peripheral */
#define ADC1_BASEADDR                           (APB2PERIPH_BASEADDR + 0x2000)         /*!< 	Base Address of ADC1 Peripheral */
#define SDIO_BASEADDR                           (APB2PERIPH_BASEADDR + 0x2C00)         /*!< 	Base Address of SDIO Peripheral */
#define SPI1_BASEADDR                           (APB2PERIPH_BASEADDR + 0x3000)         /*!< 	Base Address of SPI1 Peripheral */
#define SPI4_BASEADDR                           (APB2PERIPH_BASEADDR + 0x3400)         /*!< 	Base Address of SPI4 Peripheral */
#define SYSCFG_BASEADDR                         (APB2PERIPH_BASEADDR + 0x3800)         /*!< 	Base Address of System Configuration Controller */
#define EXTI_BASEADDR                           (APB2PERIPH_BASEADDR + 0x3C00)         /*!< 	Base Address of External Interrupt Peripheral */
#define TIM9_BASEADDR                           (APB2PERIPH_BASEADDR + 0x4000)         /*!< 	Base Address of Timer9 Peripheral */
#define TIM10_BASEADDR                          (APB2PERIPH_BASEADDR + 0x4400)         /*!< 	Base Address of Timer10 Peripheral */
#define TIM11_BASEADDR                          (APB2PERIPH_BASEADDR + 0x4800)         /*!< 	Base Address of Timer11 Peripheral */

/*
 * Base addresses of AHB1 Bus Peripherals
 */
#define GPIOA_BASEADDR                          (AHB1PERIPH_BASEADDR + 0x0000)         /*!< 	Base Address of GPIOA Peripheral */
#define GPIOB_BASEADDR                          (AHB1PERIPH_BASEADDR + 0x0400)         /*!< 	Base Address of GPIOB Peripheral */
#define GPIOC_BASEADDR                          (AHB1PERIPH_BASEADDR + 0x0800)         /*!< 	Base Address of GPIOC Peripheral */
#define GPIOD_BASEADDR                          (AHB1PERIPH_BASEADDR + 0x0C00)         /*!< 	Base Address of GPIOD Peripheral */
#define GPIOE_BASEADDR                          (AHB1PERIPH_BASEADDR + 0x1000)         /*!< 	Base Address of GPIOE Peripheral */
#define GPIOH_BASEADDR                          (AHB1PERIPH_BASEADDR + 0x1C00)         /*!<		Base Address of GPIOH Peripheral */
#define CRC_BASEADDR                            (AHB1PERIPH_BASEADDR + 0x3000)         /*!< 	Base Address of CRC Peripheral */
#define RCC_BASEADDR                            (AHB1PERIPH_BASEADDR + 0x3800)         /*!< 	Base Address of RCC Peripheral */

/*
 * Defining Peripherals Structures
 */
typedef struct{
	__vo uint32_t CR1;					/*!<	Offset 0x00*/
	__vo uint32_t CR2;					/*!<	Offset 0x04*/
	__vo uint32_t SMCR;					/*!<	Offset 0x08*/
	__vo uint32_t DIER;					/*!<	Offset 0x0C*/
	__vo uint32_t SR;					/*!<	Offset 0x10*/
	__vo uint32_t EGR;					/*!<	Offset 0x14*/
	__vo uint32_t CCMR1;   				/*!<	Offset 0x18 ; OCM : 0x18 / ICM : 0x1A*/
	__vo uint32_t CCMR2;				/*!<	Offset 0x1C ; OCM : 0x1C / ICM : 0x1E*/
	__vo uint32_t CCER;					/*!<	Offset 0x20*/
	__vo uint32_t CNT;					/*!<	Offset 0x24*/
	__vo uint32_t PSC;					/*!<	Offset 0x28*/
	__vo uint32_t ARR;					/*!<	Offset 0x2C*/
	uint32_t Reserved1;					/*!< RESERVED */
	__vo uint32_t CCR1;					/*!<	Offset 0x34*/
	__vo uint32_t CCR2;					/*!<	Offset 0x38*/
	__vo uint32_t CCR3;					/*!<	Offset 0x3C*/
	__vo uint32_t CCR4;					/*!<	Offset 0x40*/
	uint32_t Reserved2;					/*!< RESERVED */
	__vo uint32_t DCR;					/*!<	Offset 0x48*/
	__vo uint32_t DMAR;					/*!<	Offset 0x4C*/
	__vo uint32_t OR;					/*!<	Offset 0x50*/
}TIMx_t;

typedef struct{
	__vo uint32_t TR;					/*!<	Offset 0x00*/
	__vo uint32_t DR;					/*!<	Offset 0x04*/
	__vo uint32_t CR;					/*!<	Offset 0x08*/
	__vo uint32_t ISR;					/*!<	Offset 0x0C*/
	__vo uint32_t PRER;					/*!<	Offset 0x10*/
	__vo uint32_t WUTR;					/*!<	Offset 0x14*/
	__vo uint32_t CALIBR;				/*!<	Offset 0x18*/
	__vo uint32_t ALRMAR;				/*!<	Offset 0x1C*/
	__vo uint32_t ALRMBR;				/*!<	Offset 0x20*/
	__vo uint32_t WPR;					/*!<	Offset 0x24*/
	__vo uint32_t SSR;					/*!<	Offset 0x28*/
	__vo uint32_t TSTR;					/*!<	Offset 0x30*/
	__vo uint32_t TSSSR;				/*!<	Offset 0x38*/
	__vo uint32_t CALR;					/*!<	Offset 0x3C*/
	__vo uint32_t TAFCR;				/*!<	Offset 0x40*/
	__vo uint32_t ALRMASSR;				/*!<	Offset 0x44*/
	__vo uint32_t ALRMBSSR;				/*!<	Offset 0x48*/
	__vo uint32_t BKP;					/*!<	Offset 0x50*/
}RTC_t;

typedef struct{
	__vo uint32_t CR;					/*!<	Offset 0x00*/
	__vo uint32_t CFR;					/*!<	Offset 0x04*/
	__vo uint32_t SR;					/*!<	Offset 0x08*/
}WWDG_t;

typedef struct{
	__vo uint32_t KR;					/*!<	Offset 0x00*/
	__vo uint32_t PR;					/*!<	Offset 0x04*/
	__vo uint32_t RLR;					/*!<	Offset 0x08*/
	__vo uint32_t SR;					/*!<	Offset 0x0C*/
}IWDG_t;

typedef struct{
	__vo uint32_t CR1[2];				/*!<	CR1[0]: Offset 0x00 ,CR1[1]: Offset 0x04 */
	__vo uint32_t SR;					/*!<	Offset 0x08*/
	__vo uint32_t DR;					/*!<	Offset 0x0C*/
	__vo uint32_t CRCPR;				/*!<	Offset 0x10*/
	__vo uint32_t RXCRCR;				/*!<	Offset 0x14*/
	__vo uint32_t TXCRCR;				/*!<	Offset 0x18*/
	__vo uint32_t I2SCFGR;				/*!<	Offset 0x1C*/
	__vo uint32_t I2SPR;				/*!<	Offset 0x20*/
}SPI_t;

typedef struct{
	__vo uint32_t SR;					/*!<	Offset 0x00*/
	__vo uint32_t DR;					/*!<	Offset 0x04*/
	__vo uint32_t BRR;					/*!<	Offset 0x08*/
	__vo uint32_t CR1;					/*!<	Offset 0x0C*/
	__vo uint32_t CR2;					/*!<	Offset 0x10*/
	__vo uint32_t CR3;					/*!<	Offset 0x14*/
	__vo uint32_t GTPR;					/*!<	Offset 0x18*/
}USART_t;


typedef struct{
	__vo uint32_t CR1;					/*!<	Offset 0x00*/
	__vo uint32_t CR2;					/*!<	Offset 0x04*/
	__vo uint32_t OAR1;					/*!<	Offset 0x08*/
	__vo uint32_t OAR2;					/*!<	Offset 0x0C*/
	__vo uint32_t DR;					/*!<	Offset 0x10*/
	__vo uint32_t SR1;					/*!<	Offset 0x14*/
	__vo uint32_t SR2;					/*!<	Offset 0x18*/
	__vo uint32_t CCR;					/*!<	Offset 0x1C*/
	__vo uint32_t TRISE;				/*!<	Offset 0x20*/
	__vo uint32_t FLTR;					/*!<	Offset 0x24*/
}I2C_t;

typedef struct{
	__vo uint32_t CR;					/*!<	Offset 0x00*/
	__vo uint32_t CSR;					/*!<	Offset 0x04*/
}PWR_t;

typedef struct{
	__vo uint32_t CR1;					/*!<	Offset 0x00*/
	__vo uint32_t CR2;					/*!<	Offset 0x04*/
	__vo uint32_t SMCR;					/*!<	Offset 0x08*/
	__vo uint32_t DIER;					/*!<	Offset 0x0C*/
	__vo uint32_t SR;					/*!<	Offset 0x10*/
	__vo uint32_t EGR;					/*!<	Offset 0x14*/
	__vo uint32_t CCMR1; 				/*!<	Offset 0x18 ; OCM : 0x18 , ICM : 0x1A*/
	__vo uint32_t CCMR2;				/*!<	Offset 0x1C ; OCM : 0x1C / ICM : 0x1E*/
	__vo uint32_t CCER;					/*!<	Offset 0x20*/
	__vo uint32_t CNT;					/*!<	Offset 0x24*/
	__vo uint32_t PSC;					/*!<	Offset 0x28*/
	__vo uint32_t ARR;					/*!<	Offset 0x2C*/
	__vo uint32_t RCR;					/*!<	Offset 0x30*/
	__vo uint32_t CCR1;					/*!<	Offset 0x34*/
	__vo uint32_t CCR2;					/*!<	Offset 0x38*/
	__vo uint32_t CCR3;					/*!<	Offset 0x3C*/
	__vo uint32_t CCR4;					/*!<	Offset 0x40*/
	__vo uint32_t BDTR;					/*!<	Offset 0x44*/
	__vo uint32_t DCR;					/*!<	Offset 0x48*/
	__vo uint32_t DMAR;					/*!<	Offset 0x4C*/
}TIM1_t;

typedef struct{
	__vo uint32_t SR;					/*!<	Offset 0x00*/
	__vo uint32_t CR1;					/*!<	Offset 0x04*/
	__vo uint32_t CR2;					/*!<	Offset 0x08*/
	__vo uint32_t SMPR1;				/*!<	Offset 0x0C*/
	__vo uint32_t SMPR2;				/*!<	Offset 0x10*/
	__vo uint32_t JOFR1;				/*!<	Offset 0x14*/
	__vo uint32_t JOFR2;				/*!<	Offset 0x18*/
	__vo uint32_t JOFR3;				/*!<	Offset 0x1C*/
	__vo uint32_t JOFR4;				/*!<	Offset 0x20*/
	__vo uint32_t HTR;					/*!<	Offset 0x24*/
	__vo uint32_t LTR;					/*!<	Offset 0x28*/
	__vo uint32_t SQR1;					/*!<	Offset 0x2C*/
	__vo uint32_t SQR2;					/*!<	Offset 0x30*/
	__vo uint32_t SQR3;					/*!<	Offset 0x34*/
	__vo uint32_t JSQR;					/*!<	Offset 0x38*/
	__vo uint32_t JDR1;					/*!<	Offset 0x3C*/
	__vo uint32_t JDR2;					/*!<	Offset 0x40*/
	__vo uint32_t JDR3;					/*!<	Offset 0x44*/
	__vo uint32_t JDR4;					/*!<	Offset 0x48*/
	__vo uint32_t DR;					/*!<	Offset 0x4C*/
	__vo uint32_t CCR;					/*!<	Offset 0x04*/
}ADC_t;

typedef struct{
	__vo uint32_t POWER;				/*!<	Offset 0x00*/
	__vo uint32_t CLKCR;				/*!<	Offset 0x04*/
	__vo uint32_t ARG;					/*!<	Offset 0x08*/
	__vo uint32_t CMD;					/*!<	Offset 0x0C*/
	__vo uint32_t RESPCMD;				/*!<	Offset 0x10*/
	__vo uint32_t RESP1;				/*!<	Offset 0x14*/
	__vo uint32_t RESP2;				/*!<	Offset 0x18*/
	__vo uint32_t RESP3;				/*!<	Offset 0x1C*/
	__vo uint32_t RESP4;				/*!<	Offset 0x20*/
	__vo uint32_t DTIMER;				/*!<	Offset 0x24*/
	__vo uint32_t DLEN;					/*!<	Offset 0x28*/
	__vo uint32_t DCTRL;				/*!<	Offset 0x2C*/
	__vo uint32_t DCOUNT;				/*!<	Offset 0x30*/
	__vo uint32_t STA;					/*!<	Offset 0x34*/
	__vo uint32_t ICR;					/*!<	Offset 0x38*/
	__vo uint32_t MASK;					/*!<	Offset 0x3C*/
	uint32_t Reserved1[2];				/*!< 	Offset 0x40*/
	__vo uint32_t FIFOCNT;				/*!<	Offset 0x48*/
	uint32_t Reserved2[14];
	__vo uint32_t FIFO;					/*!<	Offset 0x80*/
}SDIO_t;

typedef struct{
	__vo uint32_t MEMRMP;				/*!<	Offset 0x00*/
	__vo uint32_t PMC;					/*!<	Offset 0x04*/
	__vo uint32_t EXTICR[4];			/*!<	Offset 0x08*/
	uint32_t Reserved[2];				/*!< Reserved */
	__vo uint32_t CMPCR;				/*!<	Offset 0x20*/
}SYSCFG_t;

typedef struct{
	__vo uint32_t IMR;					/*!<	Offset 0x00*/
	__vo uint32_t EMR;					/*!<	Offset 0x04*/
	__vo uint32_t RTSR;					/*!<	Offset 0x08*/
	__vo uint32_t FTSR;					/*!<	Offset 0x0C*/
	__vo uint32_t SWIER;				/*!<	Offset 0x10*/
	__vo uint32_t PR;					/*!<	Offset 0x14*/
}EXTI_t;

typedef struct{
	__vo uint32_t CR1[2];				/*!<	Offset 0x00*/
	__vo uint32_t SMCR;					/*!<	Offset 0x08*/
	__vo uint32_t DIER;					/*!<	Offset 0x0C*/
	__vo uint32_t SR;					/*!<	Offset 0x10*/
	__vo uint32_t EGR;					/*!<	Offset 0x14*/
	__vo uint32_t CCMR1; 				/*!<	Offset 0x18*/
	uint32_t Reserved1;					/*!< Reserved */
	__vo uint32_t CCER;					/*!<	Offset 0x20*/
	__vo uint32_t CNT;					/*!<	Offset 0x24*/
	__vo uint32_t PSC;					/*!<	Offset 0x28*/
	__vo uint32_t ARR;					/*!<	Offset 0x2C*/
	uint32_t Reserved2;
	__vo uint32_t CCR1;					/*!<	Offset 0x34*/
	__vo uint32_t CCR2;					/*!<	Offset 0x38*/
	uint32_t Reserved3[4];				/*!< Reserved */
}TIM9_t;

typedef struct{
	__vo uint32_t CR1[2];				/*!<	Offset 0x00*/
	__vo uint32_t SMCR;					/*!<	Offset 0x08*/
	__vo uint32_t DIER;					/*!<	Offset 0x0C*/
	__vo uint32_t SR;					/*!<	Offset 0x10*/
	__vo uint32_t EGR;					/*!<	Offset 0x14*/
	__vo uint32_t CCMR1;  				/*!<	Offset 0x18*/
	uint32_t Reserved1;					/*!< Reserved */
	__vo uint32_t CCER;					/*!<	Offset 0x20*/
	__vo uint32_t CNT;					/*!<	Offset 0x24*/
	__vo uint32_t PSC;					/*!<	Offset 0x28*/
	__vo uint32_t ARR;					/*!<	Offset 0x2C*/
	uint32_t Reserved2;					/*!< Reserved */
	__vo uint32_t CCR1;					/*!<	Offset 0x34*/
	uint32_t Reerved3[5];				/*!< Reserved */
	__vo uint32_t OR;					/*!<	Offset 0x50*/
}TIM1011_t;

typedef struct{
	__vo uint32_t MODER;				/*!<GPIO port mode register,				Offset 0x00*/
	__vo uint32_t OTYPER;				/*!<GPIO port output type register,			Offset 0x04*/
	__vo uint32_t OSPEEDR;				/*!<GPIO port output speed register,		Offset 0x08*/
	__vo uint32_t PUPDR;				/*!<GPIO port pull-up/pull-down register,	Offset 0x0C*/
	__vo uint32_t IDR;					/*!<GPIO port input data register,			Offset 0x10*/
	__vo uint32_t ODR;					/*!<GPIO port output data register,			Offset 0x14*/
	__vo uint32_t BSRR;					/*!<GPIO port bit set/reset register,		Offset 0x18*/
	__vo uint32_t LCKR;					/*!<GPIO port configuration lock register,	Offset 0x1C*/
	__vo uint32_t AFR[2];				/*!<AFR[0] : Alternate function low register , AFR[1] : Alternate function high register ,		 Offset 0x20*/
}GPIOx_t;

typedef struct{
	__vo uint32_t DR;					/*!<	Offset 0x00*/
	__vo uint32_t IDR;					/*!<	Offset 0x04*/
	__vo uint32_t CR;					/*!<	Offset 0x08*/
}CRC_t;

typedef struct{
	__vo uint32_t CR;					/*!<	Offset 0x00*/
	__vo uint32_t PLLCFGR;				/*!<	Offset 0x04*/
	__vo uint32_t CFGR;					/*!<	Offset 0x08*/
	__vo uint32_t CIR;					/*!<	Offset 0x0C*/
	__vo uint32_t AHB1RSTR;				/*!<	Offset 0x10*/
	__vo uint32_t AHB2RSTR;				/*!<	Offset 0x14*/
	uint32_t Reserved1[2]; 				/*!< Reserved */
	__vo uint32_t APB1RSTR;				/*!<	Offset 0x20*/
	__vo uint32_t APB2RSTR;				/*!<	Offset 0x24*/
	uint32_t Reserved2[2];				/*!< Reserved */
	__vo uint32_t AHB1ENR;				/*!<	Offset 0x30*/
	__vo uint32_t AHB2ENR;				/*!<	Offset 0x34*/
	uint32_t Reserved3[2]; 				/*!<Reserved */
	__vo uint32_t APB1ENR;				/*!<	Offset 0x40*/
	__vo uint32_t APB2ENR;				/*!<	Offset 0x44*/
	uint32_t Reserved4[2]; 				/*!<Reserved */
	__vo uint32_t AHB1LPENR;			/*!<	Offset 0x50*/
	__vo uint32_t AHB2LPENR;			/*!<	Offset 0x54*/
	uint32_t Reserved5[2]; 				/*!<Reserved */
	__vo uint32_t APB1LPENR;			/*!<	Offset 0x60*/
	__vo uint32_t APB2LPENR;			/*!<	Offset 0x64*/
	uint32_t Reserved6[2]; 				/*!<Reserved */
	__vo uint32_t BDCR;					/*!<	Offset 0x70*/
	__vo uint32_t CSR;					/*!<	Offset 0x74*/
	uint32_t Reserved7[2]; 				/*!<Reserved */
	__vo uint32_t SSCGR;				/*!<	Offset 0x80*/
	__vo uint32_t PLLI2SCFGR;			/*!<	Offset 0x84*/
	__vo uint32_t DCKCFGR;				/*!<	Offset 0x8C*/
}RCC_t;

typedef struct{
	__vo uint32_t ACR;					/*!<	Offset 0x00*/
	__vo uint32_t KEYR;					/*!<	Offset 0x04*/
	__vo uint32_t OPTKEYR;				/*!<	Offset 0x08*/
	__vo uint32_t SR;					/*!<	Offset 0x0C*/
	__vo uint32_t CR;					/*!<	Offset 0x10*/
	__vo uint32_t OPTCR;				/*!<	Offset 0x14*/
}FLASH_t;

typedef struct{
	__vo uint32_t LISR;					/*!<	Offset 0x0000*/
	__vo uint32_t HISR;					/*!<	Offset 0x0004*/
	__vo uint32_t LIFCR;				/*!<	Offset 0x0008*/
	__vo uint32_t HIFCR;				/*!<	Offset 0x000C*/
	__vo uint32_t S0CR;					/*!<	Offset 0x0010*/
	__vo uint32_t S0NDTR;				/*!<	Offset 0x0014*/
	__vo uint32_t S0PAR;				/*!<	Offset 0x0018*/
	__vo uint32_t S0M0AR;				/*!<	Offset 0x001C*/
	__vo uint32_t S0M1AR;				/*!<	Offset 0x0020*/
	__vo uint32_t S0FCR;				/*!<	Offset 0x0024*/
	__vo uint32_t S1CR;					/*!<	Offset 0x0028*/
	__vo uint32_t S1NDTR;				/*!<	Offset 0x002C*/
	__vo uint32_t S1PAR;				/*!<	Offset 0x0030*/
	__vo uint32_t S1M0AR;				/*!<	Offset 0x0034*/
	__vo uint32_t S1M1AR;				/*!<	Offset 0x0038*/
	__vo uint32_t S1FCR;				/*!<	Offset 0x003C*/
	__vo uint32_t S2CR;					/*!<	Offset 0x0040*/
	__vo uint32_t S2NDTR;				/*!<	Offset 0x0044*/
	__vo uint32_t S2PAR;				/*!<	Offset 0x0048*/
	__vo uint32_t S2M0AR;				/*!<	Offset 0x004C*/
	__vo uint32_t S2M1AR;				/*!<	Offset 0x0050*/
	__vo uint32_t S2FCR;				/*!<	Offset 0x0054*/
	__vo uint32_t S3CR;					/*!<	Offset 0x0058*/
	__vo uint32_t S3NDTR;				/*!<	Offset 0x005C*/
	__vo uint32_t S3PAR;				/*!<	Offset 0x0060*/
	__vo uint32_t S3M0AR;				/*!<	Offset 0x0064*/
	__vo uint32_t S3M1AR;				/*!<	Offset 0x0068*/
	__vo uint32_t S3FCR;				/*!<	Offset 0x006C*/
	__vo uint32_t S4CR;					/*!<	Offset 0x0070*/
	__vo uint32_t S4NDTR;				/*!<	Offset 0x0074*/
	__vo uint32_t S4PAR;				/*!<	Offset 0x0078*/
	__vo uint32_t S4M0AR;				/*!<	Offset 0x007C*/
	__vo uint32_t S4M1AR;				/*!<	Offset 0x0080*/
	__vo uint32_t S4FCR;				/*!<	Offset 0x0084*/
	__vo uint32_t S5CR;					/*!<	Offset 0x0088*/
	__vo uint32_t S5NDTR;				/*!<	Offset 0x008C*/
	__vo uint32_t S5PAR;				/*!<	Offset 0x0090*/
	__vo uint32_t S5M0AR;				/*!<	Offset 0x0094*/
	__vo uint32_t S5M1AR;				/*!<	Offset 0x0098*/
	__vo uint32_t S5FCR;				/*!<	Offset 0x009C*/
	__vo uint32_t S6CR;					/*!<	Offset 0x00A0*/
	__vo uint32_t S6NDTR;				/*!<	Offset 0x00A4*/
	__vo uint32_t S6PAR;				/*!<	Offset 0x00A8*/
	__vo uint32_t S6M0AR;				/*!<	Offset 0x00AC*/
	__vo uint32_t S6M1AR;				/*!<	Offset 0x00B0*/
	__vo uint32_t S6FCR;				/*!<	Offset 0x00B4*/
	__vo uint32_t S7CR;					/*!<	Offset 0x00B8*/
	__vo uint32_t S7NDTR;				/*!<	Offset 0x00BC*/
	__vo uint32_t S7PAR;				/*!<	Offset 0x00C0*/
	__vo uint32_t S7M0AR;				/*!<	Offset 0x00C4*/
	__vo uint32_t S7M1AR;				/*!<	Offset 0x00C8*/
	__vo uint32_t S7FCR;				/*!<	Offset 0x00CC*/
}DMA_t;



/*
 * Peripherals Base Addresses typecasted to  GPIOx_t*
 */
#define GPIOA			((GPIOx_t*)GPIOA_BASEADDR)                                  		/*!< 	Typecasting GPIOA_BASEADDR Peripheral to GPIOx_t* */
#define GPIOB			((GPIOx_t*)GPIOB_BASEADDR)                                 			/*!< 	Typecasting GPIOB_BASEADDR Peripheral to GPIOx_t* */
#define GPIOC			((GPIOx_t*)GPIOC_BASEADDR)                              			/*!< 	Typecasting GPIOC_BASEADDR Peripheral to GPIOx_t* */
#define GPIOD			((GPIOx_t*)GPIOD_BASEADDR)                                 			/*!< 	Typecasting GPIOD_BASEADDR Peripheral to GPIOx_t* */
#define GPIOE			((GPIOx_t*)GPIOE_BASEADDR)                               			/*!< 	Typecasting GPIOE_BASEADDR Peripheral to GPIOx_t* */
#define GPIOH			((GPIOx_t*)GPIOH_BASEADDR)                          	        	/*!<	Typecasting GPIOH_BASEADDR Peripheral to GPIOx_t* */

/*
 * RCC Base Address typecasted to  RCC_t*
 */
#define RCC 			((RCC_t*)RCC_BASEADDR)												/*!<	Typecasting RCC_BASEADDR to RCC_t* */

/*
 * EXTI Base Address typecasted to EXTI_t*
 */
#define EXTI 			((EXTI_t*)EXTI_BASEADDR)											/*!<	Typecasting RCC_BASEADDR to EXTI_t* */

/*
 * SYSCFG Base Address typecasted to SYSCFG_t*
 */
#define SYSCFG 			((SYSCFG_t*)SYSCFG_BASEADDR)										/*!<	Typecasting RCC_BASEADDR to EXTI_t* */

/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define  GPIOA_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 0) )
#define  GPIOB_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 1) )
#define  GPIOC_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 2) )
#define  GPIOD_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 3) )
#define  GPIOE_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 4) )
#define  GPIOH_PCLK_EN()		( RCC->AHB1ENR |= ( 1 << 7) )

/*
 * Clock Enable Macros for I2Cx
 */
#define I2C1_PCLK_EN()			( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()			( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()			( RCC->APB1ENR |= (1 << 23) )

/*
 * Clock Enable Macros for SPIx
 */
#define SPI1_PCLK_EN()			( RCC->APB2ENR |= (1 << 12) )
#define SPI2_PCLK_EN()			( RCC->APB1ENR |= (1 << 14) )
#define SPI3_PCLK_EN()			( RCC->APB1ENR |= (1 << 15) )
#define SPI4_PCLK_EN()			( RCC->APB2ENR |= (1 << 13) )

/*
 * Clock Enable Macros for USARTx
 */
#define USART1_PCLK_EN()			( RCC->APB2ENR |= (1 << 4) )
#define USART2_PCLK_EN()			( RCC->APB1ENR |= (1 << 17) )
#define USART6_PCLK_EN()			( RCC->APB2ENR |= (1 << 5) )

/*
 * Clock Enable Macros for TIMx
 */
#define TIM1_PCLK_EN()			( RCC->APB2ENR |= (1 << 0) )
#define TIM2_PCLK_EN()			( RCC->APB1ENR |= (1 << 0) )
#define TIM3_PCLK_EN()			( RCC->APB1ENR |= (1 << 1) )
#define TIM4_PCLK_EN()			( RCC->APB1ENR |= (1 << 2) )
#define TIM5_PCLK_EN()			( RCC->APB1ENR |= (1 << 3) )
#define TIM9_PCLK_EN()			( RCC->APB2ENR |= (1 << 16) )
#define TIM10_PCLK_EN()			( RCC->APB2ENR |= (1 << 17) )
#define TIM11_PCLK_EN()			( RCC->APB2ENR |= (1 << 18) )



/*
 * Clock Enable Macros for DMA
 */
#define DMA1_PCLK_EN()			( RCC->AHB1ENR |= (1 << 21) )
#define DMA2_PCLK_EN()			( RCC->AHB1ENR |= (1 << 22) )

/*
 * Clock Enable Macro for SYSCFG
 */
#define SYSCFG_PCLK_EN()		( RCC->APB2ENR |= (1 << 14) )
/*
 * Clock Disable Macros for GPIOx peripherals
 */
#define  GPIOA_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 0) )
#define  GPIOB_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 1) )
#define  GPIOC_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 2) )
#define  GPIOD_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 3) )
#define  GPIOE_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 4) )
#define  GPIOH_PCLK_DI()		( RCC->AHB1ENR &= ~( 1 << 7) )

/*
 * Clock Disable Macros for I2Cx
 */
#define I2C1_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 23) )

/*
 * Clock Disable Macros for SPIx
 */
#define SPI1_PCLK_DI()			( RCC->APB2ENR &= ~(1 << 12) )
#define SPI2_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 14) )
#define SPI3_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 15) )
#define SPI4_PCLK_DI()			( RCC->APB2ENR &= ~(1 << 13) )

/*
 * Clock Disable Macros for USARTx
 */
#define USART1_PCLK_DI()			( RCC->APB2ENR &= ~(1 << 4) )
#define USART2_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 17) )
#define USART6_PCLK_DI()			( RCC->APB2ENR &= ~(1 << 5) )

/*
 * Clock Enable Macros for TIMx
 */
#define TIM1_PCLK_DI()			( RCC->APB2ENR &= ~(1 << 0) )
#define TIM2_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 0) )
#define TIM3_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 1) )
#define TIM4_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 2) )
#define TIM5_PCLK_DI()			( RCC->APB1ENR &= ~(1 << 3) )
#define TIM9_PCLK_DI()			( RCC->APB2ENR &= ~(1 << 16) )
#define TIM10_PCLK_DI()			( RCC->APB2ENR &= ~(1 << 17) )
#define TIM11_PCLK_DI()			( RCC->APB2ENR &= ~(1 << 18) )

/*
 * Clock Disable Macros for DMA
 */
#define DMA1_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 21) )
#define DMA2_PCLK_DI()			( RCC->AHB1ENR &= ~(1 << 22) )

/*
 * Clock Disable Macro for SYSCFG
 */
#define SYSCFG_PCLK_DI()		( RCC->APB2END &= ~(1 << 14) )
/*
 * GPIO Reset
 */
#define GPIOA_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<0)); (RCC->AHB1RSTR &= ~(1<<0)); }while(0)
#define GPIOB_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<1)); (RCC->AHB1RSTR &= ~(1<<1)); }while(0)
#define GPIOC_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<2)); (RCC->AHB1RSTR &= ~(1<<2)); }while(0)
#define GPIOD_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<3)); (RCC->AHB1RSTR &= ~(1<<3)); }while(0)
#define GPIOE_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<4)); (RCC->AHB1RSTR &= ~(1<<4)); }while(0)
#define GPIOH_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<7)); (RCC->AHB1RSTR &= ~(1<<7)); }while(0)

/*
 * I2C Reset
 */
#define I2C1_REG_RESET()		do{ (RCC->APB1RSTR  |= (1<<21)); (RCC->APB1RSTR &= ~(1<<21));}while(0)
#define I2C2_REG_RESET()		do{ (RCC->APB1RSTR  |= (1<<22)); (RCC->APB1RSTR &= ~(1<<22));}while(0)
#define I2C3_REG_RESET()		do{ (RCC->APB1RSTR  |= (1<<23)); (RCC->APB1RSTR &= ~(1<<23));}while(0)

/*
 * SPI Reset
 */
#define SPI1_REG_RESET()		do{ (RCC->APB2RSTR  |= (1<<12)); (RCC->APB2RSTR &= ~(1<<12));}while(0)
#define SPI2_REG_RESET()		do{ (RCC->APB1RSTR  |= (1<<14)); (RCC->APB1RSTR &= ~(1<<14));}while(0)
#define SPI3_REG_RESET()		do{ (RCC->APB1RSTR  |= (1<<15)); (RCC->APB1RSTR &= ~(1<<15));}while(0)
#define SPI4_REG_RESET()		do{ (RCC->APB2RSTR  |= (1<<13)); (RCC->APB2RSTR &= ~(1<<13));}while(0)

/*
 * USART Reset
 */
#define USART1_REG_RESET()		do{ (RCC->APB2RSTR  |= (1<<4)); (RCC->APB2RSTR &= ~(1<<4));  }while(0)
#define USART2_REG_RESET()		do{ (RCC->APB1RSTR  |= (1<<17)); (RCC->APB1RSTR &= ~(1<<17));}while(0)
#define USART6_REG_RESET()		do{ (RCC->APB2RSTR  |= (1<<5)); (RCC->APB2RSTR &= ~(1<<5));  }while(0)

/*
 * TIM Reset
 */
#define TIM1_REG_RESET()		do{ (RCC->APB2RSTR  |= (1<<0)); (RCC->APB2RSTR &= ~(1<<0));  }while(0)
#define TIM2_REG_RESET()		do{ (RCC->APB1RSTR  |= (1<<0)); (RCC->APB1RSTR &= ~(1<<0));  }while(0)
#define TIM3_REG_RESET()		do{ (RCC->APB1RSTR  |= (1<<1)); (RCC->APB1RSTR  &= ~(1<<1)); }while(0)
#define TIM4_REG_RESET()		do{ (RCC->APB1RSTR  |= (1<<2)); (RCC->APB1RSTR  &= ~(1<<2)); }while(0)
#define TIM5_REG_RESET()		do{ (RCC->APB1RSTR  |= (1<<3)); (RCC->APB1RSTR  &= ~(1<<3)); }while(0)
#define TIM9_REG_RESET()		do{ (RCC->APB2RSTR  |= (1<<16)); (RCC->APB2RSTR &= ~(1<<16));}while(0)
#define TIM10_REG_RESET()		do{ (RCC->APB2RSTR  |= (1<<17)); (RCC->APB2RSTR &= ~(1<<17));}while(0)
#define TIM11_REG_RESET()		do{ (RCC->APB2RSTR  |= (1<<18)); (RCC->APB2RSTR &= ~(1<<18));}while(0)

/*
 * DMA Reset
 */
#define DMA1_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<21)); (RCC->AHB1RSTR &= ~(1<<21)); }while(0)
#define DMA2_REG_RESET()		do{ (RCC->AHB1RSTR |= (1<<22)); (RCC->AHB1RSTR &= ~(1<<22)); }while(0)

/*
 * Used Functions Macros
 */
#define GPIO_TO_CODE(x)			( (x == GPIOA)?0:\
								  (x == GPIOB)?1:\
								  (x == GPIOC)?2:\
								  (x == GPIOD)?3:\
								  (x == GPIOE)?4:\
								  (x == GPIOH)?7:0 )





#include "stm32f401xx_gpio.h"
#endif /* INC_STM32F401XX_H_ */

