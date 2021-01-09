/*
 * stm32f0discovery.h
 *
 *  Created on: Dec 19, 2020
 *      Author: Babanazar
 */

#ifndef INC_STM32F051XX_H_
#define INC_STM32F051XX_H_

#include <stdint.h>


#define __IO volatile

/************************START:Processor Specific Details****************************
 *
 * ARM Cortex Mx Processor NVIC ISER register addresses
 */
#define NVIC_ISER				((__IO uint32_t*)0xE000E100)


/*
 * ARM Cortex Mx Processor NVIC ICER register addresses
 */
#define NVIC_ICER				((__IO uint32_t*)0xE000E180)


/*
 * ARM Cortex Mx Processor Priority Register Address Calculation
 */
#define NVIC_PR_BASE_ADDR		((__IO uint32_t*)0xE000E400)


/*
 * ARM Cortex Mx Processor number of priority bits implemented in Priority Register
 */
#define NO_PR_BITS_IMPLEMENTED	4

/*
 * base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR			0x08000000U		/* Flash base address */
#define SRAM					0x20000000U		/* SRAM base address */
#define ROM_BASEADDR			0x1FFFEC00		/* ROM base address */


/*
 * AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR			0x40000000
#define APB1PERIPH_BASE			PERIPH_BASEADDR
#define APB2PERIPH_BASE			0x40010000
#define AHB1PERIPH_BASE			0x40020000
#define AHB2PERIPH_BASE			0x48000000


/*
 * Base addresses of peripheral which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR			(AHB2PERIPH_BASE + 0x0000)
#define GPIOB_BASEADDR			(AHB2PERIPH_BASE + 0x0400)
#define GPIOC_BASEADDR			(AHB2PERIPH_BASE + 0x0800)
#define GPIOD_BASEADDR			(AHB2PERIPH_BASE + 0x0C00)
#define GPIOE_BASEADDR			(AHB2PERIPH_BASE + 0x1000)
#define GPIOF_BASEADDR			(AHB2PERIPH_BASE + 0x1400)
#define RCC_BASEADDR			(AHB1PERIPH_BASE + 0x1000)

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */
#define I2C1_BASEADDR			(APB1PERIPH_BASE + 0x5400)
#define I2C2_BASEADDR			(APB1PERIPH_BASE + 0x5800)
#define SPI2_BASEADDR			(APB1PERIPH_BASE + 0x3800)

/*
 * Base addresses of peripherals which are hanging on APB2 bus
 */
#define SYSCFG_BASEADDR			(APB2PERIPH_BASE)
#define EXTI_BASEADDR			(APB2PERIPH_BASE + 0x0400)
#define SPI1_BASEADDR			(APB2PERIPH_BASE + 0x3000)


/*************************Peripheral Register Definition Structures*********************************/

/*
 * Note: Registers of a peripheral are specific to MCU
 *
 * e.g.: Number of Registers of SPI peripheral of STM32F0x family of MCUs may be different (more or less)
 * Compared to number of registers of SPI peripheral of STM32Lx or STM32F0x family of MCUs
 * Please check you Device RM
 */


typedef struct
{
	__IO uint32_t MODER;										/*!<Give a short description,			Address offset: 0x00>*/
	__IO uint32_t OTYPER;
	__IO uint32_t OSPEEDR;
	__IO uint32_t PUPDR;
	__IO uint32_t IDR;
	__IO uint32_t ODR;
	__IO uint32_t BSRR;
	__IO uint32_t LCKR;
	__IO uint32_t AFR[2];
	__IO uint32_t BRR;											/*!<GPIO port bit reset register,		Address offset: 0x28>*/
}GPIO_RegDef_t;


/*
 * Peripheral register definition structure for RCC
 */
typedef struct
{
	__IO uint32_t CR;
	__IO uint32_t CFGR;
	__IO uint32_t CIR;
	__IO uint32_t APB2RSTR;
	__IO uint32_t APB1RSTR;
	__IO uint32_t AHBENR;
	__IO uint32_t APB2ENR;
	__IO uint32_t APB1ENR;
	__IO uint32_t BDCR;
	__IO uint32_t CSR;
	__IO uint32_t AHBRSTR;
	__IO uint32_t CFGR2;
	__IO uint32_t CFGR3;
	__IO uint32_t CR2;
}RCC_RegDef_t;


/*
 * Peripheral register definition structure for EXTI
 */
typedef struct
{
	__IO uint32_t IMR;
	__IO uint32_t EMR;
	__IO uint32_t RTSR;
	__IO uint32_t FTSR;
	__IO uint32_t SWIER;
	__IO uint32_t PR;
}EXTI_RegDef_t;


/*
 * Peripheral register definition structure for SPI
 */
typedef struct
{
	__IO uint32_t CR1;
	__IO uint32_t CR2;
	__IO uint32_t SR;
	__IO uint32_t DR;
	__IO uint32_t CRCPR;
	__IO uint32_t RXCRCR;
	__IO uint32_t TXCRCR;
	__IO uint32_t I2SCFGR;
	__IO uint32_t I2SSPR;
}SPI_RegDef_t;

/*
 * Peripheral register definition structure for SYSCFG
 */
typedef struct
{
	__IO uint32_t CFGR1;		/*!< Address offset: 0x00*/
	uint32_t RESERVED;			/*!< Address offset: 0x04*/
	__IO uint32_t EXTICR[4];	/*!< Address offset: 0x08-0x14*/
	__IO uint32_t CFGR2;		/*!< Address offset: 0x18*/
}SYSCFG_RegDef_t;


/*
 * Peripheral Definitions
 */
#define GPIOA 						((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 						((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 						((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 						((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 						((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 						((GPIO_RegDef_t*)GPIOF_BASEADDR)

#define RCC							((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI						((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG						((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1						((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2						((SPI_RegDef_t*)SPI2_BASEADDR)

/*
 * Clock Enable Macros for GPIOx peripherals
 */
#define GPIOA_PCLOCK_ENABLE()		(RCC->AHBENR |= (1 << 17))
#define GPIOB_PCLOCK_ENABLE()		(RCC->AHBENR |= (1 << 18))
#define GPIOC_PCLOCK_ENABLE()		(RCC->AHBENR |= (1 << 19))
#define GPIOD_PCLOCK_ENABLE()		(RCC->AHBENR |= (1 << 20))
#define GPIOE_PCLOCK_ENABLE()		(RCC->AHBENR |= (1 << 21))
#define GPIOF_PCLOCK_ENABLE()		(RCC->AHBENR |= (1 << 22))


/*
 * Clock Enable Macros for I2Cx peripherals
 */
#define I2C1_PCLK_EN()				(RCC->APB2ENR |= (1 << 0))


/*
 * Clock Enable Macros for SPIx peripherals
 */
#define SPI1_PCLOCK_EN()			(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLOCK_EN()			(RCC->APB1ENR |= (1 << 14))

/*
 * Clock Enable Macros for USARTx peripherals
 */


/*
 * Clock Enable Macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN()	(RCC->APB2ENR)


/*
 * Clock Disable Macros for SYSCFG peripheral
 */

/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()			do{ (RCC->AHBRSTR |= (1 << 0)); (RCC->AHBRSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET()			do{ (RCC->AHBRSTR |= (1 << 1)); (RCC->AHBRSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET()			do{ (RCC->AHBRSTR |= (1 << 2)); (RCC->AHBRSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET()			do{ (RCC->AHBRSTR |= (1 << 3)); (RCC->AHBRSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET()			do{ (RCC->AHBRSTR |= (1 << 4)); (RCC->AHBRSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET()			do{ (RCC->AHBRSTR |= (1 << 5)); (RCC->AHBRSTR &= ~(1 << 5));}while(0)


/*
 * return port code for given GPIOx base address
 * This macro returns a code (between 0 to 5) for a given GPIO base address(x)
 */
#define GPIO_BASEADDR_TO_CODE(x)  ( (x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
									(x == GPIOE) ? 4 :\
									(x == GPIOF) ? 5 :0 )




/*
 * IRQ(Interrupt Request) Numbers of STM32F0Discovery MCU
 */
#define IRQ_NO_EXTI0_1		5
#define IRQ_NO_EXTI2_3		6
#define IRQ_NO_EXTI4_15		7

/*
 * macros for all the possible priority levels
 */
#define NVIC_IRQ_PRI_0		0
#define NVIC_IRQ_PRI_15		15

// some generic macros
#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET		SET
#define GPIO_PIN_RESET		RESET
#define FLAG_RESET			RESET
#define FLAG_SET			SET


/********************************************************************************
 * Bit position definitions of SPI peripheral
 *******************************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA		0
#define SPI_CR1_CPOL		1
#define SPI_CR1_MSTR		2
#define SPI_CR1_BR			3
#define SPI_CR1_SPE			6
#define SPI_CR1_LSBFIRST	7
#define SPI_CR1_SSI			8
#define SPI_CR1_SSM			9
#define SPI_CR1_RXONLY		10
#define SPI_CR1_CRCL		11
#define SPI_CR1_CRCNEXT		12
#define SPI_CR1_CRCEN		13
#define SPI_CR1_BIDIOE		14
#define SPI_CR1_BIDIMODE	15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		0
#define SPI_CR2_TXDMAEN		1
#define SPI_CR2_SSOE		2
#define SPI_CR2_NSSP		3
#define SPI_CR2_FRF			4
#define SPI_CR2_ERRIE		5
#define SPI_CR2_RXNEIE		6
#define SPI_CR2_TXEIE		7
#define SPI_CR2_DS			8
#define SPI_CR2_FRXTH		12
#define SPI_CR2_LDMA_RX		13
#define SPI_CR2_LDMA_TX		14


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE			0
#define SPI_SR_TXE			1
#define SPI_SR_CHSIDE		2
#define SPI_SR_UDR			3
#define SPI_SR_CRCERR		4
#define SPI_SR_MODF			5
#define SPI_SR_OVR			6
#define SPI_SR_BSY			7
#define SPI_SR_FRE			8
#define SPI_SR_FRLVL		9
#define SPI_SR_FTLVL		11


#include <stm32f051xx_gpio_driver.h>
#include <stm32f051xx_spi_driver.h>

#endif /* INC_STM32F051XX_H_ */
