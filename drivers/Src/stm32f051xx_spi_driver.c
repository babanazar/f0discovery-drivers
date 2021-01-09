/*
 * stm32f051xx_spi_driver.c
 *
 *  Created on: Jan 8, 2021
 *      Author: Babanazar
 */

#include "stm32f051xx_spi_driver.h"

/*
 * Peripheral Clock Setup
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLOCK_EN();

		}
		else if(pSPIx == SPI2)
		{
			SPI2_PCLOCK_EN();
		}

	}
	else
	{
		// use macros to disable as in above code
	}
}


/*
 * Init and De-Init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	/* Peripheral clock enable */
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	/* First lets configure the SPI_CR1 register */
	uint32_t tempreg = 0;

	/* 1. Configure the device mode */
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	/* 2. Configure the bus config */
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		/* bidi mode should be cleared */
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE );
	}else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		/* bidi mode should be set */
		tempreg |= ( 1 << SPI_CR1_BIDIMODE );
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		/* BIDI mode should be cleared */
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE );
		/* RXONLY bit must be set */
		tempreg |= ( 1 << SPI_CR1_RXONLY );
	}

	/* 3. Configure the spi serial clock speed (baud rate) */
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	/* 4. Configure the DFF */
	//tempreg |= pSPIHandle->SPIConfig.SPI_DFF << 11;

	/* 5. Configure the CPOL */
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	/* 6. Configure the CPHA */
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	pSPIHandle->pSPIx->CR1 = tempreg;
}
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	//TODO
}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName)
{
	if(pSPIx->SR & flagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		/* 1. wait until TXE is set */
		while( SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET );

		/* 2. Check the DFF bit in CR1 */
		if((pSPIx->CR1 & (1 << SPI_CR1_CRCL)))
		{
			// 16 bit DFF
			/* 1. load the data in to the DR */
			pSPIx->DR = *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			// 8 bit DFF
			pSPIx->DR = *(pTxBuffer);
			Len--;
			pTxBuffer++;
		}
	}
}
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{

}

/*
 * IRQ Configuration and ISR handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{

}
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{

}
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE)
	{
		pSPIx->CR1 |= (1 << SPI_CR1_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1 << SPI_CR1_SPE);
	}
}
