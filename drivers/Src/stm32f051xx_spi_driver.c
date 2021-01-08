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

}
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	//TODO
}

/*
 * Data Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{

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
