/*
 * stm32f0discovery_gpio_driver.c
 *
 *  Created on: Dec 20, 2020
 *      Author: Babanazar
 */


#include <stm32f051xx_gpio_driver.h>



/*
 * Peripheral Clock Setup
 */
/***********************************************************************
 * @fn				- GPIO_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		- base address of the GPIO peripheral
 * @param[in]		- ENABLE or DISABLE macros
 * @param[in]		-
 *
 * @return 			- none
 *
 * @Note			- none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLOCK_ENABLE();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLOCK_ENABLE();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLOCK_ENABLE();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLOCK_ENABLE();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLOCK_ENABLE();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLOCK_ENABLE();
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
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0;
	// 1. configure the mode of gpio pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		// non-interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else
	{
		if(GPIO_MODE_IT_FT == pGPIOHandle->GPIO_PinConfig.GPIO_PinMode)
		{
			/* 1. Configure the FTSR */
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			/* Clear the corresponding RTSR bit */
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(GPIO_MODE_IT_RT == pGPIOHandle->GPIO_PinConfig.GPIO_PinMode)
		{
			/* 1. Configure the RTSR */
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			/* Clear the corresponding RTSR bit */
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}
		else if(GPIO_MODE_IT_RFT == pGPIOHandle->GPIO_PinConfig.GPIO_PinMode)
		{
			/* 1. Configure the FTSR and RTSR */
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			/* Clear the corresponding RTSR bit */
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		/* 2. Configure the GPIO port selection in SYSCFG_EXTICR */
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << (temp2 * 4);

		/* 3. Enable the EXTI interrupt delivery using IMR */
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}


	temp = 0;

	// 2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	temp = 0;

	// 3. configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	temp = 0;

	// 4.configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType));
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;

	// 5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		// configure the alt function registers.
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;

		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
}

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = ((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)pGPIOx->IDR;
	return value;
}
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}

/*
 * IRQ Configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(ENABLE == EnorDi)
	{
		if(IRQNumber <= 31)
		{
			/* program ISER register */
			*NVIC_ISER |= (1 << IRQNumber);
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			/* program ICER register */
			*NVIC_ICER |= (1 << IRQNumber);
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	/* 1. First lets find out the ipr register */
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = ( 8 * iprx_section ) + (8 - NO_PR_BITS_IMPLEMENTED);
	*(NVIC_PR_BASE_ADDR + iprx * 4) |= ( IRQPriority << shift_amount );
}

void GPIO_IRQHandling(uint8_t PinNumber)
{
	/* Clear the exti pr register corresponding to the pin number */
	if(EXTI->PR & (1 << PinNumber))
	{
		/* Clear */
		EXTI->PR |= (1 << PinNumber);
	}
}
