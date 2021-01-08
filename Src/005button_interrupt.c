/*
 * 005button_interrupt.c
 *
 *  Created on: Jan 8, 2021
 *      Author: Babanazar
 */


#include <stm32f051xx.h>

#define HIGH			ENABLE
#define BUTTON_PRESSED	HIGH


int main(void)
{
	GPIO_Handle_t gpioLed, gpioButton;

	gpioLed.pGPIOx = GPIOC;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&gpioLed);

	/* Button GPIO configuration */
	gpioButton.pGPIOx = GPIOA;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&gpioButton);


	/* IRQ configuration */
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI0_1, NVIC_IRQ_PRI_15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI0_1, ENABLE);

	while(1);

	return 0;
}


void EXTI0_1_IRQHandler(void)
{
	GPIO_IRQHandling(GPIO_PIN_NO_0);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_0);
}
