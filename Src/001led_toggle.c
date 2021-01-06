/*
 * 001led_toggle.c
 *
 *  Created on: Dec 22, 2020
 *      Author: Babanazar
 */
#include "stm32f0discovery.h"


void delay(void)
{
	for(uint32_t i = 0; i < 1000000; i++);
}

int main(void)
{
	GPIO_Handle_t gpioLed;

	gpioLed.pGPIOx = GPIOC;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&gpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_9);
		delay();
	}

	return 0;
}
