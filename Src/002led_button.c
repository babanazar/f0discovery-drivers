/*
 * 002led_button.c
 *
 *  Created on: Jan 6, 2021
 *      Author: Babanazar
 */


#include "stm32f0discovery.h"

#define HIGH			ENABLE
#define BUTTON_PRESSED	HIGH

void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{
	GPIO_Handle_t gpioLed, gpioButton;

	gpioLed.pGPIOx = GPIOC;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_9;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPPD;

	GPIO_PeriClockControl(GPIOC, ENABLE);

	GPIO_Init(&gpioLed);


	gpioButton.pGPIOx = GPIOA;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPPD;

	GPIO_PeriClockControl(GPIOA, ENABLE);

	GPIO_Init(&gpioButton);


	while(1)
	{
		if(BUTTON_PRESSED == GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0))
		{
			delay();
			GPIO_ToggleOutputPin(GPIOC, GPIO_PIN_NO_9);
		}

	}

	return 0;
}
