/*
 * 001led_toggle.c
 *
 *  Created on: Sep 15, 2022
 *      Author: henrikass
 */

#include "stm32f407xx.h"

int main(void)
{
	GPIO_Handle_t LED;
	LED.pGPIOx = GPIOA;
	LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
	LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	LED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	LED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	LED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	GPIO_Handle_t Button;
	Button.pGPIOx = GPIOB;
	Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&LED);
	GPIO_Init(&Button);

	for(;;)
	{
		if(1 == GPIO_ReadFromInputPin(GPIOB, 12))
		{
			GPIO_WriteToOutputPin(GPIOA, 1, 0);
		}
		else
		{
			GPIO_WriteToOutputPin(GPIOA, 1, 1);
		}
	}

	return 0;
}

void EXTI0_IRQHandler(void)
{
    GPIO_IRQHandling(0); /* 0 is the pin number, which will be detecting interrupt requests (IRQs) */
}