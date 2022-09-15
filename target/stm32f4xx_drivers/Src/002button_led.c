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
	LED.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	LED.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	LED.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	LED.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
	LED.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	GPIO_Handle_t Button;
	Button.pGPIOx = GPIOB;
	Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PD;
	Button.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOB, ENABLE);
	GPIO_Init(&LED);
	GPIO_Init(&Button);

	for(;;)
	{
		if(1 == GPIO_ReadFromInputPin(GPIOB, 12))
		{
			GPIO_WriteToOutputPin(GPIOA, 14, 1);
		}
		else
		{
			GPIO_WriteToOutputPin(GPIOA, 14, 0);
		}
	}

	return 0;
}
