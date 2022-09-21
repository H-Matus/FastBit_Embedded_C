/*
 * 003IRQ_led.c
 *
 *  Created on: Sep 21, 2022
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
	Button.pGPIOx = GPIOD;
	Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_PeriClockControl(GPIOD, ENABLE);
	GPIO_Init(&LED);
	GPIO_Init(&Button);

    GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE);
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIORITY_15);

	for(;;)
	{

	}

	return 0;
}

void EXTI9_5_IRQHandler(void)
{
    GPIO_IRQHandling(GPIO_PIN_NO_5);
    GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_NO_1);
}

