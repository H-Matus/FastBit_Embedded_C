/*
 * 015uart_tx.c
 *
 *  Created on: 14 May 2023
 *      Author: Admin
 */

#include <string.h>
#include "stm32f407xx.h"

char msg[1024] = "UART Tx testing...\n\r";

USART_Handle_t usart1_handle;

void delay(void)
{
    for(uint32_t i = 0; i < 500000/2; i++);
}

void GPIO_ButtonInit(void)
{
    GPIO_Handle_t Button;

    Button.pGPIOx = GPIOA;
    Button.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    Button.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    Button.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    Button.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PD;
    Button.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    GPIO_Init(&Button);
}

void USART1_Init(void)
{
    usart1_handle.pUSARTx = USART1;
    usart1_handle.USART_Config.USART_Baud = USART_STD_BAUD_115200;
    usart1_handle.USART_Config.USART_HWFlowControl = USART_HW_FLOW_CTRL_NONE;
    usart1_handle.USART_Config.USART_Mode = USART_MODE_ONLY_TX;
    usart1_handle.USART_Config.USART_NoOfStopBits = USART_STOPBITS_1;
    usart1_handle.USART_Config.USART_WordLength = USART_WORDLEN_8BITS;
    usart1_handle.USART_Config.USART_ParityControl = USART_PARITY_DISABLE;
    USART_Init(&usart1_handle);
}

void USART1_GPIOInit(void)
{
    GPIO_Handle_t usart_gpios;

    usart_gpios.pGPIOx = GPIOA;
    usart_gpios.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    usart_gpios.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    usart_gpios.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
    usart_gpios.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    usart_gpios.GPIO_PinConfig.GPIO_PinAltFunMode = 7;

    // USART2 TX
    usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_2;
    GPIO_Init(&usart_gpios);

    // USART2 RX
    usart_gpios.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_3;
    GPIO_Init(&usart_gpios);
}

int main(void)
{
    GPIO_ButtonInit();

	USART1_GPIOInit();
    USART1_Init();

    USART_PeripheralControl(USART1, ENABLE);

    // wait till the button is pressed
    while(! GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0));

    // to avoid button debouncing
    delay();

    USART_SendData(&usart1_handle, (uint8_t*)msg, strlen(msg));

    while(1);

	return 0;
}
