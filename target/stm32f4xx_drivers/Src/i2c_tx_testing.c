/*
 * i2c_tx_testing.c
 *
 *  Created on: 18 Oct 2022
 *      Author: henrikass
 */

#include "stm32f407xx.h"
#include <string.h>

/**
 * PB6 --> I2C1_SCL
 * PB7 --> I2C1_SDA
 * Alternate mode function = 4
 */

void I2C1_GPIOInits(void)
{
    GPIO_Handle_t I2CPins;

    I2CPins.pGPIOx = GPIOB;
    I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
    I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    I2CPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    // SCL
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_6;
    GPIO_Init(&I2CPins);

    // SDA
    I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_7;
    GPIO_Init(&I2CPins);
}

void Button_GPIOInit(void)
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

void I2C1_Inits(I2C_Handle_t *pI2C_Handle)
{
    pI2C_Handle->pI2Cx = I2C1;
    pI2C_Handle->I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
    pI2C_Handle->I2C_Config.I2C_DeviceAddress = 0x1;
    pI2C_Handle->I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE; // Generates sclk of 8MHz

    I2C_Init(pI2C_Handle);
}

int main(void)
{
    I2C_Handle_t I2C1Handle;
    char user_data[] = "Hello World";

    I2C1_GPIOInits();
    Button_GPIOInit();

    I2C1_Inits(&I2C1Handle);

    // Enable I2C1 peripheral
    I2C_PeripheralControl(I2C1, ENABLE);

    if(GPIO_ReadFromInputPin(GPIOA, 0))
    {
        I2C_ControllerSendData(&I2C1Handle, (uint8_t*)user_data, strlen(user_data), 1);
    }

    while(1);

    return 0;
}
