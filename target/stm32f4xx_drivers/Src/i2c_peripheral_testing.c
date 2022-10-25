/*
 * i2c_peripheral_testing.c
 *
 *  Created on: 25 Oct 2022
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
    // Configs
    I2C_Handle_t I2C1Handle;

    I2C1_GPIOInits();

    I2C1_Inits(&I2C1Handle);

    // Enable I2C1 peripheral
    I2C_PeripheralControl(I2C1, ENABLE);

    // Main code
    uint8_t len_request = 0x51;
    uint8_t data_request = 0x52;

    uint8_t data_length = 29;
    char data[29] = "This is STM32 board replying";

    while(1)
    {
        if(I2C_PeripheralReceiveData(I2C1) == len_request)
        {
            I2C_PeripheralSendData(I2C1, data_length);
        }

        if(I2C_PeripheralReceiveData(I2C1) == data_request)
        {
            I2C_PeripheralSendData(I2C1, data);
        }

    }

    return 0;
}