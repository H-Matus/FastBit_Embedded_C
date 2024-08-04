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
    I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
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
    pI2C_Handle->I2C_Config.I2C_DeviceAddress = 0x0;
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

    I2C_ManageACKing(I2C1,I2C_ACK_ENABLE);

    // Main code
    //uint8_t len_request = 0x51;
    //uint8_t data_request = 0x52;

    //uint8_t data_length = 0x8;
    //uint8_t data = 0x53;

    char data_input[8] = "";

    while(1)
    {
    	(void)I2C_PeripheralReceiveData2( &I2C1Handle, (uint8_t *)data_input, 8 );
    }

    return 0;
}
