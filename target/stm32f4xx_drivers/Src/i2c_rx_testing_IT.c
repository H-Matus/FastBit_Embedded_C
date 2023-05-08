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
    // Configs
    I2C_Handle_t I2C1Handle;

    I2C1_GPIOInits();
    Button_GPIOInit();

    I2C1_Inits(&I2C1Handle);

    // Enable I2C1 peripheral
    I2C_PeripheralControl(I2C1, ENABLE);

    // ACK bit is made 1 after PE=1
    I2C_ManageACKing(I2C1, I2C_ACK_ENABLE);

    // Main code
    uint8_t DataLength = 0;
    char data[DataLength];
    uint8_t len_request = 0x51;
    uint8_t data_request = 0x52;

    uint8_t *pDataLength = &DataLength;

    while( ! GPIO_ReadFromInputPin(GPIOA,GPIO_PIN_NO_0));

    // Ask for length
    while(I2C_ControllerSendDataIT(&I2C1Handle, len_request, sizeof(uint8_t), 0x2, I2C_ENABLE_SR) != I2C_READY);
        
    // Get the length 
    while(I2C_ControllerReceiveDataIT(&I2C1Handle, &pDataLength, sizeof(uint8_t), 0x2, I2C_ENABLE_SR) != I2C_READY);

    // Send request for data
    while(I2C_ControllerSendDataIT(&I2C1Handle, data_request, sizeof(uint8_t), 0x2, I2C_ENABLE_SR) != I2C_READY);

    // Get the data
    while(I2C_ControllerReceiveDataIT(&I2C1Handle, DataLength, sizeof(uint8_t), 0x2, I2C_DISABLE_SR) != I2C_READY);

    data[DataLength+1] = '\0';

    while(1);

    return 0;
}

