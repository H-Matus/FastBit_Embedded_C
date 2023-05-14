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

#define PERI_ADDR   0x68
#define MY_ADDR     PERI_ADDR

void delay(void)
{
    for(uint32_t i = 0; i < 500000/2; i++);
}

I2C_Handle_t I2C1Handle;

uint8_t tx_buf[32] = "Tx is complete.\n";

/**
 * PB6 -> SCL 
 * PB7 -> SDA
 */

void I2C1_GPIOInits(void)
{
    GPIO_Handle_t I2CPins;

    I2CPins.pGPIOx = GPIOB;
    I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    I2CPins.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
    I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PU;
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

void I2C1_Inits(void)
{
    I2C1Handle.pI2Cx = I2C1;
    I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;
    I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
    I2C1Handle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE; // Generates sclk of 8MHz
    I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;

    I2C_Init(&I2C1Handle);
}

int main(void)
{
    // Configs
    I2C1_GPIOInits();
    Button_GPIOInit();

    I2C1_Inits(&I2C1Handle);

    // I2C IRQ Configs
    I2C_IRQInterruptConfig(IRQ_NO_I2C1_EV, ENABLE);
    I2C_IRQInterruptConfig(IRQ_NO_I2C1_ER, ENABLE);

    // Enables interrupt control bits in the CR register
    I2C_PeriEnableDisableCallbackEvents(I2C1, ENABLE);

    // Enable I2C1 peripheral
    I2C_PeripheralControl(I2C1, ENABLE);

    // ACK bit is made 1 after PE=1
    I2C_ManageACKing(I2C1, I2C_ACK_ENABLE);

    while(1){ };
}

void I2C1_EV_IRQHandler(void)
{
    I2C_EV_IRQHandling(&I2C1Handle);
}

void I2C1_ER_IRQHandler(void)
{
    I2C_ER_IRQHandling(&I2C1Handle);
}


void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle, uint8_t AppEv)
{
    // Static variables are actually in global memory space, but they can only be accessed locally in this function.
    static uint8_t commandCode = 0;
    static uint8_t counter = 0;

    if(AppEv == I2C_EV_DATA_REQ)
    {
        // Main wants some data. Peri has to send it.
        if(commandCode == 0x51)
        {
            // Send the length information to the master
            I2C_PeripheralSendData(pI2CHandle->pI2Cx, strlen((char*)tx_buf));
            // After this I2C_ERROR_AF (ACK Failure) happens to indicate that peripheral finished sending data.
        }
        else if(commandCode == 0x52)
        {
            // Send the contents of Tx_buf
            I2C_PeripheralSendData(pI2CHandle->pI2Cx, tx_buf[counter++]);
        }
    }
    else if(AppEv == I2C_EV_DATA_RCV)
    {
        // Data is waiting for the peri to read. Peripheral has to read the data.
        commandCode = I2C_PeripheralReceiveData(pI2CHandle->pI2Cx);
    }
    else if(AppEv == I2C_ERROR_AF)
    {
        // This happens only during peri Txing;
        // Main has to send NACK,
        // so that peri would understand that main doesn't need more data.
        commandCode = 0xFF;
        counter = 0;
    }
    else if(AppEv == I2C_EV_STOP)
    {
        // This happens only during peri reception.
        // Main has ended the I2C comms with peri.
    }

}
