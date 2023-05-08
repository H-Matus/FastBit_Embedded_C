/*
 * 007spi_tx_arduino_only.c
 *
 *  Created on: 5 Jan 2023
 *      Author: Admin
 */

#include "stm32f407xx.h"
#include <string.h>

void delay(void)
{
    for(uint32_t i = 0; i < 500000/2; i++);
}

/**
 * PB14 --> SPI2_MISO
 * PB15 --> SPI2_MOSI
 * PB13 --> SPI2_SCLK
 * PB12 --> SPI2_NSS
 * Alternate mode function = 5
 */

void SPI2_GPIOInits(void)
{
    GPIO_Handle_t SPIPins;

    SPIPins.pGPIOx = GPIOB;
    SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
    SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    SPIPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    // SCLK
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
    GPIO_Init(&SPIPins);

    // MOSI
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
    GPIO_Init(&SPIPins);

    // MISO
    // SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    // GPIO_Init(&SPIPins);

    // NSS
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
    SPI_Handle_t SPI2Handle;

    SPI2Handle.pSPIx = SPI2;
    SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
    SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MAIN;
    SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; // Generates sclk of 2MHz
    SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI; // Sofware slave management enabled for NSS pin

    SPI_Init(&SPI2Handle);
}

void Button_GPIOInits(void)
{
    GPIO_Handle_t ButtonPin;

    ButtonPin.pGPIOx = GPIOA;
    ButtonPin.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_0;
    ButtonPin.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    ButtonPin.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    ButtonPin.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

    GPIO_PeriClockControl(GPIOA, ENABLE);

    GPIO_Init(&ButtonPin);
}

int main(void)
{
    char user_data[] = "Hello World";

    SPI2_GPIOInits();

    Button_GPIOInits();

    SPI2_Inits();

    //SPI_SSIConfig(SPI2, ENABLE);

    SPI_SSOEConfig(SPI2, ENABLE);

    while(1)
    {
        while( !GPIO_ReadFromInputPin(GPIOA, GPIO_PIN_NO_0) );

        delay();

        // Enable SPI2 peripheral
        SPI_PeripheralControl(SPI2, ENABLE);

        SPI_SendData(SPI2, (uint8_t*)user_data, strlen(user_data));

        // Let's confirm SPI is not busy.
        while( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) );

        SPI_PeripheralControl(SPI2, DISABLE);
    }

    return 0;
}
