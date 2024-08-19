/*
 * 006spi_tx_testing.c
 *
 *  Created on: 7 Oct 2022
 *      Author: henrikass
 */

#include "stm32f407xx.h"
#include <string.h>

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
    SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
    GPIO_Init(&SPIPins);

    // NSS
    // SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    // GPIO_Init(&SPIPins);
}

void SPI2_Inits(void)
{
    SPI_Handle_t SPI2Handle;

    SPI2Handle.pSPIx = SPI2;
    SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_HD;
    SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_PERI;
    SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV256; // Generates sclk of 8MHz
    SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BITS;
    SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
    SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
    SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN; // Sofware slave management enabled for NSS pin

    SPI_Init(&SPI2Handle);
}

int main(void)
{
    char tx_data[] = "Hello";
    char tx_data2[] = "Data.";
    char tx_err_data[] = "Error";
    uint8_t rx_data = 0x0;
    char rx_data2[5] = "";

    SPI2_GPIOInits();

    SPI2_Inits();

    SPI_SSIConfig(SPI2, DISABLE);

    SPI2->CR1 &= ~( 1 << SPI_CR1_MSTR );

    SPI2->CR1 &= ~( 1 << SPI_CR1_BIDIOE );
    SPI_PeripheralControl(SPI2, ENABLE);

    SPI_ReceiveData(SPI2, &rx_data, 1);
    //SPI_ReceiveData(SPI2, (uint8_t *)rx_data, 4);
    while( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) );

    SPI_PeripheralControl(SPI2, DISABLE);
    SPI2->CR1 |= ( 1 << SPI_CR1_BIDIOE );

    // Enable SPI2 peripheral
    SPI_PeripheralControl(SPI2, ENABLE);

    if(0x1 == rx_data)
    {
        SPI_SendData(SPI2, (uint8_t *)tx_data, strlen(tx_data));
    }
    else if(0x2 == rx_data)
    {
        SPI_SendData(SPI2, (uint8_t *)tx_data2, strlen(tx_data));
    }
    else
    {
        SPI_SendData(SPI2, (uint8_t *)tx_err_data, strlen(tx_data));
    }

    while( SPI_GetFlagStatus(SPI2, SPI_TXE_FLAG) == FLAG_RESET );
    while( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) );

    SPI_PeripheralControl(SPI2, DISABLE);
    SPI2->CR1 &= ~( 1 << SPI_CR1_BIDIOE );
    SPI_PeripheralControl(SPI2, ENABLE);

    SPI_ReceiveData(SPI2, (uint8_t *)rx_data2, 5);
    while( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) );

    SPI_PeripheralControl(SPI2, DISABLE);

    //SPI_ReceiveData(SPI2, (uint8_t *)rx_data, 1);

//    //lets confirm SPI is not busy
    //while( SPI_GetFlagStatus(SPI2,SPI_BUSY_FLAG) );

//
//    SPI_PeripheralControl(SPI2, DISABLE);
//
//    SPI2->CR1 |= ( 1 << SPI_CR1_BIDIOE );
//
//    SPI_PeripheralControl(SPI2, ENABLE);
//
//    if(rx_data[0] == 0x51)
//    {
//    	SPI_SendData(SPI2, (uint8_t *)tx_data, strlen(tx_data));
//    }
//    else
//    {
//    	SPI_SendData(SPI2, (uint8_t *)tx_error_data, strlen(tx_data));
//    }
//
//    //lets confirm SPI is not busy
//    while( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) );

//    SPI_ReceiveData(SPI2, &rx_data, 1);
//    while( SPI_GetFlagStatus(SPI2, SPI_BUSY_FLAG) );
//
//    if(rx_data == 0x1)
//    {
//    	SPI_SendData(SPI2, (uint8_t *)tx_data, 2);
//    	SPI_ReceiveData(SPI2, &rx_data2, 1);
//    }
//    else if(rx_data == 0x2)
//    {
//    	SPI_SendData(SPI2, (uint8_t *)tx_data2, 2);
//    	SPI_ReceiveData(SPI2, &rx_data2, 1);
//    }
//    else
//    {
//
//    }



    while(1)
    {

    }

    return 0;
}
