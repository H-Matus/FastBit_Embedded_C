/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 5 Oct 2022
 *      Author: henrikass
 */

#include "stm32f407xx_spi_driver.h"

/**
 * @brief SPI Clock init
 * 
 * @param pSPIx 
 * @param EnorDi 
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if(pSPIx == SPI1)
        {
            SPI1_PCLK_EN();
        }
        else if(pSPIx == SPI2)
        {
            SPI2_PCLK_EN();
        }
        else if(pSPIx == SPI3)
        {
            SPI3_PCLK_EN();
        }
    }
    else
    {
        if(pSPIx == SPI1)
        {
            SPI1_PCLK_DI();
        }
        else if(pSPIx == SPI2)
        {
            SPI2_PCLK_DI();
        }
        else if(pSPIx == SPI3)
        {
            SPI3_PCLK_DI();
        }
    }
}

/**
 * @brief SPI Init 
 * 
 * @param pSPIHandle 
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
    uint32_t tempreg = 0;

    // 1. Configure the device mode
    tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << 2;

    // 2. Configure the bus config
    if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
    {
        // bidi mode should be cleared
        tempreg &= ~( 1 << SPI_CR1_BIDIMODE );
    }
    else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
    {
        // bidi mode should be set
        tempreg |= ( 1 << SPI_CR1_BIDIMODE );
    }
    else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
    {
        // bidi mode should be cleared
        tempreg &= ~( 1 << SPI_CR1_BIDIMODE );

        // RXONLY bit must be set
        tempreg |= ( 1 << SPI_CR1_RXONLY );
    }

    // 3. Configure the clock speed
    tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

    // 4. Configure the DFF
    tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

    // 5. Configure the CPOL
    tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

    // 6. Configure the CPHA
    tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

    pSPIHandle->pSPIx->CR1 = tempreg;
}

/**
 * @brief SPI De-Init
 * 
 * @param pSPIx 
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
    /* Resetting all registers in the peripheral */
    if(pSPIx == SPI1)
    {
        SPI1_REG_RESET();
    }
    else if(pSPIx == SPI2)
    {
        SPI2_REG_RESET();
    }
    else if(pSPIx == SPI3)
    {
        SPI3_REG_RESET();
    }
}