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
        tempreg &= ~( 1 << 15 );
    }
    else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
    {
        // bidi mode should be set
        tempreg |= ( 1 << 15 );
    }
    else if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
    {
        // bidi mode should be cleared
        tempreg &= ~( 1 << 15 );

        // RXONLY bit must be set
        tempreg |= ( 1 << 10 );
    }

    // 3. Configure the clock speed
    if(pSPIHandle->SPIConfig.SPI_SclkSpeed == SPI_SCLK_SPEED_DIV2)
    {
        tempreg &= ~( 1 << 3 );
    }
    else if(pSPIHandle->SPIConfig.SPI_SclkSpeed == SPI_SCLK_SPEED_DIV4)
    {
        tempreg |= ( 1 <<  )
    }
    else if(pSPIHandle->SPIConfig.SPI_SclkSpeed == SPI_SCLK_SPEED_DIV8)
    {

    }
    else if(pSPIHandle->SPIConfig.SPI_SclkSpeed == SPI_SCLK_SPEED_DIV16)
    {

    }
    else if(pSPIHandle->SPIConfig.SPI_SclkSpeed == SPI_SCLK_SPEED_DIV32)
    {

    }
    else if(pSPIHandle->SPIConfig.SPI_SclkSpeed == SPI_SCLK_SPEED_DIV64)
    {

    }
    else if(pSPIHandle->SPIConfig.SPI_SclkSpeed == SPI_SCLK_SPEED_DIV128)
    {

    }
    else if(pSPIHandle->SPIConfig.SPI_SclkSpeed == SPI_SCLK_SPEED_DIV256)
    {

    }

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