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

    // Enable SPI clock so that the user wouldn't have to do it themselves
    SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

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

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
    if(pSPIx->SR & FlagName)
    {
        return FLAG_SET;
    }
    return FLAG_RESET;
}

/**
 * @brief SPI Send Data API
 * 
 * @param pSPIx 
 * @param pTxBuffer 
 * @param Len 
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
    while(Len > 0)
    {
        // 1. Wait until TXE is set
        while( SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_RESET );

        // 2. Check the DFF bit in CR2
        if(pSPIx->CR1 & ( 1 << SPI_CR1_DFF ))
        {
            // 16-bit DFF
            pSPIx->DR = *((uint16_t*)pTxBuffer);
            Len--;
            Len--;
            (uint16_t*)pTxBuffer++;
        }
        else
        {
            // 8-bit DFF
            pSPIx->DR = *(pTxBuffer);
            Len--;
            pTxBuffer++;
        }
    }
}
