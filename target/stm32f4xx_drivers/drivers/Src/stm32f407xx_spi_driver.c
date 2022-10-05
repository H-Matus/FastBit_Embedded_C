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
    uint32_t temp = 0;

    

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