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

void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
    while(Len > 0)
    {
        // 1. Wait until TXE is set
        while( SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_RESET );

        // 2. Check the DFF bit in CR2
        if(pSPIx->CR1 & ( 1 << SPI_CR1_DFF ))
        {
            // 16-bit DFF
            *((uint16_t*)pRxBuffer) = pSPIx->DR; 
            Len--;
            Len--;
            (uint16_t*)pRxBuffer++;
        }
        else
        {
            // 8-bit DFF
            *(pRxBuffer) = pSPIx->DR;
            Len--;
            pRxBuffer++;
        }
    }

}

void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        pSPIx->CR1 |= ( 1 << SPI_CR1_SPE );
    }
    else
    {
        pSPIx->CR1 &= ~( 1 << SPI_CR1_SPE );
    }
}

void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if(IRQNumber <= 31)
        {
            *NVIC_ISER0 |= ( 1 << IRQNumber );
        }
        else if(IRQNumber > 31 && IRQNumber < 64)
        {
            *NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
        }
        else if(IRQNumber > 64 && IRQNumber < 96)
        {
            *NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
        }
    }
    else
    {
        if(IRQNumber <= 31)
        {
            *NVIC_ICER0 |= ( 1 << IRQNumber );
        }
        else if(IRQNumber > 31 && IRQNumber < 64)
        {
            *NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
        }
        else if(IRQNumber > 64 && IRQNumber < 96)
        {
            *NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
        }
    }
}

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED); /* We add the last part, as we need to shift from the beginning of not implemented bits. in Interrupt priority registers. */
    *(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount);
}

uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
    uint8_t state = pSPIHandle->TxState;

    if(state != SPI_BUSY_IN_TX)
    {
        // 1. Save the Tx buffer address and Len information in some global variables
        pSPIHandle->pTxBuffer = pTxBuffer;
        pSPIHandle->TxLen = Len;

        // 2. Mark the SPI state as busy in transmission so that
        //      no other code can take over same SPI peripheral until transmission is over
        pSPIHandle->TxState = SPI_BUSY_IN_TX;

        // 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
        pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE );
    }

    return state;
}

uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
    uint8_t state = pSPIHandle->RxState;

    if(state != SPI_BUSY_IN_RX)
    {
        // 1. Save the Tx buffer address and Len information in some global variables
        pSPIHandle->pRxBuffer = pRxBuffer;
        pSPIHandle->RxLen = Len;

        // 2. Mark the SPI state as busy in transmission so that
        //      no other code can take over same SPI peripheral until transmission is over
        pSPIHandle->RxState = SPI_BUSY_IN_RX;

        // 3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
        pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE );
    }

    return state;

}

void SPI_IRQHandling(SPI_Handle_t *pHandle)
{

}