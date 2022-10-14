/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: 14 Oct 2022
 *      Author: henrikass
 */

#include "stm32f407xx_i2c_driver.h"

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if(EnorDi == ENABLE)
    {
        if(pI2Cx == I2C1)
        {
            I2C1_PCLK_EN();
        }
        else if(pI2Cx == I2C2)
        {
            I2C2_PCLK_EN();
        }
        else if(pI2Cx == I2C3)
        {
            I2C3_PCLK_EN();
        }
    }
    else
    {
        if(pI2Cx == I2C1)
        {
            I2C1_PCLK_DI();
        }
        else if(pI2Cx == I2C2)
        {
            I2C2_PCLK_DI();
        }
        else if(pI2Cx == I2C3)
        {
            I2C3_PCLK_DI();
        }
    }
}

/**
 * @brief I2C DeInit
 * 
 * @param pI2Cx 
 */
void I2C_DeInit(I2C_RegDef_t *pI2Cx)
{
    /* Resetting all registers in the peripheral */
    if(pI2Cx == I2C1)
    {
        I2C1_REG_RESET();
    }
    else if(pI2Cx == I2C2)
    {
        I2C2_REG_RESET();
    }
    else if(pI2Cx == I2C3)
    {
        I2C3_REG_RESET();
    }
}

/**
 * @brief 
 * 
 * @param IRQNumber 
 * @param EnorDi 
 */
void I2C_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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

/**
 * @brief I2C IRQ Priority Config
 * 
 * @param IRQNumber 
 * @param IRQPriority 
 */
void I2C_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED); /* We add the last part, as we need to shift from the beginning of not implemented bits. in Interrupt priority registers. */
    *(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount);
}

/**
 * @brief 
 * 
 * @param pI2Cx 
 * @param EnOrDi 
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        pI2Cx->CR1 |= ( 1 << I2C_CR1_PE );
    }
    else
    {
        pI2Cx->CR1 &= ~( 1 << I2C_CR1_PE );
    }
}
