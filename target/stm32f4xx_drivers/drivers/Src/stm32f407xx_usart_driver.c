/*
 * stm32f407xx_usart_driver.c
 *
 *  Created on: 25 Oct 2022
 *      Author: henrikass
 */

#include "stm32f407xx_usart_driver.h"

/**
 * @brief Peripheral Clock Control Setup
 * 
 * @param pUSARTx 
 * @param EnOrDi 
 */
void USART_PeriClockControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        if(pUSARTx == USART1)
        {
            USART1_PCLK_EN();
        }
        else if(pUSARTx == UART2)
        {
            UART2_PCLK_EN();
        }
        else if(pUSARTx == UART3)
        {
            UART3_PCLK_EN();
        }
        else if(pUSARTx == UART4)
        {
            UART4_PCLK_EN();
        }
        else if(pUSARTx == UART5)
        {
            UART5_PCLK_EN();
        }
        else if(pUSARTx == USART6)
        {
            USART6_PCLK_EN();
        }
    }
    else
    {
        if(pUSARTx == USART1)
        {
            USART1_PCLK_DI();
        }
        else if(pUSARTx == UART2)
        {
            UART2_PCLK_DI();
        }
        else if(pUSARTx == UART3)
        {
            UART3_PCLK_DI();
        }
        else if(pUSARTx == UART4)
        {
            UART4_PCLK_DI();
        }
        else if(pUSARTx == UART5)
        {
            UART5_PCLK_DI();
        }
        else if(pUSARTx == USART6)
        {
            USART6_PCLK_DI();
        }
    }
}

void USART_Init(USART_Handle_t *pUSARTHandle)
{

}

void USART_DeInit(USART_RegDef_t *pUSARTx)
{

}

/**
 * @brief Interrupt Config Setup
 * 
 * @param IRQNumber 
 * @param EnorDi 
 */
void USART_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @brief Interrupt Priority Config Setup
 * 
 * @param IRQNumber 
 * @param IRQPriority 
 */
void USART_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
    uint8_t iprx = IRQNumber / 4;
    uint8_t iprx_section = IRQNumber % 4;

    uint8_t shift_amount = (8 * iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED); /* We add the last part, as we need to shift from the beginning of not implemented bits. in Interrupt priority registers. */
    *(NVIC_PR_BASE_ADDR + (iprx * 4)) |= (IRQPriority << shift_amount);
}

/**
 * @brief USART Peripheral Control
 * 
 * @param pUSARTx 
 * @param EnOrDi 
 */
void USART_PeripheralControl(USART_RegDef_t *pUSARTx, uint8_t EnOrDi)
{
    if(EnOrDi == ENABLE)
    {
        pUSARTx->CR1 |= ( 1 << USART_CR1_UE );
    }
    else
    {
        pUSARTx->CR1 &= ~( 1 << USART_CR1_UE );
    }
}

/**
 * @brief USART Flag Status Getter
 * 
 * @param pUSARTx 
 * @param FlagName 
 * @return uint8_t 
 */
uint8_t USART_GetFlagStatus(USART_RegDef_t *pUSARTx , uint32_t FlagName)
{
    if(pUSARTx->SR & FlagName)
    {
        return FLAG_SET;
    }
    return FLAG_RESET;
}

/**
 * @brief USART Flag Clearing function
 * 
 * @param pUSARTx 
 * @param StatusFlagName 
 */
void USART_ClearFlag(USART_RegDef_t *pUSARTx, uint16_t StatusFlagName)
{
    pUSARTx->SR &= ~( 1 << StatusFlagName );
}