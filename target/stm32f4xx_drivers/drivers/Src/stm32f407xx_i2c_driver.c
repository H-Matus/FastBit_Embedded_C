/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: 14 Oct 2022
 *      Author: henrikass
 */

#include "stm32f407xx_i2c_driver.h"

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint16_t APB1_PreScaler[4] = {2, 4, 8, 16};

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

uint32_t RCC_GetPLLOutput()
{
    return;
}

uint32_t RCC_GetPCLK1Value(void)
{
    uint32_t pclk1, SystemClk;
    uint8_t clksrc, temp, ahbp, apb1p;
    clksrc = (( RCC->CFGR >> 2 ) & 0x3 );

    if( clksrc == 0 )
    {
        SystemClk = 16000000;
    }
    else if( clksrc == 1 )
    {
        SystemClk = 8000000;
    }
    else if( clksrc == 2 )
    {
        SystemClk = RCC_GetPLLOutput();
    }

    temp = ( ( RCC->CFGR >> 4 ) & 0xF );

    // AHB Prescaler calculations
    if(temp < 8)
    {
        ahbp = 1;
    }
    else
    {
        ahbp = AHB_PreScaler[temp-8];
    }

    temp = ( ( RCC->CFGR >> 10 ) & 0x7 );

    // APB Prescaler calculations
    if(temp < 4)
    {
        apb1p = 1;
    }
    else
    {
        apb1p = APB1_PreScaler[temp-4];
    }

    pclk1 = ( SystemClk / ahbp ) / apb1p;

    return pclk1;
}

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
    uint32_t tempreg = 0;

    // ACK control bit (CR1)
    tempreg |= ( pI2CHandle->I2C_Config.I2C_ACKControl << 10 );
    pI2CHandle->pI2Cx->CR1 = tempreg;

    // Configure the FREQ field of CR2 (CR2)
    tempreg = 0;
    tempreg |= RCC_GetPCLK1Value() / 1000000U;
    pI2CHandle->pI2Cx->CR2 = ( tempreg & 0x3F ); // We are bit masking the first 6 bits for safety purposes.

    // Program device's own address (OAR1)
    tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
    tempreg |= ( 1 << 14 );
    pI2CHandle->pI2Cx->OAR1 = tempreg;

    // Init CCR (CCR calculations)
    uint16_t ccr_value = 0;
    tempreg = 0;
    if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
    {
        // mode is standard
        ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        tempreg |= (ccr_value & 0xFFF); // Masking out other bits, as CCR is only 12-bit length, whereas ccr_value is 16-bit
    }
    else
    {
        // mode is fast
        tempreg |= ( 1 << 15 ); // Fast mode is configured
        tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

        if(pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
        {
            ccr_value = (RCC_GetPCLK1Value() / (3 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        }
        else
        {
            ccr_value = (RCC_GetPCLK1Value() / (25 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        }
        tempreg |= (ccr_value & 0xFFF);
    }
    pI2CHandle->pI2Cx->CCR = tempreg;
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
