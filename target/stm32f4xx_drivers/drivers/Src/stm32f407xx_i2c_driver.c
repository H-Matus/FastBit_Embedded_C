/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: 14 Oct 2022
 *      Author: henrikass
 */

#include "stm32f407xx_i2c_driver.h"

uint16_t AHB_PreScaler[8] = {2, 4, 8, 16, 64, 128, 256, 512};
uint16_t APB1_PreScaler[4] = {2, 4, 8, 16};

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase_Write(I2C_RegDef_t *pI2Cx, uint8_t PeripheralAddr);
static void I2C_ExecuteAddressPhase_Read(I2C_RegDef_t *pI2Cx, uint8_t PeripheralAddr);
static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx);
static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx);

static void I2C_ExecuteAddressPhase_Write(I2C_RegDef_t *pI2Cx, uint8_t PeripheralAddr)
{
    // Shift address by 1 bit to make space for R/W bit
    PeripheralAddr = PeripheralAddr << 1;
    PeripheralAddr &= ~(1); // PeripheralAddr is Peripheral Address + ( R/W bit = 0 )
    pI2Cx->DR = PeripheralAddr;
}

static void I2C_ExecuteAddressPhase_Read(I2C_RegDef_t *pI2Cx, uint8_t PeripheralAddr)
{
    // Shift address by 1 bit to make space for R/W bit
    PeripheralAddr = PeripheralAddr << 1;
    PeripheralAddr |= 1; // PeripheralAddr is Peripheral Address + ( R/W bit = 1 )
    pI2Cx->DR = PeripheralAddr;
}

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->CR1 |= ( 1 << I2C_CR1_START );
}

static void I2C_ClearADDRFlag(I2C_RegDef_t *pI2Cx)
{
    uint32_t dummyRead = pI2Cx->SR1;
    dummyRead = pI2Cx->SR2;
    (void)dummyRead;
}

static void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->CR1 |= ( 1 << I2C_CR1_STOP );
}

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

    // TRISE configuration
    if(pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
    {
        // mode is standard
        tempreg = (RCC_GetPCLK1Value() / 1000000) + 1;
    }
    else
    {
        // mode is fast mode
        tempreg = ((RCC_GetPCLK1Value() * 300) / 1000000) + 1;
    }
    pI2CHandle->pI2Cx->TRISE = tempreg & 0x3F;

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

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
    if(pI2Cx->SR1 & FlagName)
    {
        return FLAG_SET;
    }
    return FLAG_RESET;
}

void I2C_ControllerSendData(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t PeripheralAddr)
{
    // 1. Generate the START condition
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    // 2. Confirm that start generation is completed by checking the SB flag in the SR1
    // NOTE: until SB is cleared SCL will be stretched (pulled to LOW)
    while(! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)) );

    // 3. Send the address of the slave with R/W bit set to w(0) (total 8 bits)
    I2C_ExecuteAddressPhase_Write(pI2CHandle->pI2Cx, PeripheralAddr);

    // 4. Confirm that address phase is completed by checking the ADDR flag in the SR1
    while(! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)));

    // 5. Clear the ADDR flag according to its software sequence
    // Note: Until ADDr s cleared SCL will be stretched (pulled to LOW)
    I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

    // 6. Send the data until Len becomes 0
    while(Len > 0)
    {
        while(! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)));
        pI2CHandle->pI2Cx->DR = *pTxBuffer;
        pTxBuffer++;
        Len--;
    }

    // 7. When Len become zero wait for TxE = 1 and BTF = 1 before generating the STOP condition
    //  Note: TxE = 1, BTF = 1, means that both SR and DR are empty and next transmission should begin when BTF = 1 SCL will be stretched (pulled to LOW)
    while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE));

    while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF));

    // 8. Generate STOP condition and Controller need not to wait for the completion of stop condition
    //  Note: generating STOP, automatically clears the BTF
    I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
}

void I2C_ControllerReceiveData(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t PeripheralAddr)
{
    // 1. Generate START condition
    I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

    // 2. Confirm that START generation is completed by checking the SB flag in the SR1
    //  Note: Until SB is cleared SCL will be stretched (pulled to LOW)
    while(! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)) );

    // 3. Send the address of the peripheral with R/notW bit set to R(1) (total 8 bits)
    I2C_ExecuteAddressPhase_Read(pI2CHandle->pI2Cx, PeripheralAddr);

    // 4. Wait until address phase is completed by checking the ADDR flag in the SR1
    while(! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)));

    // 5. Procedure to read only 1 byte from peripheral
    if(Len == 1)
    {
        // Disable Acking
        I2C_ManageACKing(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);
        
        // Clear the ADDR Flag
        I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

        // Wait until RXNE becomes 1
        while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE));

        // Generate STOP condition
        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

        // Read data into buffer
        *pRxBuffer = pI2CHandle->pI2Cx->DR;

        return;
    }

    // 5. Procedure to read data from peripheral when Len > 1
    if(Len > 1)
    {
        // Clear the ADDR flag
        I2C_ClearADDRFlag(pI2CHandle->pI2Cx);

        // Read the data until Len becomes zero
        for(uint32_t i = Len; i > 0; i--)
        {
            // Wait until RXNE becomes 1
            while(! (I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)));

            if(i == 2) // if last 2 bytes are remaining
            {
                // Clear the ACK bit
                I2C_ManageACKing(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

                // Generate STOP condition
                I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
            }
            
            // Read the data from data register into buffer
            *pRxBuffer = pI2CHandle->pI2Cx->DR;

            // Increment the buffer address
            pRxBuffer++;
        }
    }

    // re-enable ACKing
    if(pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
    {
        I2C_ManageACKing(pI2CHandle->pI2Cx, I2C_ACK_ENABLE);
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

void I2C_ManageACKing(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
    if(EnOrDi == I2C_ACK_ENABLE)
    {
        // enable the ack
        pI2Cx->CR1 |= ( 1 << I2C_CR1_ACK );
    }
    else
    {
        // disable the ack
        pI2Cx->CR1 &= ~( 1 << I2C_CR1_ACK );
    }
}


/*********************************************************************
 * @fn      		  - I2C_MasterSendDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              -  Complete the below code . Also include the function prototype in header file

 */
uint8_t I2C_ControllerSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t PeripheralAddr)
{
	uint8_t busystate = pI2CHandle->?;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pTxBuffer = ?;
		pI2CHandle->TxLen = ?;
		pI2CHandle->TxRxState = ?;
		pI2CHandle->DevAddr = ?;
		pI2CHandle->Sr = ?;

		//Implement code to Generate START Condition
		

		//Implement the code to enable ITBUFEN Control Bit
		pI2CHandle->pI2Cx->CR2 |= ( 1 << I2C_CR2_ITBUFEN);

		//Implement the code to enable ITEVFEN Control Bit
		

		//Implement the code to enable ITERREN Control Bit
		

	}

	return busystate;
}

/*********************************************************************
 * @fn      		  - I2C_MasterReceiveDataIT
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the below code . Also include the fn prototype in header file

 */
uint8_t I2C_ControllerReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t PeripheralAddr)
{
	uint8_t busystate = pI2CHandle->?;

	if( (busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
	{
		pI2CHandle->pRxBuffer = ?;
		pI2CHandle->RxLen = ?;
		pI2CHandle->TxRxState = ?;
		pI2CHandle->RxSize = Len; //Rxsize is used in the ISR code to manage the data reception 
		pI2CHandle->DevAddr = ?;
		pI2CHandle->Sr = Sr;

		//Implement code to Generate START Condition
		

		//Implement the code to enable ITBUFEN Control Bit
		

		//Implement the code to enable ITEVFEN Control Bit
		

		//Implement the code to enable ITERREN Control Bit
		
	}

	return busystate;
}
