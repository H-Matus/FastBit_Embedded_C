/*
 * stm32f407xx_i2c_driver.c
 *
 *  Created on: 14 Oct 2022
 *      Author: henrikass
 */

#include "stm32f407xx_i2c_driver.h"

static void I2C_GenerateStartCondition(I2C_RegDef_t *pI2Cx);
static void I2C_ExecuteAddressPhase_Write(I2C_RegDef_t *pI2Cx, uint8_t PeripheralAddr);
static void I2C_ExecuteAddressPhase_Read(I2C_RegDef_t *pI2Cx, uint8_t PeripheralAddr);
static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle);
static void I2C_ControllerHandleTXEInterrupt(I2C_Handle_t *pI2CHandle);
static void I2C_ControllerHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle);

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
    pI2Cx->CR1 |= (1 << I2C_CR1_START);
}

static void I2C_ClearADDRFlag(I2C_Handle_t *pI2CHandle)
{
    uint32_t dummy_read;
    // Check the device mode
    if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
    {
        // device is in controller mode
        if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
        {
            if (pI2CHandle->RxSize == 1)
            {
                // first disable the ack
                I2C_ManageACKing(pI2CHandle->pI2Cx, DISABLE);

                // clear the ADDR flag
                dummy_read = pI2CHandle->pI2Cx->SR1;
                dummy_read = pI2CHandle->pI2Cx->SR2;
                (void)dummy_read;
            }
        }
        else
        {
            // clear the addr flag ( read sr1, read sr2 )
            dummy_read = pI2CHandle->pI2Cx->SR1;
            dummy_read = pI2CHandle->pI2Cx->SR2;
            (void)dummy_read;
        }
    }
    else
    {
        // Device is in Peripheral mode
        // clear the addr flag ( read sr1, read sr2 )
        dummy_read = pI2CHandle->pI2Cx->SR1;
        dummy_read = pI2CHandle->pI2Cx->SR2;
        (void)dummy_read;
    }
}

void I2C_GenerateStopCondition(I2C_RegDef_t *pI2Cx)
{
    pI2Cx->CR1 |= (1 << I2C_CR1_STOP);
}

void I2C_PeriEnableDisableCallbackEvents(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);
        pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);
        pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    }
    else
    {
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);
        pI2Cx->CR2 &= ~(1 << I2C_CR2_ITERREN);
    }
}

void I2C_PeriClockControl(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
    if (EnorDi == ENABLE)
    {
        if (pI2Cx == I2C1)
        {
            I2C1_PCLK_EN();
        }
        else if (pI2Cx == I2C2)
        {
            I2C2_PCLK_EN();
        }
        else if (pI2Cx == I2C3)
        {
            I2C3_PCLK_EN();
        }
    }
    else
    {
        if (pI2Cx == I2C1)
        {
            I2C1_PCLK_DI();
        }
        else if (pI2Cx == I2C2)
        {
            I2C2_PCLK_DI();
        }
        else if (pI2Cx == I2C3)
        {
            I2C3_PCLK_DI();
        }
    }
}

void I2C_Init(I2C_Handle_t *pI2CHandle)
{
    uint32_t tempreg = 0;

    I2C_PeriClockControl(I2C1, ENABLE);

    // ACK control bit (CR1)
    tempreg = 0;
    tempreg |= (pI2CHandle->I2C_Config.I2C_ACKControl << 10);
    pI2CHandle->pI2Cx->CR1 = tempreg;

    // Configure the FREQ field of CR2 (CR2)
    tempreg = 0;
    tempreg |= RCC_GetPCLK1Value() / 1000000U;
    pI2CHandle->pI2Cx->CR2 = (tempreg & 0x3F); // We are bit masking the first 6 bits for safety purposes.

    // Program device's own address (OAR1)
    tempreg = 0;
    tempreg |= pI2CHandle->I2C_Config.I2C_DeviceAddress << 1;
    tempreg |= (1 << 14);
    pI2CHandle->pI2Cx->OAR1 = tempreg;

    // Init CCR (CCR calculations)
    uint16_t ccr_value = 0;
    tempreg = 0;
    if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
    {
        // mode is standard
        ccr_value = (RCC_GetPCLK1Value() / (2 * pI2CHandle->I2C_Config.I2C_SCLSpeed));
        tempreg |= (ccr_value & 0xFFF); // Masking out other bits, as CCR is only 12-bit length, whereas ccr_value is 16-bit
    }
    else
    {
        // mode is fast
        tempreg |= (1 << 15); // Fast mode is configured
        tempreg |= (pI2CHandle->I2C_Config.I2C_FMDutyCycle << 14);

        if (pI2CHandle->I2C_Config.I2C_FMDutyCycle == I2C_FM_DUTY_2)
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
    if (pI2CHandle->I2C_Config.I2C_SCLSpeed <= I2C_SCL_SPEED_SM)
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
    if (pI2Cx == I2C1)
    {
        I2C1_REG_RESET();
    }
    else if (pI2Cx == I2C2)
    {
        I2C2_REG_RESET();
    }
    else if (pI2Cx == I2C3)
    {
        I2C3_REG_RESET();
    }
}

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint32_t FlagName)
{
    if (pI2Cx->SR1 & FlagName)
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
    while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)))
        ;

    // 3. Send the address of the slave with R/W bit set to w(0) (total 8 bits)
    I2C_ExecuteAddressPhase_Write(pI2CHandle->pI2Cx, PeripheralAddr);

    // 4. Confirm that address phase is completed by checking the ADDR flag in the SR1
    while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)))
        ;

    // 5. Clear the ADDR flag according to its software sequence
    // Note: Until ADDr s cleared SCL will be stretched (pulled to LOW)
    I2C_ClearADDRFlag(pI2CHandle);

    // 6. Send the data until Len becomes 0
    while (Len > 0)
    {
        while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE)))
            ;
        pI2CHandle->pI2Cx->DR = *pTxBuffer;
        pTxBuffer++;
        Len--;
    }

    // 7. When Len become zero wait for TxE = 1 and BTF = 1 before generating the STOP condition
    //  Note: TxE = 1, BTF = 1, means that both SR and DR are empty and next transmission should begin when BTF = 1 SCL will be stretched (pulled to LOW)
    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_TXE))
        ;

    while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_BTF))
        ;

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
    while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_SB)))
        ;

    // 3. Send the address of the peripheral with R/notW bit set to R(1) (total 8 bits)
    I2C_ExecuteAddressPhase_Read(pI2CHandle->pI2Cx, PeripheralAddr);

    // 4. Wait until address phase is completed by checking the ADDR flag in the SR1
    while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_ADDR)))
        ;

    // 5. Procedure to read only 1 byte from peripheral
    if (Len == 1)
    {
        // Disable Acking
        I2C_ManageACKing(pI2CHandle->pI2Cx, I2C_ACK_DISABLE);

        // Clear the ADDR Flag
        I2C_ClearADDRFlag(pI2CHandle);

        // Wait until RXNE becomes 1
        while (!I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE))
            ;

        // Generate STOP condition
        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

        // Read data into buffer
        *pRxBuffer = pI2CHandle->pI2Cx->DR;

        return;
    }

    // 5. Procedure to read data from peripheral when Len > 1
    if (Len > 1)
    {
        // Clear the ADDR flag
        I2C_ClearADDRFlag(pI2CHandle);

        // Read the data until Len becomes zero
        for (uint32_t i = Len; i > 0; i--)
        {
            // Wait until RXNE becomes 1
            while (!(I2C_GetFlagStatus(pI2CHandle->pI2Cx, I2C_FLAG_RXNE)))
                ;

            if (i == 2) // if last 2 bytes are remaining
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
    if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
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
    if (EnorDi == ENABLE)
    {
        if (IRQNumber <= 31)
        {
            *NVIC_ISER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber > 31 && IRQNumber < 64)
        {
            *NVIC_ISER1 |= (1 << (IRQNumber % 32));
        }
        else if (IRQNumber > 64 && IRQNumber < 96)
        {
            *NVIC_ISER2 |= (1 << (IRQNumber % 64));
        }
    }
    else
    {
        if (IRQNumber <= 31)
        {
            *NVIC_ICER0 |= (1 << IRQNumber);
        }
        else if (IRQNumber > 31 && IRQNumber < 64)
        {
            *NVIC_ICER1 |= (1 << (IRQNumber % 32));
        }
        else if (IRQNumber > 64 && IRQNumber < 96)
        {
            *NVIC_ICER2 |= (1 << (IRQNumber % 64));
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
 * @brief Enabling or disabling an I2Cx peripheral ( based on MCU can be I2C1, I2C2, I2C3, etc. )
 *
 * @param pI2Cx
 * @param EnOrDi
 */
void I2C_PeripheralControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
    if (EnOrDi == ENABLE)
    {
        pI2Cx->CR1 |= (1 << I2C_CR1_PE);
    }
    else
    {
        pI2Cx->CR1 &= ~(1 << I2C_CR1_PE);
    }
}

void I2C_ManageACKing(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi)
{
    if (EnOrDi == I2C_ACK_ENABLE)
    {
        // enable the ack
        pI2Cx->CR1 |= (1 << I2C_CR1_ACK);
    }
    else
    {
        // disable the ack
        pI2Cx->CR1 &= ~(1 << I2C_CR1_ACK);
    }
}

void I2C_CloseReceiveData(I2C_Handle_t *pI2CHandle)
{
    // Implement the code to disable ITBUFEN Control Bit
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

    // Implement the code to disable ITEVFEN Control Bit
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pRxBuffer = NULL;
    pI2CHandle->RxLen = 0;
    pI2CHandle->RxSize = 0;

    if (pI2CHandle->I2C_Config.I2C_ACKControl == I2C_ACK_ENABLE)
    {
        I2C_ManageACKing(pI2CHandle->pI2Cx, ENABLE);
    }
}

void I2C_CloseSendData(I2C_Handle_t *pI2CHandle)
{
    // Implement the code to disable ITBUFEN Control Bit
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITBUFEN);

    // Implement the code to disable ITEVFEN Control Bit
    pI2CHandle->pI2Cx->CR2 &= ~(1 << I2C_CR2_ITEVTEN);

    pI2CHandle->TxRxState = I2C_READY;
    pI2CHandle->pTxBuffer = NULL;
    pI2CHandle->TxLen = 0;
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
uint8_t I2C_ControllerSendDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pTxBuffer, uint8_t Len, uint8_t PeripheralAddr, uint8_t Sr)
{
    uint8_t busystate = pI2CHandle->TxRxState;

    if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
    {
        pI2CHandle->pTxBuffer = pTxBuffer;
        pI2CHandle->TxLen = Len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_TX;
        pI2CHandle->DevAddr = PeripheralAddr;
        pI2CHandle->Sr = Sr;

        // Implement code to Generate START Condition
        I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        // Implement the code to enable ITBUFEN Control Bit
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

        // Implement the code to enable ITEVFEN Control Bit
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

        // Implement the code to enable ITERREN Control Bit
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
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
uint8_t I2C_ControllerReceiveDataIT(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len, uint8_t PeripheralAddr, uint8_t Sr)
{
    uint8_t busystate = pI2CHandle->TxRxState;

    if ((busystate != I2C_BUSY_IN_TX) && (busystate != I2C_BUSY_IN_RX))
    {
        pI2CHandle->pRxBuffer = pRxBuffer;
        pI2CHandle->RxLen = Len;
        pI2CHandle->TxRxState = I2C_BUSY_IN_RX;
        pI2CHandle->RxSize = Len; // Rxsize is used in the ISR code to manage the data reception
        pI2CHandle->DevAddr = PeripheralAddr;
        pI2CHandle->Sr = Sr;

        // Implement code to Generate START Condition
        I2C_GenerateStartCondition(pI2CHandle->pI2Cx);

        // Implement the code to enable ITBUFEN Control Bit
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITBUFEN);

        // Implement the code to enable ITEVFEN Control Bit
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITEVTEN);

        // Implement the code to enable ITERREN Control Bit
        pI2CHandle->pI2Cx->CR2 |= (1 << I2C_CR2_ITERREN);
    }

    return busystate;
}

void I2C_PeripheralSendData(I2C_RegDef_t *pI2Cx, uint8_t data)
{
    pI2Cx->DR = data;
}

uint8_t I2C_PeripheralReceiveData2(I2C_Handle_t *pI2CHandle, uint8_t *pRxBuffer, uint8_t Len)
{
	uint8_t temp = 0;
	uint8_t temp2 = 0;

	while(!(pI2CHandle->pI2Cx->SR1 & (1 << 1)));

    // 5. Procedure to read only 1 byte from peripheral
    if (Len == 1)
    {
        // Clear the ADDR Flag
        I2C_ClearADDRFlag(pI2CHandle);

        // Read data into buffer and clear RxNE flag
        *pRxBuffer = pI2CHandle->pI2Cx->DR;

        // Generate STOP condition
        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

        // Clear STOPF flag
        // Read SR1
        temp = I2C1->SR1;
        // Write into CR1
        I2C1->CR1 |= 0x0000;
    }

    // 5. Procedure to read data from peripheral when Len > 1
    if (Len > 1)
    {
        // Clear the ADDR flag
        I2C_ClearADDRFlag(pI2CHandle);

        // Read the data until Len becomes zero
        for (uint32_t i = Len; i > 0; i--)
        {
        	//wait until RXNE becomes 1
        	while(! I2C_GetFlagStatus(pI2CHandle->pI2Cx,I2C_FLAG_RXNE) );

        	// Read data into buffer and clear RxNE flag
        	*pRxBuffer = pI2CHandle->pI2Cx->DR;

            // Increment the buffer address
            pRxBuffer++;
        }

        // Read data into buffer and clear RxNE flag
        //*pRxBuffer = pI2CHandle->pI2Cx->DR;

        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);

        // Clear STOPF flag
        // Read SR1
        temp = I2C1->SR1;
        // Write into CR1
        I2C1->CR1 |= 0x0000;
    }

    return (uint8_t)(pI2CHandle->pI2Cx->DR);
}

uint8_t I2C_PeripheralReceiveData(I2C_RegDef_t *pI2Cx)
{
	return (uint8_t)pI2Cx->DR;
}

static void I2C_ControllerHandleTXEInterrupt(I2C_Handle_t *pI2CHandle)
{
    if (pI2CHandle->TxLen > 0)
    {
        // 1. load the data into the DR
        pI2CHandle->pI2Cx->DR = *(pI2CHandle->pTxBuffer);

        // 2. decrement the TxLen
        pI2CHandle->TxLen--;

        // 3. Increment the buffer address
        pI2CHandle->pTxBuffer++;
    }
}

static void I2C_ControllerHandleRXNEInterrupt(I2C_Handle_t *pI2CHandle)
{
    // We have to do the data reception
    if (pI2CHandle->RxSize == 1)
    {
        *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
        pI2CHandle->RxLen--;
    }

    if (pI2CHandle->RxSize > 1)
    {
        if (pI2CHandle->RxLen == 2)
        {
            // clear the ack bit
            I2C_ManageACKing(pI2CHandle->pI2Cx, DISABLE);
        }

        // read DR
        *pI2CHandle->pRxBuffer = pI2CHandle->pI2Cx->DR;
        pI2CHandle->pRxBuffer++;
        pI2CHandle->RxLen--;
    }

    if (pI2CHandle->RxLen == 0)
    {
        // close I2C data reception and notify the application

        // 1. generate the stop condition
        if (pI2CHandle->Sr == I2C_DISABLE_SR)
        {
            I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
        }

        // 2. close the I2C rx
        I2C_CloseReceiveData(pI2CHandle);

        // 3. notify the application
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_RX_CMPLT);
    }
}

void I2C_EV_IRQHandling(I2C_Handle_t *pI2CHandle)
{
    // Interrupt handling for both Controller and Peripheral mode of a device

    uint32_t temp1, temp2, temp3;

    temp1 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITEVTEN);
    temp2 = pI2CHandle->pI2Cx->CR2 & (1 << I2C_CR2_ITBUFEN);

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_SB);
    // 1. Handling for interrupt generated by SB event
    //  Note: SB flag is only applicable in Controller mode
    if (temp1 && temp3)
    {
        // SB flag is set
        if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
        {
            I2C_ExecuteAddressPhase_Read(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
        }
        else if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
        {
            I2C_ExecuteAddressPhase_Write(pI2CHandle->pI2Cx, pI2CHandle->DevAddr);
        }
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_ADDR);
    // 2. Handling for interrupt generated by ADDR event
    //  Note:   When Controller mode: Address is sent
    //          When Peripheral mode: Address is matched with own address
    if (temp1 && temp3)
    {
        // ADDR Flag is set
        I2C_ClearADDRFlag(pI2CHandle);
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_BTF);
    // 3. Handling for interrupt generated by BTF(Byte Transfer Finished) event
    if (temp1 && temp3)
    {
        // BTF flag is set
        if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
        {
            if (pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE))
            {
                // BTF and TXE are both set.
                if (pI2CHandle->TxLen == 0)
                {
                    // 1. Generate STOP condition
                    if (pI2CHandle->Sr == I2C_DISABLE_SR)
                    {
                        I2C_GenerateStopCondition(pI2CHandle->pI2Cx);
                    }

                    // 2. reset all the member elements of the handle structure
                    I2C_CloseSendData(pI2CHandle);

                    // 3. notify the application about transmission completion
                    I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_TX_COMPLETE);
                }
            }
        }
        else if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
        {
            // We don't close RX like TX
            ;
        }
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_STOPF);
    // 4. Handling for interrupt generated by STOPF event
    //  Note: Stop detection flag is applicable only Peripheral mode.
    // STOPF block will not be executed by the controller because STOPF will not be set in controller mode
    if (temp1 && temp3)
    {
        // STOPF flag is set
        // Clear the STOPF ( Read SR1, Write CR1 )
        // We have already read into SR1 before the if statement ^, so now just work on CR1
        pI2CHandle->pI2Cx->CR1 |= 0x0000;

        // Notify the application that STOP is detected
        I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_STOP);
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_TXE);
    // 5. Handling for interrupt generated by TXE event
    if (temp1 && temp2 && temp3)
    {
        // TXE flag is set
        // Do the data transmission

        // Check the device mode (only if device is controller, we can execute the Tx code)
        if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
        {
            // If the device is a controller
            if (pI2CHandle->TxRxState == I2C_BUSY_IN_TX)
            {
                I2C_ControllerHandleTXEInterrupt(pI2CHandle);
            }
        }
        else
        {
            // If the device is a peripheral
            // Making sure if the device is in transmitter mode:
            if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA))
            {
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_REQ);
            }
        }
    }

    temp3 = pI2CHandle->pI2Cx->SR1 & (1 << I2C_SR1_RXNE);
    // 6. Handling for interrupt generated by RXNE event
    if (temp1 && temp2 && temp3)
    {
        // Check for device mode
        if (pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_MSL))
        {
            // device is controller.
            // RXNE flag is set
            if (pI2CHandle->TxRxState == I2C_BUSY_IN_RX)
            {
                I2C_ControllerHandleRXNEInterrupt(pI2CHandle);
            }
        }
        else
        {
            // device is a peripheral
            // make sure that the peripheral is in receiver mode
            if (!(pI2CHandle->pI2Cx->SR2 & (1 << I2C_SR2_TRA)))
            {
                I2C_ApplicationEventCallback(pI2CHandle, I2C_EV_DATA_RCV);
            }
        }
    }
}

/*********************************************************************
 * @fn      		  - I2C_ER_IRQHandling
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Complete the code also define these macros in the driver
                        header file
                        #define I2C_ERROR_BERR  3
                        #define I2C_ERROR_ARLO  4
                        #define I2C_ERROR_AF    5
                        #define I2C_ERROR_OVR   6
                        #define I2C_ERROR_TIMEOUT 7

 */
void I2C_ER_IRQHandling(I2C_Handle_t *pI2CHandle)
{

    uint32_t temp1, temp2;

    // Know the status of  ITERREN control bit in the CR2
    temp2 = (pI2CHandle->pI2Cx->CR2) & (1 << I2C_CR2_ITERREN);

    /***********************Check for Bus error************************************/
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_BERR);
    if (temp1 && temp2)
    {
        // This is Bus error

        // Implement the code to clear the buss error flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_BERR);

        // Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_BERR);
    }

    /***********************Check for arbitration lost error************************************/
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_ARLO);
    if (temp1 && temp2)
    {
        // This is arbitration lost error

        // Implement the code to clear the arbitration lost error flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_ARLO);

        // Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_ARLO);
    }

    /***********************Check for ACK failure  error************************************/

    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_AF);
    if (temp1 && temp2)
    {
        // This is ACK failure error

        // Implement the code to clear the ACK failure error flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_AF);

        // Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_AF);
    }

    /***********************Check for Overrun/underrun error************************************/
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_OVR);
    if (temp1 && temp2)
    {
        // This is Overrun/underrun

        // Implement the code to clear the Overrun/underrun error flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_OVR);

        // Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_OVR);
    }

    /***********************Check for Time out error************************************/
    temp1 = (pI2CHandle->pI2Cx->SR1) & (1 << I2C_SR1_TIMEOUT);
    if (temp1 && temp2)
    {
        // This is Time out error

        // Implement the code to clear the Time out error flag
        pI2CHandle->pI2Cx->SR1 &= ~(1 << I2C_SR1_TIMEOUT);

        // Implement the code to notify the application about the error
        I2C_ApplicationEventCallback(pI2CHandle, I2C_ERROR_TIMEOUT);
    }
}

void I2C_PeriEnDisCallbacks(I2C_RegDef_t *pI2Cx, uint8_t EnorDi)
{
}
