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

/*********************************************************************
 * @fn      		  - USART_Init
 *
 * @brief             -
 *
 * @param[in]         -
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -
 *
 * @Note              - Resolve all the TODOs 

 */
void USART_Init(USART_Handle_t *pUSARTHandle)
{

	//Temporary variable
	uint32_t tempreg=0;

/******************************** Configuration of CR1******************************************/

	//Implement the code to enable the Clock for given USART peripheral
    USART_PeriClockControl(pUSARTHandle->pUSARTx, ENABLE);

	//Enable USART Tx and Rx engines according to the USART_Mode configuration item
	if ( pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_RX)
	{
		//Implement the code to enable the Receiver bit field 
		tempreg|= (1 << USART_CR1_RE);
	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_ONLY_TX)
	{
		//Implement the code to enable the Transmitter bit field 
		tempreg |= ( 1 << USART_CR1_TE );

	}else if (pUSARTHandle->USART_Config.USART_Mode == USART_MODE_TXRX)
	{
		//Implement the code to enable the both Transmitter and Receiver bit fields 
		tempreg |= ( ( 1 << USART_CR1_RE) | ( 1 << USART_CR1_TE) );
	}

    //Implement the code to configure the Word length configuration item 
	tempreg |= pUSARTHandle->USART_Config.USART_WordLength << USART_CR1_M ;


    //Configuration of parity control bit fields
	if ( pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_EVEN)
	{
		//Implement the code to enale the parity control 
		tempreg |= ( 1 << USART_CR1_PCE);

		//Implement the code to enable EVEN parity 
		//Not required because by default EVEN parity will be selected once you enable the parity control 

	}else if (pUSARTHandle->USART_Config.USART_ParityControl == USART_PARITY_EN_ODD )
	{
		//Implement the code to enable the parity control 
	    tempreg |= ( 1 << USART_CR1_PCE);

	    //Implement the code to enable ODD parity 
	    tempreg |= ( 1 << USART_CR1_PS);

	}

   //Program the CR1 register 
	pUSARTHandle->pUSARTx->CR1 = tempreg;

/******************************** Configuration of CR2******************************************/

	tempreg=0;

	//Implement the code to configure the number of stop bits inserted during USART frame transmission 
	tempreg |= pUSARTHandle->USART_Config.USART_NoOfStopBits << USART_CR2_STOP;

	//Program the CR2 register 
	pUSARTHandle->pUSARTx->CR2 = tempreg;

/******************************** Configuration of CR3******************************************/

	tempreg=0;
	
	//Configuration of USART hardware flow control 
	if ( pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS)
	{
		//Implement the code to enable CTS flow control 
		tempreg |= ( 1 << USART_CR3_CTSE);


	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_RTS)
	{
		//Implement the code to enable RTS flow control 
		tempreg |= ( 1 << USART_CR3_RTSE );

	}else if (pUSARTHandle->USART_Config.USART_HWFlowControl == USART_HW_FLOW_CTRL_CTS_RTS)
	{
		//Implement the code to enable both CTS and RTS Flow control 
		tempreg |= ( ( 1 << USART_CR3_CTSE ) | ( 1 << USART_CR3_RTSE ) );
	}

	pUSARTHandle->pUSARTx->CR3 = tempreg;

/******************************** Configuration of BRR(Baudrate register)******************************************/

	//Implement the code to configure the baud rate
	//We will cover this in the lecture. No action required here 

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