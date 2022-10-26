/*
 * stm32f407xx_rcc_driver.h
 *
 *  Created on: 26 Oct 2022
 *      Author: henrikass
 */

#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_

#include "stm32f407xx.h"

uint32_t RCC_GetPLLOutput();
uint32_t RCC_GetPCLK1Value(void); // Returns APB1 clock value
uint32_t RCC_GetPCLK2Value(void); // Returns APB2 clock value

#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */
