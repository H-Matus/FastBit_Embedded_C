/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: 5 Oct 2022
 *      Author: henrikass
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_

#include "stm32f407xx.h"

/**
 * @brief Configuration structure for SPIx peripheral
 * 
 */
typedef struct
{
    uint8_t SPI_DeviceMode;
    uint8_t SPI_BusConfig;
    uint8_t SPI_SclkSpeed;
    uint8_t SPI_DFF;
    uint8_t SPI_CPOL;
    uint8_t SPI_CPHA;
    uint8_t SPI_SSM;
}SPI_Config_t;

/**
 * @brief Handle structure for SPIx peripheral 
 * 
 */
typedef struct
{
    SPI_RegDef_t *pSPIx;
    SPI_Config_t SPIConfig;
}SPI_Handle_t;

/**
 * @brief Peripheral Clock setup
 * 
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);

/**
 * @brief Init and De-init
 * 
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/**
 * @brief Data Send and Receive
 * 
 */

/**
 * @brief IRQ Configuration and ISR handling
 * 
 */

#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
