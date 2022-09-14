/*
 * stm32f407xx.h
 *
 *  Created on: Sep 14, 2022
 *      Author: henrikass
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define __vo volatile

/*
 * base addresses of FLASH ROM and SRAM
 */
#define FLASH_BASEADDR  0x08000000U                     /* FLASH ROM base address */
#define SRAM1_BASEADDR  0x20000000U                     /* SRAM1 base address */
#define SRAM1_SIZE      0x1C00U                         /* SRAM1 size = 112 KB */
#define SRAM2_BASEADDR  (SRAM1_BASEADDR + SRAM1_SIZE)   /* SRAM2 Base address, as it starts instantly after SRAM1 */
#define SRAM2_SIZE      0x3E80U                         /* SRAM2 size = 16 KB */
#define SRAM            SRAM1_BASEADDR                  /* SRAM1 is usually the main of both SRAMs*/
#define ROM_BASEADDR    0x1FFF0000U                     /* ROM base address */

/*
 * AHBx and APBx Bus Peripheral base addresses.
 */
#define PERIPH_BASEADDR         0x40000000U
#define APB1PERIPH_BASEADDR     PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR     0x40010000U
#define AHB1PERIPH_BASEADDR     0x40020000U
#define AHB2PERIPH_BASEADDR     0x50000000U

/*
    AHB1 bus peripheral addresses.
*/
#define GPIOA_BASEADDR      (AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR      (AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR      (AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR      (AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR      (AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR      (AHB1PERIPH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR      (AHB1PERIPH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR      (AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR      (AHB1PERIPH_BASEADDR + 0x2000)
#define RCC_BASEADDR        (AHB1PERIPH_BASEADDR + 0x3800)

/*
    APB1 bus peripheral addresses.
*/
#define SPI2_BASEADDR       (APB1PERIPH_BASEADDR + 0x3800U)
#define SPI3_BASEADDR       (APB1PERIPH_BASEADDR + 0x3C00U)
#define USART2_BASEADDR     (APB1PERIPH_BASEADDR + 0x4400U)
#define USART3_BASEADDR     (APB1PERIPH_BASEADDR + 0x4800U)
#define UART4_BASEADDR      (APB1PERIPH_BASEADDR + 0x4C00U)
#define UART5_BASEADDR      (APB1PERIPH_BASEADDR + 0x5000U)
#define I2C1_BASEADDR       (APB1PERIPH_BASEADDR + 0x5400U)
#define I2C2_BASEADDR       (APB1PERIPH_BASEADDR + 0x5800U)
#define I2C3_BASEADDR       (APB1PERIPH_BASEADDR + 0x5C00U)

/*
    APB2 bus peripheral addresses.
*/
#define USART1_BASEADDR     (APB2PERIPH_BASEADDR + 0x1000U)
#define USART6_BASEADDR     (APB2PERIPH_BASEADDR + 0x1400U)
#define SPI1_BASEADDR       (APB2PERIPH_BASEADDR + 0x3000U)
#define SYSCFG_BASEADDR     (APB2PERIPH_BASEADDR + 0x3800U)
#define EXTI_BASEADDR       (APB2PERIPH_BASEADDR + 0x3C00U)

/*
    GPIO peripheral register definition structure.
*/
typedef struct
{
    __vo uint32_t MODER;     /* GPIO port mode register */
    __vo uint32_t TYPER;     /* GPIO port output type register */
    __vo uint32_t OSPEEDR;   /* GPIO port output speed register */
    __vo uint32_t PUPDR;     /* GPIO port pull-up/pull-down register */
    __vo uint32_t IDR;       /* GPIO port input data register */
    __vo uint32_t ODR;       /* GPIO port output data register */
    __vo uint32_t BSRR;      /* GPIO port bit set/reset register */
    __vo uint32_t LCKR;      /* GPIO port configuration lock register */
    __vo uint32_t AFR[2];    /* AFR[0]: GPIO alternate function low register; AFR[1]: GPIO alternate function high register; */
}GPIO_RegDef_t;

/*
    RCC peripheral register definition structure.
*/
typedef struct 
{
    __vo uint32_t CR;
    __vo uint32_t PLLCFGR;
    __vo uint32_t CFGR;
    __vo uint32_t CIR;
    __vo uint32_t AHB1RSTR;
    __vo uint32_t AHB2RSTR;
    __vo uint32_t AHB3RSTR;
    uint32_t Reserved1;
    __vo uint32_t APB1RSTR;
    __vo uint32_t APB2RSTR;
    uint32_t Reserved2;
    uint32_t Reserved3;
    __vo uint32_t AHB1ENR;
    __vo uint32_t AHB2ENR;
    __vo uint32_t AHB3ENR;
    uint32_t Reserved4;
    __vo uint32_t APB1ENR;
    __vo uint32_t APB2ENR;
    uint32_t Reserved5;
    uint32_t Reserved6;
    __vo uint32_t AHB1LPENR;
    __vo uint32_t AHB2LPENR;
    __vo uint32_t AHB3LPENR;
    uint32_t Reserved7;
    __vo uint32_t APB1LPENR;
    __vo uint32_t APB2LPENR;
    uint32_t Reserved8;
    uint32_t Reserved9;
    __vo uint32_t BDCR;
    __vo uint32_t CSR;
    uint32_t Reserved10;
    __vo uint32_t SSCGR;
    __vo uint32_t PLLI2SCFGR;
}RCC_RegDef_t;


/*
    Peripheral definitions.
*/
#define GPIOA       ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB       ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC       ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD       ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE       ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF       ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG       ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH       ((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI       ((GPIO_RegDef_t *)GPIOI_BASEADDR)

#define RCC         ((RCC_RegDef_t *)RCC_BASEADDR)

/*
    Clock enable macros for GPIOx peripherals
*/
#define GPIOA_PCLK_EN()     (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()     (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()     (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()     (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()     (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()     (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()     (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()     (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()     (RCC->AHB1ENR |= (1 << 8))

/*
    Clock enable macros for I2Cx peripherals
*/
#define I2C1_PCLK_EN()      (RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()      (RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()      (RCC->APB1ENR |= (1 << 23))

/*
    Clock enable macros for SPIx peripherals
*/
#define SPI1_PCLK_EN()      (RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()      (RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()      (RCC->APB1ENR |= (1 << 15))

/*
    Clock enable macros for USARTx peripherals
*/
#define USART1_PCLK_EN()    (RCC->APB2ENR |= (1 << 4))
#define USART6_PCLK_EN()    (RCC->APB2ENR |= (1 << 5))
#define USART2_PCLK_EN()    (RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()    (RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()     (RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()     (RCC->APB1ENR |= (1 << 20))

/*
    Clock enable macros for SYSCFG peripheral
*/
#define SYSCFG_PCLK_EN()     (RCC->APB2ENR |= (1 << 14))

/*
    Clock disable macros for GPIOx peripherals
*/
#define GPIOA_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()     (RCC->AHB1ENR &= ~(1 << 8))

/*
    Clock disable macros for I2Cx peripherals
*/
#define I2C1_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 23))

/*
    Clock disable macros for SPIx peripherals
*/
#define SPI1_PCLK_DI()      (RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()      (RCC->APB1ENR &= ~(1 << 15))

/*
    Clock disable macros for USARTx peripherals
*/
#define USART1_PCLK_DI()    (RCC->APB2ENR &= ~(1 << 4))
#define USART6_PCLK_DI()    (RCC->APB2ENR &= ~(1 << 5))
#define USART2_PCLK_DI()    (RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()    (RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()     (RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()     (RCC->APB1ENR &= ~(1 << 20))

/*
    Clock disable macros for SYSCFG peripheral
*/
#define SYSCFG_PCLK_DI()     (RCC->APB2ENR &= ~(1 << 14))

/* Generic Macros */
#define ENABLE          1
#define DISABLE         0
#define SET             ENABLE
#define RESET           DISABLE
#define GPIO_PIN_SET    SET
#define GPIO_PIN_RESET  RESET

#endif /* INC_STM32F407XX_H_ */
