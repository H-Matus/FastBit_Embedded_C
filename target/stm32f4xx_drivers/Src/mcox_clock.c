/*
 * mcox_clock.c
 *
 *  Created on: 3 Jan 2023
 *      Author: Admin
 */

#include "stm32f407xx.h"

#define RCC_BASE_ADDR           0x40023800UL
#define RCC_CR_REG_OFFSET       0x0UL
#define RCC_CFGR_REG_OFFSET     0x08UL
#define RCC_CR_REG_ADDR         (RCC_BASE_ADDR + RCC_CR_REG_OFFSET)
#define RCC_CFGR_ADDR           (RCC_BASE_ADDR + RCC_CFGR_REG_OFFSET)

int main(void)
{
    uint32_t *pRccCfgrRegAddr = (uint32_t*) RCC_CFGR_ADDR;
    uint32_t *pRccCrRegAddr = (uint32_t*) RCC_CR_REG_ADDR;

    //*pRccCfgrRegAddr &= ~(0x3 << 21); // Clear the MCO1 bit positions
    *pRccCrRegAddr |= (0x1 << 16);      // Turn the HSE ON
    while(!(*pRccCrRegAddr & (1 << 17)));
    *pRccCfgrRegAddr |= (0x1 << 0);     // Switch system clock to HSE
    *pRccCfgrRegAddr |= (0x2 << 21);

	GPIO_Handle_t GpioClock;
	GpioClock.pGPIOx = GPIOA;
	GpioClock.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_8;
	GpioClock.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GpioClock.GPIO_PinConfig.GPIO_PinAltFunMode = 0;
	GpioClock.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioClock.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	GpioClock.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;

	GPIO_PeriClockControl(GPIOA, ENABLE);
	GPIO_Init(&GpioClock);
    
    GPIO_WriteToOutputPin(GPIOA, GPIO_PIN_NO_8, GPIO_PIN_SET);

	//for(;;)
	//{
	//	GPIO_ToggleOutputPin(GPIOA, 8);
	//}

	return 0;
}
