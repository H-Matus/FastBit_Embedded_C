/*
 * ds1307.c
 *
 *  Created on: 24 May 2023
 *      Author: Admin
 */

#include <stdint.h>
#include <string.h>
#include "ds1307.h"

static void ds1307_i2c_pin_config(void);
static void ds1307_i2c_config(void);
static void ds1307_write(uint8_t value, uint8_t reg_addr);
static uint8_t ds1307_read(uint8_t reg_addr);

I2C_Handle_t g_ds1307I2cHandle;

// Inits everything related to ds1307
// and returns 1 or 0 depending on the value of CH.
// if CH == 1 -> init failed
uint8_t ds1307_init(void)
{
    /* Initialise I2C pins */
    ds1307_i2c_pin_config();

    /* Initialise I2C peripheral */
    ds1307_i2c_config();

    /* Enable the I2C peripheral */
    I2C_PeripheralControl(DS1307_I2C, ENABLE);

    /* Clock halt = 0 */
    ds1307_write(0x00, DS1307_ADDR_SEC);

    /* Read back clock halt bit */
    uint8_t clock_state = ds1307_read(DS1307_ADDR_SEC);
    
    return ((clock_state >> 7) & 0x1);
}

void ds1307_set_current_time(RTC_time_t *rtc_time)
{

}

void ds1307_get_current_time(RTC_time_t *rtc_time)
{

}

void ds1307_set_current_date(RTC_date_t *rtc_date)
{

}

void ds1307_get_current_date(RTC_date_t *rtc_date)
{

}

static void ds1307_i2c_pin_config(void)
{
    GPIO_Handle_t i2c_sda, i2c_scl;

    memset(&i2c_scl, 0, sizeof(i2c_scl));
    memset(&i2c_sda, 0, sizeof(i2c_sda));

    /**
     * I2C1_SCL ==> PB6 
     * I2C1_SDA ==> PB7
     */

    i2c_sda.pGPIOx = DS1307_I2C_GPIO_PORT;
    i2c_sda.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
    i2c_sda.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    i2c_sda.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SDA_PIN;
    i2c_sda.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    i2c_sda.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
    i2c_sda.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    GPIO_Init(&i2c_sda);

    i2c_scl.pGPIOx = DS1307_I2C_GPIO_PORT;
    i2c_scl.GPIO_PinConfig.GPIO_PinAltFunMode = 4;
    i2c_scl.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
    i2c_scl.GPIO_PinConfig.GPIO_PinNumber = DS1307_I2C_SCL_PIN;
    i2c_scl.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
    i2c_scl.GPIO_PinConfig.GPIO_PinPuPdControl = DS1307_I2C_PUPD;
    i2c_scl.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

    GPIO_Init(&i2c_scl);
}
 
static void ds1307_i2c_config(void)
{
    g_ds1307I2cHandle.pI2Cx = DS1307_I2C;
    g_ds1307I2cHandle.I2C_Config.I2C_ACKControl = I2C_ACK_ENABLE;
    g_ds1307I2cHandle.I2C_Config.I2C_SCLSpeed = DS1307_I2C_SPEED;
    I2C_Init(&g_ds1307I2cHandle);
}

static void ds1307_write(uint8_t value, uint8_t reg_addr)
{
    uint8_t tx[2];
    tx[0] = reg_addr;
    tx[1] = value;
    I2C_ControllerSendData(&g_ds1307I2cHandle, tx, 2, DS1307_I2C_ADDRESS); // Generates memory space for the I2C_ADDRESS and sends data

}

static uint8_t ds1307_read(uint8_t reg_addr)
{
    uint8_t data;

    I2C_ControllerSendData(&g_ds1307I2cHandle, &reg_addr, 1, DS1307_I2C_ADDRESS);
    I2C_ControllerReceiveData(&g_ds1307I2cHandle, &data, 1, DS1307_I2C_ADDRESS);

    return data;
}