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

static uint8_t binary_to_bcd(uint8_t value)
{
    uint8_t bcd;
    uint8_t m, n;

    // if value < 10 then it's the same as bcd
    bcd = value;

    // binary to bcd algorithm
    if(value >= 10)
    {
        m = value / 10;
        n = value % 10;
        bcd = (uint8_t) ((m << 4) | n);
    }

    return bcd;
}

static uint8_t bcd_to_binary(uint8_t value)
{
    uint8_t m, n;

    m = (uint8_t)((value >> 4) * 10);
    n = value & (uint8_t)0x0F;

    return (m+n);
}

void ds1307_set_current_time(RTC_time_t *rtc_time)
{
    uint8_t seconds, minutes, hours;

    seconds = binary_to_bcd(rtc_time->seconds);
    minutes = binary_to_bcd(rtc_time->minutes);
    hours = binary_to_bcd(rtc_time->hours);
    
    seconds &= ~(1 << 7);

    ds1307_write(seconds, DS1307_ADDR_SEC);
    ds1307_write(minutes, DS1307_ADDR_MIN);

    if(rtc_time->time_format == TIME_FORMAT_24HRS)
    {
        hours &= ~(1 << 6);
    }
    else if(rtc_time->time_format == TIME_FORMAT_12HRS_AM)
    {
        hours |= (1 << 6);
        hours &= ~(1 << 5);
    }
    else if(rtc_time->time_format == TIME_FORMAT_12HRS_PM)
    {
        hours |= (1 << 6);
        hours |= (1 << 5);
    }

    ds1307_write(hours, DS1307_ADDR_HRS);
}

void ds1307_get_current_time(RTC_time_t *rtc_time)
{
    uint8_t seconds, minutes, hours;

    seconds = ds1307_read(DS1307_ADDR_SEC);
    seconds &= ~(1 << 7);

    minutes = ds1307_read(DS1307_ADDR_MIN);

    rtc_time->seconds = bcd_to_binary(seconds);
    rtc_time->minutes = bcd_to_binary(minutes);

    hours = ds1307_read(DS1307_ADDR_HRS);

    if(0 == (hours >> 6))
    {
        rtc_time->time_format = TIME_FORMAT_24HRS;
    }
    else if(2 == (hours >> 5))
    {
        rtc_time->time_format = TIME_FORMAT_12HRS_AM;
    }
    else if(3 == (hours >> 5))
    {
        rtc_time->time_format = TIME_FORMAT_12HRS_PM;
    }

    hours &= ~(3 << 5);
    rtc_time->hours = bcd_to_binary(hours);
}

void ds1307_set_current_date(RTC_date_t *rtc_date)
{
    uint8_t day, date, month, year;

    day = binary_to_bcd(rtc_date->day);
    date = binary_to_bcd(rtc_date->date);
    month = binary_to_bcd(rtc_date->month);
    year = binary_to_bcd(rtc_date->year);

    ds1307_write(day, DS1307_ADDR_DAY);
    ds1307_write(date, DS1307_ADDR_DATE);
    ds1307_write(month, DS1307_ADDR_MONTH);
    ds1307_write(year, DS1307_ADDR_YEAR);
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
