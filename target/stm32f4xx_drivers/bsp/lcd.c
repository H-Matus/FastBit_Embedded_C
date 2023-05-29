/*
 * lcd.c
 *
 *  Created on: 24 May 2023
 *      Author: Admin
 */

#include "lcd.h"

static void write_4_bits(uint8_t value);

/* Sending command code on what command we want from LCD */
void lcd_send_command(uint8_t cmd)
{
    // sending command for instructing the LCD what to do.
    /* RS = 0, for LCD command */
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

    /* RW = 0, writing to LCD */
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

    write_4_bits(cmd >> 4);     // Writing the higher nibble
    write_4_bits(cmd >> 0x0F);  // Writing the lower nibble
}

/**
 * @brief
 * This function sends a character to the LCD.
 * Here we used 4 bit parallel data transmission.
 * First higher nibble of the data will be sent on to the data lines D4, D5, D6, D7
 * Then lower nibble of the data will be set on to the data lines D4, D5, D6, D7
 * 
 * @param data 
 */
void lcd_send_char(uint8_t data)
{
    /* RS = 0, for LCD command */
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RS, GPIO_PIN_RESET);

    /* RW = 0, writing to LCD */
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_RW, GPIO_PIN_RESET);

    write_4_bits(data >> 4);     // Writing the higher nibble
    write_4_bits(data >> 0x0F);  // Writing the lower nibble

}

static void lcd_enable()
{
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_SET);
    udelay(10);
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_EN, GPIO_PIN_RESET);
    udelay(100);
}

void lcd_init(void)
{
    // 1. Configure gpio pins which are used for lcd connections
    GPIO_Handle_t lcd_signal;

    lcd_signal.pGPIOx = LCD_GPIO_PORT;
    lcd_signal.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RS;
    lcd_signal.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    lcd_signal.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    lcd_signal.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GPIO_Init(&lcd_signal);

    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_RW;
    GPIO_Init(&lcd_signal);

    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_EN;
    GPIO_Init(&lcd_signal);

    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D4;
    GPIO_Init(&lcd_signal);

    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D5;
    GPIO_Init(&lcd_signal);

    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D6;
    GPIO_Init(&lcd_signal);

    lcd_signal.GPIO_PinConfig.GPIO_PinNumber = LCD_GPIO_D7;
    GPIO_Init(&lcd_signal);

    GPIO_WriteToOutputPin(lcd_signal.pGPIOx, LCD_GPIO_RS, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(lcd_signal.pGPIOx, LCD_GPIO_RW, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(lcd_signal.pGPIOx, LCD_GPIO_EN, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(lcd_signal.pGPIOx, LCD_GPIO_D4, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(lcd_signal.pGPIOx, LCD_GPIO_D5, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(lcd_signal.pGPIOx, LCD_GPIO_D6, GPIO_PIN_RESET);
    GPIO_WriteToOutputPin(lcd_signal.pGPIOx, LCD_GPIO_D7, GPIO_PIN_RESET);

    // 2. Do LCD initialisation
    mdelay(40);

    /* RS = 0, for LCD command */
    GPIO_WriteToOutputPin(lcd_signal.pGPIOx, LCD_GPIO_RS, GPIO_PIN_RESET);

    write_4_bits(0x3);

    mdelay(5);

    write_4_bits(0x3);

    udelay(150);

    write_4_bits(0x3);
    write_4_bits(0x2);

    // function send command
    lcd_send_command(LCD_CMD_4DL_2N_5X8F);

    // display on or off control
    lcd_send_command(LCD_CMD_DON_CURON);

    // display clear
    lcd_send_command(LCD_CMD_DIS_CLEAR);

    // After DIS_CLEAR command, we need to wait for 2ms
    mdelay(10);

    // entry mode set
    lcd_send_command(LCD_CMD_INCADD);

}

static void write_4_bits(uint8_t value)
{
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D7, ((value >> 0) & 0x1));
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D6, ((value >> 1) & 0x1));
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D5, ((value >> 2) & 0x1));
    GPIO_WriteToOutputPin(LCD_GPIO_PORT, LCD_GPIO_D4, ((value >> 3) & 0x1));

    lcd_enable();
}
