/*
 * 017rtc_lcd.c
 *
 *  Created on: 24 May 2023
 *      Author: Admin
 */
#include <stdio.h>
#include "ds1307.h"

// System generated interrupt every 1 second.
void init_systick_timer(uint32_t tick_hz)
{
	uint32_t *pSRVR = (uint32_t*)0xE000E014;
	uint32_t *pSCSR = (uint32_t*)0xE000E010;

    /* calculation of reload value */
    uint32_t count_value = (SYSTICK_TIM_CLK/tick_hz)-1;

    //Clear the value of SVR
    *pSRVR &= ~(0x00FFFFFFFF);

    //load the value in to SVR
    *pSRVR |= count_value;

    //do some settings
    *pSCSR |= ( 1 << 1); //Enables SysTick exception request:
    *pSCSR |= ( 1 << 2);  //Indicates the clock source, processor clock source

    //enable the systick
    *pSCSR |= ( 1 << 0); //enables the counter

}

char* get_day_of_week(uint8_t i);
char* time_to_string(RTC_time_t *rtc_time);
char* date_to_string(RTC_date_t *rtc_date);
void number_to_string(uint8_t num, char* buf);

int main(void)
{
    RTC_time_t current_time;
    RTC_date_t current_date;

    // If init returns non-zero, then something has failed in there.
    if(ds1307_init())
    {
        printf("RTC init has failed\n");
        while(1){};
    }

    current_date.day = MONDAY;
    current_date.date = 29;
    current_date.month = 5;
    current_date.year = 23;

    current_time.seconds = 10;
    current_time.minutes = 20;
    current_time.hours = 5;
    current_time.time_format = TIME_FORMAT_12HRS_PM;

    ds1307_set_current_date(&current_date);
    ds1307_set_current_time(&current_time);

    ds1307_get_current_time(&current_time);
    ds1307_get_current_date(&current_date);

    char *ampm;
    if(current_time.time_format != TIME_FORMAT_24HRS)
    {
        ampm = (current_time.time_format) ? "PM" : "AM";
        printf("Current time = %s %s\n", time_to_string(&current_time), ampm); //04:24:41 PM
    }
    else
    {
        printf("Current time = %s\n", time_to_string(&current_time));
    }

    //print date
    printf("Current date = %s <%s>\n", date_to_string(&current_date), get_day_of_week(current_date.day));
     

	return 0;
}

char* get_day_of_week(uint8_t i)
{
    char* days[] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

    return days[i-1];
}

void number_to_string(uint8_t num, char* buf)
{
    if(num < 10)
    {
        buf[0] = '0';
        buf[1] = num+48;
    }
    else if(num >= 10 && num < 99)
    {
        buf[0] = (num/10) + 48;
        buf[1] = (num % 10) + 48;
    }
}

// returns time information in string format hh:mm:ss
char* time_to_string(RTC_time_t *rtc_time)
{
    char buf[9];

    buf[2] = ':';
    buf[5] = ':';

    number_to_string(rtc_time->hours, buf);
    number_to_string(rtc_time->minutes, &buf[3]);
    number_to_string(rtc_time->seconds, &buf[6]);

    buf[8] = '\n';

    return buf;
}

char* date_to_string(RTC_date_t *rtc_date)
{
    static char buf[9];

    buf[2] = '/';
    buf[5] = '/';

    number_to_string(rtc_date->date, buf);
    number_to_string(rtc_date->month, &buf[3]);
    number_to_string(rtc_date->year, &buf[6]);

    buf[8] = '\n';

    return buf;
}