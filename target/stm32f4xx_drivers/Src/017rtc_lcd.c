/*
 * 017rtc_lcd.c
 *
 *  Created on: 24 May 2023
 *      Author: Admin
 */
#include <stdio.h>
#include "ds1307.h"

char* get_day_of_week(uint8_t i);
char* time_to_string(RTC_time_t *rtc_time);

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

// returns time information in string format hh:mm:ss
char* time_to_string(RTC_time_t *rtc_time)
{

}