/*
 * main.c
 *
 *  Created on: Sep 5, 2022
 *      Author: henrikass
 */

#include <stdint.h>
#include <stdio.h>

#define BITMASK 0x1

void wait_for_user_input(void);

int main(void)
{

	int32_t num1;
	printf("Enter a number:");
	scanf("%d", &num1);

	if(num1 & BITMASK)
	{
		printf("The number is odd.\n");
	}
	else
	{
		printf("The number is even.\n");
	}

	wait_for_user_input();

	return 0;
}

void wait_for_user_input()
{
	printf("Please enter any key to continue:\n");
	getchar();
	getchar();
}
