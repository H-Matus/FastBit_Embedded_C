/*
 * main.c
 *
 *  Created on: Sep 2, 2022
 *      Author: henrikass
 */

#include <stdint.h>
#include <stdio.h>

int main(void)
{
	char c1;
	char c2;
	char c3;
	char c4;
	char c5;
	char c6;

	scanf("%c %c %c %c %c %c", &c1, &c2, &c3, &c4, &c5, &c6);

	printf("ASCII codes are: %d, %d, %d, %d, %d, %d\n", c1, c2, c3, c4, c5, c6);

	printf("Please enter any key to exit the application.\n");
	while(getchar() != '\n')
	{

	}

	return 0;
}
