/*
 * main.c
 *
 *  Created on: Sep 2, 2022
 *      Author: henrikass
 */

#include <stdio.h>
#include <stdint.h>



int main(void)
{
	char val = 100;
	printf("Address of the val = %p\n", &val);

	char* ptrVal = &val;

	char value = *ptrVal;

	printf("Read operation yield from value = %d\n", value);
	*ptrVal = 65;

	printf("Address of the val = %d\n", val);

	getchar();
	getchar();

}
