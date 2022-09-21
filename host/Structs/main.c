 /*
 * main.c
 *
 *  Created on: 6 Sep 2022
 *      Author: henrikass
 */

#include <stdint.h>
#include <stdio.h>

void wait_for_user_input(void);

struct DataSet
{
	char data1;
	char data3;
	short data4;
	int data2;
};

struct DataSet data;

int main(void)
{
	data.data1 = 0xaa;
	data.data2 = 0x11223344;
	data.data3 = 0xbb;
	data.data4 = 0x5566;
	printf("Sizeof of struct carModel is %I64llu\n", sizeof(data));

}

void wait_for_user_input()
{
	printf("Please enter any key to continue:\n");
	getchar();
	getchar();
}
