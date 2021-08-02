#include "main.h"
#include "xiic.h"
#include "stdio.h"
#include "stdlib.h"
#include <sleep.h>
#include <inttypes.h>
#include <stdint.h>
// device slave addr, r/w,

int main(void)
{
	CLK_DIV_BAUD[0] = 100000000/115200;
	sleep(1);

	// HEADER: 	5 bytes. => 2 bytes initial + 1 byte protocol select + 2 bytes protocol message size.
	// Protocol selection: 00 for UART, 01 for RS232, 02 for RS422, 03 for IIC, 04 for SPI.

	// UART: 	16 bytes. => 1 byte enable + 4 bytes Baud Rate + 1 byte UART message size + 10 bytes UART message.
	// RS232: 	16 bytes. => 1 byte enable + 4 bytes Baud Rate + 1 byte UART message size + 10 bytes UART message.
	// RS422: 	16 bytes. => 1 byte enable + 4 bytes Baud Rate + 1 byte UART message size + 10 bytes UART message.

	// IIC:

	// SPI:

	int header_data_counter = 0; // after receiving first 8 bits of the package, this variable should be zeroed.
	int data_counter,message_counter=0;
	int *message_array_ptr;
	u16 message_size=0;
	u8 rxdata;
	//u64 test_data = 0xAA55BBCCDDEEFF99;
	u8 header_specs[5];
	//u8 test_array[18];
	u8 check_sum = 0; // should be zeroed after getting the whole package.

	while (1)
	{
		//rxdata = RX_BUF_DATA[0];
//https://www.programiz.com/c-programming/c-dynamic-memory-allocation
		while((RX_BUF_EMPTY[0] == 0)){
			//TX_BUF_DATA[0] = RX_BUF_DATA[0];
			if(header_data_counter<5)
			{
			header_specs[header_data_counter] = RX_BUF_DATA[0];
			header_data_counter = header_data_counter + 1;
			}
			else if(header_data_counter==5)
			{
			message_size = (header_specs[3] << 8) + header_specs[4];
			message_array_ptr = (int*) malloc( (int)message_size * sizeof(int) );
			}
			//message[(int)message_size];
			//message[message_size] = RX_BUF_DATA[0];
			else if(message_counter < (int)message_size)
			{
			(message_array_ptr) + message_counter == RX_BUF_DATA[0];
			TX_BUF_DATA[0] = *message_array_ptr + message_counter;
			message_counter = message_counter + 1;

			}
		}
		/*if(data_counter2 == 18){
		data_counter2=0;
		TX_BUF_DATA[0] = test_array[5];
		}*/

		if(RX_BUF_EMPTY[0] == 0)
		{
			//TX_BUF_DATA[0] = rxdata;
			//TX_BUF_DATA[0] = RX_BUF_DATA[0];

			if(data_counter == 0 && rxdata == 0xAA)
			{
				header_specs[data_counter] = rxdata;
				data_counter = data_counter + 1;
				//TX_BUF_DATA[0] = header_specs[0];
			}

			else if(data_counter == 1 && header_specs[0] == 0xAA)
			{
				header_specs[data_counter] = rxdata;
				data_counter = data_counter + 1;
				//TX_BUF_DATA[0] = header_specs[1];
			}

			else if (header_specs[0] == 0xAA && header_specs[1] == 0x55 && data_counter < 8)
			{
				header_specs[data_counter] = rxdata;
				data_counter = data_counter + 1;

				if(data_counter == 8)
				{
					//TX_BUF_DATA[0] = header_specs[0];
					//TX_BUF_DATA[0] = header_specs[1];
					//TX_BUF_DATA[0] = header_specs[2];
					//TX_BUF_DATA[0] = header_specs[3];
					//TX_BUF_DATA[0] = header_specs[4];
					//TX_BUF_DATA[0] = header_specs[5];
					//TX_BUF_DATA[0] = header_specs[6];
					TX_BUF_DATA[0] = header_specs[7];
					data_counter = 0;
				}
			}

			else//control statement, will not be used later.
				TX_BUF_DATA[0] = 0xBC;

		}
		else
		{
			//TX_BUF_DATA[0] = 0xCB;
		}


		/*if ([0] == 1)
				CLK_DIV_BAUD_U0[0] = 100000000/1,2,3,4;
				sleep(1);
				TX_BUF_DATA_U0[0] = 5;
		if([6] == 1)
				CLK_DIV_BAUD_U1[0] = 100000000/7,8,9,10;
				sleep(1);
				TX_BUF_DATA_U1[0] = 11;
				*/


	}
return 0;
}

//}
