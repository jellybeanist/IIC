#include "main.h"
#include "xiic.h"
#include "stdio.h"
#include "stdlib.h"
#include <sleep.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
// device slave addr, r/w,

int main(void)
{
	CLK_DIV_BAUD_DBG[0] = 100000000/115200;
	sleep(1);

	// HEADER: 	5 bytes. => 2 bytes initial + 1 byte protocol select + 2 bytes protocol message size.
	// Protocol selection: 00 for UART, 01 for RS232, 02 for RS422, 03 for IIC, 04 for SPI.

	// UART: 	16 bytes. => 1 byte enable + 4 bytes Baud Rate + 1 byte UART message size + 10 bytes UART message.
	// RS232: 	16 bytes. => 1 byte enable + 4 bytes Baud Rate + 1 byte UART message size + 10 bytes UART message.
	// RS422: 	16 bytes. => 1 byte enable + 4 bytes Baud Rate + 1 byte UART message size + 10 bytes UART message.

	// IIC:

	// SPI:

	int header_data_counter,data_counter,message_counter = 0; // after receiving first 8 bits of the package, these variables should be zeroed.
	//int *message_array_ptr;
	int receiving_done = 0;
	u16 message_size = 0;

	u8 msg_uart[16], valid_uart_msg_size;
	//u8 msg_iic[];
	//u8 msg_spi[];
	u8 header_specs[5];
	u8 check_sum = 0; // should be zeroed after getting the whole package.
	bool valid_initials = false;

	while (1)
	{
		while((RX_BUF_EMPTY[0] == 0)){
			//TX_BUF_DATA[0] = RX_BUF_DATA[0];
			if(header_data_counter<5)
			{
				header_specs[header_data_counter] = RX_BUF_DATA[0];
				//TX_BUF_DATA[0] = header_specs[header_data_counter];
				header_data_counter = header_data_counter + 1;
			}

			else if(header_data_counter == 5 && header_specs[2] == 00 && receiving_done == 0)
			{
				//message received properly. check sum is correct.
				msg_uart[message_counter] = RX_BUF_DATA[0];
				check_sum = check_sum + msg_uart[message_counter];
					if(message_counter == 13)
					{
						message_counter = 0;
						receiving_done = 1;
					}
			}
			/*
			else if(header_data_counter==5 && header_specs[2] == 01 && receiving_done == 0)
			{

			}
			else if(header_data_counter==5 && header_specs[2] == 02 && receiving_done == 0)
			{

			}
			else if(header_data_counter==5 && header_specs[2] == 03)
			{

			}
			*/

		}



		if(header_specs[0] == 0xAA && header_specs[1] == 0x55)
		{
			valid_initials = true;
		}

		if(valid_initials == true)
		{
			if(msg_uart[0] == 1)
			{

			}
		}


	}
return 0;
}





/*

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

*/

		/*if ([0] == 1)
				CLK_DIV_BAUD_U0[0] = 100000000/1,2,3,4;
				sleep(1);
				TX_BUF_DATA_U0[0] = 5;
		if([6] == 1)
				CLK_DIV_BAUD_U1[0] = 100000000/7,8,9,10;
				sleep(1);
				TX_BUF_DATA_U1[0] = 11;
				*/

