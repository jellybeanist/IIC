#include "main.h"
#include "xiic.h"
#include "stdio.h"
#include "stdlib.h"
#include <sleep.h>
#include <inttypes.h>
#include <stdint.h>

u32 temp=0;
u32 adder=0;
u32 right=0;
u32 shift=0;
u32 sent=0;

static int IIC_SensorInitialized = FALSE ;

void IIC_Initialize(void)
{
	IIC_SensorInitialized = TRUE;
}

void i2cFree(void)
{
	IIC_SensorInitialized = FALSE ;
}


XStatus IIC_MCP4725_Write(u8 regAddr, u8 data0, u8 data1)
{
	u8 wrData[2];

	wrData[0] = regAddr;
	wrData[0] = data0;
	wrData[1] = data1;

	/* register write single byte */
	if(XIic_Send(I2C_DEVICE_BASE_ADDR, MCP4725_DEV_ADDRESS, wrData, sizeof(wrData), XIIC_STOP) != sizeof(wrData))
	{
		return XST_FAILURE;
	}
	return XST_SUCCESS;
}


XStatus IIC_MCP4725_Read(u8 regAddr, u8* dataPtr, u8 dataCount)
{
	/* register select */
	if(XIic_Send(I2C_DEVICE_BASE_ADDR, MCP4725_DEV_ADDRESS, &regAddr, 2, XIIC_STOP) != 1)
	{
		return XST_FAILURE;
	}
	/* register read */
	if(XIic_Recv(I2C_DEVICE_BASE_ADDR, MCP4725_DEV_ADDRESS, dataPtr, dataCount, XIIC_STOP) != dataCount)
	{
		return XST_FAILURE;
	}
	return XST_SUCCESS;
}


XStatus IIC_QMC5883_Write(u8 regAddr, u8 slaveAddr, u8 data)
{
	u8 wrData[3];

	wrData[0] = regAddr;
	wrData[1] = slaveAddr;
	wrData[2] = data;

	/* register write single byte */
	if(XIic_Send(I2C_DEVICE_BASE_ADDR, QMC5883_DEV_ADDRESS, wrData, sizeof(wrData), XIIC_STOP) != sizeof(wrData))
	{
		return XST_FAILURE;
	}
	return XST_SUCCESS;
}


XStatus IIC_QMC5883_Read(u8 regAddr, u8* dataPtr, u8 dataCount)
{
	/* register select */
	if(XIic_Send(I2C_DEVICE_BASE_ADDR, QMC5883_DEV_ADDRESS, &regAddr, 1, XIIC_STOP) != 1)
	{
		return XST_FAILURE;
	}
	/* register read */
	if(XIic_Recv(I2C_DEVICE_BASE_ADDR, QMC5883_DEV_ADDRESS, dataPtr, dataCount, XIIC_STOP) != dataCount)
	{
		return XST_FAILURE;
	}
	return XST_SUCCESS;
}

int main(void)
{
	u8 datas[8];
	CLK_DIV_BAUD[0] = 100000000/115200;
	sleep(1);
	// int counter=0;
	// u8 voltageLow,voltageHigh=0x00;
	//IIC_MCP4725_Write(0x40,0x00,0x00);
	while(1)
	{
		if (IIC_QMC5883_Read(0x00, datas, 8) == XST_FAILURE)
		{
			TX_BUF_DATA[0] = 0xAA;
		}
		else
		{
			for (int i= 0; i<8; i++)
				TX_BUF_DATA[0] = datas[i];
		}
		sleep(1);
	}


	//Write the data comes from UART to the 7 segment display.
/*	while (1)
	{
		//IIC_MCP4725_Write(MCP4725_WRITEDAC_ADDRESS,voltageHigh,voltageLow);
		if(RX_BUF_EMTY[0] == 0){
			TX_BUF_DATA[0]=RX_BUF_DATA[0];
			voltageHigh = RX_BUF_DATA[0];

			if (counter==1){
				voltageLow = RX_BUF_DATA[0];
				IIC_MCP4725_Write(MCP4725_WRITEDAC_ADDRESS,voltageHigh,voltageLow);
				counter=0;
			}

				counter=counter+1;

			//SEVEN_SEGW[0] = shift+RX_BUF_DATA[0];



			}
		//shift = temp<<8;
		}
*/
	//12 bit DAC function.
	//IIC_MCP4725_Write(MCP4725_WRITEDAC_ADDRESS,0x00,0xFF);



	return XST_SUCCESS;
}
