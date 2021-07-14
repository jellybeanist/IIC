#include "main.h"
#include "xiic.h"
#include "stdio.h"
#include "stdlib.h"

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
	u8 wrData[3];

	wrData[0] = regAddr;
	wrData[1] = data0;
	wrData[2] = data1;

	if(XIic_Send(I2C_DEVICE_BASE_ADDR, MCP4725_DEV_ADDRESS, wrData, sizeof(wrData), XIIC_STOP) != sizeof(wrData))
	{
		return XST_FAILURE;
	}
	return XST_SUCCESS;
}


XStatus IIC_MCP4725_Read(u8 regAddr, u8* dataPtr, u8 dataCount)
{
	if(XIic_Send(I2C_DEVICE_BASE_ADDR, MCP4725_DEV_ADDRESS, &regAddr, 2, XIIC_STOP) != 1)
	{
		return XST_FAILURE;
	}
	if(XIic_Recv(I2C_DEVICE_BASE_ADDR, MCP4725_DEV_ADDRESS, dataPtr, dataCount, XIIC_STOP) != dataCount)
	{
		return XST_FAILURE;
	}
	return XST_SUCCESS;
}


int main(void)
{
	//u8 readData;
	//u8* dataPtr;
	//IIC_MCP4725_Read(0x40);
	IIC_MCP4725_Write(0x40,0x0F,0x00);
	//IIC_MCP4725_Read(0x40,*dataPtr,2);
	//readData=dataPtr;
	return XST_SUCCESS;
}
