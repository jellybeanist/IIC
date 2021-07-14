/******************************************************************************
*
* (c) Copyright MELSiS Inc & ASELSAN. All rights reserved.
*
* THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS PART OF THIS FILE
* AT ALL TIMES.
*
******************************************************************************/

/****************************************************************************/
/**
*
* @file i2c_if.c
*
* Contains required functions I2C Interface Communication of ADAPTOR Software
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who  			Date     	Changes
* ----- ----------		-------- 	-----------------------------------------------
* 1.00a B.SEKERLISOY  	01/04/2018 	First release
*
* </pre>
*
*****************************************************************************/

/***************************** Include Files ********************************/
#include "i2c_if.h"
#include "xiic.h"
#include "stdio.h"
#include "stdlib.h"
#include "aux_func.h"

/************************** Constant Definitions ****************************/

/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions ********************/

/************************** Variable Definitions ****************************/
static int i2cSensorInitialized = FALSE ;
unsigned int board_temparature;

/*
 ***************************************************************************
 *  Function 	:
 *  	i2cInitialize
 *
 *  Description	:
 *  	sets i2cSensorInitialized variable to TRUE as initialization
 *
 *  Parameters	:
 *      NA
 *
 *  Return value:
 *		NA
 *************************************************************************
 */
void i2cInitialize(void)
{
	i2cSensorInitialized = TRUE;
	ltc2991Init();
}

/*
 ***************************************************************************
 *  Function 	:
 *  	i2cFree
 *
 *  Description	:
 *  	sets i2cSensorInitialized variable to FALSE as deinitialization
 *
 *  Parameters	:
 *      NA
 *
 *  Return value:
 *		NA
 *************************************************************************
 */
void i2cFree(void)
{
	i2cSensorInitialized = FALSE ;
}

/*
 ****************************************************************
 **     *     *     *     *     *     *     *     *     *     * *
 * *   * *   * *   * *   * *   * *   * *   * *   * *   * *   *  *
 *  * *   * *   * *   * *   * *   * *   * *   * *   * *   * *   *
 *   *	   *	 *	   *	 *	   *	 *	   *	 *	   *	*
 ****************************************************************
 **						AD7414 FUNCTIONS					   **
 ****************************************************************
 **     *     *     *     *     *     *     *     *     *     * *
 * *   * *   * *   * *   * *   * *   * *   * *   * *   * *   *  *
 *  * *   * *   * *   * *   * *   * *   * *   * *   * *   * *   *
 *   *	   *	 *	   *	 *	   *	 *	   *	 *	   *	*
 ****************************************************************
 */
/*
 ***************************************************************************
 *  Function 	:
 *  	i2cAD7414Write
 *
 *  Description	:
 *  	write one byte to AD7414 register through I2C
 *
 *  Parameters	:
 *      u8 regAddr
 *      	register address of AD7414 (located in i2c_if.h)
 *    	u8 data
 *       	register data to be written to AD7414 register
 *
 *  Return value:
 *		XStatus
 *			XST_SUCCESS	: write is successful
 *			XST_FAILURE	: write failed
 *************************************************************************
 */
XStatus i2cAD7414Write(u8 regAddr, u8 data)
{
	u8 wrData[2];

	if(regAddr==AD7414_REG_TEMP)
	{
		return XST_FAILURE;
	}
	wrData[0] = regAddr;
	wrData[1] = data;

	/* register write single byte */
	if(XIic_Send(I2C_DEVICE_BASE_ADDR, AD7414_DEV_ADDRESS, wrData, sizeof(wrData), XIIC_STOP) != sizeof(wrData))
	{
		return XST_FAILURE;
	}
	return XST_SUCCESS;
}

/*
 ***************************************************************************
 *  Function 	:
 *  	i2cAD7414Read
 *
 *  Description	:
 *  	read several bytes from AD7414 register through I2C
 *
 *  Parameters	:
 *      u8 regAddr
 *      	read start register address of AD7414 (located in i2c_if.h)
 *    	u8* dataPtr
 *       	register data pointer to be read from AD7414 register
 *		u8 dataCount
 *			number of bytes to be read successively starting from regAddr
 *
 *  Return value:
 *		XStatus
 *			XST_SUCCESS	: read is successful
 *			XST_FAILURE	: read failed
 *************************************************************************
 */
XStatus i2cAD7414Read(u8 regAddr, u8* dataPtr, u8 dataCount)
{
	/* register select */
	if(XIic_Send(I2C_DEVICE_BASE_ADDR, AD7414_DEV_ADDRESS, &regAddr, 1, XIIC_STOP) != 1)
	{
		return XST_FAILURE;
	}
	/* register read */
	if(XIic_Recv(I2C_DEVICE_BASE_ADDR, AD7414_DEV_ADDRESS, dataPtr, dataCount, XIIC_STOP) != dataCount)
	{
		return XST_FAILURE;
	}
	return XST_SUCCESS;
}

/*
 ***************************************************************************
 *  Function 	:
 *  	i2cAD7414ReadTemp
 *
 *  Description	:
 *  	read temparature value from AD7414 
 *
 *  Parameters	:
 *      int* tempValPtr
 *      	temparature value pointer
 *
 *  Return value:
 *		XStatus
 *			XST_SUCCESS	: read is successful
 *			XST_FAILURE	: read failed
 *************************************************************************
 */
XStatus i2cAD7414ReadTemp(int* tempValPtr)
{
	u8 tempPtr[2];
	int tempVal;

	if( i2cAD7414Read(AD7414_REG_TEMP, tempPtr, 2) != XST_SUCCESS )
	{
		return XST_FAILURE;
	}

	tempVal = (int)(tempPtr[0]*4) + (int)(tempPtr[1]>>6);
	if(tempVal>512)
	{
		tempVal = tempVal - 512;
	}
	tempVal = tempVal;
	tempValPtr[0] = tempVal;

	return XST_SUCCESS;
}

/*
 ****************************************************************
 **     *     *     *     *     *     *     *     *     *     * *
 * *   * *   * *   * *   * *   * *   * *   * *   * *   * *   *  *
 *  * *   * *   * *   * *   * *   * *   * *   * *   * *   * *   *
 *   *	   *	 *	   *	 *	   *	 *	   *	 *	   *	*
 ****************************************************************
 **						DS1682 FUNCTIONS					   **
 ****************************************************************
 **     *     *     *     *     *     *     *     *     *     * *
 * *   * *   * *   * *   * *   * *   * *   * *   * *   * *   *  *
 *  * *   * *   * *   * *   * *   * *   * *   * *   * *   * *   *
 *   *	   *	 *	   *	 *	   *	 *	   *	 *	   *	*
 ****************************************************************
 */
/*
 ***************************************************************************
 *  Function 	:
 *  	i2cDS1682Write
 *
 *  Description	:
 *  	write several bytes to DS1682 registers through I2C
 *
 *  Parameters	:
 *      u8 regAddr
 *      	write start register address of DS1682 (located in i2c_if.h)
 *    	u8* dataPtr
 *       	register data pointer to write to DS1682 registers
 *		u8 dataCount
 *			number of bytes to write successively starting from regAddr
 *
 *  Return value:
 *		XStatus
 *			XST_SUCCESS	: write is successful
 *			XST_FAILURE	: write failed
 *************************************************************************
 */
 XStatus i2cDS1682Write(u8 regAddr, u8* dataPtr, u8 dataCount)
{
	u8 wrData[16];
	u8 i;

	wrData[0] = regAddr;
	for(i=0;i<dataCount;i++)
	{
		wrData[i+1] = dataPtr[i];
	}

	if(XIic_Send(I2C_DEVICE_BASE_ADDR, DS1682_DEV_ADDRESS, wrData, dataCount+1, XIIC_STOP) != dataCount+1)
	{
		return XST_FAILURE;
	}
	return XST_SUCCESS;
}

/*
 ***************************************************************************
 *  Function 	:
 *  	i2cDS1682Read
 *
 *  Description	:
 *  	read several bytes from DS1682 registers through I2C
 *
 *  Parameters	:
 *      u8 regAddr
 *      	read start register address of DS1682 (located in i2c_if.h)
 *    	u8* dataPtr
 *       	register data pointer to be read from DS1682 registers
 *		u8 dataCount
 *			number of bytes to read successively starting from regAddr
 *
 *  Return value:
 *		XStatus
 *			XST_SUCCESS	: read is successful
 *			XST_FAILURE	: read failed
 *************************************************************************
 */
XStatus i2cDS1682Read(u8 regAddr, u8* dataPtr, u8 dataCount)
{
	if(XIic_Send(I2C_DEVICE_BASE_ADDR, DS1682_DEV_ADDRESS, &regAddr, 1, XIIC_STOP) != 1)
	{
		return XST_FAILURE;
	}

	if(XIic_Recv(I2C_DEVICE_BASE_ADDR, DS1682_DEV_ADDRESS, dataPtr, dataCount, XIIC_STOP) != dataCount)
	{
		return XST_FAILURE;
	}
	return XST_SUCCESS;
}

/*
 ***************************************************************************
 *  Function 	:
 *  	i2cDS1682ReadETC
 *
 *  Description	:
 *  	read electronic timer counter value from DS1682
 *
 *  Parameters	:
 *      unsigned int* etcValPtr
 *      	electronic counter value pointer
 *
 *  Return value:
 *		XStatus
 *			XST_SUCCESS	: read is successful
 *			XST_FAILURE	: read failed
 *************************************************************************
 */
XStatus i2cDS1682ReadETC(unsigned int* etcValPtr)
{
	u8 etctr[4];
	unsigned int etcVal;

	if( i2cDS1682Read(DS1682_REG_ETC, etctr, 4) != XST_SUCCESS )
	{
		return XST_FAILURE;
	}

	etcVal = (unsigned int)(etctr[3]<<24) + (unsigned int)(etctr[2]<<16) + (unsigned int)(etctr[1]<<8) + (unsigned int)(etctr[0]);
	etcValPtr[0] = etcVal >> 2;
	return XST_SUCCESS;
}

/*
 ***************************************************************************
 *  Function 	:
 *  	i2cDS1682Reset
 *
 *  Description	:
 *  	resets electronic counter of DS1682
 *
 *  Parameters	:
 *      NA
 *
 *  Return value:
 *		XStatus
 *			XST_SUCCESS	: reset is successful
 *			XST_FAILURE	: reset failed
 *************************************************************************
 */
XStatus i2cDS1682Reset(void)
{
	u8 rstData[4];
	rstData[0] = 0;
	rstData[1] = 0;
	rstData[2] = 0;
	rstData[3] = 0;
	if( i2cDS1682Write(DS1682_REG_ETC, rstData, sizeof(rstData)) != XST_SUCCESS )
	{
		return XST_FAILURE;
	}
	return XST_SUCCESS;
}

/*
 ****************************************************************
 **     *     *     *     *     *     *     *     *     *     * *
 * *   * *   * *   * *   * *   * *   * *   * *   * *   * *   *  *
 *  * *   * *   * *   * *   * *   * *   * *   * *   * *   * *   *
 *   *	   *	 *	   *	 *	   *	 *	   *	 *	   *	*
 ****************************************************************
 **						24LCXX FUNCTIONS					   **
 ****************************************************************
 **     *     *     *     *     *     *     *     *     *     * *
 * *   * *   * *   * *   * *   * *   * *   * *   * *   * *   *  *
 *  * *   * *   * *   * *   * *   * *   * *   * *   * *   * *   *
 *   *	   *	 *	   *	 *	   *	 *	   *	 *	   *	*
 ****************************************************************
 */
/*
 ***************************************************************************
 *  Function 	:
 *  	eepromTest
 *
 *  Description	:
 *  	writes and reads 16 bytes to/from eeprom and tests and compare
 *		read/write data
 *
 *  Parameters	:
 *      NA
 *
 *  Return value:
 *		XStatus
 *			XST_SUCCESS	: eeprom test is successful
 *			XST_FAILURE	: eeprom test failed
 *************************************************************************
 */
 XStatus eepromTest(void)
{
	u8 wrBuf[16];
	u8 rdBuf[16];
	u16 i;

	for(i=0; i<16; i++)
	{
		wrBuf[i] = (u8)(i+1);
		rdBuf[i] = 0;
	}

	eepromWrite(EEPROM_TEST_ADDR, wrBuf, 16);
	eepromRead(EEPROM_TEST_ADDR, rdBuf, 16);

	for(i=0; i<16; i++)
	{
		if(wrBuf[i]!=rdBuf[i])
		{
			return XST_FAILURE;
		}
	}
	return XST_SUCCESS;
}

/*
 ***************************************************************************
 *  Function 	:
 *  	eepromWrite
 *
 *  Description	:
 *  	write several bytes to I2C Eeprom
 *
 *  Parameters	:
 *      u16 addr
 *      	eeprom write start address
 *    	u8* bufPtr
 *       	write data pointer
 *		unsigned int dataCount
 *			number of bytes to write successively starting from addr
 *
 *  Return value:
 *		XStatus
 *			XST_SUCCESS	: write is successful
 *			XST_FAILURE	: write failed
 *************************************************************************
 */
XStatus eepromWrite(u16 addr, u8* bufPtr, unsigned int dataCount)
{
	u16 i;
	u8 wrData;

	for(i=0; i<dataCount; i++)
	{
		wrData = bufPtr[i];
		if(i2cEepromWriteByte(addr+i, wrData)!=1)
		{
			return XST_FAILURE;
		}
	}
	return XST_SUCCESS;
}

/*
 ***************************************************************************
 *  Function 	:
 *  	eepromRead
 *
 *  Description	:
 *  	read several bytes from I2C Eeprom
 *
 *  Parameters	:
 *      u16 addr
 *      	eeprom read start address
 *    	u8* bufPtr
 *       	read data pointer
 *		unsigned int dataCount
 *			number of bytes to read successively starting from addr
 *
 *  Return value:
 *		XStatus
 *			XST_SUCCESS	: read is successful
 *			XST_FAILURE	: read failed
 *************************************************************************
 */
XStatus eepromRead(u16 addr, u8* bufPtr, unsigned int dataCount)
{
	u16 i;
	u8 rdData;

	for(i=0; i<dataCount; i++)
	{
		if(i2cEepromReadByte(addr+i, &rdData)!=1)
		{
			return XST_FAILURE;
		}
		bufPtr[i] = rdData;
	}
	return XST_SUCCESS;
}

/*
 ***************************************************************************
 *  Function 	:
 *  	i2cEepromWriteByte
 *
 *  Description	:
 *  	write single byte to I2C Eeprom
 *
 *  Parameters	:
 *      u16 Address
 *      	eeprom write address
 *    	u8 data
 *       	write data
 *
 *  Return value:
 *		unsigned int 
 *			number of bytes written to eeprom
 *************************************************************************
 */
unsigned int i2cEepromWriteByte(u16 Address, u8 data)
{
	unsigned int SentByteCount;
	unsigned int AckByteCount;
	u8 WriteBuffer[sizeof(Address) + 1];

	/* prepare buffer for single byte write */
	WriteBuffer[0] = (u8)(Address >> 8);
	WriteBuffer[1] = (u8)(Address);
	WriteBuffer[2] = data;

	/* set address pointer of eeprom */
	do {
		SentByteCount = XIic_Send(I2C_DEVICE_BASE_ADDR, I2C_EEPROM_DEV_ADDRESS, (u8 *)&Address, sizeof(Address), XIIC_STOP);
		if (SentByteCount != sizeof(Address))
		{
			/* Send is aborted so reset Tx FIFO */
			XIic_WriteReg(I2C_DEVICE_BASE_ADDR, XIIC_CR_REG_OFFSET, XIIC_CR_TX_FIFO_RESET_MASK);
			XIic_WriteReg(I2C_DEVICE_BASE_ADDR, XIIC_CR_REG_OFFSET, XIIC_CR_ENABLE_DEVICE_MASK);
		}
	} while (SentByteCount != sizeof(Address));

	/* Write a single byte at the specified address to the EEPROM. */
	SentByteCount = XIic_Send(I2C_DEVICE_BASE_ADDR, I2C_EEPROM_DEV_ADDRESS,  WriteBuffer, sizeof(WriteBuffer), XIIC_STOP);

	/*
	 * Wait for the write to be complete by trying to do a write and
	 * the device will not ack if the write is still active.
	 */
	do {
		AckByteCount = XIic_Send(I2C_DEVICE_BASE_ADDR, I2C_EEPROM_DEV_ADDRESS, WriteBuffer, sizeof(Address), XIIC_STOP);
		if (AckByteCount != sizeof(Address))
		{
			/* Send is aborted so reset Tx FIFO */
			XIic_WriteReg(I2C_DEVICE_BASE_ADDR, XIIC_CR_REG_OFFSET, XIIC_CR_TX_FIFO_RESET_MASK);
			XIic_WriteReg(I2C_DEVICE_BASE_ADDR, XIIC_CR_REG_OFFSET, XIIC_CR_ENABLE_DEVICE_MASK);
		}
	} while (AckByteCount != sizeof(Address));

	/* Return the number of bytes written to the EEPROM */
	return SentByteCount - sizeof(Address);
}

/*
 ***************************************************************************
 *  Function 	:
 *  	i2cEepromReadByte
 *
 *  Description	:
 *  	read single byte from I2C Eeprom
 *
 *  Parameters	:
 *      u16 Address
 *      	eeprom read address
 *    	u8* data
 *       	read data pointer
 *
 *  Return value:
 *		unsigned int 
 *			number of bytes read from eeprom
 *************************************************************************
 */
unsigned int i2cEepromReadByte(u16 Address, u8* data)
{
	unsigned int ReceivedByteCount;
	u16 StatusReg;
	u8 WriteBuffer[sizeof(Address)];

	/* prepare buffer for single byte write */
	WriteBuffer[0] = (u8)(Address >> 8);
	WriteBuffer[1] = (u8)(Address);

	/* set address pointer of eeprom */
	do {
		StatusReg = XIic_ReadReg(I2C_DEVICE_BASE_ADDR, XIIC_SR_REG_OFFSET);
		if(!(StatusReg & XIIC_SR_BUS_BUSY_MASK))
		{
			ReceivedByteCount = XIic_Send(I2C_DEVICE_BASE_ADDR, I2C_EEPROM_DEV_ADDRESS, WriteBuffer, sizeof(Address), XIIC_STOP);
			if (ReceivedByteCount != sizeof(Address)) {
				/* Send is aborted so reset Tx FIFO */
				XIic_WriteReg(I2C_DEVICE_BASE_ADDR, XIIC_CR_REG_OFFSET, XIIC_CR_TX_FIFO_RESET_MASK);
				XIic_WriteReg(I2C_DEVICE_BASE_ADDR, XIIC_CR_REG_OFFSET, XIIC_CR_ENABLE_DEVICE_MASK);
			}
		}
	} while (ReceivedByteCount != sizeof(Address));

	/* Read single byte at the specified address from the EEPROM. */
	ReceivedByteCount = XIic_Recv(I2C_DEVICE_BASE_ADDR, I2C_EEPROM_DEV_ADDRESS, data, 1, XIIC_STOP);

	/* Return the number of bytes read from the EEPROM. */
	return ReceivedByteCount;
}

/*
 ****************************************************************
 **     *     *     *     *     *     *     *     *     *     * *
 * *   * *   * *   * *   * *   * *   * *   * *   * *   * *   *  *
 *  * *   * *   * *   * *   * *   * *   * *   * *   * *   * *   *
 *   *	   *	 *	   *	 *	   *	 *	   *	 *	   *	*
 ****************************************************************
 **						LTC2945 FUNCTIONS					   **
 ****************************************************************
 **     *     *     *     *     *     *     *     *     *     * *
 * *   * *   * *   * *   * *   * *   * *   * *   * *   * *   *  *
 *  * *   * *   * *   * *   * *   * *   * *   * *   * *   * *   *
 *   *	   *	 *	   *	 *	   *	 *	   *	 *	   *	*
 ****************************************************************
 */
/*
 ***************************************************************************
 *  Function 	:
 *  	i2cLtc2945Write
 *
 *  Description	:
 *  	write one byte to LTC2945 register through I2C
 *
 *  Parameters	:
 *      u8 regAddr
 *      	register address of LTC2945 (located in i2c_if.h)
 *    	u8 data
 *       	register data to be written to LTC2945 register
 *
 *  Return value:
 *		XStatus
 *			XST_SUCCESS	: write is successful
 *			XST_FAILURE	: write failed
 *************************************************************************
 */
XStatus i2cLtc2945Write(u8 regAddr, u8 data)
{
	u8 wrData[2];

	wrData[0] = regAddr;
	wrData[1] = data;

	/* register write single byte */
	if(XIic_Send(I2C_DEVICE_BASE_ADDR, LTC2945_DEV_ADDRESS, wrData, sizeof(wrData), XIIC_STOP) != sizeof(wrData))
	{
		return XST_FAILURE;
	}
	return XST_SUCCESS;
}

/*
 ***************************************************************************
 *  Function 	:
 *  	i2cLtc2945Read
 *
 *  Description	:
 *  	read several bytes from Ltc2945 register through I2C
 *
 *  Parameters	:
 *      u8 regAddr
 *      	read start register address of Ltc2945 (located in i2c_if.h)
 *    	u8* dataPtr
 *       	register data pointer to be read from Ltc2945 register
 *		u8 dataCount
 *			number of bytes to be read successively starting from regAddr
 *
 *  Return value:
 *		XStatus
 *			XST_SUCCESS	: read is successful
 *			XST_FAILURE	: read failed
 *************************************************************************
 */
XStatus i2cLtc2945Read(u8 regAddr, u8* dataPtr, u8 dataCount)
{
	/* register select */
	if(XIic_Send(I2C_DEVICE_BASE_ADDR, LTC2945_DEV_ADDRESS, &regAddr, 1, XIIC_REPEATED_START) != 1)
	{
		return XST_FAILURE;
	}
	/* register read */
	if(XIic_Recv(I2C_DEVICE_BASE_ADDR, LTC2945_DEV_ADDRESS, dataPtr, dataCount, XIIC_STOP) != dataCount)
	{
		return XST_FAILURE;
	}
	return XST_SUCCESS;
}

/*
 ***************************************************************************
 *  Function 	:
 *  	i2cLtc2945ReadCurrent
 *
 *  Description	:
 *  	read current value from LTC2945
 *
 *  Parameters	:
 *      int* curValPtr
 *      	current value pointer
 *
 *  Return value:
 *		XStatus
 *			XST_SUCCESS	: read is successful
 *			XST_FAILURE	: read failed
 *************************************************************************
 */
XStatus i2cLtc2945ReadCurrent(int* curValPtr)
{
	u8 tempPtr[2];
	int tempVal;

	if( i2cLtc2945Read(LTC2945_CURR_DSENSE_MSB_REG, tempPtr, 2) != XST_SUCCESS )
	{
		return XST_FAILURE;
	}

	tempVal = 16*((int)(tempPtr[0])) + ((int)(tempPtr[1]>>4));
	tempVal = 5*tempVal;
	curValPtr[0] = tempVal;

	return XST_SUCCESS;
}

/*
 ***************************************************************************
 *  Function 	:
 *  	i2cLtc2945ReadVoltage
 *
 *  Description	:
 *  	read voltage value from LTC2945
 *
 *  Parameters	:
 *      int* voltValPtr
 *      	voltage value pointer
 *
 *  Return value:
 *		XStatus
 *			XST_SUCCESS	: read is successful
 *			XST_FAILURE	: read failed
 *************************************************************************
 */
XStatus i2cLtc2945ReadVoltage(int* voltValPtr)
{
	u8 tempPtr[2];
	int tempVal;

	if( i2cLtc2945Read(LTC2945_CURR_VIN_MSB_REG, tempPtr, 2) != XST_SUCCESS )
	{
		return XST_FAILURE;
	}

	tempVal = 16*((int)(tempPtr[0])) + ((int)(tempPtr[1]>>4));
	tempVal = 25*tempVal;
	voltValPtr[0] = tempVal;

	return XST_SUCCESS;
}

/*
 ****************************************************************
 **     *     *     *     *     *     *     *     *     *     * *
 * *   * *   * *   * *   * *   * *   * *   * *   * *   * *   *  *
 *  * *   * *   * *   * *   * *   * *   * *   * *   * *   * *   *
 *   *	   *	 *	   *	 *	   *	 *	   *	 *	   *	*
 ****************************************************************
 **						LTC2991 FUNCTIONS					   **
 ****************************************************************
 **     *     *     *     *     *     *     *     *     *     * *
 * *   * *   * *   * *   * *   * *   * *   * *   * *   * *   *  *
 *  * *   * *   * *   * *   * *   * *   * *   * *   * *   * *   *
 *   *	   *	 *	   *	 *	   *	 *	   *	 *	   *	*
 ****************************************************************
 */
/*
 ***************************************************************************
 *  Function 	:
 *  	ltc2991Init
 *
 *  Description	:
 *  	initialize LTC2991 chip to read voltages
 *
 *  Parameters	:
 *      NA
 *
 *  Return value:
 *		XStatus
 *			XST_SUCCESS	: init is successful
 *			XST_FAILURE	: init failed
 *************************************************************************
 */
XStatus ltc2991Init(void)
{
	u8 regVal ;

	regVal = LTC2991_VCTRL_VALUE ;
	if( ltc2991Write(LTC2991_V1234_CONTROL, &regVal, 1) != XST_SUCCESS )
	{
		return XST_FAILURE ;
	}

	regVal = LTC2991_VCTRL_VALUE ;
	if( ltc2991Write(LTC2991_V5678_CONTROL, &regVal, 1) != XST_SUCCESS )
	{
		return XST_FAILURE ;
	}

	regVal = (u8) LTC2991_CHANNEL_VALUE ;
	if( ltc2991Write(LTC2991_CHANNEL_EN, &regVal, 1) != XST_SUCCESS )
	{
		return XST_FAILURE ;
	}

	regVal = LTC2991_ACQUISITION_VALUE ;
	if( ltc2991Write(LTC2991_ACQUISITION, &regVal, 1) != XST_SUCCESS )
	{
		return XST_FAILURE ;
	}

	return XST_SUCCESS ;
}

/*
 ***************************************************************************
 *  Function 	:
 *  	ltc2991Write
 *
 *  Description	:
 *  	write byte(s) to ltc2991Write register through I2C
 *
 *  Parameters	:
 *      u8 regAddr
 *      	register address of LTC2945 (located in i2c_if.h)
 *    	u8* data
 *       	start address of register data to be written to LTC2991
 *      u8 dataCount
 *      	number of data to be written to LTC2991
 *
 *  Return value:
 *		XStatus
 *			XST_SUCCESS	: write is successful
 *			XST_FAILURE	: write failed
 *************************************************************************
 */
XStatus ltc2991Write(u8 regAddr, u8* dataPtr, u8 dataCount)
{
	u8 wrData[16];
	u8 i;

	wrData[0] = regAddr;
	for(i=0;i<dataCount;i++)
	{
		wrData[i+1] = dataPtr[i];
	}

	if(XIic_Send(I2C_DEVICE_BASE_ADDR, LTC2991_DEV_ADDRESS, wrData, (dataCount+1), XIIC_STOP) != (dataCount+1))
	{
		return XST_FAILURE;
	}
	return XST_SUCCESS;
}

/*
 ***************************************************************************
 *  Function 	:
 *  	ltc2991Read
 *
 *  Description	:
 *  	read several bytes from LTC2991 register through I2C
 *
 *  Parameters	:
 *      u8 regAddr
 *      	read start register address of LTC2991 (located in i2c_if.h)
 *    	u8* dataPtr
 *       	register data pointer to be read from LTC2991 register
 *		u8 dataCount
 *			number of bytes to be read successively starting from regAddr
 *
 *  Return value:
 *		XStatus
 *			XST_SUCCESS	: read is successful
 *			XST_FAILURE	: read failed
 *************************************************************************
 */
XStatus ltc2991Read(u8 regAddr, u8* dataPtr, u8 dataCount)
{
	if(XIic_Send(I2C_DEVICE_BASE_ADDR, LTC2991_DEV_ADDRESS, &regAddr, 1, XIIC_REPEATED_START) != 1)
	{
		return XST_FAILURE;
	}
	if(XIic_Recv(I2C_DEVICE_BASE_ADDR, LTC2991_DEV_ADDRESS, dataPtr, dataCount, XIIC_STOP) != dataCount)
	{
		return XST_FAILURE;
	}
	lSleep(10000);
	return XST_SUCCESS;
}

/*
 ***************************************************************************
 *  Function 	:
 *  	ltc2991ReadBoardTemparature
 *
 *  Description	:
 *  	read temparature value from LTC2991
 *
 *  Parameters	:
 *      int* boardTempPtr
 *      	temparature value pointer
 *
 *  Return value:
 *		XStatus
 *			XST_SUCCESS	: read is successful
 *			XST_FAILURE	: read failed
 *************************************************************************
 */
XStatus ltc2991ReadBoardTemparature(int* boardTempPtr)
{
	u8 dBuffer[2];
	unsigned int rdTempData ;
	float temp;
	int temp_int;

	if( ltc2991Read(LTC2991_TDATA_OFFSET, dBuffer, LTC2991_VDATA_SIZE) != XST_SUCCESS)
	{
		return XST_FAILURE ;
	}
	rdTempData 	= (dBuffer[0] & 0x1F) ;
	rdTempData 	= (rdTempData << 8) | dBuffer[1];
	boardTempPtr[0] = rdTempData;

	temp = ((float)(rdTempData))*0.0625;
	if(temp>256)
	{
		temp = temp-512;
	}
	temp_int = (int)temp;
	boardTempPtr[0] = temp_int;
	return XST_SUCCESS ;
}

/*
 ***************************************************************************
 *  Function 	:
 *  	ltc2991PrintVoltage
 *
 *  Description	:
 *  	read voltages value from LTC2991
 *
 *  Parameters	:
 *      int* voltBuffer
 *      	voltage value pointer
 *
 *  Return value:
 *		XStatus
 *			XST_SUCCESS	: read is successful
 *			XST_FAILURE	: read failed
 *************************************************************************
 */
XStatus ltc2991PrintVoltage(int* voltBuffer)
{
	int chanId ;
	u8 statusMask = 0x01 ;
	u8 dBuffer[16];
	int rdVoltageData ;
	int sign ;
	float voltage ;

	lSleep(10000);
	if( ltc2991Read(LTC2991_VDATA_OFFSET,dBuffer,LTC2991_VDATA_SIZE*LTC2991_NOF_CHANNELS) != XST_SUCCESS)
	{
		return XST_FAILURE ;
	}

	for(chanId = 0 ; chanId < LTC2991_NOF_CHANNELS ; chanId++)
	{
		if( (dBuffer[chanId*LTC2991_VDATA_SIZE] & 0x80))
		{
			sign   			= (dBuffer[chanId*LTC2991_VDATA_SIZE] & 0x40) ;
			rdVoltageData 	= (dBuffer[chanId*LTC2991_VDATA_SIZE] & 0x3F) ;
			rdVoltageData 	= (rdVoltageData << 8) | dBuffer[chanId*LTC2991_VDATA_SIZE+1];

			if(sign > 0)
			{
				voltage = -5 + ( rdVoltageData * 305.18 ) / 1000000 ;
			}
			else
			{
				voltage = ( rdVoltageData * 305.18 ) / 1000000 ;
			}
			voltage = 1000*voltage;
			voltBuffer[chanId] = (int)voltage; /* in miliVolts */
		}
		statusMask = statusMask << 1 ;
	}

	return XST_SUCCESS ;
}
