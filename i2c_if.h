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
* @file i2c_if.h
*
* Contains the implementation of the I2C Interface Communication component
* of ADAPTOR Software.
*
* <pre>
* MODIFICATION HISTORY:
*
* Ver   Who  			Date     	Changes
* ----- ----------		-------- 	-----------------------------------------------
* 1.00a B.SEKERLISOY  	04/01/2018 	First release
*
* </pre>
*
*****************************************************************************/

#ifndef SRC_I2C_H_
#define SRC_I2C_H_

/***************************** Include Files ********************************/
#include "xstatus.h"
#include "xparameters.h"

/************************** Constant Definitions ****************************/
#define I2C_DEVICE_BASE_ADDR			XPAR_IIC_0_BASEADDR

/*
 * AD7414 CONSTANTS
 */
#define AD7414_DEV_ADDRESS				0x4A
#define AD7414_REG_TEMP					0x00
#define AD7414_REG_CONFIG				0x01
#define AD7414_REG_THIGH				0x02
#define AD7414_REG_TLOW					0x03

/*
 * DS1682 CONSTANTS
 */
#define DS1682_DEV_ADDRESS				0x6B
#define DS1682_REG_CONFIG				0x00
#define DS1682_REG_ALARM				0x01
#define DS1682_REG_ETC					0x05
#define DS1682_REG_RESET				0x1D
#define DS1682_REG_WRITEDIS				0x1E

/*
 * 24LC32 CONSTANTS
 */
#define I2C_EEPROM_DEV_ADDRESS			0x50
#define EEPROM_TEST_ADDR				0x0000
#define EEPROM_SW_PAR_ADDRESS			0x0100

/*
 * LTC2945 CONSTANTS
 */
#define LTC2945_DEV_ADDRESS				0x6F
#define LTC2945_CURR_CONTROL_REG 		0x00
#define LTC2945_CURR_DSENSE_MSB_REG 	0x14
#define LTC2945_CURR_VIN_MSB_REG 		0x1E

/*
 * LTC2991 CONSTANTS
 */
#define LTC2991_DEV_ADDRESS			0x4C

#define LTC2991_STATUS_LOW			0x00
#define LTC2991_CHANNEL_EN			0x01
#define LTC2991_V1234_CONTROL		0x06
#define LTC2991_V5678_CONTROL		0x07
#define LTC2991_ACQUISITION			0x08
#define LTC2991_VDATA_OFFSET		0x0A
#define LTC2991_TDATA_OFFSET		0x1A
#define LTC2991_VDATA_SIZE			0x02

#define LTC2991_CHANNEL_VALUE		0xF8	// All four channel pairs are enabled
#define LTC2991_VCTRL_VALUE			0x00	// Filter Disabled Single Ended Voltage in Celsius for each pair
#define LTC2991_ACQUISITION_VALUE	0x10	// Acquisition in Repeated Mode

#define LTC2991_NOF_CHANNELS		0x08

/**************************** Type Definitions ******************************/

/***************** Macros (Inline Functions) Definitions ********************/

/************************** Function Prototypes *****************************/
void i2cInitialize(void);
void i2cFree(void);

XStatus i2cAD7414Write(u8 regAddr, u8 data);
XStatus i2cAD7414Read(u8 regAddr, u8* dataPtr, u8 dataCount);
XStatus i2cAD7414ReadTemp(int* tempValPtr);

XStatus i2cDS1682Write(u8 regAddr, u8* dataPtr, u8 dataCount);
XStatus i2cDS1682Read(u8 regAddr, u8* dataPtr, u8 dataCount);
XStatus i2cDS1682ReadETC(unsigned int* tempValPtr);
XStatus i2cDS1682Reset(void);

unsigned int i2cEepromWriteByte(u16 Address, u8 data);
unsigned int i2cEepromReadByte(u16 Address, u8* data);
XStatus eepromWrite(u16 addr, u8* bufPtr, unsigned int dataCount);
XStatus eepromRead(u16 addr, u8* bufPtr, unsigned int dataCount);
XStatus eepromTest(void);

XStatus i2cLtc2945Write(u8 regAddr, u8 data);
XStatus i2cLtc2945Read(u8 regAddr, u8* dataPtr, u8 dataCount);
XStatus i2cLtc2945ReadCurrent(int* curValPtr);
XStatus i2cLtc2945ReadVoltage(int* voltValPtr);

XStatus ltc2991Init(void);
XStatus ltc2991Write(u8 regAddr, u8* dataPtr, u8 dataCount);
XStatus ltc2991Read(u8 regAddr, u8* dataPtr, u8 dataCount);
XStatus ltc2991ReadBoardTemparature(int* boardTempPtr);
XStatus ltc2991PrintVoltage(int* voltBuffer);

#endif /* SRC_I2C_H_ */
