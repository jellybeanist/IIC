#ifndef SRC_MAIN_H_
#define SRC_MAIN_H_

#include "xstatus.h"
#include "xparameters.h"

#define BRAM_WRITE_BASE_ADDRESS XPAR_AXI_BRAM_CTRL_0_S_AXI_BASEADDR
#define BRAM_READ_BASE_ADDRESS  XPAR_AXI_BRAM_CTRL_0_S_AXI_BASEADDR + 4*(0x800)

#define I2C_DEVICE_BASE_ADDR	XPAR_IIC_0_BASEADDR

#define CLK_DIV_BAUD 	((u32*) (BRAM_WRITE_BASE_ADDRESS + 4*(0x00)))
#define TX_BUF_DATA 	((u32*) (BRAM_WRITE_BASE_ADDRESS + 4*(0x01)))
#define LEDS		 	((u32*) (BRAM_WRITE_BASE_ADDRESS + 4*(0x02)))
#define SEVEN_SEGW		((u32*) (BRAM_WRITE_BASE_ADDRESS + 4*(0x03)))

#define RX_BUF_EMTY 	((u32*) (BRAM_READ_BASE_ADDRESS + 4*(0x00)))
#define RX_BUF_DATA 	((u32*) (BRAM_READ_BASE_ADDRESS + 4*(0x01)))
#define TX_BUF_FULL 	((u32*) (BRAM_READ_BASE_ADDRESS + 4*(0x02)))

/*
 * For MCP4725 => 12 Bit DAC :
 */
#define MCP4725_DEV_ADDRESS					0x62 //A0 is logic zero, wired to the ground. 0x63 if its logic 1.
#define MCP4725_WRITEDAC_ADDRESS			0x40
#define MCP4725_WRITEDACEEPROM_ADDRESS		0x60

/*
 * For GY-231 (QMC5883) => 3 axis compass :
 * => Temperature Output Registers (07H and 08H).
 * => Mode register (09H).
 */

#define QMC5883_DEV_ADDRESS					0x0D //Device address (0001101)
#define QMC5883_X_AXIS_LSB					0x00 //X axis information (LSB)
#define QMC5883_X_AXIS_MSB					0x01 //X axis information (MSB)
#define QMC5883_Y_AXIS_LSB					0x02 //Y axis information (LSB)
#define QMC5883_Y_AXIS_MSB					0x03 //Y axis information (MSB)
#define QMC5883_Z_AXIS_LSB					0x04 //Z axis information (LSB)
#define QMC5883_Z_AXIS_MSB					0x05 //Z axis information (MSB)
#define QMC5883_TEMPERATURE_LSB				0x07 //Temperature value (LSB)
#define QMC5883_TEMPERATURE_MSB				0x08 //Temperature value (MSB)
#define QMC5883_WRITE_ADDRESS				0x09 //Temperature value (MSB)


/*
 * For GY-521 (MPU6050) => 6 axis accelerometer :
 */

#define MPU6050_DEV_ADDRESS     			0x68 //A0 is logic zero, wired to the ground. 0x69 if its logic 1.
#define MPU6050_TEMPERATURE_MSB       		0x41
#define MPU6050_TEMPERATURE_LSB       		0x42

/*
 * For PCF8574 => IIC LCD :
 */

#define	PCF8574_DEV_ADDRESS     			0x27
#define	PCF8574_WRITE_ADDRESS     			0x4E
#define	PCF8574_READ_ADDRESS     			0x4F

XStatus IIC_PCF8574_Write(u8 regAddr, u8 data);
XStatus IIC_PCF8574_Read(u8 regAddr, u8* dataPtr, u8 dataCount);

XStatus IIC_MCP4725_Write(u8 regAddr, u8 data0, u8 data1);
XStatus IIC_MCP4725_Read(u8 regAddr, u8* dataPtr, u8 dataCount);

XStatus IIC_QMC5883_Write(u8 regAddr, u8 slaveAddr, u8 data);
XStatus IIC_QMC5883_Read(u8 regAddr, u8* dataPtr, u8 dataCount);

XStatus IIC_MPU6050_Write(u8 regAddr, u8 data);
XStatus IIC_MPU6050_Read(u8 regAddr, u8* dataPtr, u8 dataCount);

#endif


