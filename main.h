#ifndef SRC_MAIN_H_
#define SRC_MAIN_H_

#include "xstatus.h"
#include "xparameters.h"

#define BRAM_WRITE_BASE_ADDRESS XPAR_AXI_BRAM_CTRL_0_S_AXI_BASEADDR
#define BRAM_READ_BASE_ADDRESS  XPAR_AXI_BRAM_CTRL_0_S_AXI_BASEADDR + 4*(0x800)

#define I2C_DEVICE_BASE_ADDR	XPAR_IIC_0_BASEADDR

#define CLK_DIV_BAUD 	((u32*) (BRAM_WRITE_BASE_ADDRESS + 4*(0x00)))
#define TX_BUF_DATA 	((u32*) (BRAM_WRITE_BASE_ADDRESS + 4*(0x01)))
#define LEDS		 	    ((u32*) (BRAM_WRITE_BASE_ADDRESS + 4*(0x02)))
#define SEVEN_SEGW		((u32*) (BRAM_WRITE_BASE_ADDRESS + 4*(0x03)))

#define RX_BUF_EMTY 	((u32*) (BRAM_READ_BASE_ADDRESS + 4*(0x00)))
#define RX_BUF_DATA 	((u32*) (BRAM_READ_BASE_ADDRESS + 4*(0x01)))
#define TX_BUF_FULL 	((u32*) (BRAM_READ_BASE_ADDRESS + 4*(0x02)))

/*
 * For MCP4725:
 */
#define MCP4725_DEV_ADDRESS					    0x62 //A0 is logic zero, wired to the ground. 0x63 if its logic 1.
#define MCP4725_WRITEDAC_ADDRESS			  0x40
#define MCP4725_WRITEDACEEPROM_ADDRESS	0x60

XStatus i2cAD7414Write(u8 regAddr, u8 data);
XStatus i2cAD7414Read(u8 regAddr, u8* dataPtr, u8 dataCount);

#endif


