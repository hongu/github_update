#ifndef __AD25128_H
#define __AD25128_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal_spi.h"
#include "spi.h"

#define	READ_OPCODE			6			//		1			  10				A6 ~ A0
#define	EWEN_OPCODE			4			//		1			  00				11xxxxx
#define	ERASE_OPCODE		7			//		1			  11				A6 ~ A0
#define	WRITE_OPCODE		5			//		1			  01				A6 ~ A0
#define	ERAL_OPCODE			4			//		1			  00				10xxxxx
#define	WRAL_OPCODE			4			//		1			  00				01xxxxx
#define	EWDS_OPCODE			4			//		1			  00				00xxxxx

#define	WREN			0x06			//Set Write Enable Latch
#define	WRDI			0x04			//Reset Write Enable Latch
#define	RDSR			0x05			//Read Status Register
#define	WRSR			0x01			//Write Status Register
#define	READ			0x03			//Read Data from Memory Array
#define	WRITE			0x02			//Write Data to Memory Array
	
#define	EWEN_Addr		0x60
#define	ERAL_Addr		0x40
#define	WRAL_Addr		0x20
#define	EWDS_Addr		0x00

//////// AT93C56 Data Siza /////////////
#define	Command_Data_Size		0x09
#define	Data_Size				0x07

//////// Chip_Select //////////////
#define	AT93C56_Enable				HAL_GPIO_WritePin(ROM_CS_GPIO_Port, ROM_CS_Pin, GPIO_PIN_RESET)
#define	AT93C56_Disable				HAL_GPIO_WritePin(ROM_CS_GPIO_Port, ROM_CS_Pin, GPIO_PIN_SET)

#define	AT25128_WP_High				HAL_GPIO_WritePin(GPIOB, ROM_WP_Pin, GPIO_PIN_SET)
#define	AT25128_WP_Low				HAL_GPIO_WritePin(GPIOB, ROM_WP_Pin, GPIO_PIN_RESET)

#define	Tag_Addr_Num				7
#define	Parameter_Add_Offset			100
#define	Course_Addr_Offset			1792

extern void AT93C46_SPIResetting(unsigned int Size);
extern void AT25128_Write_EEPROM(unsigned int Addr, unsigned char Data);
extern unsigned char AT25128_READ_EEPROM(unsigned int Addr);
extern unsigned char Read_WRSR(void);
extern void Write_WRSR(unsigned char Data);
extern void AT25128_EEPROM_Control(unsigned char OPCODE);

#ifdef __cplusplus
}
#endif

#endif /* __AD25128_H */