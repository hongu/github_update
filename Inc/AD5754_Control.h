#ifndef __AD5754_CONTROL_H
#define __AD5754_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal_spi.h"
#include "spi.h"

///DAC AD5754 Control Register Address/////////////////
#define	DAC_Register						0
#define	Output_Range_Select_Register		1
#define	Power_Control_Register				2
#define	Control_Regiser						3

///DAC AD5754 Channel Address/////////////////
#define	DACA		0
#define	DACB		1
#define	DACC		2
#define	DACD		3
#define	All_DAC		4

//Read-Write Data////
#define	AD5754_Write	0
#define	AD5754_Read		1

//Command Byte Bit Number//Output_Range_Select_Register/
#define	R_W_Bit			7
#define	Zero_Bit		6
#define	REG_Bit			3
#define	Addr_Bit		0

//Control Register Bit Number////
#define	TSD_Enable		3
#define	Clamp_Enable	2
#define	CLR_Enable		1
#define	SDO_Enable		0

///Power Control Bit Nember//////
#define	OC_Start_Bit	7
#define	TSD_Shutdown	5
#define	PU_Ref			4
#define	PU_Start		0

extern void AD5754_Init(void);
//extern void AD5754_DA_Out_Function(unsigned char * Data, char Num);
extern void AD5754_SPI_Write(unsigned int Data, unsigned char Addr);
extern void AD5754_Read_Funciton(unsigned char Data, unsigned char Addr);
//extern void AD5754_DA_Out_Function(unsigned int Data1, unsigned int Data2, unsigned int Data3, unsigned int Data4, char Num);
extern void AD5754_DA_Out_Function(unsigned int *Data, char Num);

#ifdef __cplusplus
}
#endif

#endif /* __AD5754_CONTROL_H */