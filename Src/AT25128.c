#include "spi.h"
#include "main.h"
#include "AT25128.h"

void AT25128_Write_EEPROM(unsigned int Addr, unsigned char Data)
{
	unsigned char Temp = 0, i;

	AT93C56_Enable;
	for(i = 0; i < 40; i++)AT93C56_Enable;
	Temp = WRITE;
	HAL_SPI_Transmit(&hspi1,&Temp, 1, 10);
	
	Temp = (Addr >> 8) & 0x00FF;
	HAL_SPI_Transmit(&hspi1,&Temp, 1, 10);
	Temp = Addr & 0x00FF;
	HAL_SPI_Transmit(&hspi1,&Temp, 1, 10);
	
	Temp = Data;
	HAL_SPI_Transmit(&hspi1,&Temp, 1, 10);
	for(i = 0; i < 20; i++)AT93C56_Enable;
	
	AT93C56_Disable;
}

unsigned char AT25128_READ_EEPROM(unsigned int Addr)
{
	unsigned char Read, Temp = 0, i;

	AT93C56_Enable;
	for(i = 0; i < 40; i++)AT93C56_Enable;
	Temp = READ;
	HAL_SPI_Transmit(&hspi1,&Temp, 1, 10);
	
	Temp = (Addr >> 8) & 0x00FF;
	HAL_SPI_Transmit(&hspi1,&Temp, 1, 10);
	Temp = Addr & 0x00FF;
	HAL_SPI_Transmit(&hspi1,&Temp, 1, 10);

	Temp = 0xFF;	
	HAL_SPI_TransmitReceive(&hspi1, &Temp, &Read, 1, 10);
	HAL_Delay(1);
	AT93C56_Disable;
		
	return Read;
}

void AT25128_EEPROM_Control(unsigned char OPCODE)
{
	unsigned char Temp = 0, i;	
	
	AT93C56_Enable;	
	for(i = 0; i < 40; i++)AT93C56_Enable;
	
	Temp = OPCODE & 0x00FF;
	HAL_SPI_Transmit(&hspi1,&Temp, 1, 10);

	for(i = 0; i < 100; i++)AT93C56_Enable;
	//HAL_Delay(1);
	AT93C56_Disable;
}

unsigned char Read_WRSR(void)
{
	unsigned char Read = 0, Temp = 0, i;	
	
	AT93C56_Enable;
	for(i = 0; i < 40; i++)AT93C56_Enable;
	
	Temp = RDSR;
	HAL_SPI_Transmit(&hspi1,&Temp, 1, 10);
	
	Temp = 0xFF;	
	HAL_SPI_TransmitReceive(&hspi1, &Temp, &Read, 1, 10);
	HAL_Delay(1);
	AT93C56_Disable;
	
	return Read;
}

void Write_WRSR(unsigned char Data)
{	
	unsigned char Temp = 0, i;
	
	AT93C56_Enable;
	for(i = 0; i < 40; i++)AT93C56_Enable;
	
	Temp = WRSR;
	HAL_SPI_Transmit(&hspi1,&Temp, 1, 10);

	Temp = Data;
	HAL_SPI_Transmit(&hspi1,&Temp, 1, 10);
	HAL_Delay(1);
	AT93C56_Disable;	
}


