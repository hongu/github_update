#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_spi.h"
#include "stm32f4xx_hal_def.h"
#include "AD5754_Control.h"
#include "main.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"


// void AD5754_NSS(unsigned char Port_State)
void AD5754_NSS(GPIO_PinState Port_State)
{
	HAL_GPIO_WritePin(DAC_NSS_GPIO_Port, DAC_NSS_Pin, Port_State);
}

void AD5754_SPI_Write(unsigned int Data, unsigned char Addr)
{
	unsigned char SPI_Send_Buff[3], i;

	SPI_Send_Buff[0] = Addr;
	SPI_Send_Buff[1] = (Data >> 8) & 0x00FF;
	SPI_Send_Buff[2] = Data & 0x00FF;

	for(i = 0; i < 20; i++)AD5754_NSS(LOW);
	HAL_SPI_Transmit(&hspi2,(unsigned char *)&SPI_Send_Buff, 3, 10);
	for(i = 0; i < 20; i++)AD5754_NSS(HIGH);

	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_SET);
}

void AD5754_Init(void)
{
	unsigned char Register_Buff, i;
	unsigned int Data_Buff = 0;
	unsigned char SPI_Send_Buff[3];

	HAL_GPIO_WritePin(GPIOH, DAC_CLR_Pin|DAC_BIN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(DAC_NSS_GPIO_Port, DAC_NSS_Pin, GPIO_PIN_SET);

	Register_Buff = (AD5754_Write << R_W_Bit) | (0 << Zero_Bit) | (Output_Range_Select_Register << REG_Bit) | (All_DAC << 0);
	Data_Buff = 0x0001;

	SPI_Send_Buff[0] = Register_Buff;
	SPI_Send_Buff[1] = (Data_Buff >> 8) & 0x00FF;
	SPI_Send_Buff[2] = Data_Buff & 0x00FF;

	AD5754_NSS(LOW);
	HAL_SPI_Transmit(&hspi2,(unsigned char *)&SPI_Send_Buff, 3, 10);
	for(i = 0; i < 20; i++)AD5754_NSS(HIGH);

	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_SET);

	//HAL_Delay(100);

	Register_Buff = (AD5754_Write << R_W_Bit) | (0 << Zero_Bit) | (Control_Regiser << REG_Bit) | (DACB << 0);
	Data_Buff = (unsigned int)(0 << TSD_Enable) | (0 << Clamp_Enable) | (0 << CLR_Enable) | (0 << SDO_Enable);

	SPI_Send_Buff[0] = Register_Buff;
	SPI_Send_Buff[1] = (Data_Buff >> 8) & 0x00FF;
	SPI_Send_Buff[2] = Data_Buff & 0x00FF;

	for(i = 0; i < 20; i++)AD5754_NSS(LOW);
	HAL_SPI_Transmit(&hspi2,(unsigned char *)&SPI_Send_Buff, 3, 10);
	for(i = 0; i < 20; i++)AD5754_NSS(HIGH);

	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_SET);

	//HAL_Delay(100);

	Register_Buff = (AD5754_Write << R_W_Bit) | (0 << Zero_Bit) | (Power_Control_Register << REG_Bit) | (0 << 0);
	Data_Buff = (unsigned int)(0 << OC_Start_Bit) | (0 << TSD_Shutdown) | (1 << PU_Ref) | (15 << PU_Start);

	SPI_Send_Buff[0] = Register_Buff;
	SPI_Send_Buff[1] = (Data_Buff >> 8) & 0x00FF;
	SPI_Send_Buff[2] = Data_Buff & 0x00FF;

	for(i = 0; i < 20; i++)AD5754_NSS(LOW);
	HAL_SPI_Transmit(&hspi2,(unsigned char *)&SPI_Send_Buff, 3, 10);
	for(i = 0; i < 20; i++)AD5754_NSS(HIGH);

	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_SET);

	//HAL_Delay(5);

}

void AD5754_Read_Funciton(unsigned char Data, unsigned char Addr)
{
	unsigned char Register_Buff, i;
	unsigned int Data_Buff = 0;
	unsigned char SPI_Send_Buff[3];

	Register_Buff = (AD5754_Read << R_W_Bit) | (0 << Zero_Bit) | (Data << REG_Bit) | (Addr << 0);
	Data_Buff = 0;

	SPI_Send_Buff[0] = Register_Buff;
	SPI_Send_Buff[1] = (Data_Buff >> 8) & 0x00FF;
	SPI_Send_Buff[2] = Data_Buff & 0x00FF;

	for(i = 0; i < 20; i++)AD5754_NSS(LOW);
	HAL_SPI_Transmit(&hspi2,(unsigned char *)&SPI_Send_Buff, 3, 10);
	for(i = 0; i < 20; i++)AD5754_NSS(HIGH);

	for(i = 0; i < 20; i++)HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_SET);

	//HAL_Delay(100);

	Register_Buff = 0;
	Data_Buff = 0;

	SPI_Send_Buff[0] = Register_Buff;
	SPI_Send_Buff[1] = (Data_Buff >> 8) & 0x00FF;
	SPI_Send_Buff[2] = Data_Buff & 0x00FF;

	AD5754_NSS(LOW);
	HAL_SPI_Transmit(&hspi2,(unsigned char *)&SPI_Send_Buff, 3, 10);
	for(i = 0; i < 20; i++)AD5754_NSS(HIGH);

	for(i = 0; i < 20; i++)HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_SET);
}


//void AD5754_DA_Out_Function(unsigned int Data1, unsigned int Data2, unsigned int Data3, unsigned int Data4, char Num)
void AD5754_DA_Out_Function(unsigned int *Data, char Num)
{
	unsigned char Register_Buff = 0, j, DA_Count = 0; //
	// unsigned int Data_Buff = 0;
	unsigned char SPI_Send_Buff[3];

	// HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_SET);
	for(j = 0; j < Num; j++){
		SPI_Send_Buff[0] = Register_Buff = (DA_Count << 0);
		SPI_Send_Buff[1] = (Data[DA_Count] >> 8) & 0x00FF;
		SPI_Send_Buff[2] = Data[DA_Count] & 0x00FF;
		DA_Count++;

		AD5754_NSS(LOW);
		HAL_SPI_Transmit(&hspi2,(unsigned char *)&SPI_Send_Buff, 3, 10);
		AD5754_NSS(HIGH);

		HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_SET);
		}
	// HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_RESET);

	// HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_SET);
	// for(j = 0; j < Num; j++){
	// 	SPI_Send_Buff[0] = DA_Count; // Register_Buff = (DA_Count << 0);
	// 	SPI_Send_Buff[1] = (Data[DA_Count] >> 8) & 0x00FF;
	// 	SPI_Send_Buff[2] = Data[DA_Count] & 0x00FF;
	// 	DA_Count++;

	// 	AD5754_NSS(LOW);
	// 	HAL_SPI_Transmit(&hspi2,(unsigned char *)&SPI_Send_Buff, 3, 10);
	// 	AD5754_NSS(HIGH);

	// 	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	// 	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	// 	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_SET);
	// 	}
	// HAL_GPIO_WritePin(GPIOG, GPIO_PIN_15, GPIO_PIN_RESET);
/*****************************************************************************************************************/

	/*
	SPI_Send_Buff[0] = Register_Buff = (DA_Count++ << 0);
	SPI_Send_Buff[1] = (Data1 >> 8) & 0x00FF;
	SPI_Send_Buff[2] = Data1 & 0x00FF;

	for(i = 0; i < 20; i++)AD5754_NSS(LOW);
	HAL_SPI_Transmit(&hspi2,(unsigned char *)&SPI_Send_Buff, 3, 10);
	for(i = 0; i < 20; i++)AD5754_NSS(HIGH);

	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	for(i = 0; i < 20; i++)HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_SET);


	SPI_Send_Buff[0] = Register_Buff = (DA_Count++ << 0);
	SPI_Send_Buff[1] = (Data2 >> 8) & 0x00FF;
	SPI_Send_Buff[2] = Data2 & 0x00FF;

	for(i = 0; i < 20; i++)AD5754_NSS(LOW);
	HAL_SPI_Transmit(&hspi2,(unsigned char *)&SPI_Send_Buff, 3, 10);
	for(i = 0; i < 20; i++)AD5754_NSS(HIGH);

	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	for(i = 0; i < 20; i++)HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_SET);

	SPI_Send_Buff[0] = Register_Buff = (DA_Count++ << 0);
	SPI_Send_Buff[1] = (Data3 >> 8) & 0x00FF;
	SPI_Send_Buff[2] = Data3 & 0x00FF;

	for(i = 0; i < 20; i++)AD5754_NSS(LOW);
	HAL_SPI_Transmit(&hspi2,(unsigned char *)&SPI_Send_Buff, 3, 10);
	for(i = 0; i < 20; i++)AD5754_NSS(HIGH);

	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	for(i = 0; i < 20; i++)HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_SET);

	SPI_Send_Buff[0] = Register_Buff = (DA_Count++ << 0);
	SPI_Send_Buff[1] = (Data4 >> 8) & 0x00FF;
	SPI_Send_Buff[2] = Data4 & 0x00FF;

	for(i = 0; i < 20; i++)AD5754_NSS(LOW);
	HAL_SPI_Transmit(&hspi2,(unsigned char *)&SPI_Send_Buff, 3, 10);
	for(i = 0; i < 20; i++)AD5754_NSS(HIGH);

	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_RESET);
	for(i = 0; i < 20; i++)HAL_GPIO_WritePin(MT_DAC_EN_GPIO_Port, MT_DAC_EN_Pin, GPIO_PIN_SET);
	*/
}


