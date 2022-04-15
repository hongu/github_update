#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_def.h"
#include "HS_Uart.h"
#include "stdarg.h"
#include "string.h"
#include "stdio.h"

uint8_t Com0_Receive, Com0_RecBuff[Com0buffsize];
unsigned int Com0_Rwi_posi = 0, Com0_Rrd_posi = 0;

uint8_t Com1_Receive, Com1_RecBuff[Com1buffsize];
unsigned int Com1_Rwi_posi = 0, Com1_Rrd_posi = 0;

uint8_t Com2_Receive, Com2_RecBuff[Com2buffsize];
unsigned int Com2_Rwi_posi = 0,Com2_Rrd_posi = 0;

uint8_t Com3_Receive, Com3_RecBuff[Com3buffsize];
unsigned int Com3_Rwi_posi = 0,Com3_Rrd_posi = 0;

uint8_t Com4_Receive, Com4_RecBuff[Com4buffsize];
unsigned int Com4_Rwi_posi = 0,Com4_Rrd_posi = 0;

uint8_t Com5_Receive, Com5_RecBuff[Com5buffsize];
unsigned int Com5_Rwi_posi = 0,Com5_Rrd_posi = 0;

uint8_t Com0_Transmit, Com0_TXBuff[Com0TxbuffSize];
//unsigned int Com0_Twi_posi = 0,Com0_Trd_posi = 0;

uint8_t Com1_Transmit, Com1_TXBuff[Com1TxbuffSize];
//unsigned int Com1_Twi_posi = 0,Com1_Trd_posi = 0;

uint8_t Com2_Transmit, Com2_TXBuff[Com2TxbuffSize];
//unsigned int Com2_Twi_posi = 0,Com2_Trd_posi = 0;

uint8_t Com3_Transmit, Com3_TXBuff[Com3TxbuffSize];
//unsigned int Com3_Twi_posi = 0,Com3_Trd_posi = 0;

uint8_t Com4_Transmit, Com4_TXBuff[Com4TxbuffSize];
//unsigned int Com4_Twi_posi = 0,Com4_Trd_posi = 0;

uint8_t Com5_Transmit, Com5_TXBuff[Com5TxbuffSize];
//unsigned int Com5_Twi_posi = 0,Com5_Trd_posi = 0;

void UART_REC_IT_Set(uint8_t Num)
{
	 switch(Num)
	 {
	 case 1 : HAL_UART_Receive_IT(&huart1, &Com0_Receive, 1); break;
	 case 2 : HAL_UART_Receive_IT(&huart2, &Com1_Receive, 1); break;
	 case 3 : HAL_UART_Receive_IT(&huart3, &Com2_Receive, 1); break;
	 case 4 : HAL_UART_Receive_IT(&huart8, &Com3_Receive, 1); break;
	 case 5 : HAL_UART_Receive_IT(&huart6, &Com4_Receive, 1); break;
	 case 6 : HAL_UART_Receive_IT(&huart7, &Com5_Receive, 1); break;
	 }
}

uint8_t USART_GetFlagStatus(USART_TypeDef* USARTx, uint16_t USART_FLAG)
{
	uint8_t bitstatus;

	if ((USARTx->SR & USART_FLAG) != (uint16_t)RESET)
	{
		bitstatus = SET;
	}
	else
	{
		bitstatus = RESET;
	}
	return bitstatus;
}

void USART_SendData(USART_TypeDef* USARTx, uint16_t Data)
{
	assert_param(IS_USART_ALL_PERIPH(USARTx));
	assert_param(IS_USART_DATA(Data));

	USARTx->DR = (Data & (uint16_t)0x01FF);
}

int StrLen(int8_t *dest)
{
	 volatile int8_t *tmp = dest;

	 if (!tmp) return -1;

	 while (*tmp!=0) tmp++;

	 return (tmp - dest);
}  // StrLen.

/*** SCIA USER FUNCTION BEGIN *********************************************************/

void SCIA_TX_Char(uint8_t Data)
{
	while(!USART_GetFlagStatus(USART1, USART_FLAG_TXE));
	USART_SendData(USART1, Data);
}

void SCIA_TX_String(int8_t *str)
{
	int count = 0;

	while (str[count] != 0x00)
	{
		while(!USART_GetFlagStatus(USART1, USART_FLAG_TXE));
		USART_SendData(USART1, str[count]);
		count++;
	}
}

void SCIA_TX_String_Num(int8_t *str, unsigned int num)
{
	int count = 0;

	while (num--)
	{
		while(!USART_GetFlagStatus(USART1, USART_FLAG_TXE));
		USART_SendData(USART1, str[count]);
		count++;
	}
}

int SCIA_TX_String_IT(int8_t *str)
{
	volatile int8_t *tmp = str;
	int i = 0;

	if (!tmp) return -1;

	while ( *tmp!=0 )
	{
		Com0_TXBuff[i] = tmp[i];
		if ( i > Com0TxbuffSize ) return -1;
		else i++;
	}

	HAL_UART_Transmit_IT(&huart1, Com0_TXBuff, i);

	return i;
}

int SCIA_TX_String_Num_IT(int8_t *str, int len)
{
	volatile int8_t *tmp = str;
	int i = 0;

	for ( i = 0; i < len; i++ )
	{
		Com0_TXBuff[i] = tmp[i];
		if ( i > Com0TxbuffSize ) return -1;
	}

	HAL_UART_Transmit_IT(&huart1, Com0_TXBuff, i);

	return i;
}

uint8_t SCIA_Read_Check ( void )
{
	if ( Com0_Rwi_posi != Com0_Rrd_posi ) return 1;
	else return 0;
}

uint8_t SCIA_Read_Char ( void )
{
	uint8_t rd = 0;

	if ( Com0_Rwi_posi != Com0_Rrd_posi ) rd = Com0_RecBuff[Com0_Rrd_posi++];
	if ( Com0_Rrd_posi >= Com0buffsize ) Com0_Rrd_posi = 0;

	return rd;
}

/*** SCIA USER FUNCTION END ***********************************************************/

/*** SCIB USER FUNCTION BEGIN *********************************************************/

void SCIB_TX_Char(uint8_t Data)
{
	while(!USART_GetFlagStatus(USART2, USART_FLAG_TXE));
	USART_SendData(USART2, Data);
}

unsigned int Read_Rwi_Position(void)
{
	return(Com1_Rwi_posi);
}

void Reset_Rwi_Position(void)
{
	Com1_Rrd_posi = Com1_Rwi_posi = 0;
}

void SCIB_TX_String(int8_t *str)
{
	int count = 0;

	while (str[count] != 0x00)
	{
		while(!USART_GetFlagStatus(USART2, USART_FLAG_TXE));
		USART_SendData(USART2, str[count]);
		count++;
	}
}

void SCIB_TX_String_Num(int8_t *str, unsigned int num)
{
	int count = 0;

	while (num--)
	{
		while(!USART_GetFlagStatus(USART2, USART_FLAG_TXE));
		USART_SendData(USART2, str[count]);
		count++;
	}
}

int SCIB_TX_String_IT(int8_t *str)
{
	volatile int8_t *tmp = str;
	int i = 0;

	if (!tmp) return -1;

	while ( *tmp != 0 )
	{
		Com1_TXBuff[i] = tmp[i];
		if ( i > Com1TxbuffSize ) return -1;
		else i++;
	}

	HAL_UART_Transmit_IT(&huart2, Com1_TXBuff, i);

	return i;
}

int SCIB_TX_String_Num_IT(int8_t *str, int len)
{
	volatile int8_t *tmp = str;
	int i = 0;

	for ( i = 0; i < len; i++ ) {
		Com1_TXBuff[i] = tmp[i];
		if ( i > Com1TxbuffSize ) return -1;
	}

	HAL_UART_Transmit_IT(&huart2, Com1_TXBuff, i);

	return i;
}


uint8_t SCIB_Read_Check ( void )
{
	if ( Com1_Rwi_posi != Com1_Rrd_posi ) return 1;
	else return 0;
}

uint8_t SCIB_Read_Char ( void )
{
	uint8_t rd = 0;

	if ( Com1_Rwi_posi != Com1_Rrd_posi ) rd = Com1_RecBuff[Com1_Rrd_posi++];
	if ( Com1_Rrd_posi >= Com1buffsize ) Com1_Rrd_posi = 0;

	return rd;
}

/*** SCIB USER FUNCTION END ***********************************************************/

/*** SCIC USER FUNCTION BEGIN *********************************************************/
void SCIC_RecBuff_Clear ( void )
{
	unsigned int i;

	for(i = 0; i < Com2buffsize; i++) Com2_RecBuff[i] = 0;

	Com2_Rwi_posi = 0;
	Com2_Rrd_posi = 0;
}

void SCIC_TX_Char(uint8_t Data)
{
	while(!USART_GetFlagStatus(USART3, USART_FLAG_TXE));
	USART_SendData(USART3, Data);
}

void SCIC_TX_String(int8_t *str)
{
	int count = 0;

	while (str[count] != 0x00)
	{
		while(!USART_GetFlagStatus(USART3, USART_FLAG_TXE));
		USART_SendData(USART3, str[count]);
		count++;
	}
}

void SCIC_TX_String_Num(int8_t *str, unsigned int num)
{
	int count = 0;

	while (num--)
	{
		while(!USART_GetFlagStatus(USART3, USART_FLAG_TXE));
		USART_SendData(USART3, str[count]);
		count++;
	}
}

int SCIC_TX_String_IT(int8_t *str)
{
	volatile int8_t *tmp = str;
	int i = 0;

	if (!tmp) return -1;

	while ( *tmp != 0 )
	{
		Com2_TXBuff[i] = tmp[i];
		if ( i > Com2TxbuffSize ) return -1;
		else i++;
	}

	HAL_UART_Transmit_IT(&huart3, Com2_TXBuff, i);

	return i;
}

int SCIC_TX_String_Num_IT(int8_t *str, int len)
{
	volatile int8_t *tmp = str;
	int i = 0;

	for ( i = 0; i < len; i++ )
	{
		Com2_TXBuff[i] = tmp[i];
		if ( i > Com2TxbuffSize ) return -1;
	}

	HAL_UART_Transmit_IT(&huart3, Com2_TXBuff, i);

	return i;
}

uint8_t SCIC_Read_Check ( void )
{
	if ( Com2_Rwi_posi != Com2_Rrd_posi ) return 1;
	else return 0;
}

uint8_t SCIC_Read_Char ( void )
{
	uint8_t rd = 0;

	if ( Com2_Rwi_posi != Com2_Rrd_posi ) rd = Com2_RecBuff[Com2_Rrd_posi++];
	if ( Com2_Rrd_posi >= Com2buffsize ) Com2_Rrd_posi = 0;

	return rd;
}

/*** SCIC USER FUNCTION END ***********************************************************/

/*** SCID USER FUNCTION BEGIN *********************************************************/

void SCID_TX_Char(uint8_t Data)
{
	while(!USART_GetFlagStatus(UART8, USART_FLAG_TXE));
	USART_SendData(UART8, Data);
}

void SCID_TX_String(int8_t *str)
{
	int count = 0;

	while (str[count] != 0x00)
	{
		while(!USART_GetFlagStatus(UART8, USART_FLAG_TXE));
		USART_SendData(UART8, str[count]);
		count++;
	}
}

int SCID_TX_String_IT(int8_t *str)
{
	volatile int8_t *tmp = str;
	int i = 0;

	if (!tmp) return -1;

	while ( *tmp != 0 )
	{
		Com3_TXBuff[i] = tmp[i];
		if ( i > Com3TxbuffSize ) return -1;
		else i++;
	}

	HAL_UART_Transmit_IT(&huart8, Com3_TXBuff, i);

	return i;
}

int SCID_TX_String_Num_IT(int8_t *str, int len)
{
	volatile int8_t *tmp = str;
	int i = 0;

	for ( i = 0; i < len; i++ )
	{
		Com3_TXBuff[i] = tmp[i];
		if ( i > Com3TxbuffSize ) return -1;
	}

	HAL_UART_Transmit_IT(&huart8, Com3_TXBuff, i);

	return i;
}

uint8_t SCID_Read_Check ( void )
{
	if ( Com3_Rwi_posi != Com3_Rrd_posi ) return 1;
	else return 0;
}

uint8_t SCID_Read_Char ( void )
{
	uint8_t rd = 0;

	if ( Com3_Rwi_posi != Com3_Rrd_posi ) rd = Com3_RecBuff[Com3_Rrd_posi++];
	if ( Com3_Rrd_posi >= Com3buffsize ) Com3_Rrd_posi = 0;

	return rd;
}

/*** SCID USER FUNCTION END ***********************************************************/

/*** SCIE USER FUNCTION BEGIN *********************************************************/

void SCIE_TX_Char(uint8_t Data)
{
	while(!USART_GetFlagStatus(USART6, USART_FLAG_TXE));
	USART_SendData(USART6, Data);
}

void SCIE_TX_String(int8_t *str)
{
	int count = 0;

	while (str[count] != 0x00)
	{
		while(!USART_GetFlagStatus(USART6, USART_FLAG_TXE));
		USART_SendData(USART6, str[count]);
		count++;
	}
}

int SCIE_TX_String_IT(int8_t *str)
{
	volatile int8_t *tmp = str;
	int i = 0;

	if (!tmp) return -1;

	while ( *tmp != 0 )
	{
		Com4_TXBuff[i] = tmp[i];
		if ( i > Com4TxbuffSize ) return -1;
		else i++;
	}

	HAL_UART_Transmit_IT(&huart6, Com4_TXBuff, i);

	return i;
}

int SCIE_TX_String_Num_IT(int8_t *str, int len)
{
	volatile int8_t *tmp = str;
	int i = 0;

	for ( i = 0; i < len; i++ )
	{
		Com4_TXBuff[i] = tmp[i];
		if ( i > Com4TxbuffSize ) return -1;
	}

	HAL_UART_Transmit_IT(&huart6, Com4_TXBuff, i);

	return i;
}

uint8_t SCIE_Read_Check ( void )
{
	if ( Com4_Rwi_posi != Com4_Rrd_posi ) return 1;
	else return 0;
}

uint8_t SCIE_Read_Char ( void )
{
	uint8_t rd = 0;

	if ( Com4_Rwi_posi != Com4_Rrd_posi ) rd = Com4_RecBuff[Com4_Rrd_posi++];
	if ( Com4_Rrd_posi >= Com4buffsize ) Com4_Rrd_posi = 0;

	return rd;
}

/*** SCIE USER FUNCTION END ***********************************************************/

/*** SCIF USER FUNCTION BEGIN *********************************************************/

void SCIF_TX_Char(uint8_t Data)
{
	while(!USART_GetFlagStatus(UART7, USART_FLAG_TXE));
	USART_SendData(UART7, Data);
}

void SCIF_TX_String(int8_t *str)
{
	int count = 0;

	while (str[count] != 0x00)
	{
		while(!USART_GetFlagStatus(UART7, USART_FLAG_TXE));
		USART_SendData(UART7, str[count]);
		count++;
	}
}

void SCIF_TX_String_Num(int8_t *str, unsigned int num)
{
	int count = 0;

	while (num--)
	{
		while(!USART_GetFlagStatus(UART7, USART_FLAG_TXE));
		USART_SendData(UART7, str[count]);
		count++;
	}
}

int SCIF_TX_String_IT(int8_t *str)
{
	volatile int8_t *tmp = str;
	int i = 0;

	if (!tmp) return -1;

	while ( *tmp != 0 )
	{
		Com5_TXBuff[i] = tmp[i];
		if ( i > Com5TxbuffSize ) return -1;
		else i++;
	}

	HAL_UART_Transmit_IT(&huart7, Com5_TXBuff, i);

	return i;
}

int SCIF_TX_String_Num_IT(int8_t *str, int len)
{
	volatile int8_t *tmp = str;
	int i = 0;

	for ( i = 0; i < len; i++ )
	{
		Com5_TXBuff[i] = tmp[i];
		if ( i > Com5TxbuffSize ) return -1;
	}

	HAL_UART_Transmit_IT(&huart7, Com5_TXBuff, i);

	return i;
}

uint8_t SCIF_Read_Check ( void )
{
	if ( Com5_Rwi_posi != Com5_Rrd_posi ) return 1;
	else return 0;
}

uint8_t SCIF_Read_Char ( void )
{
	uint8_t rd = 0;

	if ( Com5_Rwi_posi != Com5_Rrd_posi ) rd = Com5_RecBuff[Com5_Rrd_posi++];
	if ( Com5_Rrd_posi >= Com5buffsize ) Com5_Rrd_posi = 0;

	return rd;
}

/*** SCIF USER FUNCTION END ***********************************************************/

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance == USART1)
	{
		Com0_RecBuff[Com0_Rwi_posi++] = Com0_Receive;
		if( Com0_Rwi_posi >= Com0buffsize ) Com0_Rwi_posi = 0;

		HAL_UART_Receive_IT(&huart1, &Com0_Receive, 1);
	}

	else if(huart->Instance == USART2)
	{
		Com1_RecBuff[Com1_Rwi_posi++] = Com1_Receive;
		if( Com1_Rwi_posi >= Com1buffsize ) Com1_Rwi_posi = 0;

		HAL_UART_Receive_IT(&huart2, &Com1_Receive, 1);
	}

	else if(huart->Instance == USART3)
	{
		Com2_RecBuff[Com2_Rwi_posi++] = Com2_Receive;
		if( Com2_Rwi_posi >= Com2buffsize ) Com2_Rwi_posi = 0;

		HAL_UART_Receive_IT(&huart3, &Com2_Receive, 1);
	}

	else if(huart->Instance == UART8)
	{
		Com3_RecBuff[Com3_Rwi_posi++] = Com3_Receive;
		if( Com3_Rwi_posi >= Com3buffsize ) Com3_Rwi_posi = 0;

		HAL_UART_Receive_IT(&huart8, &Com3_Receive, 1);
	}

	else if(huart->Instance == USART6)
	{
		Com4_RecBuff[Com4_Rwi_posi++] = Com4_Receive;
		if( Com4_Rwi_posi >= Com4buffsize ) Com4_Rwi_posi = 0;

		HAL_UART_Receive_IT(&huart6, &Com4_Receive, 1);
	}

	else if(huart->Instance == UART7)
	{

		Com5_RecBuff[Com5_Rwi_posi++] = Com5_Receive;
		if( Com5_Rwi_posi >= Com5buffsize ) Com5_Rwi_posi = 0;

		HAL_UART_Receive_IT(&huart7, &Com5_Receive, 1);
	}
}

void Digi5_Unsigned_IntToAscii(unsigned long Data, int8_t *Buff)
{
	unsigned int Data_Buff, Count = 0;

	Data_Buff = (Data % 100000);

	Buff[Count++] = (Data_Buff / 10000) + 0x30;
	Data_Buff = (Data_Buff % 10000);

	Buff[Count++] = (Data_Buff / 1000) + 0x30;
	Data_Buff = (Data_Buff % 1000);

	Buff[Count++] = (Data_Buff / 100) + 0x30;
	Data_Buff = (Data_Buff % 100);

	Buff[Count++] = (Data_Buff / 10) + 0x30;
	Buff[Count++] = (Data_Buff % 10) + 0x30;
}


void Digi4_Unsigned_IntToAscii(unsigned int Data, int8_t *Buff)
{
	unsigned int Data_Buff, Count = 0;

	Data_Buff = (Data % 10000);

	Buff[Count++] = (Data_Buff / 1000) + 0x30;
	Data_Buff = (Data_Buff % 1000);

	Buff[Count++] = (Data_Buff / 100) + 0x30;
	Data_Buff = (Data_Buff % 100);

	Buff[Count++] = (Data_Buff / 10) + 0x30;
	Buff[Count++] = (Data_Buff % 10) + 0x30;
}

void Digi3_Unsigned_IntToAscii(unsigned int Data, int8_t *Buff)
{
	unsigned int Data_Buff, Count = 0;

	Data_Buff = (Data % 1000);

	Buff[Count++] = (Data_Buff / 100) + 0x30;
	Data_Buff = (Data_Buff % 100);

	Buff[Count++] = (Data_Buff / 10) + 0x30;
	Buff[Count++] = (Data_Buff % 10) + 0x30;
}

uint8_t Int_To_HexAscii(uint8_t *str, uint8_t Count, unsigned int Data)
{
	unsigned int Cal_Buff = 0;
	uint8_t Count_Buff = 0;

	Count_Buff = Count;

	Cal_Buff = (Data >> 12) & 0x0F;
	if(Cal_Buff < 10)str[Count_Buff++] = Cal_Buff + '0';
	if(Cal_Buff >= 10)str[Count_Buff++] = (Cal_Buff - 10) + 'A';

	Cal_Buff = (Data >> 8) & 0x0F;
	if(Cal_Buff < 10)str[Count_Buff++] = Cal_Buff + '0';
	if(Cal_Buff >= 10)str[Count_Buff++] = (Cal_Buff - 10) + 'A';

	Cal_Buff = (Data >> 4) & 0x0F;
	if(Cal_Buff < 10)str[Count_Buff++] = Cal_Buff + '0';
	if(Cal_Buff >= 10)str[Count_Buff++] = (Cal_Buff - 10) + 'A';

	Cal_Buff = Data & 0x0F;
	if(Cal_Buff < 10)str[Count_Buff++] = Cal_Buff + '0';
	if(Cal_Buff >= 10)str[Count_Buff++] = (Cal_Buff - 10) + 'A';

	return(Count_Buff);
}


unsigned int AsciiHex_To_Binary_Num(int8_t *pStr, unsigned int Count, int8_t len)
{
	int8_t *tmp = (pStr + Count), i = 0;
	unsigned int dat = 0;

	for ( i = 0; i < len; i++ )
	{
		dat <<= 4;
		if ( tmp[i] >= 'A' && tmp[i] <= 'F') dat |= tmp[i] + 10 - 'A';
		else if( tmp[i] >= '0' && tmp[i] <= '9') dat |= tmp[i] - '0';
		else return -1;
	}

	return dat;
}

int8_t Int_To_HexAscii_Num(int8_t* pStr, int8_t Count, unsigned int data, int8_t len )
{
	int8_t *tmp = (pStr + Count), i = len;
	unsigned int n = data, buf = 0;

	while( i )
	{
			buf = n & 0x0F;
			if ( buf < 10 ) tmp[--i] = buf + '0';
			else tmp[--i] = ( buf - 10 )+ 'A';
			n = n >> 4;
	}

	return ( Count + len );
}

int8_t Int_To_Ascii_Num(int8_t* pStr, int8_t Count, unsigned int data, int8_t len )
{
	int8_t *tmp = (pStr + Count), i = len;
	unsigned int n = data;

	while( i )
	{
			tmp[--i] = (n % 10) + '0';
			n /= 10;
	 }

	 return ( Count + len );
}

int8_t Long_To_HexAscii_Num(int8_t* pStr, int8_t Count, unsigned long data, int8_t len )
{
	 int8_t *tmp = (pStr + Count), i = len;
	 unsigned long n = data, buf = 0;

	 while( i )
	 {
			buf = n & 0x0F;
			if ( buf < 10 ) tmp[--i] = buf + '0';
			else tmp[--i] = ( buf - 10 )+ 'A';
			n = n >> 4;
	 }

	 return ( Count + len );
}

int8_t Long_To_Ascii_Num(int8_t* pStr, int8_t Count, unsigned long data, int8_t len )
{
	 int8_t *tmp = (pStr + Count), i = len;
	 unsigned long n = data;

	 while( i )
	 {
			tmp[--i] = (n % 10) + '0';
			n /= 10;
	 }

	 return ( Count + len );  ;
}

int8_t*  ItoA(int nNum, int8_t* pString, int radix)
{
	 int nSign=1, nIdx=1;//  nSize=0,       // 부호 변수, 정수크기(자리수), 인덱스
	 int tNum = nNum;                    // 임시 변수에 인자로 받은 정수 복사

	 if( nNum < 0 )                      // 바꿀 정수가 0 보다 작으면 음수
			nSign = -1;                      // 부호 변수 -1 대입

	 while(tNum/10 != 0)                 // 나눈 몫이 0이 아닐동안 반복
	 {
			tNum = tNum / 10;                // 나눈 몫을 임시 변수 자신에 저장
			nIdx++;                          // 카운터 증가 (자리수 확인)
	 }

	 if(nSign == -1)                     // 음수이면
			nIdx+=1;                         // 자리수 1 증가 (부호 문자 삽입 위해)

	 *(pString+(nIdx--)) = '\0';         // 종료문자를 마지막 자리에 삽입

	 while(nIdx >= (nSign==-1)?1:0)      // 음수이면 인덱스가 1일때까지 양수이면 0일때까지 반복
	 {
			*(pString+(nIdx--)) = (int8_t)((nNum%10)*nSign) + 0x30;  // 계산한 나머지를 각 인덱스(뒤에서부터)에 저장
			nNum = nNum / 10;                // 나눈 몫을 자기자신에 저장
	 }

	 if(nSign == -1)  // 음수이면
			*(pString+nIdx) = '-';  // 부호 문자 '-' 삽입 (가장 첫인덱스(0))

	 return pString;  // 저장한 문자열 반환
}

void Printf(int8_t *fmt,...)
{
	va_list ap;
	int8_t string[256];

	va_start(ap,fmt);
	vsprintf((char *)string,(char *)fmt,ap);
	SCIF_TX_String(string);
	va_end(ap);
}
