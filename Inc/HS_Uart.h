#ifndef __HS_UART_H
#define __HS_UART_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"


#define Com0buffsize           	400
#define Com1buffsize			128
#define Com2buffsize			2000
#define Com3buffsize			128
#define Com4buffsize			128
#define Com5buffsize 			128

#define Com0TxbuffSize 			100
#define Com1TxbuffSize 			100
#define Com2TxbuffSize 			100
#define Com3TxbuffSize 			100
#define Com4TxbuffSize 			100
#define Com5TxbuffSize 			100

#define Route_REC_Buff_Size     2000

#define USART_FLAG_TXE          0x80

#define     SOH             0x01
#define     STX             0x02
#define     ETX             0x03
#define     EOT             0x04
#define     ENQ 	    	0x05
#define     ACK 	    	0x06
#define     NAK 	    	0x15
#define     COM_LF			0x0A
#define     COM_CR			0x0D

extern int StrLen(int8_t *dest);

extern void SCIA_TX_Char(uint8_t Data);
extern void SCIA_TX_String(int8_t *str);
extern void SCIA_TX_String_Num(int8_t *str, unsigned int num);
extern int SCIA_TX_String_IT(int8_t *str);
extern int SCIA_TX_String_Num_IT(int8_t *str, int len);
extern uint8_t SCIA_Read_Check ( void );
extern uint8_t SCIA_Read_Char ( void );

extern void SCIB_TX_Char(uint8_t Data);
extern unsigned int Read_Rwi_Position(void);
extern void Reset_Rwi_Position(void);
extern void SCIB_TX_String(int8_t *str);
extern void SCIB_TX_String_Num(int8_t *str, unsigned int num);
extern int SCIB_TX_String_IT(int8_t *str);
extern int SCIB_TX_String_Num_IT(int8_t *str, int len);
extern uint8_t SCIB_Read_Check ( void );
extern uint8_t SCIB_Read_Char ( void );

extern void SCIC_RecBuff_Clear ( void );
extern void SCIC_TX_Char(uint8_t Data);
extern void SCIC_TX_String(int8_t *str);
extern void SCIC_TX_String_Num(int8_t *str, unsigned int num);
extern int SCIC_TX_String_IT(int8_t *str);
extern int SCIC_TX_String_Num_IT(int8_t *str, int len);
extern uint8_t SCIC_Read_Check ( void );
extern uint8_t SCIC_Read_Char ( void );

extern void SCID_TX_Char(uint8_t Data);
extern void SCID_TX_String(int8_t *str);
extern int SCID_TX_String_IT(int8_t *str);
extern int SCID_TX_String_Num_IT(int8_t *str, int len);
extern uint8_t SCID_Read_Check ( void );
extern uint8_t SCID_Read_Char ( void );

extern void SCIE_TX_Char(uint8_t Data);
extern void SCIE_TX_String(int8_t *str);
extern int SCIE_TX_String_IT(int8_t *str);
extern int SCIE_TX_String_Num_IT(int8_t *str, int len);
extern uint8_t SCIE_Read_Check ( void );
extern uint8_t SCIE_Read_Char ( void );

extern void SCIF_TX_Char(uint8_t Data);
extern void SCIF_TX_String(int8_t *str);
extern void SCIF_TX_String_Num(int8_t *str, unsigned int num);
extern int SCIF_TX_String_IT(int8_t *str);
extern int SCIF_TX_String_Num_IT(int8_t *str, int len);
extern uint8_t SCIF_Read_Check ( void );
extern uint8_t SCIF_Read_Char ( void );

extern void UART_REC_IT_Set(uint8_t Num);

extern unsigned int AsciiHex_To_Binary_Num(int8_t *pStr, unsigned int Count, int8_t len);

extern void Digi5_Unsigned_IntToAscii(unsigned long Data, int8_t *Buff);
extern void Digi4_Unsigned_IntToAscii(unsigned int Data, int8_t *Buff);
extern void Digi3_Unsigned_IntToAscii(unsigned int Data, int8_t *Buff);
extern uint8_t Int_To_HexAscii(uint8_t *str, uint8_t Count, unsigned int Data);
extern int8_t Int_To_HexAscii_Num(int8_t* pStr, int8_t Count, unsigned int data, int8_t len );
extern int8_t Int_To_Ascii_Num(int8_t* pStr, int8_t Count, unsigned int data, int8_t len );
extern int8_t Long_To_HexAscii_Num(int8_t* pStr, int8_t Count, unsigned long data, int8_t len );
extern int8_t Long_To_Ascii_Num(int8_t* pStr, int8_t Count, unsigned long data, int8_t len );
extern int8_t* ItoA(int nNum, int8_t* pString, int radix);
extern void Printf(int8_t *fmt,...);

#ifdef __cplusplus
}
#endif

#endif /* __HS_UART_H */