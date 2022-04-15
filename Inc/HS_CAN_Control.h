#ifndef __HS_CAN_CONTROL_H
#define __HS_CAN_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "can.h"

#define	STD_Mode			0x00
#define	ETX_Mode			0x04

#define Can_Output_GPIO_ID					0x00000001
#define Can_Input_GPIO_ID					0x00000305
#define R2100_1Send_ID						0x00000601
#define R2100_2Send_ID						0x00000602
#define R2100_Start_ID						0x00000000
#define Guide1_Real_ID						0x00000501
#define Guide2_Real_ID						0x00000502
#define ADC_Input_ID						0x00000401
#define Can_Output_GPIO_2_ID				0x00000306

#define R2100_1REC_ID						0x00000581
#define R2100_2REC_ID						0x00000582

#define BMS_REQUEST_ID 						0x00000370

#define BMS_SUMMARY_ID 						0x00000351
#define BMS_STATUS_ID						0x00000352
#define BMS_VOLTAGE_ID						0x00000353
#define BMS_TEMP_ID							0x00000354
#define BMS_CELL_1_4_Volt_ID 				0x00000100
#define BMS_CELL_5_8_Volt_ID 				0x00000101
#define BMS_CELL_9_12_Volt_ID 				0x00000110
#define BMS_CELL_13_Volt_Temp_ID 			0x00000111

extern uint16_t BMS_ID_Buff[];

extern uint16_t R2100_Foward_Buff[11];
extern uint16_t R2100_Backward_Buff[11];
extern int8_t Actuator_Response;
extern uint8_t Guide_Num1C, Guide_Num1L, Guide_Num1R;
extern uint8_t Guide_Num1C, Guide_Num2L, Guide_Num2R;
extern uint8_t Foward_Guide_REC_Flag, Backward_Guide_REC_Flag;

extern PORT_IO_FRAME En_InPortA;
extern PORT_IO_FRAME En_InPortB;
extern PORT_IO_FRAME En_InPortC;
extern PORT_IO_FRAME En_InPortD;

extern int Left_Motor_Torque;
extern int Right_Motor_Torque;

extern uint8_t PORT_Can1_RECV, PORT_Can2_RECV;
extern uint8_t ADC_Input_RECV, R2100_ID2_RECV;
extern uint8_t CAN_TEST_Buff1[8], CAN_TEST_Buff2[8];

typedef struct {
	int16_t L:8;
	int16_t H:8;
}Int16_Struct;

typedef union{
	int16_t all;
	Int16_Struct byte;
}Int16_Frame;

typedef struct{
	uint16_t L:8;
	uint16_t H:8;
}Uint16_Struct;

typedef union{
	uint16_t all;
	Uint16_Struct byte;
}Uint16_Frame;

typedef struct{
	Uint16_Frame BAT_Volt;
	Int16_Frame BAT_Current;
	Uint16_Frame BAT_SOC;
	Uint16_Frame BAT_Use_Cycle;
}BMS_Summary_Inform;

typedef struct{
	uint16_t OPV_Flag:2;
	uint16_t UPV_Flag:2;
	uint16_t OCV_Flag:2;
	uint16_t UCV_Flag:2;
	uint16_t OSOC_Flag:2;
	uint16_t USOC_Flag:2;
	uint16_t OT_Flag:2;
	uint16_t UT_Flag:2;
}BMS_Status1_Struct;

typedef union{
	uint16_t all;
	Int16_Struct byte;
	BMS_Status1_Struct bit;
}BMS_Status1_Frame;

typedef struct{
	uint16_t OC_Flag:2;
	uint16_t ODV_Flag:2;
	uint16_t ODT_Flag:2;
	uint16_t Charge_Comp:1;
	uint16_t Res1:1;
	uint16_t Charging:1;
	uint16_t Total_Warnning:1;
	uint16_t Total_CutOff:1;
	uint16_t Res2:5;
}BMS_Status2_Struct;

typedef union{
	uint16_t all;
	Int16_Struct byte;
	BMS_Status2_Struct bit;
}BMS_Status2_Frame;

typedef struct{
	BMS_Status1_Frame Status1;
	BMS_Status2_Frame Status2;
}BMS_Status_Inform;

typedef struct{
	Uint16_Frame CELL_MAX_VOLT;
	Uint16_Frame CELL_MIN_VOLT;
	Uint16_Frame AVG_TEMP;
	uint8_t MAX_VOLT_CELL_POS;
	uint8_t MIN_VOLT_CELL_POS;
}BMS_Voltage_Inform;

typedef struct{
	uint8_t CELL_MAX_TEMP;
	uint8_t CELL_MIN_TEMP;
	uint8_t CELL_AVG_TEMP;
	uint8_t MAX_TEMP_POS;
	uint8_t MIN_TEMP_POS;
}BMS_Temp_Inform;

typedef struct{
	Uint16_Frame Cell1;
	Uint16_Frame Cell2;
	Uint16_Frame Cell3;
	Uint16_Frame Cell4;
	Uint16_Frame Cell5;
	Uint16_Frame Cell6;
	Uint16_Frame Cell7;
	Uint16_Frame Cell8;
	Uint16_Frame Cell9;
	Uint16_Frame Cell10;
	Uint16_Frame Cell11;
	Uint16_Frame Cell12;
	Uint16_Frame Cell13;
}BMS_CELL_Voltage_Struct;

typedef union{
	uint8_t Temp[4];
}BMS_Temp_Struct;


extern BMS_Summary_Inform BMS_Summary_Data;
extern BMS_Status_Inform BMS_Status_Data;
extern BMS_Voltage_Inform BMS_Volt_Data;
extern BMS_Temp_Inform BMS_Temp1_Data;
extern BMS_CELL_Voltage_Struct BMS_Cell_Volt_Data;
extern BMS_Temp_Struct BMS_Temp2_Data;

extern void CAN_Mask_Init(CAN_HandleTypeDef *hcan, uint32_t MASK_ID, uint32_t FILTER_ID);
extern void CAN_Send_Data(CAN_HandleTypeDef *hcan, uint32_t MSG_ID, uint8_t Type, uint8_t Data_Length, uint8_t *Data);

extern uint32_t REC_ID;
#ifdef __cplusplus
}
#endif

#endif /* __HS_CAN_CONTROL_H */

