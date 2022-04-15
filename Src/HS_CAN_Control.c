#include "can.h"
#include "main.h"
#include "HS_CAN_Control.h"

uint16_t R2100_Foward_Buff[11] = {4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000};
uint16_t R2100_Backward_Buff[11]  = {4000,4000,4000,4000,4000,4000,4000,4000,4000,4000,4000};
int8_t Actuator_Response = 0;
uint8_t Guide_Num1C = 0, Guide_Num1L = 0, Guide_Num1R = 0;
uint8_t Guide_Num2C = 0, Guide_Num2L = 0, Guide_Num2R = 0;
uint8_t Foward_Guide_REC_Flag, Backward_Guide_REC_Flag;

uint16_t BMS_ID_Buff[] = { 	BMS_SUMMARY_ID,
							BMS_STATUS_ID,
							BMS_VOLTAGE_ID,
							BMS_TEMP_ID,
							BMS_CELL_1_4_Volt_ID,
							BMS_CELL_5_8_Volt_ID,
							BMS_CELL_9_12_Volt_ID,
							BMS_CELL_13_Volt_Temp_ID };

PORT_IO_FRAME En_InPortA;
PORT_IO_FRAME En_InPortB;
PORT_IO_FRAME En_InPortC;
PORT_IO_FRAME En_InPortD;

int Left_Motor_Torque = 0, Right_Motor_Torque = 0;

uint8_t PORT_Can1_RECV, PORT_Can2_RECV;
uint8_t ADC_Input_RECV = 0, R2100_ID2_RECV = 0;

BMS_Summary_Inform BMS_Summary_Data;
BMS_Status_Inform BMS_Status_Data;
BMS_Voltage_Inform BMS_Volt_Data;
BMS_Temp_Inform BMS_Temp1_Data;
BMS_CELL_Voltage_Struct BMS_Cell_Volt_Data;
BMS_Temp_Struct BMS_Temp2_Data;

void CAN_Mask_Init(CAN_HandleTypeDef *hcan, uint32_t MASK_ID, uint32_t FILTER_ID)
{
	if(hcan->Instance == CAN1){
		Can1Filter.FilterMaskIdHigh = ( MASK_ID & 0xFFFF ) << 5;
		Can1Filter.FilterIdHigh = ( FILTER_ID & 0xFFFF ) << 5;
		Can1Filter.FilterMaskIdLow = ( MASK_ID & 0xFFFF ) << 5;
		Can1Filter.FilterIdLow = ( FILTER_ID & 0xFFFF ) << 5;
		Can1Filter.FilterMode = CAN_FILTERMODE_IDMASK;
		Can1Filter.FilterScale = CAN_FILTERSCALE_16BIT;
		Can1Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
		Can1Filter.FilterBank = 0;

		Can1Filter.FilterActivation = ENABLE;

		HAL_CAN_ConfigFilter(&hcan1, &Can1Filter );
		HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

		PORT_Can1_RECV = 0;
	}
	else if(hcan->Instance == CAN2){
		Can2Filter.FilterMaskIdHigh = ( MASK_ID & 0xFFFF ) << 5;
		Can2Filter.FilterIdHigh = ( FILTER_ID & 0xFFFF ) << 5;
		Can2Filter.FilterMaskIdLow = ( MASK_ID & 0xFFFF ) << 5;
		Can2Filter.FilterIdLow = ( FILTER_ID & 0xFFFF ) << 5;
		Can2Filter.FilterMode = CAN_FILTERMODE_IDMASK;
		Can2Filter.FilterScale = CAN_FILTERSCALE_16BIT;
		Can2Filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
		Can2Filter.FilterBank = 0;

		Can2Filter.FilterActivation = ENABLE;

		HAL_CAN_ConfigFilter(&hcan2, &Can2Filter );
		HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);

		PORT_Can2_RECV = 0;
	}


}

void CAN_Send_Data(CAN_HandleTypeDef *hcan, uint32_t MSG_ID, uint8_t Type, uint8_t Data_Length, uint8_t *Data)
{
	if(hcan->Instance==CAN1){
		can1TxHeader.StdId = MSG_ID;
		can1TxHeader.RTR = CAN_RTR_DATA;
		can1TxHeader.IDE = Type;
		if(Data_Length > 8)Data_Length = 8;
		can1TxHeader.DLC = Data_Length;

	  	txMailBox1 = HAL_CAN_GetTxMailboxesFreeLevel(&hcan1);	//비워져 있는 메일박스 찾아서 저장
		HAL_CAN_AddTxMessage(&hcan1, &can1TxHeader, (uint8_t *)Data, (uint32_t *)txMailBox1);
	}
	else if (hcan->Instance==CAN2){
  		can2TxHeader.StdId = MSG_ID;
		can2TxHeader.RTR = CAN_RTR_DATA;
		can2TxHeader.IDE = Type;
		if(Data_Length > 8)Data_Length = 8;
		can2TxHeader.DLC = Data_Length;

		txMailBox2 = HAL_CAN_GetTxMailboxesFreeLevel(&hcan2);	//비워져 있는 메일박스 찾아서 저장
		HAL_CAN_AddTxMessage(&hcan2, &can2TxHeader, (uint8_t *)Data, (uint32_t *)txMailBox2);
	}
}

uint32_t REC_CAN1_ID = 0, REC_CAN2_ID = 0;
uint8_t CAN_TEST_Buff1[8], CAN_TEST_Buff2[8];
uint8_t Status_Arr[8];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if(hcan->Instance == CAN1){

		HAL_CAN_GetRxMessage(&hcan1, CAN_RX_FIFO0, &can1RxHeader, (uint8_t *)can1Rx0Data);
		// PORT_Can1_RECV = 1;

		REC_CAN1_ID = can1RxHeader.StdId;

		switch( REC_CAN1_ID )
		{
			case Can_Input_GPIO_ID :
				Ex_Board1_In_Port.byte[0] = can1Rx0Data[1]; // IN 0 ~ 7
				Ex_Board1_In_Port.byte[1] = can1Rx0Data[0]; // IN 8 ~ 15
				Ex_Board1_In_Port.byte[2] = can1Rx0Data[2]; // IN 16 ~ 23
				Ex_Board1_In_Port.byte[3] = can1Rx0Data[3]; // IN 24 ~ 31
				PORT_Can1_RECV = 1;
				break;

			case Can_Output_GPIO_2_ID :
				Ex_Board2_In_Port.byte[0] = can1Rx0Data[1]; // IN 0 ~ 7
				Ex_Board2_In_Port.byte[1] = can1Rx0Data[0]; // IN 8 ~ 15
				Ex_Board2_In_Port.byte[2] = can1Rx0Data[2]; // IN 16 ~ 23
				Ex_Board2_In_Port.byte[3] = can1Rx0Data[3]; // IN 24 ~ 31
				PORT_Can1_RECV = 2;
				break;

			case BMS_SUMMARY_ID :
				BMS_Summary_Data.BAT_Volt.byte.H = can1Rx0Data[0];
				BMS_Summary_Data.BAT_Volt.byte.L = can1Rx0Data[1];
				BMS_Summary_Data.BAT_Current.byte.H = can1Rx0Data[2];
				BMS_Summary_Data.BAT_Current.byte.L = can1Rx0Data[3];
				BMS_Summary_Data.BAT_SOC.byte.H = can1Rx0Data[4];
				BMS_Summary_Data.BAT_SOC.byte.L = can1Rx0Data[5];
				BMS_Summary_Data.BAT_Use_Cycle.byte.H = can1Rx0Data[6];
				BMS_Summary_Data.BAT_Use_Cycle.byte.L = can1Rx0Data[7];
				PORT_Can2_RECV = 3;
				break;

			case BMS_STATUS_ID :
				BMS_Status_Data.Status1.byte.L = Status_Arr[0] = can1Rx0Data[0];
				BMS_Status_Data.Status1.byte.H = Status_Arr[1] = can1Rx0Data[1];
				BMS_Status_Data.Status2.byte.L = Status_Arr[2] = can1Rx0Data[2];
				BMS_Status_Data.Status2.byte.H = Status_Arr[3] = can1Rx0Data[3];
				Status_Arr[4] = can1Rx0Data[4];
				Status_Arr[5] = can1Rx0Data[5];
				Status_Arr[6] = can1Rx0Data[6];
				Status_Arr[7] = can1Rx0Data[7];
				PORT_Can2_RECV = 4;
				break;

			case BMS_VOLTAGE_ID :
				BMS_Volt_Data.CELL_MAX_VOLT.byte.H = can1Rx0Data[0];
				BMS_Volt_Data.CELL_MAX_VOLT.byte.L = can1Rx0Data[1];
				BMS_Volt_Data.CELL_MIN_VOLT.byte.H = can1Rx0Data[2];
				BMS_Volt_Data.CELL_MIN_VOLT.byte.L = can1Rx0Data[3];
				BMS_Volt_Data.AVG_TEMP.byte.H = can1Rx0Data[4];
				BMS_Volt_Data.AVG_TEMP.byte.L = can1Rx0Data[5];
				BMS_Volt_Data.MAX_VOLT_CELL_POS = can1Rx0Data[6];
				BMS_Volt_Data.MIN_VOLT_CELL_POS = can1Rx0Data[7];
				PORT_Can2_RECV = 5;
				break;

			case BMS_TEMP_ID :
				BMS_Temp1_Data.CELL_MAX_TEMP = can1Rx0Data[0];
				BMS_Temp1_Data.CELL_MIN_TEMP = can1Rx0Data[1];
				BMS_Temp1_Data.CELL_AVG_TEMP = can1Rx0Data[2];
				BMS_Temp1_Data.MAX_TEMP_POS = can1Rx0Data[3];
				BMS_Temp1_Data.MIN_TEMP_POS = can1Rx0Data[4];
				PORT_Can2_RECV = 6;
				break;

			case BMS_CELL_1_4_Volt_ID :
				BMS_Cell_Volt_Data.Cell1.byte.H = can1Rx0Data[0];
				BMS_Cell_Volt_Data.Cell1.byte.L = can1Rx0Data[1];
				BMS_Cell_Volt_Data.Cell2.byte.H = can1Rx0Data[2];
				BMS_Cell_Volt_Data.Cell2.byte.L = can1Rx0Data[3];
				BMS_Cell_Volt_Data.Cell3.byte.H = can1Rx0Data[4];
				BMS_Cell_Volt_Data.Cell3.byte.L = can1Rx0Data[5];
				BMS_Cell_Volt_Data.Cell4.byte.H = can1Rx0Data[6];
				BMS_Cell_Volt_Data.Cell4.byte.L = can1Rx0Data[7];
				PORT_Can2_RECV = 7;
				break;

			case BMS_CELL_5_8_Volt_ID :
				BMS_Cell_Volt_Data.Cell5.byte.H = can1Rx0Data[0];
				BMS_Cell_Volt_Data.Cell5.byte.L = can1Rx0Data[1];
				BMS_Cell_Volt_Data.Cell6.byte.H = can1Rx0Data[2];
				BMS_Cell_Volt_Data.Cell6.byte.L = can1Rx0Data[3];
				BMS_Cell_Volt_Data.Cell7.byte.H = can1Rx0Data[4];
				BMS_Cell_Volt_Data.Cell7.byte.L = can1Rx0Data[5];
				BMS_Cell_Volt_Data.Cell8.byte.H = can1Rx0Data[6];
				BMS_Cell_Volt_Data.Cell8.byte.L = can1Rx0Data[7];
				PORT_Can2_RECV = 8;
				break;

			case BMS_CELL_9_12_Volt_ID :
				BMS_Cell_Volt_Data.Cell9.byte.H = can1Rx0Data[0];
				BMS_Cell_Volt_Data.Cell9.byte.L = can1Rx0Data[1];
				BMS_Cell_Volt_Data.Cell10.byte.H = can1Rx0Data[2];
				BMS_Cell_Volt_Data.Cell10.byte.L = can1Rx0Data[3];
				BMS_Cell_Volt_Data.Cell11.byte.H = can1Rx0Data[4];
				BMS_Cell_Volt_Data.Cell11.byte.L = can1Rx0Data[5];
				BMS_Cell_Volt_Data.Cell12.byte.H = can1Rx0Data[6];
				BMS_Cell_Volt_Data.Cell12.byte.L = can1Rx0Data[7];
				PORT_Can2_RECV = 8;
				break;

			case BMS_CELL_13_Volt_Temp_ID :
				BMS_Cell_Volt_Data.Cell13.byte.H = can1Rx0Data[0];
				BMS_Cell_Volt_Data.Cell13.byte.L = can1Rx0Data[1];
				BMS_Temp2_Data.Temp[0] = can1Rx0Data[2];
				BMS_Temp2_Data.Temp[1] = can1Rx0Data[3];
				BMS_Temp2_Data.Temp[2] = can1Rx0Data[4];
				BMS_Temp2_Data.Temp[3] = can1Rx0Data[5];
				break;

			case Guide1_Real_ID :
				break;

			case Guide2_Real_ID :
				break;

			case ADC_Input_ID :
				break;

            case R2100_1REC_ID :
				break;

			}
		}

	else if(hcan->Instance == CAN2){

		HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &can2RxHeader, (uint8_t *)can2Rx0Data);

		REC_CAN2_ID = can2RxHeader.StdId;

		switch( REC_CAN2_ID ){
			// case Can_Input_GPIO_ID :
			// 	Ex_Board1_In_Port.byte[0] = can2Rx0Data[1]; // IN 0 ~ 7
			// 	Ex_Board1_In_Port.byte[1] = can2Rx0Data[0]; // IN 8 ~ 15
			// 	Ex_Board1_In_Port.byte[2] = can2Rx0Data[2]; // IN 16 ~ 23
			// 	Ex_Board1_In_Port.byte[3] = can2Rx0Data[3]; // IN 24 ~ 31
			// 	PORT_Can2_RECV = 1;
			// 	break;

			// case Can_Output_GPIO_2_ID :
			// 	Ex_Board2_In_Port.byte[0] = can2Rx0Data[1]; // IN 0 ~ 7
			// 	Ex_Board2_In_Port.byte[1] = can2Rx0Data[0]; // IN 8 ~ 15
			// 	Ex_Board2_In_Port.byte[2] = can2Rx0Data[2]; // IN 16 ~ 23
			// 	Ex_Board2_In_Port.byte[3] = can2Rx0Data[3]; // IN 24 ~ 31
			// 	PORT_Can2_RECV = 2;
			// 	break;

			// case BMS_SUMMARY_ID :
			// 	Summary_Buff.BAT_Volt.byte.H = can2Rx0Data[0];
			// 	Summary_Buff.BAT_Volt.byte.L = can2Rx0Data[1];
			// 	Summary_Buff.BAT_Current.byte.H = can2Rx0Data[2];
			// 	Summary_Buff.BAT_Current.byte.L = can2Rx0Data[3];
			// 	Summary_Buff.BAT_SOC.byte.H = can2Rx0Data[4];
			// 	Summary_Buff.BAT_SOC.byte.L = can2Rx0Data[5];
			// 	Summary_Buff.BAT_Use_Cycle.byte.H = can2Rx0Data[6];
			// 	Summary_Buff.BAT_Use_Cycle.byte.L = can2Rx0Data[7];

			// 	BMS_Summary_Data = Summary_Buff;
			// 	PORT_Can2_RECV = 3;
			// 	break;

			// case BMS_STATUS_ID :
			// 	Status_Buff.Status1.byte.L = can2Rx0Data[0];
			// 	Status_Buff.Status1.byte.H = can2Rx0Data[1];
			// 	Status_Buff.Status2.byte.L = can2Rx0Data[2];
			// 	Status_Buff.Status2.byte.H = can2Rx0Data[3];

			// 	BMS_Status_Data = Status_Buff;
			// 	PORT_Can2_RECV = 4;
			// 	break;

			// case BMS_VOLTAGE_ID :
			// 	Voltage_Buff.CELL_MAX_VOLT.byte.H = can2Rx0Data[0];
			// 	Voltage_Buff.CELL_MAX_VOLT.byte.L = can2Rx0Data[1];
			// 	Voltage_Buff.CELL_MIN_VOLT.byte.H = can2Rx0Data[2];
			// 	Voltage_Buff.CELL_MIN_VOLT.byte.L = can2Rx0Data[3];
			// 	Voltage_Buff.AVG_TEMP.byte.H = can2Rx0Data[4];
			// 	Voltage_Buff.AVG_TEMP.byte.L = can2Rx0Data[5];
			// 	Voltage_Buff.MAX_VOLT_CELL_POS = can2Rx0Data[6];
			// 	Voltage_Buff.MIN_VOLT_CELL_POS = can2Rx0Data[7];

			// 	BMS_Volt_Data = Voltage_Buff;
			// 	PORT_Can2_RECV = 5;
			// 	break;

			// case BMS_TEMP_ID :
			// 	Temp_Buff.CELL_MAX_TEMP = can2Rx0Data[0];
			// 	Temp_Buff.CELL_MIN_TEMP = can2Rx0Data[1];
			// 	Temp_Buff.CELL_AVG_TEMP = can2Rx0Data[2];
			// 	Temp_Buff.MAX_TEMP_POS = can2Rx0Data[3];
			// 	Temp_Buff.MIN_TEMP_POS = can2Rx0Data[4];

			// 	BMS_Temp1_Data = Temp_Buff;
			// 	PORT_Can2_RECV = 6;
			// 	break;
			}
		}
}

// BMS_Summary_Inform BMS_Summary_Data;
// BMS_Status_Inform BMS_Status_Data;
// BMS_Voltage_Inform BMS_Volt_Data;
// BMS_Temp_Inform BMS_Temp_Data;