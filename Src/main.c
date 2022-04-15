/* USER CODE BEGIN Header */
/**
	******************************************************************************
	* @file           : main.c
	* @brief          : Main program body
	******************************************************************************
	* @attention
	*
	* <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
	* All rights reserved.</center></h2>
	*
	* This software component is licensed by ST under BSD 3-Clause license,
	* the "License"; You may not use this file except in compliance with the
	* License. You may obtain a copy of the License at:
	*                        opensource.org/licenses/BSD-3-Clause
	*
	******************************************************************************
	*/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "fatfs.h"
#include "sdio.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "fmc.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "HS_Uart.h"
#include "HS_CAN_Control.h"
#include "dwt_delay.h"
#include "AD5754_Control.h"
#include "AT25128.h"
// #include "HS_M2I_Melsec_Q.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


// add work test 

uint8_t add_work1 = 0;
uint8_t add_work2 = 3;

// finish work 
//add_work2 = add_work1 + 10;


// branch dev_heo
uint8_t FIRMWARE_VERSION[5]	= {"v6.30"};

#define	Wheel_Size				150
#define	Wheel_Pi				(Wheel_Size * 314) / 100

#define	Drive_Wheel_Distance	590
#define	Spin_Drive_Pi			(Drive_Wheel_Distance * 314) / 100

#define Encoder_Resolution  	471 	//0.2616 * 10000   753//0.2616 * 10000
#define Check_Frequency   		20		//20

#define	Spin_360_Encoder_Pulse	(Spin_Drive_Pi * 10000) / Encoder_Resolution
#define	Spin_180_Encoder_Pulse	Spin_360_Encoder_Pulse / 2
#define	Spin_90_Encoder_Pulse	Spin_360_Encoder_Pulse / 4

#define	Spin_Slow_Start_Rate	10

#define	Spin_180_Slow_Point		Spin_180_Encoder_Pulse - ((Spin_180_Encoder_Pulse / 100) * Spin_Slow_Start_Rate)
#define	Spin_90_Slow_Point		Spin_90_Encoder_Pulse - ((Spin_90_Encoder_Pulse / 100) * Spin_Slow_Start_Rate)


typedef struct{
	unsigned int PGV_Command;
	unsigned int PGV_Structs;
	unsigned int PGV_ID;
}PGV_Req_Struct;

typedef struct{
	unsigned long Node_ID;
	unsigned char AGV_Direction;
	unsigned char Driving_Speed;
	unsigned int Turn_Angle;
	unsigned char Safety_CMD;
}Auto_Dir_Information_Struct;

typedef union{
	unsigned char Buff[sizeof(Auto_Dir_Information_Struct)];
	Auto_Dir_Information_Struct Data;
}Auto_Dir_Information_Frame;

typedef struct{
	unsigned char Course_ID;
	unsigned char Dir_RFID;
	unsigned char Driving_Speed;
	unsigned char Action_Num;
	unsigned char Another_Cont;
	unsigned char Safety_Cont;
	unsigned char Turn_Speed;
	unsigned char Turn_Angle;
	unsigned char Course_Change;
}Manual_Dir_Set_Struct;

typedef union{
	unsigned char     Buff[sizeof(Manual_Dir_Set_Struct)];
	Manual_Dir_Set_Struct   Data;
}Manual_Dir_Set_Frame;

Manual_Dir_Set_Frame Dir_Setting_Frame, Action_Status_Buff;

Motion_Struct AGVMotionData;
PGV_Req_Struct PGV_Structs_Send_Fram;

unsigned char Old_DRV_Buff = 0;

Auto_Dir_Information_Frame Auto_Dir_Information_Buff[200];
Auto_Dir_Information_Frame Auto_Dir_Info_Rec_Buff[200];

Manual_Dir_Set_Frame Dir_Setting_Frame, Action_Status_Buff;

unsigned int Error_Flag = 0;
unsigned char Line_Err_Include_Flag = 0;

unsigned int SpinTurn_Code_Detect = 0, Spinturn_Err_Flag = 0;
unsigned int Line_Err_Detect = 0, Tag_Detect = 0;

AGVTX_Struct AGV_RunData;

unsigned char LoadingToBackward_ForTEST = 0;
unsigned char LoadingToForward_ForTEST = 0;

unsigned int Spin_Tim_Sensor_Buff = 0;

unsigned int Spin_Stop_Count = 0, Spin_Stop_Max_Count = 3, Spin_Stop_Max_Count_Buff = 3;
unsigned int Spin_Slow_Flag = 0;
unsigned int Straight_Counre_Reading_Count = 0; 

unsigned int LCD_Display_Count = 0;

unsigned int Present_Course_Number = 0x01;
unsigned int Course_Number = 0x01;

int Motor_DA_Buff[4];
uint16_t Mot_Alamr_Cnt[4] = {0};

unsigned int Test_Count = 0, Start_Slope_Count = 0, Bat_Status_Count = 0; // Error_Stop_Count = 0,
unsigned char R2100_CH_Count = 1;

int Slow_Speed_UpDown_R = 0;
int Slow_Speed_UpDown_L = 0;
unsigned int Slow_Speed_UpDown_R_Count = 0;
unsigned int Slow_Speed_UpDown_L_Count = 0;

unsigned char Line_Drive = 0;

unsigned char OPERATE_MODE = Auto_Operate;
/*** Log and Debug ************************************************************/
SEQ_NUM_DATA SEQ_NUM;
EVENT_FLAG_DATA EVENT_FLAG;
uint8_t Err_Event_Debug_Flag = 0;
/******************************************************************************/

/*** ADC VARIABLE DEFINE ******************************************************/
#define P25V_Data       3900
#define PLow_Data       3510

volatile uint16_t ADC_Val_Buff[5] = {0, 0, 0, 0, 0};
/*** PARAMETER VARIABLE DEFINE ************************************************/

#define Size_Of_Auto_Dir_Information_Buff   9

#define P_Gain        0   // 12.0
#define I_Gain        1   // 2.5
#define D_Gain        2   // 4.0

#define Max_error     32768 / (P_Gain + 1)
#define Max_Sum_Error   2147483647L / (I_Gain + 1)

#define ComBuff_Size    128

#define Distance_Angle_Back   4
#define Y_Axis_Cal_Data_Back    1

unsigned int AGV_Speed_Buff[4] = {110, 250, 400, 650};
unsigned int Manual_AGV_Speed = 200, Manual_Spin_Speed = 150;
unsigned int Distance_Cal_Buff = 1, Angle_Cal_Buff = 5, HSpeed_Angle_Cal_Buff = 5;

unsigned char Pause_Num = 0, Pause_Flag = 0, Solw_Stop_Flag = 0;
unsigned long AGV_VEL = 0;

unsigned int PID_Gain_Buff[3] = {4,2,1};
unsigned int AGV_Number = 0x01;

unsigned long ADC_Result;
unsigned int ADC_Count = 0;

// PID
#define SCALING_FACTOR  128

long Sumerror[4] = {0, 0, 0, 0};
int Last_Value[4] = {0, 0, 0, 0};
int P_Factor = P_Gain * SCALING_FACTOR, I_Factor = I_Gain * SCALING_FACTOR, D_Factor = D_Gain * SCALING_FACTOR;
int MaxError = 0;
long MaxSumError = 0;
int PID_Target_Speed = 0;

// ENCODER, SPEED, D/A
unsigned char R_Count_flag = 0, L_Count_flag = 0;

int RightMotor_Int_Count = 0, LeftMotor_Int_Count = 0;

unsigned int PID_Control_Count = 0;
int Left_Target_Speed = 0, Right_Target_Speed = 0;
unsigned long Target_Speed = 0, Old_Target_Speed = 0;
unsigned long Left_Old_Target_Speed = 0, Right_Old_Target_Speed = 0;

int Old_Left_target_Speed = 0, Old_Right_target_Speed = 0;
int Left_Speed_Slope = 0, Right_Speed_Slope = 0;

unsigned char Action_After_Speed = 0, Reset_Flag = 1;

char Start_Speed_UpFlag = 0, Speed_Up_Complete_Flag = 0;

// int Motor_DA_Buff[4];
int R_Sp_Old = 0, L_Sp_Old = 0;

unsigned char Lift_PIO_Control = 0, TIM_IN_Port_Control = 0;
unsigned int Lift_Action = Lift_Wait; //, Action_Count = 1000;
unsigned int Lift_Reset_Cnt = 0;
unsigned int Lift_Error_Cnt = 0;
unsigned int Lift_Key_Action_Flag = 0;
unsigned char Lift_Init_Flag = 0;

unsigned int Left_Mag_Count = 0, Right_Mag_Count = 0;
int Guide_New_Buff, Guide_Old_Buff = 0, Global_Guide = 0, LastGuide_Buff = 0;
int T_Guide_Old = 0, Angel_Guide = 0;
//unsigned int Spot_Line_State = QR_Tag, Spot_Line_Check = 0;
unsigned int Spot_Line_State = QR_Tag, Spot_Line_Check = 0;

unsigned char Slow_Speed_Change = 0, Pre_Spot_Line_State = 3, Line_Speed = 0, TEST_QR_Line_Change_Flag = 0;

unsigned int Spin_Slow_Start_Buff = 0, Spin_Slow_Start_90Count = 0, Spin_Slow_Start_180Count = 0;

unsigned long Distance_Sum_Buff = 0;

int Turn_Count_Buff = 0;
int Turn_Angle = 0;
int Turn_Speed = 0;
int RFID_Angle = 0;
int RFID_Speed = 0;
unsigned int Turn_Angle_Revision = 0;

int Error_Angle = 0;

unsigned char Manual_Spin_Flag = 0;

/*** USART VARIABLE DEFINE ****************************************************/
uint8_t Com0_Data[Com0buffsize];
unsigned int Com0Data_Count = 0;
uint8_t Com1_Data[Com1buffsize];
unsigned int  Com1Data_Count = 0;
uint8_t Com2_Data[Com2buffsize];
unsigned int Com2Data_Count = 0;
uint8_t Com3_Data[Com3buffsize];
unsigned int Com3Data_Count = 0;
uint8_t Com4_Data[Com4buffsize];
unsigned int Com4Data_Count = 0;
uint8_t Com5_Data[Com5buffsize];
unsigned int Com5Data_Count = 0;

int8_t ACS_Response_Buff[200];
int8_t AGV_Event_Req_Buff[50];

uint16_t ACS_Data_Send_Count = 0;
// job
unsigned long Old_JobID = 0;
//unsigned int JoB_Cancel_Even_Flag = 0;
unsigned int Job_Cancel_Stop_Flag = 0;
// unsigned int JoB_Cancel_Btn_Flag = 0;
/*** TOUCH HMI VARIABLE DEFINE ************************************************/
volatile uint16_t Coil_Buff[20];
// volatile unsigned int Register_Buff[130];
//volatile int16_t Register_Buff[150];
volatile int16_t Register_Buff[200];

unsigned char Modbus_Frame_Ack = Modbus_Frame_Wait, Modbus_Check_Count = 0;

#define Modbus_Slave_ID     11   //0x01 //11

/*** QR Code & RFID VARAIABLE DEFINE ******************************************/
unsigned long Destiantion_Imsi_Buff = 0;
unsigned long Destinations_Full_Num = 0;

// unsigned int Destinations_Root_Num[4] = {0,0,0,0};
// unsigned int Present_Root_Num[4] = {0,0,0,0};

unsigned long Old_ID, Last_Node = 0, Last_JobID = 0, Last_CarID1 = 0, Last_CarID2 = 0;
uint8_t Node_Chagne_Event_Flag = 0;
uint8_t Rec_JobID = 0;
// unsigned int AGV_GoolIN = 0;

// unsigned int Count_25mS = 0;
// unsigned char Count_100mS = 0;

unsigned int Node_Check_Count = 0;
unsigned int Node_Check_Count_Buff = 0;

// QR
unsigned int Left_Sample_Count = 0, Right_Sample_Count = 0;
unsigned int Sample_Acq_Flag = 0;

unsigned int Spin_Max_Degrees = 0, Spin_Stop_Scan_Flag = 0;

unsigned int Surface_Input = 1;
unsigned int GQ_Surface_Input = 1;

// unsigned int ACS_Car_Tagging_Flag = 0, ACS_Car_Tagging_Err_Count = 0, Tagging_Err_Check_Count = 0, RFID_Tagging_Err = 0;
unsigned char Pallet_Detect_Err_Flag = 0;
unsigned char Pallet_Detect_Enable = 0;
unsigned char ACS_Com_Check = 0;

int16_t X_Axis_Data = 0, Y_Axis_Data = 0;
long PGV_Tag_Num_Data = 0, Old_Tag_Num = 0;
int PGV_Angle_Data = 0, PGV_Guide_Angle = 0;
unsigned int PGV_Data_Set_Flag = 0, PGV_Data_Rec_Buff_Size = 3, PGV_Data_Send_Flag = 0;
unsigned int PGV_Start_Flag = 1;
unsigned int Line_Select_Buff = Dir_Left;
unsigned int Color_Select_Buff = Color_Blue;

int16_t PGV_Angle_Offset = 0, PGV_X_Axis_Offset = 0, PGV_Y_Axis_Offset = 0;
int16_t Spin_Stop_Degree = 25, Mark_Stop_Dimension = 35;
unsigned int Distance_Angle = 5, Y_Axis_Cal_Data = 2;
unsigned int LowSpeed_Distance_Angle = 5, LowSpeed_Y_Axis_Cal_Data = 2;
unsigned int HighSpeed_Distance_Angle = 5, HighSpeed_Y_Axis_Cal_Data = 2;
int Line_X_Correction = 0, Line_Deg_Correction = 0, Motor_Speed_Sum = 10, Motor_Speed_Divide = 10;
unsigned int On_Tag_Falg = 0;
uint16_t Lift_DA_Value = 0;

unsigned int Spin_P_Factor = 8;
unsigned int Spin_I_Factor = 2;
unsigned int Spin_D_Factor = 1;

int Motor_Balance = 0;

unsigned int Torque_Cal_Buff = 12;

unsigned int PGV_ID = 0;
unsigned int Setting_Flag = 0;

int Check_SEQ_Num_F = 0;
int Check_SEQ_Num_B = 0;

unsigned char Can_Sen_Count = 0;
unsigned int Tim_Reset_Count = 0;
unsigned int Sampling_Time_Count = 0;
uint16_t BAT_POW_Off_Flag = 0;

/*** MANUAL KEY INPUT VARAIABLE DEFINE ****************************************/
KEY_IN_FRAME Key_Inp;

KEY_IN_FRAME KeyIN_Buff, Old_KeyIN;//, LiftSW_Buff, Old_LiftIN;
unsigned char PreDriving_State_By_Key = 0, Key_Input_Stop = 0;

/*** SEQUENCE VARAIABLE DEFINE ************************************************/
unsigned int Course_Driving_Status = 0;

unsigned int Direct_Stop_Status = 0;
unsigned int Forward_Mark_Stop_Status = 0;
unsigned int Forward_Mark_Delay = 0;
unsigned int Guide_Off_Status = 0;
unsigned int Forward_Flag = 0;
unsigned int Left_Spin_Function_Sequence = 0;
unsigned int Right_Spin_Function_Sequence = 0;
unsigned int Lift_Interface_Sequence_Status = 0;
unsigned int Stop_Action_Function_Sequence = 0;
unsigned int Charge_Seq_Num = 0, DisCharge_Seq_Num = 0;
unsigned int Stop_Action_Num = 0;
unsigned int Restart_Action_Num = 0;
unsigned int Lift_Action_To_Moving_SEQ_Num = 0;

unsigned int Right_Turn_Forward_Status = 0;
unsigned int Left_Turn_Forward_Status = 0;

unsigned int Start_Direction = 0;

unsigned int AGVC_Charge_Move = 0;

unsigned int Spin_Count = 0;

unsigned int Course_Position = 0;

unsigned int AGV_Manual_Course_Num = 0;

unsigned int Correctio_Speed_Section = 2;

volatile uint32_t Charge_Check_Count = 0;
volatile uint32_t Charge_Error_Count = 0;
volatile uint32_t Battery_Low_Check_Count = 0;
volatile uint32_t Battery_Low_Error_Count = 0;
volatile uint32_t Error_Continue_Flag = 0;
volatile uint32_t Error_Time_Check_Count = 0;
volatile uint32_t Error_Continue_Count = 0;
volatile uint32_t Error_1_min_Check_Count = 0;

unsigned int Sensor_Spin_Stop_Flag = 0;
// unsigned int Sensor_Spin_Stop_Action = 0;
/*** SENSOR INPUT VARAIABLE DEFINE ********************************************/
// PIN
unsigned int PIN_State = 0;
// unsigned char Acquire_State = 0, Acquire_Err_Scan = 0, Acquire_Count = 0, Acquire_Err_Scan_Start = 0;
unsigned int Pin_Motor_Target_Speed = 1250;

unsigned char Lift_Down_Flag = 0;

// MARK SENSOR
unsigned int Mark_Stop = 0;
unsigned int Mark_Stop_Check = 0;
unsigned char Mark_Select = 0;
unsigned char Mark_Spin_Stop_Sel = 0;

unsigned char Mark_Stop_Sel = Left_Mark_En;
unsigned char Mark_Scan_Able = 1;
unsigned int Marker_Wait_Count = 5;

unsigned char Mark_Sensing_Flag = 0, Mark_Input_Signal = 0;
unsigned int Mark_Count = 0;

unsigned int Mark_Check = 0;
// unsigned int Mark_On = 0;

// TIM & SAFETY SENSOR
unsigned int TIM_Sensor_Status = 0;
unsigned char TIM_Special_Area_OnOff = 0;
unsigned char Tim_2Area_Sensing = 0;
unsigned char Back_Tim_2Area_Sensing = 0;

unsigned char R2100_Enable = 0, R2100_F_Enable = 0, R2100_B_Enable = 0;

unsigned char Front_Safety_Status = 1, Rear_Safety_Status = 2;

unsigned int Safety_Sensor_Event_Count = 0, Safety_Sensor_Event = 0;

unsigned int Front_Safety_Event = 0;
unsigned int Rear_Safety_Event = 0;

// Extend IO Board

EX_IO_FRAME Ex_Board1_Out_Port = {0}, Ex_Board1_In_Port = {0};
EX_IO_FRAME Ex_Board2_Out_Port = {0}, Ex_Board2_In_Port = {0};

// SCIB, COM to ACS
unsigned int DRV_Complete_Flag = 0, DRV_Complete_Count = 0, DRV_Complete_Data = 0, DRV_COM_TEST = 0, Event_Send_Flag = 0;
unsigned int Lift_Job_Comp = 0;
unsigned int Lift_Complete_Data = 0, Route_End_Complete_Data = 0, RFID_Read_Complete_Data = 0, Job_Cancel_Data = 0;
unsigned int Charge_Action_Complete_Data = 0;

unsigned char AGV_TEST_SEQ = 0;
unsigned int Forward_Mark_Act_Status = 0;
unsigned char StopMarkSen = 0;
unsigned char StopToLiftAct = 0;
unsigned char StopToMotAct = 0;
unsigned char LiftPioActFlag = 0;

// unsigned char Pause_Num = 0, Pause_Flag = 0, AGV_VEL = 0, Solw_Stop_Flag = 0;

unsigned char Guide_Read_Flag = 0, Stop_Delay_Flag = 0, Stop_Delay_Count = 0;

unsigned long Pre_Node_Num = 0, CUR_Node_Num = 0, NEXT_Node_Num = 0, CUR_Link = 0, Old_CUR_Node_Num = 0, DIR_NEXT_NODE = 0;

unsigned char Torque_OnOff = 0;

unsigned char Next_Node_Action = 0;

unsigned int Spin_Action_Start = 0;

unsigned int StopToLift_Flag = Normal_Stop;

// unsigned char Upper_Cart_Check = 0;

unsigned long Manual_Destination_Num = 0;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define  CPLD_adr 0x60000000 // CPLD Base address

#define outpw(a, b) *(__IO uint16_t*) (a) = b
#define inpw(a) *(__IO uint16_t*) (a)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

// unsigned char MT1H_Port = 0, MT1L_Port = 0, MT2H_Port = 0, MT2L_Port = 0, MT3H_Port = 0, MT3L_Port = 0, MT4H_Port = 0, MT4L_Port = 0;
// unsigned char Sound_Out_Port = 0, TIM_Out_Port = 0, Lift_Out_Port = 0, PIO_Out_Port = 0;

unsigned char Motor_Control_Status = MOTOR_STATUS_STOP;
unsigned char Motor_Pow = 0;

MOTOR_ALARM_FRAME MOT_ALARM;
MOT_OUT_FRAME MT1_Port, MT2_Port, MT3_Port, MT4_Port;

SOUND_OUT_FRAME Sound_Out_Port;

TIM_IO_FRAME TIM_Out_Port;
TIM_IO_FRAME TIM_IN_PORT;

uint8_t Tim_Set_Buff = 0;

LIFT_OUT_FRAME Lift_Out_Port;

PIO_OUT_FRAME PIO_Out_Port;

SPARE_OUT_FRAME SPARE_Out_Port;

PORT_BYTE_FRAME SPARE_IN;

ETC_IO_FRAME ETC_IO;

REMOTE_IO_FRAME Remote_In;

unsigned char tim7cnt = 0;

unsigned int Timer7_25mS_Count = 0;
unsigned int Timer7_100mS_Cnt = 0;
unsigned int Can_25ms_Count = 0;
unsigned long BitShift = 0;
// unsigned char UART1_TX_String_Buff[100];

unsigned int LED_Blue = 0, LED_RED = 0, LED_Green = 0;
unsigned int LED_Flicker_Flag = 0, LED_Flicker_Count = 0, Flicker_Buff = 0;

// unsigned int Motor_DAC_Out_Data[4] = {10000,30000,30000,30000};
unsigned int Motor_DAC_Out_Data[4] = { 0 ,0 ,0 ,0 };

int Right_PID_Count = 0, Left_PID_Count = 0;

unsigned int EEP_Test_Count = 0;

// Carriage define
unsigned char Carriage_Flag, PastCarriage_Flag = 0;
unsigned char Carriage_Status = 0;
unsigned int CarriageCheck_Cnt = 0;

typedef struct {
	unsigned char pastIn;
	unsigned char currentIn;
	unsigned char status;
	unsigned char cnt;
}CARRIAGE_STRUCT;

CARRIAGE_STRUCT FCarriage = {0, 0, 0, 0};
CARRIAGE_STRUCT RCarriage = {0, 0, 0, 0};

unsigned char Motor_Err_Read = 0;
unsigned int Com_Check_Count = 0;

unsigned char PGV_Send_Timmer_Count = 0;
unsigned char Prev_Speed_Buff = 0;

unsigned int GPIO_Read_Port(GPIO_TypeDef* GPIOx);
void GPIO_Write_Port(GPIO_TypeDef* GPIOx, uint8_t type, uint16_t Data );

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void Carriage_Status_Check ( unsigned int checkT )   /* Timer1 1 -> 25mS, 40 -> 1000mS */
{
	FCarriage.pastIn =  FCarriage.currentIn;
	FCarriage.currentIn = FRONT_CART_IN;

	if ( FCarriage.pastIn == FCarriage.currentIn ) FCarriage.cnt++;
	else FCarriage.cnt = 0;

	if ( FCarriage.cnt >= checkT ){
		FCarriage.status = FCarriage.currentIn;
		FCarriage.cnt = checkT;
		}

	RCarriage.pastIn =  RCarriage.currentIn;
	RCarriage.currentIn = REAR_CART_IN;

	if ( RCarriage.pastIn == RCarriage.currentIn ) RCarriage.cnt++;
	else RCarriage.cnt = 0;

	if ( RCarriage.cnt >= checkT ){
		RCarriage.status = RCarriage.currentIn;
		RCarriage.cnt = checkT;
		}
}

void Motor_Relay_ON ( void )
{
	HAL_GPIO_WritePin(MOTOR_ON_GPIO_Port, MOTOR_ON_Pin, GPIO_PIN_SET);
	ETC_IO.bit.MT_ON = 1;
}

void Motor_Relay_OFF ( void )
{
	HAL_GPIO_WritePin(MOTOR_ON_GPIO_Port, MOTOR_ON_Pin, GPIO_PIN_RESET);
	ETC_IO.bit.MT_ON = 0;
}

void LIFT_POW_ON ( void )
{
	// HAL_GPIO_WritePin(MOTOR_ON_GPIO_Port, MOTOR_ON_Pin, GPIO_PIN_RESET);
	Lift_Out_Port.bit.PIN_UP = 1;
	ETC_IO.bit.LF_ON = 1;

	ADDR16(PORTG) = Lift_Out_Port.all;
}

void LIFT_POW_OFF ( void )
{
	Lift_Out_Port.bit.PIN_UP = 0;
	ETC_IO.bit.LF_ON = 0;

	ADDR16(PORTG) = Lift_Out_Port.all;
}

void CHARGE_MC_ON ( void )
{
	ETC_IO.bit.MC_ON = SPARE_Out_Port.bit.IO1 = 1;
	ADDR16(PORTJ) = SPARE_Out_Port.all;
}

void CHARGE_MC_OFF ( void )
{
	ETC_IO.bit.MC_ON = SPARE_Out_Port.bit.IO1 = 0;
	ADDR16(PORTJ) = SPARE_Out_Port.all;
}

// int PID_Control(int Target_Value, int processValue, int Motor_num)
long PID_Control(int Target_Value, int processValue, int Motor_num)
{
	long error, p_term, d_term;
	long i_term, ret, temp;

	error = Target_Value - processValue;

	if (error > MaxError){
		p_term = MAX_INT;
		}
	else if (error < -MaxError){
		p_term = -MAX_INT;
		}
	else{
		p_term = P_Factor * error;
		}

	temp = Sumerror[Motor_num] + error;

	if(temp > MaxSumError){
		i_term = MAX_I_TERM;
		Sumerror[Motor_num] = MaxSumError;
		}
	else if(temp < -MaxSumError){
		i_term = -MAX_I_TERM;
		Sumerror[Motor_num] = -MaxSumError;
		}
	else{
		Sumerror[Motor_num] = temp;
		i_term = I_Factor * Sumerror[Motor_num];
		}

	d_term = D_Factor * (Last_Value[Motor_num] - processValue);

	Last_Value[Motor_num] = processValue;

	ret = (p_term + i_term + d_term) / SCALING_FACTOR;

	if(ret > 27000){
		ret = 27000;
		}
	else if(ret < 0){
		ret = 0;
		}

	return ((int)ret);
}

// void EEPROM_Write(unsigned int Add, unsigned int Data, unsigned char Len)
void EEPROM_Write(unsigned int Add, int Data, unsigned char Len)
{
	unsigned int i;

	if(Len == 1){
		HAL_Delay(5);
		AT25128_EEPROM_Control(WREN);
		for(i = 0; i < 200; i++)AT93C56_Disable;
		AT25128_Write_EEPROM(Add, (Data & 0x00FF));
		//AT25128_Write_EEPROM(i + 1, 0);
		for(i = 0; i < 200; i++)AT93C56_Disable;
		AT25128_EEPROM_Control(WRDI);
		}

	if(Len == 2){
		AT25128_EEPROM_Control(WREN);
		AT25128_Write_EEPROM(Add, ((Data >> 8) & 0x00FF));
		//AT25128_Write_EEPROM(i + 1, 0);
		AT25128_EEPROM_Control(WRDI);
		HAL_Delay(5);
		AT25128_EEPROM_Control(WREN);
		AT25128_Write_EEPROM(Add + 1, (Data & 0x00FF));
		//AT25128_Write_EEPROM(i + 1, 0);
		AT25128_EEPROM_Control(WRDI);
		HAL_Delay(5);
		}
}

// int EEPROM_Read(unsigned int Add, unsigned char Len)
unsigned int EEPROM_Read(unsigned int Add, unsigned char Len)
{
	unsigned int EEP_Data = 0;
	// int EEP_Data = 0;

	if(Len == 1){
		EEP_Data = AT25128_READ_EEPROM(Add) & 0x00FF;
		HAL_Delay(1);
		return(EEP_Data);
		}

	if(Len == 2){
		EEP_Data = (AT25128_READ_EEPROM(Add) << 8) & 0xFF00;
		HAL_Delay(1);
		EEP_Data |= AT25128_READ_EEPROM(Add+1) & 0x00FF;
		HAL_Delay(1);
		return(EEP_Data);
		}

	//return(EEP_Data);
}

void LED_Color_Select(unsigned int Red, unsigned int Green, unsigned int Blue)
{
	if(Red > TIM8->ARR)LED_RED = Red = TIM8->ARR;
	if(Green > TIM8->ARR)LED_Green = Green = TIM8->ARR;
	if(Blue > TIM8->ARR)LED_Blue = Blue = TIM8->ARR;

	if ( Red >= 10000 ) ETC_IO.bit.LED_RED = 1;
	else ETC_IO.bit.LED_RED = 0;

	if ( Green >= 10000 ) ETC_IO.bit.LED_GRN = 1;
	else ETC_IO.bit.LED_GRN = 0;

	if ( Blue >= 10000 ) ETC_IO.bit.LED_BLU = 1;
	else ETC_IO.bit.LED_BLU = 0;

	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, Blue);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_2, Red);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, Green);
}

void PIO_Out(unsigned int Num)
{
	// Lift_PIO_Control &= ~0x3F;
	// DELAY_US(1);
	// Lift_PIO_Control |= Num;
	// ADDR16(PORTE) = PIO_Out_Port;
}

PORT_IO_FRAME PIO_IN_PORT;

unsigned char PIO_Read(void)
{
	unsigned char Data;

	// Data = (GpioDataRegs.GPBDAT.all >> 12) & 0x0000000F;
	PIO_IN_PORT.bit.IO1 = HAL_GPIO_ReadPin(PIO_IN1_GPIO_Port, PIO_IN1_Pin);
	PIO_IN_PORT.bit.IO2 = HAL_GPIO_ReadPin(PIO_IN2_GPIO_Port, PIO_IN2_Pin);
	PIO_IN_PORT.bit.IO3 = HAL_GPIO_ReadPin(PIO_IN3_GPIO_Port, PIO_IN3_Pin);
	PIO_IN_PORT.bit.IO4 = HAL_GPIO_ReadPin(PIO_IN4_GPIO_Port, PIO_IN4_Pin);
	PIO_IN_PORT.bit.IO5 = HAL_GPIO_ReadPin(PIO_GO_GPIO_Port, PIO_GO_Pin);

	Data = PIO_IN_PORT.all;

	return(Data);
}

unsigned char Near_Sensor_Read (void)
{
	unsigned char Data;

	TIM_IN_PORT.all = 0;
	TIM_IN_PORT.bit.IO1_1 = HAL_GPIO_ReadPin(TIM1_OUT1_GPIO_Port, TIM1_OUT1_Pin); 	// C10-11
	TIM_IN_PORT.bit.IO1_2 = HAL_GPIO_ReadPin(TIM1_OUT2_GPIO_Port, TIM1_OUT2_Pin);	// C10-12
	TIM_IN_PORT.bit.IO2_1 = HAL_GPIO_ReadPin(TIM2_OUT1_GPIO_Port, TIM2_OUT1_Pin);	// C10-15
	TIM_IN_PORT.bit.IO2_2 = HAL_GPIO_ReadPin(TIM2_OUT2_GPIO_Port, TIM2_OUT2_Pin);	// C10-16

	Data = TIM_IN_PORT.all;

	return (Data);
}



uint8_t EMG_Signal_Read ( void )
{
	EMG_IO_FRMAE Data;

	Data.all = 0;
	Data.bit.EMG_FL = HAL_GPIO_ReadPin(KEY_IN_GPIO_Port, KEY_IN6_Pin);
	Data.bit.EMG_FR = HAL_GPIO_ReadPin(KEY_IN_GPIO_Port, KEY_IN7_Pin);
	Data.bit.EMG_RL = HAL_GPIO_ReadPin(SPARE_IN1_GPIO_Port, SPARE_IN1_Pin);
	Data.bit.EMG_RR = HAL_GPIO_ReadPin(SPARE_IN2_GPIO_Port, SPARE_IN2_Pin);
	Data.bit.BUMP1 = HAL_GPIO_ReadPin(SPARE_IN9_GPIO_Port, SPARE_IN9_Pin);

	return (Data.all);
}

unsigned int GetCRC16(char *p, int len, int Type)
{
	unsigned int CRC16 = 0xffff;
	int i;

	if(Type == 0){
		for(i = 0; i < len; i++){
			CRC16 = (CRC16 >> 8) ^ Modbus_CRC16[(CRC16 ^ p[i]) & 0x00ff];
			}
		}

	else if(Type == 1){
		for(i = 1; i < len; i++){
			CRC16 = (CRC16 >> 8) ^ Modbus_CRC16[(CRC16 ^ p[i]) & 0x00ff];
			}
		}

	else if (Type == 2){
		CRC16 = (CRC16 >> 8) ^ Modbus_CRC16[(CRC16 ^ STX) & 0x00ff];
		for(i = 0; i < len; i++) CRC16 = (CRC16 >> 8) ^ Modbus_CRC16[(CRC16 ^ p[i]) & 0x00ff];
		}

	return CRC16;
}

unsigned int CheckSum_Generation_Func( unsigned char *str, unsigned char Count)
{
	unsigned int Check_Sum_Buff = 0, Cal_Buff = 0;

	Check_Sum_Buff = GetCRC16(( char *)str, Count, 1);

	Cal_Buff = (Check_Sum_Buff >> 12) & 0x000F;
	if(Cal_Buff < 10)str[Count++] = Cal_Buff + '0';
	if(Cal_Buff >= 10)str[Count++] = (Cal_Buff - 10) + 'A';

	Cal_Buff = (Check_Sum_Buff >> 8) & 0x000F;
	if(Cal_Buff < 10)str[Count++] = Cal_Buff + '0';
	if(Cal_Buff >= 10)str[Count++] = (Cal_Buff - 10) + 'A';

	Cal_Buff = (Check_Sum_Buff >> 4) & 0x000F;
	if(Cal_Buff < 10)str[Count++] = Cal_Buff + '0';
	if(Cal_Buff >= 10)str[Count++] = (Cal_Buff - 10) + 'A';

	Cal_Buff = Check_Sum_Buff & 0x0F;
	if(Cal_Buff < 10)str[Count++] = Cal_Buff + '0';
	if(Cal_Buff >= 10)str[Count++] = (Cal_Buff - 10) + 'A';

	str[Count++] = ETX;
	str[Count] = 0;

	return (Count);
}

void PGV_Line_Select(unsigned int Sel_Data, unsigned int Dev_ID)
{
	unsigned int Send_Buff = 0;

	Reset_Rwi_Position();

	Send_Buff = Sel_Direction << 5;
	Send_Buff |= (Sel_Data << 2);
	Send_Buff |= Dev_ID & 0x03;

	SCIB_485_TX;
	DELAY_US(100);

	SCIB_TX_Char(Send_Buff);
	SCIB_TX_Char(~Send_Buff);

	DELAY_US(300);
	// DELAY_US(200);
	SCIB_485_RX;

	PGV_Data_Rec_Buff_Size = 3;
	PGV_Data_Set_Flag = 1;
	PGV_Data_Send_Flag = 1;

	//Reset_Rwi_Position();
}


void PGV_Color_Select(unsigned int Sel_Color, unsigned int Dev_ID)
{
	unsigned int Send_Buff = 0;

	Reset_Rwi_Position();

	if(Sel_Color == Color_Blue)Send_Buff = Sel_Blue << 5;
	//else if(Sel_Color == Color_Red)Send_Buff = Sel_RED << 5;
	else Send_Buff = Sel_Green << 5;  // 6 << 5

	Send_Buff |= (Sel_Color << 2);	// 1 << 4
	Send_Buff |= Dev_ID & 0x03;

	SCIB_485_TX;
	DELAY_US(300);

	SCIB_TX_Char(Send_Buff);
	SCIB_TX_Char(~Send_Buff);

	DELAY_US(300);
	SCIB_485_RX;

	PGV_Data_Rec_Buff_Size = 2;
	PGV_Data_Set_Flag = 2;
	PGV_Data_Send_Flag = 1;

	//Reset_Rwi_Position();
}


void PGV_Com_Send(unsigned int PGV_Set_Flag)
{
	unsigned int Send_Buff = 0;

	Reset_Rwi_Position();

	Send_Buff = (PGV_Structs_Send_Fram.PGV_Command << 5);		// 6 << 5
	Send_Buff |= (PGV_Structs_Send_Fram.PGV_Structs << 2);		// 2 << 2
	Send_Buff |= PGV_Structs_Send_Fram.PGV_ID & 0x03;

	SCIB_485_TX;
	DELAY_US(200);

	SCIB_TX_Char(Send_Buff);
	SCIB_TX_Char(~Send_Buff);

	DELAY_US(300);
	SCIB_485_RX;

	//Register_Buff[36] = Send_Buff;

	PGV_Data_Set_Flag = 3;
	PGV_Data_Rec_Buff_Size = 21;
	PGV_Data_Send_Flag = 1;
}


// unsigned int RFID_Test_Count = 0;
// void RFID_Read(unsigned int On_Off_Flag)
// {
// 	unsigned char Com_Buff[10];

// 	Com_Buff[0] = '~';
// 	Com_Buff[1] = 'a';
// 	if(On_Off_Flag == ON){
// 		Com_Buff[2] = 'f';
// 		}
// 	else if(On_Off_Flag == OFF){
// 		Com_Buff[2] = 's';
// 		RFID_Test_Count = 0;
// 		}

// 	Com_Buff[3] = COM_CR;
// 	Com_Buff[4] = COM_LF;
// 	Com_Buff[5] = 0;

// 	DELAY_US(1);

// 	SCIE_TX_String((char *)Com_Buff);
// }

// void One_Read_Only(void)
// {
// 	unsigned char Com_Buff[10];

// 	Com_Buff[0] = '~';
// 	Com_Buff[1] = 'w';
// 	Com_Buff[2] = 'c';
// 	Com_Buff[3] = '0';
// 	Com_Buff[4] = COM_CR;
// 	Com_Buff[5] = COM_LF;
// 	Com_Buff[6] = 0;

// 	SCIE_TX_String((char *)Com_Buff);
// }

// unsigned int ISR_ON_OFF_Flag = 0;
// void ISR_On_Off(unsigned int On_Flag)
// {

// }


unsigned int SpinTurn_Deg = 0;
void Spin_Start(unsigned char Spin_Dir, unsigned char Spd, unsigned char Dir, unsigned int Angle, unsigned int Start_Point, unsigned int Job_Comp_Flag)
{
	Course_Driving_Status = 1;

	Action_After_Speed = Spd;
	Forward_Flag = Dir;

	if(Job_Comp_Flag == 1){
		//if(PGV_Tag_Num_Data == Destinations_Full_Num)DRV_Complete_Flag = 1;
		DRV_Complete_Flag = 1;
		}

	if(Spin_Dir == Left_Spin){

		Left_Spin_Function_Sequence = Start_Point;
		Right_Spin_Function_Sequence = 0;
		Forward_Mark_Stop_Status = 0;

		if(Start_Point == 1){
			if(Surface_Input == 1){
				if(Angle == 90)Spin_Max_Degrees = 2700;
				else if(Angle == 180)Spin_Max_Degrees = 1800;
				}
			else if(Surface_Input == 2){
				if(Angle == 90)Spin_Max_Degrees = 0;
				else if(Angle == 180)Spin_Max_Degrees = 2700;
				}
			else if(Surface_Input == 3){
				if(Angle == 90)Spin_Max_Degrees = 900;
				else if(Angle == 180)Spin_Max_Degrees = 0;
				}
			else if(Surface_Input == 4){
				if(Angle == 90)Spin_Max_Degrees = 1800;
				else if(Angle == 180)Spin_Max_Degrees = 900;
				}
			}

		if(Start_Point == 3){
			if((PGV_Angle_Data >= 3150) || (PGV_Angle_Data < 450))Spin_Max_Degrees = 2700;
			else if((PGV_Angle_Data >= 450) && (PGV_Angle_Data < 1350))Spin_Max_Degrees = 0;
			else if((PGV_Angle_Data >= 1350) && (PGV_Angle_Data < 2250))Spin_Max_Degrees = 900;
			else if((PGV_Angle_Data >= 2250) && (PGV_Angle_Data < 3150))Spin_Max_Degrees = 1800;
			}
		//Mark_Scan_Able = 0;
		}

	if(Spin_Dir == Right_Spin){

		Right_Spin_Function_Sequence = Start_Point;
		Left_Spin_Function_Sequence = 0;
		Forward_Mark_Stop_Status = 0;

		if(Start_Point == 1){
			if(Surface_Input == 1){
				if(Angle == 90)Spin_Max_Degrees = 900;
				else if(Angle == 180)Spin_Max_Degrees = 1800;
				}
			else if(Surface_Input == 2){
				if(Angle == 90)Spin_Max_Degrees = 1800;
				else if(Angle == 180)Spin_Max_Degrees = 2700;
				}
			else if(Surface_Input == 3){
				if(Angle == 90)Spin_Max_Degrees = 2700;
				else if(Angle == 180)Spin_Max_Degrees = 0;
				// else if(Angle == 180)Spin_Max_Degrees = 3600;
				}
			else if(Surface_Input == 4){
				// if(Angle == 90)Spin_Max_Degrees = 3600;
				if(Angle == 90)Spin_Max_Degrees = 0;
				else if(Angle == 180)Spin_Max_Degrees = 900;
				}
			}

		if(Start_Point == 3){
			if((PGV_Angle_Data >= 3150) || (PGV_Angle_Data < 450)){
				if(Angle == 90)Spin_Max_Degrees = 900;
				else if(Angle == 180)Spin_Max_Degrees = 1800;
				}
			else if((PGV_Angle_Data >= 450) && (PGV_Angle_Data < 1350)){
				if(Angle == 90)Spin_Max_Degrees = 1800;
				else if(Angle == 180)Spin_Max_Degrees = 2700;
				}
			else if((PGV_Angle_Data >= 1350) && (PGV_Angle_Data < 2250)){
				if(Angle == 90)Spin_Max_Degrees = 2700;
				else if(Angle == 180)Spin_Max_Degrees = 0;
				}
			else if((PGV_Angle_Data >= 2250) && (PGV_Angle_Data < 3150)){
				if(Angle == 90)Spin_Max_Degrees = 0;
				else if(Angle == 180)Spin_Max_Degrees = 900;
				}
			}
		//Mark_Scan_Able = 0;
		}

	SpinTurn_Deg = Angle;
	Spin_Stop_Scan_Flag = 0;
}

void Stop_Action_Start(unsigned int Stop_Flag, unsigned int Stop_Action, unsigned int Start_Flag)
{
	Stop_Action_Function_Sequence = 1;
	Stop_Action_Num = Stop_Action;
	Restart_Action_Num = Start_Flag;
}

unsigned int Angle_Cal_Temp = 0;
#define CAL_TAGET_SPEED(SPD)  (((((SPD * 100) * 10000) / Encoder_Resolution) / 60) / Check_Frequency)

void Moving_Speed_Set(unsigned long Spd)
{
	// unsigned long Balance_Buff = 0, Speed_Cal_Buff = 0;

	AGV_VEL = Spd;

	Old_Target_Speed = Target_Speed = ((((Spd * 100) * 10000) / Encoder_Resolution) / 60) / Check_Frequency;
	Left_Target_Speed = Right_Target_Speed = Target_Speed;

	if((AGVMotionData.Drive == Forward_DRV) || (AGVMotionData.Drive == Backward_DRV)){
		P_Factor = ((float)PID_Gain_Buff[P_Gain] / 10) * SCALING_FACTOR;
		I_Factor = ((float)PID_Gain_Buff[I_Gain] / 10) * SCALING_FACTOR;
		D_Factor = ((float)PID_Gain_Buff[D_Gain] / 10) * SCALING_FACTOR;
		}

	if(AGV_VEL <= 200){
		Y_Axis_Cal_Data = LowSpeed_Y_Axis_Cal_Data;
		Distance_Angle = LowSpeed_Distance_Angle;
		}
	else{
		Y_Axis_Cal_Data = HighSpeed_Y_Axis_Cal_Data;
		Distance_Angle = HighSpeed_Distance_Angle;
		}

	if(Old_Left_target_Speed > Left_Target_Speed)Left_Speed_Slope = 1;
	else if(Old_Left_target_Speed < Left_Target_Speed)Left_Speed_Slope = 2;
	else Left_Speed_Slope = 0;

	if(Old_Right_target_Speed > Right_Target_Speed)Right_Speed_Slope = 1;
	else if(Old_Right_target_Speed < Right_Target_Speed)Right_Speed_Slope = 2;
	else Right_Speed_Slope = 0;

	Old_Left_target_Speed = Left_Target_Speed;
	Old_Right_target_Speed = Right_Target_Speed;

}

// unsigned int Stop_Flag = 0, Stop_Flag_Count = 0;
unsigned int Brake_Release_Err_Flag = 0;

void Forward_GPIO(void)
{
	if ( ( OPERATE_MODE == Auto_Operate ) && ( Key_Inp.bit.KeyBRK_REL == 1 ) ){
		Brake_Release_Err_Flag = 1;
		}
	else {
		Brake_Release_Err_Flag = 0;

		// Stop_Flag = 0;
		// Stop_Flag_Count = 0;
		Front_Safety_Event = 0;

		Motor_DA_Buff[0] = 0; //1300;
		Motor_DA_Buff[2] = 0; // 1300;
		AD5754_DA_Out_Function((unsigned int *)Motor_DA_Buff, 4); // ????
		AD5754_DA_Out_Function((unsigned int *)Motor_DA_Buff, 4); // ????

		if ( ETC_IO.bit.MT_ON == 0 ) {
			Motor_Relay_ON();
			DELAY_MS(1000);
			}

		Motor_Control_Status = MOTOR_STATUS_RUN;                      // Motor_Controrl_GPIO &= ~Motor_RUN
		
		// MT1_PORT_START_PIN = 1;             // MT1 SERVO ON
		// MT1_PORT_BREAK_PIN = 1;             // MT1 BREAK OFF
		MT1_PORT_DIR_CW_PIN = 1;            // MT1 DIR CW
		MT1_PORT_ADDR_WRITE;

		// MT3_PORT_START_PIN = 1;             // MT3 SERVO ON
		// MT3_PORT_BREAK_PIN = 1;             // MT3 BREAK OFF;
		MT3_PORT_DIR_CW_PIN = 0;            // MT3 DIR CCW
		MT3_PORT_ADDR_WRITE;

		DELAY_MS(100);

		MT1_PORT_START_PIN = 1;             // MT1 SERVO ON
		MT1_PORT_BREAK_PIN = 1;             // MT1 BREAK OFF
		// MT1_PORT_DIR_CW_PIN = 1;            // MT1 DIR CW
		MT1_PORT_ADDR_WRITE;

		MT3_PORT_START_PIN = 1;             // MT3 SERVO ON
		MT3_PORT_BREAK_PIN = 1;             // MT3 BREAK OFF;
		// MT3_PORT_DIR_CW_PIN = 0;            // MT3 DIR CCW
		MT3_PORT_ADDR_WRITE;

		DELAY_MS(100);

		RightMotor_Int_Count = LeftMotor_Int_Count = Right_PID_Count = Left_PID_Count = 0;
		Right_Mag_Count = Left_Mag_Count = 0;

		Start_Speed_UpFlag = 1;

		Sumerror[0] = Sumerror[2] = 0;

		P_Factor = ((float)PID_Gain_Buff[P_Gain] / 10) * SCALING_FACTOR;
		I_Factor = ((float)PID_Gain_Buff[I_Gain] / 10) * SCALING_FACTOR;
		D_Factor = ((float)PID_Gain_Buff[D_Gain] / 10) * SCALING_FACTOR;

		// DELAY_MS(200);

		Old_DRV_Buff = AGVMotionData.Drive = Forward_DRV;
		}
}

void Backward_GPIO(void)
{
	if ( ( OPERATE_MODE == Auto_Operate ) && ( Key_Inp.bit.KeyBRK_REL == 1 ) ){
		Brake_Release_Err_Flag = 1;
		}
	else {
		Brake_Release_Err_Flag = 0;

		// Stop_Flag = 0;
		// Stop_Flag_Count = 0;
		Rear_Safety_Event = 0;

		Motor_DA_Buff[0] = 0; // 1300;
		Motor_DA_Buff[2] = 0; // 1300;
		AD5754_DA_Out_Function((unsigned int *)Motor_DA_Buff, 4); // ????
		AD5754_DA_Out_Function((unsigned int *)Motor_DA_Buff, 4); // ????

		if ( ETC_IO.bit.MT_ON == 0 ) {
			Motor_Relay_ON();
			DELAY_MS(1000);
			}

		Motor_Control_Status = MOTOR_STATUS_RUN;

		// MT1_PORT_START_PIN = 1;             // MT1 SERVO ON
		// MT1_PORT_BREAK_PIN = 1;             // MT1 BREAK OFF
		MT1_PORT_DIR_CW_PIN = 0;            // MT1 DIR CCW
		MT1_PORT_ADDR_WRITE;

		// MT3_PORT_START_PIN = 1;             // MT3 SERVO ON
		// MT3_PORT_BREAK_PIN = 1;             // MT3 BREAK OFF;
		MT3_PORT_DIR_CW_PIN = 1;            // MT3 DIR CW
		MT3_PORT_ADDR_WRITE;

		DELAY_MS(100);

		MT1_PORT_START_PIN = 1;             // MT1 SERVO ON
		MT1_PORT_BREAK_PIN = 1;             // MT1 BREAK OFF
		// MT1_PORT_DIR_CW_PIN = 0;            // MT1 DIR CCW
		MT1_PORT_ADDR_WRITE;

		MT3_PORT_START_PIN = 1;             // MT3 SERVO ON
		MT3_PORT_BREAK_PIN = 1;             // MT3 BREAK OFF;
		// MT3_PORT_DIR_CW_PIN = 1;            // MT3 DIR CW
		MT3_PORT_ADDR_WRITE;

		DELAY_MS(100);

		RightMotor_Int_Count = LeftMotor_Int_Count = Right_PID_Count = Left_PID_Count = 0;
		Right_Mag_Count = Left_Mag_Count = 0;

		Start_Speed_UpFlag = 1;

		Sumerror[0] = Sumerror[2] = 0;

		P_Factor = ((float)PID_Gain_Buff[P_Gain] / 10) * SCALING_FACTOR;
		I_Factor = ((float)PID_Gain_Buff[I_Gain] / 10) * SCALING_FACTOR;
		D_Factor = ((float)PID_Gain_Buff[D_Gain] / 10) * SCALING_FACTOR;

		// DELAY_MS(200);

		Old_DRV_Buff = AGVMotionData.Drive = Backward_DRV;
		}
}

void Left_Spin_GPIO(void)
{
	if ( ( OPERATE_MODE == Auto_Operate ) && ( Key_Inp.bit.KeyBRK_REL == 1 ) ){
		Brake_Release_Err_Flag = 1;
		}
	else {
		Brake_Release_Err_Flag = 0;

		// Stop_Flag = 0;
		// Stop_Flag_Count = 0;
		Front_Safety_Event = Rear_Safety_Event = 0;

		Motor_DA_Buff[0] = 0; // 1000;
		Motor_DA_Buff[2] = 0; //1000;
		AD5754_DA_Out_Function((unsigned int *)Motor_DA_Buff, 4); // ????
		AD5754_DA_Out_Function((unsigned int *)Motor_DA_Buff, 4); // ????

		if ( ETC_IO.bit.MT_ON == 0 ) {
			Motor_Relay_ON();
			DELAY_MS(1000);
			}

		Motor_Control_Status = MOTOR_STATUS_RUN;

		// MT1_PORT_START_PIN = 1;             // MT1 SERVO ON
		//MT1_PORT_BREAK_PIN = 1;             // MT1 BREAK OFF
		MT1_PORT_DIR_CW_PIN = 1;            // MT1 DIR CW
		MT1_PORT_ADDR_WRITE;

		// MT3_PORT_START_PIN = 1;             // MT3 SERVO ON
		//MT3_PORT_BREAK_PIN = 1;             // MT3 BREAK OFF;
		MT3_PORT_DIR_CW_PIN = 1;            // MT3 DIR CW
		MT3_PORT_ADDR_WRITE;

		DELAY_MS(100);

		MT1_PORT_START_PIN = 1;             // MT1 SERVO ON
		MT1_PORT_BREAK_PIN = 1;             // MT1 BREAK OFF
		// MT1_PORT_DIR_CW_PIN = 1;            // MT1 DIR CW
		MT1_PORT_ADDR_WRITE;

		MT3_PORT_START_PIN = 1;             // MT3 SERVO ON
		MT3_PORT_BREAK_PIN = 1;             // MT3 BREAK OFF;
		// MT3_PORT_DIR_CW_PIN = 1;            // MT3 DIR CW
		MT3_PORT_ADDR_WRITE;

		DELAY_MS(100);

		Start_Speed_UpFlag = 1;

		Right_PID_Count = Left_PID_Count = 0;
		Right_Mag_Count = Left_Mag_Count = 0;

		Sumerror[0] = Sumerror[2] = 0;

		P_Factor = ((float)Spin_P_Factor / 10) * SCALING_FACTOR;
		I_Factor = ((float)Spin_I_Factor / 10) * SCALING_FACTOR;
		D_Factor = ((float)Spin_D_Factor / 10) * SCALING_FACTOR;

		Spin_Slow_Flag = 1;
		AGVMotionData.Drive = Left_Spin_DRV;
		}
}

void Right_Spin_GPIO(void)
{
	if ( ( OPERATE_MODE == Auto_Operate ) && ( Key_Inp.bit.KeyBRK_REL == 1 ) ){
		Brake_Release_Err_Flag = 1;
		}
	else {
		Brake_Release_Err_Flag = 0;

		// Stop_Flag = 0;
		// Stop_Flag_Count = 0;
		Front_Safety_Event = Rear_Safety_Event = 0;

		Motor_DA_Buff[0] = 0; // 1000;
		Motor_DA_Buff[2] = 0; // 1000;
		AD5754_DA_Out_Function((unsigned int *)Motor_DA_Buff, 4); // ????
		AD5754_DA_Out_Function((unsigned int *)Motor_DA_Buff, 4); // ????

		if ( ETC_IO.bit.MT_ON == 0 ) {
			Motor_Relay_ON();
			DELAY_MS(1000);
			}

		Motor_Control_Status = MOTOR_STATUS_RUN;

		// MT1_PORT_START_PIN = 1;             // MT1 SERVO ON
		// MT1_PORT_BREAK_PIN = 1;             // MT1 BREAK OFF
		MT1_PORT_DIR_CW_PIN = 0;            // MT1 DIR CCW
		MT1_PORT_ADDR_WRITE;

		// MT3_PORT_START_PIN = 1;             // MT3 SERVO ON
		// MT3_PORT_BREAK_PIN = 1;             // MT3 BREAK OFF;
		MT3_PORT_DIR_CW_PIN = 0;            // MT3 DIR CCW
		MT3_PORT_ADDR_WRITE;

		DELAY_MS(100);

		MT1_PORT_START_PIN = 1;             // MT1 SERVO ON
		MT1_PORT_BREAK_PIN = 1;             // MT1 BREAK OFF
		// MT1_PORT_DIR_CW_PIN = 0;            // MT1 DIR CCW
		MT1_PORT_ADDR_WRITE;

		MT3_PORT_START_PIN = 1;             // MT3 SERVO ON
		MT3_PORT_BREAK_PIN = 1;             // MT3 BREAK OFF;
		// MT3_PORT_DIR_CW_PIN = 0;            // MT3 DIR CCW
		MT3_PORT_ADDR_WRITE;

		DELAY_MS(100);

		Start_Speed_UpFlag = 1;

		Right_PID_Count = Left_PID_Count = 0;
		Right_Mag_Count = Left_Mag_Count = 0;

		Sumerror[0] = Sumerror[2] = 0;
		//Last_Value[0] = Last_Value[1] = 0;

		P_Factor = ((float)Spin_P_Factor / 10) * SCALING_FACTOR;
		I_Factor = ((float)Spin_I_Factor / 10) * SCALING_FACTOR;
		D_Factor = ((float)Spin_D_Factor / 10) * SCALING_FACTOR;

		Spin_Slow_Flag = 1;
		AGVMotionData.Drive = Right_Spin_DRV;
		}
}

void Stop_GPIO(void)
{
	unsigned int Motor_DA_Data[4];

	Motor_Control_Status = MOTOR_STATUS_STOP;       // Motor_Controrl_GPIO |= Motor_RUN

	MT1_PORT_START_PIN = 0;
	MT1_PORT_BREAK_PIN = 0;
	MT1_PORT_DIR_CW_PIN = 0;
	MT1_PORT_ADDR_WRITE;

	MT3_PORT_START_PIN = 0;
	MT3_PORT_BREAK_PIN = 0;
	MT3_PORT_DIR_CW_PIN = 0;
	MT3_PORT_ADDR_WRITE;

	Sumerror[0] = Sumerror[2] = 0;

	Motor_DA_Data[0] = 0;
	Motor_DA_Data[2] = 0;
	Motor_DA_Buff[DACA] = Motor_DA_Buff[DACC] = 0;
	AD5754_DA_Out_Function(Motor_DA_Data, 4); // ????
	AD5754_DA_Out_Function(Motor_DA_Data, 4); // ????

	Right_PID_Count = Left_PID_Count = 0;
	Right_Mag_Count = Left_Mag_Count = 0;

 	Direct_Stop_Flag = 0;
	// Stop_Flag = 1;
	// Stop_Flag_Count = 0;
}

void Motor_Reset(void)
{
	/* Lead shine  */
	Motor_Relay_ON();

	DELAY_MS(500);

	MT1_PORT_RESET_PIN = 1;         // Motor_Contorl_GPIO |= Motor_RST;
	MT1_PORT_ADDR_WRITE;            // ADDR16(PORTB) = Motor_Contorl_GPIO;

	MT3_PORT_RESET_PIN = 1;
	MT3_PORT_ADDR_WRITE;
	DELAY_MS(100);


	MT1_PORT_RESET_PIN = 0;         // Motor_Contorl_GPIO &= ~Motor_RST;
	MT1_PORT_ADDR_WRITE;            // ADDR16(PORTB) = Motor_Contorl_GPIO;

	MT3_PORT_RESET_PIN = 0;
	MT3_PORT_ADDR_WRITE;
	DELAY_MS(500);
}

void Job_Status_Clear ( void )
{
	Last_JobID = 0;
	Register_Buff[13] = 0;		// Last_JobID Low byte
	Register_Buff[14] = 0;		// Last_JobID High byte
	Register_Buff[16] = 0;		// Moving Speed
	// Register_Buff[45] = (long)(PGV_Tag_Num_Data_Buff >> 16) & 0xFFFF;
	// Register_Buff[44] = PGV_Tag_Num_Data_Buff & 0xFFFF;
	Register_Buff[17] = 0;  	// Node Command
	Register_Buff[18] = 0; 		// Node Angle
	Register_Buff[47] = 0; 		// Next Node Low byte
	Register_Buff[48] = 0; 		// Next Node High byte
	Register_Buff[20] = 0; 		// Safety Command

	Register_Buff[19] = Node_Check_Count = 0; 		// Node_Check_Count;
	Register_Buff[21] = 0; 		// Node_Count;

	Destinations_Full_Num = 0;	// Target Node
	//Register_Buff[34] = Register_Buff[51] = 0;	// Target Node
	Register_Buff[51] = 0;	// Target Node LowByte
	Register_Buff[52] = 0;	// Target Node HignByte
}

void AGV_Button_Event_Send(unsigned long Node, unsigned int btn )
{
	int8_t Count = 0;

	Count = Int_To_HexAscii_Num(AGV_Event_Req_Buff, Event_Send_Header_Offset, 3, 4);

	AGV_Event_Req_Buff[Count++] = 'E';
	AGV_Event_Req_Buff[Count++] = 'B';

	Count = Int_To_Ascii_Num(AGV_Event_Req_Buff, Count, btn, 1);

	CheckSum_Generation_Func( (unsigned char*)AGV_Event_Req_Buff, Count );

	SCIC_TX_String( AGV_Event_Req_Buff );
}


void AGV_Start_Place_Event_Send(unsigned long Node)
{
	unsigned char Count = 0;

	Node_Chagne_Event_Flag = 0;

	Count = Int_To_HexAscii_Num(AGV_Event_Req_Buff, Event_Send_Header_Offset, 9, 4);  // 9 -> 10

	AGV_Event_Req_Buff[Count++] = 'E';
	AGV_Event_Req_Buff[Count++] = 'I';

	Count = Long_To_HexAscii_Num(AGV_Event_Req_Buff, Count, Node, 6);  // 6 -> 7

	AGV_Event_Req_Buff[Count++] = Surface_Input + 0x30;
	// AGV_Event_Req_Buff[Event_Send_Header_Offset + Count++] = Surface_Input + 0x30;

	CheckSum_Generation_Func((unsigned char *)AGV_Event_Req_Buff, Count );

	SCIC_TX_String( AGV_Event_Req_Buff );
}


void AGV_NodeChange_Event_Send(unsigned long Node)
{
	unsigned char Count = 0;

	Node_Chagne_Event_Flag = 0;

	Count = Int_To_HexAscii_Num(AGV_Event_Req_Buff, Event_Send_Header_Offset, 14, 4); // 14

	AGV_Event_Req_Buff[Count++] = 'E';
	AGV_Event_Req_Buff[Count++] = 'N';

	Count = Long_To_HexAscii_Num(AGV_Event_Req_Buff, Count, Last_Node, 6); 	//

	Count = Long_To_HexAscii_Num(AGV_Event_Req_Buff, Count, Last_JobID, 5);

	AGV_Event_Req_Buff[Count++] = Surface_Input + 0x30;
	// AGV_Event_Req_Buff[Event_Send_Header_Offset + Count++] = Surface_Input + 0x30;

	CheckSum_Generation_Func((unsigned char *)AGV_Event_Req_Buff, Count);

	SCIC_TX_String(AGV_Event_Req_Buff);
}

void AGV_JobCancle_Event_Send(unsigned long Node)
{
	unsigned char Count = Event_Send_Header_Offset;

	Job_Status_Clear();

	Count = Int_To_HexAscii_Num(AGV_Event_Req_Buff, Event_Send_Header_Offset, 8, 4);	// 8 -> 9

	AGV_Event_Req_Buff[Count++] = 'E';
	AGV_Event_Req_Buff[Count++] = 'C';

	Count = Long_To_HexAscii_Num(AGV_Event_Req_Buff, Count, Last_Node, 6);  // 6 -> 7

	CheckSum_Generation_Func((unsigned char *)AGV_Event_Req_Buff, Count );

	SCIC_TX_String( AGV_Event_Req_Buff );

	Job_Cancel_Data = 1;
	Event_Send_Flag = 1;
	Job_Cancel_Stop_Flag = 0;
	DRV_Complete_Count = 0;
	DRV_Complete_Data = Job_Cancel;
	Register_Buff[121] = 1;
}


void AGV_Charge_Complete_Event_Send(void)
{
	unsigned char Count = Event_Send_Header_Offset;

	Count = Int_To_HexAscii_Num(AGV_Event_Req_Buff, Count, 3, 4);

	AGV_Event_Req_Buff[Count++] = 'E';
	AGV_Event_Req_Buff[Count++] = 'G';

	if(AGVMotionData.Drive == AGV_Wait)AGV_Event_Req_Buff[Count++] = '0';
	else if(AGVMotionData.Drive == AGV_CHARGING)AGV_Event_Req_Buff[Count++] = '1';
	else AGV_Event_Req_Buff[Count++] = '2';

	CheckSum_Generation_Func((unsigned char *)AGV_Event_Req_Buff, Count);

	SCIC_TX_String(AGV_Event_Req_Buff);

	DRV_COM_TEST++;

	Charge_Action_Complete_Data = 1;
	Event_Send_Flag = 1;
	DRV_Complete_Count = 0;
	DRV_Complete_Data = Charge_Complete;
}


void AGV_DRV_Complete_Event_Send(unsigned long JobID)
{
	unsigned char Count = 0;

	// Job_Status_Clear();

	Count = Int_To_HexAscii_Num(AGV_Event_Req_Buff, Event_Send_Header_Offset, 13, 4);

	AGV_Event_Req_Buff[Count++] = 'E';
	AGV_Event_Req_Buff[Count++] = 'D';

	Count = Long_To_HexAscii_Num(AGV_Event_Req_Buff, Count, JobID, 5);

	Count = Long_To_HexAscii_Num(AGV_Event_Req_Buff, Count, Last_Node , 6);  	// 6 -> 7

	CheckSum_Generation_Func((unsigned char *)AGV_Event_Req_Buff, Count);

	SCIC_TX_String(AGV_Event_Req_Buff);

	DRV_COM_TEST++;

	// Last_JobID = 0;
	Route_End_Complete_Data = 1;
	Event_Send_Flag = 1;
	DRV_Complete_Count = 0;
	DRV_Complete_Data = DRV_Complete;
	Register_Buff[122] = 1;
}

void AGV_Lift_Event_Send(unsigned int Act)
{
	unsigned char Count = 0;

	Count = Int_To_HexAscii_Num(AGV_Event_Req_Buff, Event_Send_Header_Offset, 5, 4);

	AGV_Event_Req_Buff[Count++] = 'E';
	AGV_Event_Req_Buff[Count++] = 'L';

	if(Act == Lift_Down)AGV_Event_Req_Buff[Count++] = 'D';
	else if(Act == Lift_Up)AGV_Event_Req_Buff[Count++] = 'U';

	AGV_Event_Req_Buff[Count++] = FCarriage.status + 0x30;

	AGV_Event_Req_Buff[Count++] = RCarriage.status + 0x30;

	CheckSum_Generation_Func((unsigned char *)AGV_Event_Req_Buff, Count);

	SCIC_TX_String(AGV_Event_Req_Buff);

	Lift_Complete_Data = 1;
	Event_Send_Flag = 1;
	DRV_Complete_Count = 0;
	DRV_Complete_Data = CV_Complete;
	//Node_Check_Count = 0;
}

void AGV_CarID_Event_Send(void)
{
	unsigned char Count = 0;

	Count = Int_To_HexAscii_Num(AGV_Event_Req_Buff, Event_Send_Header_Offset, 18, 4);

	AGV_Event_Req_Buff[Count++] = 'E';
	AGV_Event_Req_Buff[Count++] = 'T';

	Count = Long_To_HexAscii_Num( AGV_Event_Req_Buff, Count, Last_CarID1, 8 );

	Count = Long_To_HexAscii_Num( AGV_Event_Req_Buff, Count, Last_CarID2, 8 );

	CheckSum_Generation_Func((unsigned char *)AGV_Event_Req_Buff, Count );

	SCIC_TX_String( AGV_Event_Req_Buff );

	if(Destinations_Full_Num == Last_Node){
		Route_End_Complete_Data = 1;
		DRV_Complete_Data = DRV_Complete;
		}

	RFID_Read_Complete_Data = 1;
	Event_Send_Flag = 1;
	DRV_Complete_Count = 0;
	DRV_Complete_Data = ET_Complete;
}

void AGV_DRV_Event_Send(unsigned int Act)
{
	unsigned char Count = 0;

	Count = Int_To_HexAscii_Num(AGV_Event_Req_Buff, Event_Send_Header_Offset, 10, 4); // 10 - > 11

	AGV_Event_Req_Buff[Count++] = 'E';
	AGV_Event_Req_Buff[Count++] = 'S';

	Count = Int_To_Ascii_Num(AGV_Event_Req_Buff, Count, Act, 2);

	Count = Long_To_HexAscii_Num(AGV_Event_Req_Buff, Count, Last_Node , 6);  // 6 -> 7

	CheckSum_Generation_Func((unsigned char *)AGV_Event_Req_Buff, Count);

	SCIC_TX_String(AGV_Event_Req_Buff);
}

int16_t Lift_Enc_Count = 0;

void Lift_Up_GPIO(void)
{
	if ( Motor_Control_Status == MOTOR_STATUS_STOP )
	{
		AGVMotionData.Drive = PIN_Moving;

		// Motor_Relay_ON();

		LIFT_POW_ON();

		DELAY_MS(200);
		//DELAY_MS(1000);

		LIFT_OUT_START_PIN = 1;             // SERVO ON
		LIFT_OUT_DIR_PIN = 0;             	// DIR CCW
		LIFT_OUT_BRK_PIN = 1;            	// BREAK OFF
		LIFT_OUT_ADDR_WRITE;

		DELAY_MS(200);

		if ( Lift_Init_Flag == 1 ){
			Motor_DA_Buff[DACB] = LIFT_SPEED_MIN_DA_VAL;
		}
		else {
			if ( Lift_DA_Value < LIFT_SPEED_MIN_DA_VAL ) Lift_DA_Value = LIFT_SPEED_MIN_DA_VAL;
			else if ( Lift_DA_Value > LIFT_SPEED_MAX_DA_VAL ) Lift_DA_Value = LIFT_SPEED_MAX_DA_VAL;
			Motor_DA_Buff[DACB] = Lift_DA_Value;
		}

		AD5754_DA_Out_Function((unsigned int *)Motor_DA_Buff, 4);

		// Acquire_State = 1;
		// Action_Count = 200; // 1 = 50mS
		// AGVMotionData.PIN_State = Lift_Up;

		Lift_Action = Lift_Up_Action;
		Lift_Error_Cnt = 0;
		}
}

void Lift_Down_GPIO(void)
{
	if ( Motor_Control_Status == MOTOR_STATUS_STOP ){
		AGVMotionData.Drive = PIN_Moving;

		// Motor_Relay_ON();

		LIFT_POW_ON();

		DELAY_MS(200);

		LIFT_OUT_START_PIN = 1;             // SERVO ON
		LIFT_OUT_DIR_PIN = 1;             	// DIR CW
		LIFT_OUT_BRK_PIN = 1;           	// BREAK OFF
		LIFT_OUT_ADDR_WRITE;

		DELAY_MS(200);

		if ( Lift_Init_Flag == 1 ){
			Motor_DA_Buff[DACB] = LIFT_SPEED_MIN_DA_VAL;
		}
		else {
			if ( Lift_DA_Value < LIFT_SPEED_MIN_DA_VAL ) Lift_DA_Value = LIFT_SPEED_MIN_DA_VAL;
			else if ( Lift_DA_Value > LIFT_SPEED_MAX_DA_VAL ) Lift_DA_Value = LIFT_SPEED_MAX_DA_VAL;
			Motor_DA_Buff[DACB] = Lift_DA_Value;
		}

		AD5754_DA_Out_Function((unsigned int *)Motor_DA_Buff, 4);

		// Action_Count = 200;			// 1 = 50mS
		// AGVMotionData.PIN_State = Lift_Down;
		
		Lift_Action = Lift_Down_Action;
		Lift_Error_Cnt = 0;
		}
}

void Lift_Stop_GPIO(void)
{
	Register_Buff[49] = Lift_Error_Cnt;
	Motor_DA_Buff[DACB] = Lift_Error_Cnt = 0;

	AD5754_DA_Out_Function((unsigned int *)Motor_DA_Buff, 4);

	DELAY_MS(200);

	LIFT_OUT_START_PIN = 0;             // SERVO OFF
	LIFT_OUT_DIR_PIN = 0;             	// DIR CCW
	LIFT_OUT_BRK_PIN = 0;           	// BREAK ON
	LIFT_OUT_ADDR_WRITE;

	Key_Event_Flag = 0;

	// Action_Count = 0;
	Lift_Action = Lift_Wait;
	// AGVMotionData.PIN_State = Lift_Down;
}

void LCD_Buff_Clear(void)
{
	unsigned int i;
	unsigned char Count = 0;

	for(i = 0; i < 20; i++)Coil_Buff[i] = 0;
	for(i = 0; i < 130; i++)Register_Buff[i] = 0;
	Register_Buff[0] = 1;
	Register_Buff[Driving_Status_Addr] = 0;
	Register_Buff[AGV_No_Buff] = 0;
	//Register_Buff[59] = Manual_Operate;

	OPERATE_MODE = Auto_Operate;
	Register_Buff[59] = OPERATE_MODE;

	Register_Buff[124] = ( FIRMWARE_VERSION[0] << 8 ) | FIRMWARE_VERSION[1];
	Register_Buff[125] = ( FIRMWARE_VERSION[2] << 8 ) | FIRMWARE_VERSION[3];
	Register_Buff[126] = ( FIRMWARE_VERSION[4] << 8 );

	for(i = 0; i < 11; i++){
		R2100_Foward_Buff[i] = 0;
		R2100_Backward_Buff[i] = 0;
		}

	for(i = 0; i < 10; i++){
		Dir_Setting_Frame.Buff[i] = 0;
		}

	for ( i = 0; i < 30; i++) AGV_Event_Req_Buff[i] = '0';

	AGV_Event_Req_Buff[Count++] = STX;
	AGV_Event_Req_Buff[Count++] = '0';

	Count = Int_To_HexAscii_Num(AGV_Event_Req_Buff, Count, AGV_Number, 2);
	Count = Long_To_HexAscii_Num(AGV_Event_Req_Buff, Count, 0x123456, 6);
}

void AGV_Promptly_Stop(void)
{
	AGV_Stop_Function();
	Direct_Stop_Status = 1;
}

void Set_Mark_Stop(void)
{
	Forward_Mark_Stop_Status = 1;
}

void Set_MarkStopAct(unsigned char MotorAct, unsigned char LiftAct, unsigned char MarkSen)
{
	StopToMotAct = MotorAct;    // 0 : Stop, Backward : 1, Forward : 2
	StopToLiftAct = LiftAct;    // 0 : Non, LiftDown: 1, LiftUp  : 2

	StopMarkSen = MarkSen;
	Forward_Mark_Act_Status = 1;
}

unsigned char Err_RST = 0;
void Lift_Action_Start(void)
{
	// unsigned int Motor_DA_Data[4] = {0,0,0,0};

	switch(Lift_Action){
		case Lift_Up_Action :

			if ( ( Key_Inp.bit.KeyLiftUP_IN == 0 ) && ( Key_Inp.bit.KeyLiftDN_IN == 1 ) ){


				Lift_Stop_GPIO();

				Lift_Enc_Count = TIM2->CNT;
				Lift_Init_Flag = 0;

				AGVMotionData.Drive = AGV_Wait;
				Register_Buff[Attach_Status] = AGVMotionData.PIN_State = Lift_Up;


				if(DRV_Complete_Flag == 1)AGV_DRV_Complete_Event_Send(Last_JobID);

				DELAY_MS(500);

				if ( Pallet_Detect_Enable == 1 ){
					if ( ( FRONT_CART_IN == 1 ) && ( REAR_CART_IN == 1 ) ){
						Pallet_Detect_Err_Flag = 0;
						if(Lift_Job_Comp == 1 ) AGV_Lift_Event_Send(AGVMotionData.PIN_State);
					}
					else {
						Pallet_Detect_Err_Flag = 1;
						Lift_Action = Lift_Up_Error;
						}
					}
				else {
					Pallet_Detect_Err_Flag = 0;
					if(Lift_Job_Comp == 1 ) AGV_Lift_Event_Send(AGVMotionData.PIN_State);
					}

				Lift_Job_Comp = 0;
				}
			break;

		case Lift_Down_Action :

			if ( ( Key_Inp.bit.KeyLiftUP_IN == 1 ) && ( Key_Inp.bit.KeyLiftDN_IN == 0 ) ){

				Lift_Stop_GPIO();
				Lift_Init_Flag = 0;

				Lift_Enc_Count = TIM2->CNT = 0;

				AGVMotionData.Drive = AGV_Wait;
				Register_Buff[Attach_Status] = AGVMotionData.PIN_State = Lift_Down;

				DELAY_MS(50);

				if(DRV_Complete_Flag == 1)AGV_DRV_Complete_Event_Send(Last_JobID);

				DELAY_MS(500);

				if(Lift_Job_Comp == 1)AGV_Lift_Event_Send(AGVMotionData.PIN_State);

				Lift_Job_Comp = 0;
				}
			break;

		case Lift_Up_Error :
			break;

		case Lift_Down_Error :
			break;

		default :
			break;
		}

	// if(Action_Count > 0)Action_Count--;
}

void Course_Reset(void)
{
	Direct_Stop_Status = 0;
	Forward_Mark_Stop_Status = 0;
	Guide_Off_Status = 0;
	Lift_Interface_Sequence_Status = 0;
	Left_Spin_Function_Sequence = 0;
	Right_Spin_Function_Sequence = 0;
	Stop_Action_Function_Sequence = 0;
	Lift_Action_To_Moving_SEQ_Num = 0;

	Right_Mag_Count = Left_Mag_Count = 0;

	Charge_Seq_Num = DisCharge_Seq_Num = 0;
	// AGV_GoolIN = 0;

	Pause_Num = Pause_Flag = 0;
	Guide_Read_Flag = 0;

	NEXT_Node_Num = 0;

	Old_Tag_Num = Distance_Sum_Buff = 0;
	Spin_Action_Start = 0;

	Course_Driving_Status = 0;
}

void AGV_Stop_Function(void)
{
	Course_Reset();

	LED_Color_Select(18000,18000,0);
	LED_Flicker_Flag = 0;

	SOUND_OUT_PIN_ALL = 0;
	SOUND_OUT_ADDR_WRITE;

	Moving_Speed_Set(0);
	AGVMotionData.Drive = AGV_Stop;
}

unsigned char Direct_Stop_Flag = 0;
void Direct_Stop(void)
{
	//P_Factor = ((float)100 / 10) * SCALING_FACTOR;
	//I_Factor = ((float)200 / 10) * SCALING_FACTOR;
	//D_Factor = ((float)10 / 10) * SCALING_FACTOR;

	Moving_Speed_Set(0);
	Direct_Stop_Flag = 1;
	AGVMotionData.Drive = AGV_Stop;
}

void Move_SEQ_Start(void)
{
	Old_CUR_Node_Num = CUR_Node_Num;
	Torque_OnOff = 1;

	Moving_Speed_Set(AGV_Speed_Buff[0]);

	DELAY_MS(10);

	Forward_GPIO();
}

void Key_Input(void)
{
	unsigned int Motor_DA_Data[4]; //key, Buff,
	// PORT_BYTE_FRAME IO_16Buff;

	KeyIN_Buff.all = Key_Inp.all & ~Old_KeyIN.all;   //  0 -> 1   ----> 1
	Old_KeyIN.all = Key_Inp.all;

	if( KeyIN_Buff.bit.KeyStart ){
		if(AGVMotionData.Drive == AGV_Wait){
			if(Old_DRV_Buff == Forward_DRV){
				Moving_Speed_Set(AGV_Speed_Buff[0]);
				Forward_GPIO();
				}
			else if(Old_DRV_Buff == Backward_DRV){
				Moving_Speed_Set(AGV_Speed_Buff[0]);
				Forward_GPIO();
				}
			else{
				Moving_Speed_Set(AGV_Speed_Buff[0]);
				Forward_GPIO();
				}
			}
		}

	if( KeyIN_Buff.bit.KeyReset ){
		//AGV_Stop_Function();
		if( ( Motor_Control_Status == MOTOR_STATUS_STOP ) || ( AGVMotionData.Drive == AGV_Stop ) ){
			
			Err_Event_Debug_Flag = 0;
			Error_Continue_Flag = 0;
			Error_Continue_Count = 0;
			Error_1_min_Check_Count = 0;

			Mot_Alamr_Cnt[0] = 0;
			Mot_Alamr_Cnt[1] = 0;

			Motor_Relay_OFF();

			DELAY_MS(1000);

			Motor_Relay_ON();

			Course_Reset();

			Key_Input_Stop = PreDriving_State_By_Key = 0;

			// Acquire_Err_Scan_Start = Acquire_Count = Acquire_Err_Scan = 0;

			// if(ACS_Car_Tagging_Flag == 1)ACS_Car_Tagging_Flag = ACS_Car_Tagging_Err_Count = Tagging_Err_Check_Count = 0;

			LED_Color_Select(3500,3500,0);
			LED_Flicker_Flag = 0;
			Flicker_Buff = 0;

			SOUND_OUT_PIN_ALL = 0;
			SOUND_OUT_ADDR_WRITE;

			TIM_Out_Port.all = 0;
			TIM_OUT_DATA_WRITE(TIM_Out_Port.all);

			PIO_Out_Port.byte.L = 0;
			ADDR16(PORTH) = PIO_Out_Port.byte.L;

			Pallet_Detect_Err_Flag = 0;
			Brake_Release_Err_Flag = 0;
			Distance_Sum_Buff = 0;    //
			Line_Err_Include_Flag = Error_Flag = Distance_Sum_Buff = 0;
			AGVMotionData.Error_Code = AGV_Wait;
			AGVMotionData.Drive = AGV_Wait;
			Lift_Action = Lift_Wait;
			Old_ID = 0;

			LIFT_OUT_PIN_ALL = 0;// Lift_Out_Port &= ~0x03;
			LIFT_OUT_ADDR_WRITE; // ADDR16(PORTF) = Lift_Out_Port;

			MT1_PORT_START_PIN = 0;
			MT1_PORT_DIR_CW_PIN = 0;
			MT1_PORT_ADDR_WRITE;

			MT3_PORT_START_PIN = 0;
			MT3_PORT_DIR_CW_PIN = 0;
			MT3_PORT_ADDR_WRITE;


			Motor_DA_Data[0] = 0;
			Motor_DA_Data[2] = 0;
			AD5754_DA_Out_Function(Motor_DA_Data, 4); // ????

			DELAY_MS(50);

			Lift_Action = Lift_Wait;

			Spinturn_Err_Flag = 0;
			Old_Tag_Num = 0;
			Register_Buff[121] = Register_Buff[122] = 0;
			Spin_Action_Start = 0;

			// Safety_Sensor_Event = 0;
			Charge_Error_Count = Charge_Check_Count = 0;

			Front_Safety_Event = Rear_Safety_Event = 0;
			}
		}

	if( KeyIN_Buff.bit.KeyStop ){
		if((AGVMotionData.Drive == Forward_DRV)||(AGVMotionData.Drive == Backward_DRV)){
			PreDriving_State_By_Key = AGVMotionData.Drive;
			Key_Input_Stop = 1;
			}

		if ( ( Lift_Action == Lift_Up_Action ) || ( Lift_Action == Lift_Down_Action ) ){
			Lift_Stop_GPIO();
			}

		PIO_Out_Port.byte.L = 0;
		ADDR16(PORTH) = PIO_Out_Port.byte.L;

		AGV_Stop_Function();
		Manual_Spin_Flag = 1;
		}

	// if ( AGV_Number == 15 ){

	// }
	// else {
	// 	if ( KeyIN_Buff.bit.KeyAUTO == 1 && KeyIN_Buff.bit.KeyMANU == 0 ){
	// 		OPERATE_MODE = Auto_Operate;
	// 		AGV_DRV_Event_Send(31);
	// 	}
	// 	else if ( KeyIN_Buff.bit.KeyAUTO == 0 && KeyIN_Buff.bit.KeyMANU == 1 ){
	// 		OPERATE_MODE = Manual_Operate;
	// 		AGV_DRV_Event_Send(30);
	// 	}
	// }
}

MANUAL_LIFT_FRAME LiftSW_IN, LiftSW_Old;
uint8_t Key_Event = 0, Key_Event_Flag = 0;

void Lift_Manual_Action ( void )
{
	LiftSW_IN.all = 0;
	LiftSW_IN.bit.MANUAL_UP = HAL_GPIO_ReadPin(SPARE_IN3_GPIO_Port, SPARE_IN3_Pin);
	LiftSW_IN.bit.MANUAL_DN = HAL_GPIO_ReadPin(SPARE_IN4_GPIO_Port, SPARE_IN4_Pin);

	if ( LiftSW_IN.all != LiftSW_Old.all ){
		if ( LiftSW_IN.all == 0x01 ) Key_Event = LIFT_KEY_UP_EVENT;
		else if ( LiftSW_IN.all == 0x02 ) Key_Event = LIFT_KEY_DOWN_EVENT;
		else Key_Event = LIFT_KEY_STOP_EVENT;

		LiftSW_Old.all = LiftSW_IN.all;
	}
	else {
		if ( AGVMotionData.Drive == AGV_Wait ){
			if ( ( Key_Event == LIFT_KEY_UP_EVENT ) && ( Key_Inp.bit.KeyLiftUP_IN == 1 ) ){
				Lift_Up_GPIO();
				Key_Event_Flag = 1;
			}
			else if ( ( Key_Event == LIFT_KEY_DOWN_EVENT ) && ( Key_Inp.bit.KeyLiftDN_IN == 1 ) ){
				Lift_Down_GPIO();
				Key_Event_Flag = 1;
			}
			else {
				Key_Event = LIFT_KEY_NONE_EVENT;
			}
		}
		else if ( AGVMotionData.Drive == PIN_Moving && Key_Event_Flag == 1 ){
			if ( ( Key_Event == LIFT_KEY_UP_EVENT ) &&
				( Key_Inp.bit.KeyLiftUP_IN == 0 ) && ( Key_Inp.bit.KeyLiftDN_IN == 1 ) ){

				Lift_Enc_Count = TIM2->CNT;

				Lift_Stop_GPIO();

				Lift_Init_Flag = 0;

				DELAY_MS(300);

				Key_Event_Flag = 0;
				Key_Event = LIFT_KEY_NONE_EVENT;
				AGVMotionData.Drive = AGV_Wait;
				Register_Buff[Attach_Status] = AGVMotionData.PIN_State = Lift_Up;
			}
			else if ( ( Key_Event == LIFT_KEY_DOWN_EVENT ) &&
				( Key_Inp.bit.KeyLiftUP_IN == 1 ) && ( Key_Inp.bit.KeyLiftDN_IN == 0 ) ){

				Lift_Stop_GPIO();

				Lift_Init_Flag = 0;

				Lift_Enc_Count = TIM2->CNT = 0;

				DELAY_MS(300);

				Key_Event_Flag = 0;
				Key_Event = LIFT_KEY_NONE_EVENT;
				AGVMotionData.Drive = AGV_Wait;
				Register_Buff[Attach_Status] = AGVMotionData.PIN_State = Lift_Down;
			}
			else if ( Key_Event ==  LIFT_KEY_STOP_EVENT ){
				Lift_Stop_GPIO();

				DELAY_MS(300);

				Key_Event = LIFT_KEY_NONE_EVENT;
				Key_Event_Flag = 0;
				AGVMotionData.Drive = AGV_Wait;
			}
		}
		else {
			Key_Event = LIFT_KEY_NONE_EVENT;
		}
	}
}

void LCD_Display_Var_Assembly(void)
{
	// PORT_IO_FRAME IO_Buff;
	PORT_BYTE_FRAME IO_16Buff;

	Register_Buff[59] = OPERATE_MODE;

	IO_16Buff.all = GPIO_Read_Port(GPIOJ);
	PIO_IN_PORT.all = IO_16Buff.byte.H;
	TIM_IN_PORT.all = 0;
	TIM_IN_PORT.bit.IO1_1 = HAL_GPIO_ReadPin(TIM1_OUT1_GPIO_Port, TIM1_OUT1_Pin); 	// C10-11
	TIM_IN_PORT.bit.IO1_2 = HAL_GPIO_ReadPin(TIM1_OUT2_GPIO_Port, TIM1_OUT2_Pin);	// C10-12
	TIM_IN_PORT.bit.IO2_1 = HAL_GPIO_ReadPin(TIM2_OUT1_GPIO_Port, TIM2_OUT1_Pin);	// C10-15
	TIM_IN_PORT.bit.IO2_2 = HAL_GPIO_ReadPin(TIM2_OUT2_GPIO_Port, TIM2_OUT2_Pin);	// C10-16

	Register_Buff[60] = TIM_IN_PORT.all;

	Register_Buff[61] = TIM_Out_Port.all;

	Register_Buff[62] = Remote_In.all;

	IO_16Buff.byte.L = PIO_IN_PORT.all;
	IO_16Buff.byte.H = PIO_Out_Port.byte.L;

	Register_Buff[63] = IO_16Buff.all;

	// Register_Buff[64] = IO_16Buff.all;

	ETC_IO.bit.F_CART = FCarriage.status;  // HAL_GPIO_ReadPin ( ATTACH1_UP_IN_GPIO_Port, ATTACH1_UP_IN_Pin );
	ETC_IO.bit.R_CART = RCarriage.status;  // HAL_GPIO_ReadPin ( ATTACH2_UP_IN_GPIO_Port, ATTACH2_UP_IN_Pin ); // HAL_GPIO_ReadPin ( ATTACH1_DOWN_IN_GPIO_Port, ATTACH1_DOWN_IN_Pin );
	IO_16Buff.byte.L = Sound_Out_Port.all;
	IO_16Buff.byte.H = ETC_IO.all;
	Register_Buff[65] = IO_16Buff.all;

	Key_Inp.all = 0;
	IO_16Buff.all = GPIO_Read_Port(KEY_IN_GPIO_Port);
	Key_Inp.byte.L = IO_16Buff.byte.H;
	Key_Inp.bit.KeyBUMP = HAL_GPIO_ReadPin(SPARE_IN9_GPIO_Port, SPARE_IN9_Pin);
	Key_Inp.bit.KeyRLEMG = HAL_GPIO_ReadPin(SPARE_IN1_GPIO_Port, SPARE_IN1_Pin);
	Key_Inp.bit.KeyRREMG = HAL_GPIO_ReadPin(SPARE_IN2_GPIO_Port, SPARE_IN2_Pin);
	Key_Inp.bit.KeyLiftUP_SW = HAL_GPIO_ReadPin(SPARE_IN3_GPIO_Port, SPARE_IN3_Pin);
	Key_Inp.bit.KeyLiftDN_SW = HAL_GPIO_ReadPin(SPARE_IN4_GPIO_Port, SPARE_IN4_Pin);
	Key_Inp.bit.KeyLiftUP_IN = HAL_GPIO_ReadPin ( ATTACH1_DOWN_IN_GPIO_Port, ATTACH1_DOWN_IN_Pin ); // HAL_GPIO_ReadPin ( ATTACH2_UP_IN_GPIO_Port, ATTACH2_UP_IN_Pin );
	Key_Inp.bit.KeyLiftDN_IN = HAL_GPIO_ReadPin ( ATTACH2_DOWN_IN_GPIO_Port, ATTACH2_DOWN_IN_Pin );

	Register_Buff[66] = Key_Inp.all;

	Register_Buff[67] = MOT_ALARM.all;

	Register_Buff[113] = AGVMotionData.PIN_State;

	Register_Buff[115] = BMS_Summary_Data.BAT_Volt.all;
	Register_Buff[116] = BMS_Summary_Data.BAT_Current.all;
	Register_Buff[117] = BMS_Summary_Data.BAT_SOC.all;
	Register_Buff[118] = BMS_Summary_Data.BAT_Use_Cycle.all;
	Register_Buff[119] = BMS_Status_Data.Status1.all;
	Register_Buff[120] = BMS_Status_Data.Status2.all;

	IO_16Buff.byte.L = ACS_Com_Check;
	IO_16Buff.byte.H = Pallet_Detect_Enable;
	Register_Buff[123] = IO_16Buff.all;

	Register_Buff[130] = BMS_Cell_Volt_Data.Cell1.all;
	Register_Buff[131] = BMS_Cell_Volt_Data.Cell2.all;
	Register_Buff[132] = BMS_Cell_Volt_Data.Cell3.all;
	Register_Buff[133] = BMS_Cell_Volt_Data.Cell4.all;
	Register_Buff[134] = BMS_Cell_Volt_Data.Cell5.all;
	Register_Buff[135] = BMS_Cell_Volt_Data.Cell6.all;
	Register_Buff[136] = BMS_Cell_Volt_Data.Cell7.all;
	Register_Buff[137] = BMS_Cell_Volt_Data.Cell8.all;
	Register_Buff[138] = BMS_Cell_Volt_Data.Cell9.all;
	Register_Buff[139] = BMS_Cell_Volt_Data.Cell10.all;
	Register_Buff[140] = BMS_Cell_Volt_Data.Cell11.all;
	Register_Buff[141] = BMS_Cell_Volt_Data.Cell12.all;
	Register_Buff[142] = BMS_Cell_Volt_Data.Cell13.all;
	Register_Buff[143] = BMS_Temp2_Data.Temp[0];
	Register_Buff[144] = BMS_Temp2_Data.Temp[1];
	Register_Buff[145] = BMS_Temp2_Data.Temp[2];
	Register_Buff[146] = BMS_Temp2_Data.Temp[3];

	if ( AGVMotionData.Drive == AGV_Wait ){
		if ( Key_Inp.bit.KeyMANU == 0 ){
			if ( OPERATE_MODE != Auto_Operate ){
				OPERATE_MODE = Auto_Operate;
				AGV_DRV_Event_Send(31);
				}
			}
		else if ( Key_Inp.bit.KeyMANU == 1 ){
			if ( OPERATE_MODE != Manual_Operate ){
				OPERATE_MODE = Manual_Operate;
				AGV_DRV_Event_Send(30);
				}
			}
		}
}


// unsigned char StopToStart_Num = 0, Left_Spin_Seq_State_Buff =0, Right_Spin_Seq_State_Buff = 0, Dir_State = 0;
// void StopToStart_SEQ(void)
// {
// 	if(StopToStart_Num > 0){
// 		if(StopToStart_Num == 1){
// 			Course_Driving_Status = 1;
// 			Dir_State = AGVMotionData.Drive;
// 			if(Course_Driving_Status == 1){
// 				if(Left_Spin_Function_Sequence != 0)Left_Spin_Seq_State_Buff = Left_Spin_Function_Sequence;
// 				if(Right_Spin_Function_Sequence != 0)Right_Spin_Seq_State_Buff = Right_Spin_Function_Sequence;
// 				}

// 			Direct_Stop();
// 			StopToStart_Num = 2;
// 			}
// 		else if(StopToStart_Num == 2){
// 			if(AGVMotionData.Drive == AGV_Wait){
// 				Spot_Line_State = QR_Tag;
// 				StopToStart_Num = 0;

// 				if(Dir_State == Forward_DRV)Forward_GPIO();
// 				else if(Dir_State == Backward_DRV)Backward_GPIO();
// 				else Forward_GPIO();

// 				if(Course_Driving_Status == 1){
// 					if(Left_Spin_Function_Sequence != 0)Left_Spin_Function_Sequence = Left_Spin_Seq_State_Buff;
// 					if(Right_Spin_Function_Sequence != 0)Right_Spin_Function_Sequence = Right_Spin_Seq_State_Buff;
// 				}
// 				Global_Guide = Guide_Old_Buff = 17;
// 				Course_Driving_Status = 0;
// 				}
// 			}
// 		}
// }

// Com2_Data[Com2buffsize], Com2Data_Count = 0;
// unsigned int CheckSum_Data = 0, CheckSum_Check_Buff = 0;
// unsigned char CMD_Buff[2];
unsigned int Com2Data_Count_Buff = 0;
unsigned char During_Stop_Action = 0;
unsigned char Index_Err_Check = 0;
int16_t Com_Change_Count = 0;

uint8_t SCIC_Communication_Control(void)
{
	unsigned int com_char = 0;//Test_Buff, Switch_Num
	unsigned char i = 0;//, com[20];
	unsigned int CheckSum_Data = 0, CheckSum_Check_Buff = 0;
	unsigned char CMD_Buff[2];

	// unsigned int AGV_ID_Buff = 0, Body_Length = 0;
	// unsigned long Message_ID_Buff = 0;

	unsigned char In_Buffer_Count = Send_Frame_HeaderOffset;//, Char_Cal_Buff = 0;
	//unsigned int Int_Cal_Buff = 0;
	unsigned int Index_Num_Buff = 0;
	unsigned long JobID_Buff = 0;//, Cal_Buff = 0;
	unsigned char Err_Flag = 1;
	unsigned long Number_of_NodeID = 0;
	unsigned int Action_Num = 0;
	//PORT_BYTE_FRAME IO_16Buff;

	com_char = SCIC_Read_Char();

	while(com_char != 0){

		// SCIF_TX_Char(com_char);

		switch(com_char){
			case STX : Com2Data_Count = 0; break;

			case ETX :

				// Com_Check_Count = 0;
				//Register_Buff[123] ^= 1;
				ACS_Com_Check ^= 1;
				// Com2Data_Count_Buff = Com2Data_Count;
				CheckSum_Data = GetCRC16((char *)Com2_Data, Com2Data_Count - 4, 0);               // CRC16 Type = 2,
				CheckSum_Check_Buff = AsciiHex_To_Binary_Num((int8_t *)Com2_Data, (Com2Data_Count - 4), 4); // CRC16 4Byte,

				ACS_Response_Buff[0] = STX;

				for(i = 0; i < Receive_Frame_HeaderOffset; i++)ACS_Response_Buff[i + 1] = Com2_Data[i];

				// if ( CheckSum_Data == CheckSum_Check_Buff ){
				CMD_Buff[0] = Com2_Data[Receive_Frame_HeaderOffset];
				CMD_Buff[1] = Com2_Data[Receive_Frame_HeaderOffset + 1];

				/* ?†µ?‹  ?””ë²„ê¹…?š© ì½”ë“œ */
				if ( Com_Change_Count > 10000 ) Com_Change_Count = 0;
				else Com_Change_Count++;

				Register_Buff[150] = Com_Change_Count;

				// for ( i = 0; i<9; i++){
				// 	IO_16Buff.all = 0;
				// 	IO_16Buff.byte.L = Com2_Data[Receive_Frame_HeaderOffset + i * 2];
				// 	IO_16Buff.byte.H = Com2_Data[Receive_Frame_HeaderOffset  + i * 2 + 1];
				// 	Register_Buff[151 + i] = IO_16Buff.all;
				// 	}

				/* ?†µ?‹  ?””ë²„ê¹…?š© ì½”ë“œ */

				// if( OPERATE_MODE == Manual_Operate){
					if((CMD_Buff[0] == 'G') && (CMD_Buff[1] == 'Q')){

						In_Buffer_Count = Send_Frame_HeaderOffset;
						ACS_Response_Buff[In_Buffer_Count++] = 'g';
						ACS_Response_Buff[In_Buffer_Count++] = 'q';

						if((Spot_Line_State == QR_Tag) && (On_Tag_Falg == 1)){
							if((Register_Buff[73] >= 3150) || (Register_Buff[73] < 450))GQ_Surface_Input = 1;
							else if((Register_Buff[73] >= 450) && (Register_Buff[73] < 1350))GQ_Surface_Input = 2;
							else if((Register_Buff[73] >= 1350) && (Register_Buff[73] < 2250))GQ_Surface_Input = 3;
							else if((Register_Buff[73] >= 2250) && (Register_Buff[73] < 3150))GQ_Surface_Input = 4;
							}

						//Battery
						In_Buffer_Count = Int_To_HexAscii_Num(ACS_Response_Buff, In_Buffer_Count, BMS_Summary_Data.BAT_SOC.all, 2 );

						//Alive
						if(AGVMotionData.Drive == AGV_Wait) ACS_Response_Buff[In_Buffer_Count++] = '2';
						else if (AGVMotionData.Drive == Pause_Wait ) ACS_Response_Buff[In_Buffer_Count++] = 'P';
						else if ((AGVMotionData.Drive == AGV_CHARGING) || (AGVMotionData.Drive == Charge_SEQ) ||
							(AGVMotionData.Drive == Charge_Stop_SEQ)) ACS_Response_Buff[In_Buffer_Count++] = 'C';
						else{
							if((AGVMotionData.Drive == AGV_EMR)||(AGVMotionData.Drive == AGV_Line_Error)
								||(AGVMotionData.Drive == AGV_Bump_Err)||(AGVMotionData.Drive == AGV_Near_F_Err)
								||(AGVMotionData.Drive == AGV_Near_B_Err)||(AGVMotionData.Drive == AGV_Direction_Err)
								||(AGVMotionData.Drive == AGV_Motor_ALR)||(AGVMotionData.Drive == Brake_Release_Err)
								||(AGVMotionData.Drive == Pallet_Detect_Err)||(AGVMotionData.Drive == AGV_Chargine_Err)
								||(AGVMotionData.Drive == Lift_Timeout_Err)||(AGVMotionData.Drive == Lift_Motor_ALR)
								||(AGVMotionData.Drive == AGV_Carrier_Err)){
								ACS_Response_Buff[In_Buffer_Count++] = 'F';
								}
							else ACS_Response_Buff[In_Buffer_Count++] = '1';
							}

						if( TIM_Out_Port.bit4.T1 < 10 ) ACS_Response_Buff[In_Buffer_Count++] = TIM_Out_Port.bit4.T1 + '0';
						else ACS_Response_Buff[In_Buffer_Count++] = TIM_Out_Port.bit4.T1 - 10 + 'A';

						if( TIM_Out_Port.bit4.T2 < 10 ) ACS_Response_Buff[In_Buffer_Count++] = TIM_Out_Port.bit4.T2 + '0';
						else ACS_Response_Buff[In_Buffer_Count++] = TIM_Out_Port.bit4.T2 - 10 + 'A';

						ACS_Response_Buff[In_Buffer_Count++] = AGVMotionData.PIN_State + 0x30;

						// Last_Node = 0x1234;
						In_Buffer_Count = Long_To_HexAscii_Num( ACS_Response_Buff, In_Buffer_Count, Last_Node, 6); // 6 -> 7

						//Last_CarID = 0x12345;
						In_Buffer_Count = Long_To_HexAscii_Num(ACS_Response_Buff, In_Buffer_Count, Last_CarID2, 8);

						//Last_JobID = 0x12345;
						In_Buffer_Count = Long_To_HexAscii_Num(ACS_Response_Buff, In_Buffer_Count, Last_JobID, 5);

						if( OPERATE_MODE == Auto_Operate)ACS_Response_Buff[In_Buffer_Count++] = '1';
						else ACS_Response_Buff[In_Buffer_Count++] = '0';

						ACS_Response_Buff[In_Buffer_Count++] = GQ_Surface_Input + 0x30;

						ACS_Response_Buff[In_Buffer_Count++] = FCarriage.status + 0x30;

						ACS_Response_Buff[In_Buffer_Count++] = RCarriage.status + 0x30;

						// SCIC_RecBuff_Clear();

						Int_To_HexAscii_Num( ACS_Response_Buff, Send_Frame_BodyCountOffset, ( In_Buffer_Count - Send_Frame_HeaderOffset ), 4 );

						CheckSum_Generation_Func((unsigned char *)ACS_Response_Buff, In_Buffer_Count );

						SCIC_TX_String( ACS_Response_Buff );

						Com2Data_Count = 0;
						return 1;
						}
					// }

				else if( OPERATE_MODE == Auto_Operate){
					if(Event_Send_Flag != 0){
						if((CMD_Buff[0] == 'e') && (CMD_Buff[1] == 'd')){
							if(Com2_Data[15] == 'S'){
								// Register_Buff[122] = Route_End_Complete_Data = Old_JobID = 0;
								Register_Buff[122] = Route_End_Complete_Data = 0;
								}
							else{
								AGV_DRV_Complete_Event_Send(Last_JobID);
								}
							}

						if((CMD_Buff[0] == 'e') && (CMD_Buff[1] == 'l')){
							if(Com2_Data[15] == 'S'){
								Register_Buff[121] = Lift_Complete_Data = 0;
								}
							else{
								AGV_Lift_Event_Send(AGVMotionData.PIN_State);
								}
							}

						if((CMD_Buff[0] == 'e') && (CMD_Buff[1] == 'g')){
							if(Com2_Data[15] == 'S'){
								Charge_Action_Complete_Data = 0;
								}
							else {
								AGV_Charge_Complete_Event_Send();
								}
							}

						if((CMD_Buff[0] == 'e') && (CMD_Buff[1] == 'c')){
							if(Com2_Data[15] == 'S'){
								// Register_Buff[121] = Job_Cancel_Data = Old_JobID = 0;
								Job_Cancel_Data = 0;
								}
							else{
								AGV_JobCancle_Event_Send(Last_Node);
								}
							}

						if( ( Route_End_Complete_Data == 0) && ( Lift_Complete_Data == 0 ) &&
	 					 	( Charge_Action_Complete_Data == 0) && ( Job_Cancel_Data == 0 ) ){
							Event_Send_Flag = DRV_Complete_Flag = DRV_Complete_Count = 0;
							}
						}

					else if(CMD_Buff[0] == 'G'){
						if(CMD_Buff[1] == 'B'){
							In_Buffer_Count = Send_Frame_HeaderOffset;
							ACS_Response_Buff[In_Buffer_Count++] = 'g';
							ACS_Response_Buff[In_Buffer_Count++] = 'b';

							In_Buffer_Count = Int_To_HexAscii_Num(ACS_Response_Buff, In_Buffer_Count, BMS_Summary_Data.BAT_SOC.all, 2 );
							}

						else if(CMD_Buff[1] == 'S'){
							In_Buffer_Count = Send_Frame_HeaderOffset;
							ACS_Response_Buff[In_Buffer_Count++] = 'g';
							ACS_Response_Buff[In_Buffer_Count++] = 's';

							if(Com2_Data[15] == 'H'){
								if( TIM_Out_Port.bit4.T1 < 10 ) ACS_Response_Buff[In_Buffer_Count++] = TIM_Out_Port.bit4.T1 + '0';
								else ACS_Response_Buff[In_Buffer_Count++] = TIM_Out_Port.bit4.T1 - 10 + 'A';
								}

							else if(Com2_Data[15] == 'T'){
								if( TIM_Out_Port.bit4.T2 < 10 ) ACS_Response_Buff[In_Buffer_Count++] = TIM_Out_Port.bit4.T2 + '0';
								else ACS_Response_Buff[In_Buffer_Count++] = TIM_Out_Port.bit4.T2 - 10 + 'A';
								}
							}

						else if(CMD_Buff[1] == 'L'){    //
							In_Buffer_Count = Send_Frame_HeaderOffset;
							ACS_Response_Buff[In_Buffer_Count++] = 'g';
							ACS_Response_Buff[In_Buffer_Count++] = 'l';

							ACS_Response_Buff[In_Buffer_Count++] = AGVMotionData.PIN_State + 0x30;
							}

						else if(CMD_Buff[1] == 'A'){    //Alive
							In_Buffer_Count= Send_Frame_HeaderOffset;
							ACS_Response_Buff[In_Buffer_Count++] = 'g';
							ACS_Response_Buff[In_Buffer_Count++] = 'a';

							if(AGVMotionData.Drive == AGV_Wait) ACS_Response_Buff[In_Buffer_Count++] = '2';
							else if (AGVMotionData.Drive == Pause_Wait ) ACS_Response_Buff[In_Buffer_Count++] = 'P';
							else if ((AGVMotionData.Drive == AGV_CHARGING) || (AGVMotionData.Drive == Charge_SEQ) ||
								(AGVMotionData.Drive == Charge_Stop_SEQ)) ACS_Response_Buff[In_Buffer_Count++] = 'C';
							else{
								if((AGVMotionData.Drive == AGV_EMR)||(AGVMotionData.Drive == AGV_Line_Error)
									||(AGVMotionData.Drive == AGV_Bump_Err)||(AGVMotionData.Drive == AGV_Near_F_Err)
									||(AGVMotionData.Drive == AGV_Near_B_Err)||(AGVMotionData.Drive == AGV_Direction_Err)){
									ACS_Response_Buff[In_Buffer_Count++] = 'F';
									}
								else ACS_Response_Buff[In_Buffer_Count++] = '1';
								}
							}

						else if(CMD_Buff[1] == 'N'){        // Node ID Requeset
							In_Buffer_Count = Send_Frame_HeaderOffset;
							ACS_Response_Buff[In_Buffer_Count++] = 'g';
							ACS_Response_Buff[In_Buffer_Count++] = 'n';

							In_Buffer_Count = Long_To_HexAscii_Num(ACS_Response_Buff, In_Buffer_Count, Last_Node, 6);  // 6 -> 7

							ACS_Response_Buff[In_Buffer_Count++] = Surface_Input + 0x30;
							}

						else if(CMD_Buff[1] == 'T'){        // Carriage ID Request
							In_Buffer_Count = Send_Frame_HeaderOffset;
							ACS_Response_Buff[In_Buffer_Count++] = 'g';
							ACS_Response_Buff[In_Buffer_Count++] = 't';

							//Last_Node = 7548;
							In_Buffer_Count = Long_To_HexAscii_Num(ACS_Response_Buff,In_Buffer_Count, Last_Node, 6); 	// 6 -> 7
							}

						else if(CMD_Buff[1] == 'J'){        // Job ID Request
							In_Buffer_Count = Send_Frame_HeaderOffset;
							ACS_Response_Buff[In_Buffer_Count++] = 'g';
							ACS_Response_Buff[In_Buffer_Count++] = 'j';

							//Last_JobID = 7584;
							In_Buffer_Count = Long_To_HexAscii_Num(ACS_Response_Buff, In_Buffer_Count ,Last_JobID, 5 );
							}

						Int_To_HexAscii_Num( ACS_Response_Buff, Send_Frame_BodyCountOffset, ( In_Buffer_Count - Send_Frame_HeaderOffset ), 4 );

						CheckSum_Generation_Func((unsigned char *)ACS_Response_Buff, In_Buffer_Count );

						SCIC_TX_String(ACS_Response_Buff);
						}

					else if(CMD_Buff[0] == 'A'){
						if((CMD_Buff[1] == 'M') || (CMD_Buff[1] == 'O') || (CMD_Buff[1] == 'E')){   //

							if( (CheckSum_Data == CheckSum_Check_Buff) && ( AGVMotionData.Drive == AGV_Wait ) ){

								Job_Cancel_Stop_Flag = 0;

								if((Register_Buff[122] != 0) || (Register_Buff[121] != 0))Register_Buff[122] = Register_Buff[121] = 0;

								JobID_Buff = AsciiHex_To_Binary_Num( ( int8_t *)Com2_Data, JobID_Frame_Offset, JobID_Len);

								Number_of_NodeID = AsciiHex_To_Binary_Num( (int8_t *)Com2_Data, NumofNode_Offset, Node_Count_Len);

								if(Old_JobID != JobID_Buff){
									Node_Check_Count = 0;
									Index_Err_Check = 0;
									}

								for(i = 0; i < Number_of_NodeID; i++){

									Index_Num_Buff = AsciiHex_To_Binary_Num( ( int8_t *)Com2_Data, Index_Num_Offset + (Frame_Len * i), Index_Num_Len );

									Auto_Dir_Info_Rec_Buff[Node_Check_Count].Data.Node_ID
												= AsciiHex_To_Binary_Num( ( int8_t *)Com2_Data, (NodeID_Num_Offset + (Frame_Len * i)), NodeID_Len );
									Auto_Dir_Info_Rec_Buff[Node_Check_Count].Data.AGV_Direction
												= AsciiHex_To_Binary_Num( ( int8_t *)Com2_Data, (Direction_Offset + (Frame_Len * i)), Direction_Len );
									Auto_Dir_Info_Rec_Buff[Node_Check_Count].Data.Driving_Speed
												= AsciiHex_To_Binary_Num( ( int8_t *)Com2_Data, (Speed_Num_Offset + (Frame_Len * i)), Speed_Len );
									Auto_Dir_Info_Rec_Buff[Node_Check_Count].Data.Turn_Angle
												= AsciiHex_To_Binary_Num( ( int8_t *)Com2_Data, (Turn_Angle_Offset + (Frame_Len * i)), Turn_Angle_Len );
									Auto_Dir_Info_Rec_Buff[Node_Check_Count].Data.Safety_CMD
												= AsciiHex_To_Binary_Num( ( int8_t *)Com2_Data, (Safety_CMD_Offset + (Frame_Len * i)), Safety_CMD_Len );

									if ( Index_Num_Buff == Node_Check_Count ) {
										Node_Check_Count++;
										Index_Err_Check = 0;
										}
									else {
										Index_Err_Check = 1;
										break;
										}
									}

									Err_Flag = 0;
								}


							In_Buffer_Count = Send_Frame_HeaderOffset;
							ACS_Response_Buff[In_Buffer_Count++] = CMD_Buff[0] + 0x20;
							ACS_Response_Buff[In_Buffer_Count++] = CMD_Buff[1] + 0x20;

							if(Err_Flag == 0){
								ACS_Response_Buff[In_Buffer_Count++] = 'S';
								DRV_Complete_Data = DRV_Job_Start;
								//Register_Buff[52] = Old_JobID = JobID_Buff;
								Old_JobID = JobID_Buff;
								Last_JobID = JobID_Buff;
								}
							else{
								Old_JobID = Last_JobID = 0;
								Node_Check_Count = 0;
								ACS_Response_Buff[In_Buffer_Count++] = 'F';
								}
							}


						else if(CMD_Buff[1] == 'L'){
							if((Register_Buff[122] != 0) || (Register_Buff[121] != 0))Register_Buff[122] = Register_Buff[121] = 0;
							In_Buffer_Count = Send_Frame_HeaderOffset;
							ACS_Response_Buff[In_Buffer_Count++] = 'a';
							ACS_Response_Buff[In_Buffer_Count++] = 'l';

							if(Com2_Data[15] == 'U'){
								Lift_Job_Comp = 1;
								Lift_Up_GPIO();
								ACS_Response_Buff[In_Buffer_Count++] = 'S';
								}

							else if(Com2_Data[15] == 'D'){
								Lift_Job_Comp = 1;
								Lift_Down_GPIO();
								ACS_Response_Buff[In_Buffer_Count++] = 'S';
								}

							else{
								ACS_Response_Buff[In_Buffer_Count++] = 'F';
								}
							}

						else if(CMD_Buff[1] == 'P'){
							In_Buffer_Count = Send_Frame_HeaderOffset;
							ACS_Response_Buff[In_Buffer_Count++] = 'a';
							ACS_Response_Buff[In_Buffer_Count++] = 'p';

							if( Com2_Data[15] == 'P') {
								if ( ( AGVMotionData.Drive == Pause_Wait ) || ( Pause_Flag == 1 ) ) {
									ACS_Response_Buff[In_Buffer_Count++] = 'p';
									ACS_Response_Buff[In_Buffer_Count++] = 'S';
									}
								else if ((AGVMotionData.Drive == Forward_DRV) || (AGVMotionData.Drive == Backward_DRV)){
									if(Course_Driving_Status == 0){
										if ( Next_Node_Action == 0x01 || Next_Node_Action == 0x08 ){
											Pause_Num = 2;
											Pause_Flag = 1;
											Solw_Stop_Flag = 1;
											LED_Color_Select(1500,1000,1500);
											LED_Flicker_Flag = 0;

											SOUND_OUT_PIN_ALL = 0;
											SOUND_OUT_ADDR_WRITE;

											P_Factor = ((float)100 / 10) * SCALING_FACTOR;
											I_Factor = ((float)200 / 10) * SCALING_FACTOR;
											D_Factor = ((float)0 / 10) * SCALING_FACTOR;

											Direct_Stop();
											Distance_Sum_Buff = 0;
											}
										else if ( Next_Node_Action == 0x02 || Next_Node_Action == 0x03
											|| Next_Node_Action ==  0x05 ||  Next_Node_Action ==  0x06 ){
											Pause_Flag = 1;
											Pause_Num = 1;
											ACS_Response_Buff[In_Buffer_Count++] = 'p';
											ACS_Response_Buff[In_Buffer_Count++] = 'S';
											}
										else {
											ACS_Response_Buff[In_Buffer_Count++] = 'p';
											ACS_Response_Buff[In_Buffer_Count++] = 'F';
											}
										}

									else{
										Pause_Flag = 1;
										Pause_Num = 1;
										ACS_Response_Buff[In_Buffer_Count++] = 'p';
										ACS_Response_Buff[In_Buffer_Count++] = 'S';
										}
									}

								else if ( ( AGVMotionData.Drive == Pause_Wait ) || ( Pause_Flag == 1 ) ) {
									ACS_Response_Buff[In_Buffer_Count++] = 'p';
									ACS_Response_Buff[In_Buffer_Count++] = 'S';
									}

								else {
									ACS_Response_Buff[In_Buffer_Count++] = 'p';
									ACS_Response_Buff[In_Buffer_Count++] = 'F';
									}

								}

							else if( Com2_Data[15] == 'C'){

								if ( AGVMotionData.Drive == Pause_Wait ) {
									if ( Course_Driving_Status == 0 ) {
										if(Old_DRV_Buff == Forward_DRV){
											DELAY_MS(500);
											Moving_Speed_Set(AGV_Speed_Buff[0]);
											Forward_GPIO();
											}
										else if(Old_DRV_Buff == Backward_DRV){
											DELAY_MS(500);
											Moving_Speed_Set(AGV_Speed_Buff[0]);
											Backward_GPIO();
											}
										}

									Pause_Num = Pause_Flag = 0;
									ACS_Response_Buff[In_Buffer_Count++] = 'c';
									ACS_Response_Buff[In_Buffer_Count++] = 'S';
									}
								else {
									ACS_Response_Buff[In_Buffer_Count++] = 'c';
									ACS_Response_Buff[In_Buffer_Count++] = 'F';
									}
								}
							}

						else if(CMD_Buff[1] == 'S'){    //
							In_Buffer_Count = Send_Frame_HeaderOffset;
							ACS_Response_Buff[In_Buffer_Count++] = 'a';
							ACS_Response_Buff[In_Buffer_Count++] = 's';

							if(( AGVMotionData.Drive == AGV_Wait )&&( Pause_Num == 0 )&&( Node_Check_Count != 0 )) { //&&( Index_Err_Check == 0)){
								if((Register_Buff[122] != 0) || (Register_Buff[121] != 0))Register_Buff[122] = Register_Buff[121] = 0;

								// TIM_Out_Port.bit4.T1 = 0;
								// TIM_Out_Port.bit4.T2 = 0;
								// TIM_OUT_DATA_WRITE(TIM_Out_Port.all);

								for( i = 0; i < Node_Check_Count; i++){
									Auto_Dir_Information_Buff[i].Data.Node_ID = Auto_Dir_Info_Rec_Buff[i].Data.Node_ID;
									Auto_Dir_Information_Buff[i].Data.AGV_Direction = Auto_Dir_Info_Rec_Buff[i].Data.AGV_Direction;
									Auto_Dir_Information_Buff[i].Data.Driving_Speed = Auto_Dir_Info_Rec_Buff[i].Data.Driving_Speed;
									Auto_Dir_Information_Buff[i].Data.Turn_Angle = Auto_Dir_Info_Rec_Buff[i].Data.Turn_Angle;
									Auto_Dir_Information_Buff[i].Data.Safety_CMD = Auto_Dir_Info_Rec_Buff[i].Data.Safety_CMD;
									}

								Register_Buff[14] = (long)(Last_JobID >> 16) & 0xFFFF;
								Register_Buff[13] = Last_JobID & 0xFFFF;
								Register_Buff[16] = Auto_Dir_Information_Buff[0].Data.Driving_Speed;
								Register_Buff[45] = (long)(Last_Node >> 16) & 0xFFFF;
								Register_Buff[44] = Last_Node & 0xFFFF;
								Register_Buff[17] = Auto_Dir_Information_Buff[0].Data.AGV_Direction;
								Register_Buff[18] = Auto_Dir_Information_Buff[0].Data.Turn_Angle;
								Register_Buff[48] = (long)(Auto_Dir_Information_Buff[1].Data.Node_ID >> 16) & 0xFFFF;
								Register_Buff[47] = Auto_Dir_Information_Buff[1].Data.Node_ID  & 0xFFFF;
								// Register_Buff[20] = Auto_Dir_Information_Buff[0].Data.Safety_CMD;
								Destinations_Full_Num = Auto_Dir_Information_Buff[Node_Check_Count - 1].Data.Node_ID;

								if ( Last_Node == 675420 || Last_Node == 677420 || Last_Node == 679420 
									|| Last_Node == 796460 || Last_Node == 796480 || Last_Node == 796500
									|| Last_Node == 301000 || Last_Node == 303010 || Last_Node == 305010 || Last_Node == 307000 ){
									Register_Buff[20] = Tim_Set_Buff = Auto_Dir_Information_Buff[0].Data.Safety_CMD;
									}
								else {
									Register_Buff[20] = Tim_Set_Buff = 0;
									}

								Next_Node_Action = Auto_Dir_Information_Buff[1].Data.AGV_Direction;

								Register_Buff[19] = Node_Check_Count;

								if(Destinations_Full_Num != 0){
									Register_Buff[51] = Destinations_Full_Num & 0xFFFF;
									Register_Buff[52] = ( Destinations_Full_Num >> 16 ) & 0xFFFF;
									}


								Key_Input_Stop = 0;

								DELAY_MS(1);

								RightMotor_Int_Count = LeftMotor_Int_Count = Right_Mag_Count = Left_Mag_Count = 0;

								Action_Num = AsciiHex_To_Binary_Num((int8_t *)Com2_Data, AGV_Action_Num_Addr, 2);

								DELAY_MS(10);

								//ACS_Response_Buff[In_Buffer_Count++] = 'S';

								switch(Action_Num){

									case 1:
										TIM_Out_Port.bit4.T1 = Tim_Set_Buff;
										TIM_Out_Port.bit4.T2 = 0;
										TIM_OUT_DATA_WRITE(TIM_Out_Port.all);

										ACS_Response_Buff[In_Buffer_Count++] = 'S';
										Mark_Input_Signal = 0;
										Moving_Speed_Set(AGV_Speed_Buff[0]);
										Forward_GPIO();
										Sample_Acq_Flag = 1;
										break;

									case 4:
										TIM_Out_Port.bit4.T1 = 0;
										TIM_Out_Port.bit4.T2 = Tim_Set_Buff;
										TIM_OUT_DATA_WRITE(TIM_Out_Port.all);

										ACS_Response_Buff[In_Buffer_Count++] = 'S';
										Mark_Input_Signal = 0;
										Moving_Speed_Set(AGV_Speed_Buff[0]);
										Backward_GPIO();
										Sample_Acq_Flag = 1;
										break;

									case 7:
										TIM_Out_Port.bit4.T1 = Tim_Set_Buff;
										TIM_Out_Port.bit4.T2 = 0;
										TIM_OUT_DATA_WRITE(TIM_Out_Port.all);

										ACS_Response_Buff[In_Buffer_Count++] = 'S';
										Mark_Input_Signal = 0;
										Start_Direction = Forward_DRV;
										Lift_Action_To_Moving_SEQ_Num = 1;
										Spin_Action_Start = 0;
										break;

									case 8:
										TIM_Out_Port.bit4.T1 = 0;
										TIM_Out_Port.bit4.T2 = Tim_Set_Buff;
										TIM_OUT_DATA_WRITE(TIM_Out_Port.all);

										ACS_Response_Buff[In_Buffer_Count++] = 'S';
										Mark_Input_Signal = 0;
										Start_Direction = Backward_DRV;
										Lift_Action_To_Moving_SEQ_Num = 1;
										Spin_Action_Start = 0;
										break;

									default :
										ACS_Response_Buff[In_Buffer_Count++] = 'F';
										break;
									}
								}

							else{
								ACS_Response_Buff[In_Buffer_Count++] = 'F';
								Index_Err_Check = 0;
								}
							}

						else if(CMD_Buff[1] == 'G'){
							In_Buffer_Count = Send_Frame_HeaderOffset;
							ACS_Response_Buff[In_Buffer_Count++] = 'a';
							ACS_Response_Buff[In_Buffer_Count++] = 'g';

							if(Com2_Data[15] == '1'){
								if(AGVMotionData.Drive == AGV_Wait  || AGVMotionData.Drive == Charge_Stop_SEQ ){
									AGVMotionData.Drive = Charge_SEQ;

									PIO_Out_Port.byte.L = 0;
									ADDR16(PORTH) = PIO_Out_Port.byte.L;

									Charge_Seq_Num = 1;
									DisCharge_Seq_Num = 0;
									ACS_Response_Buff[In_Buffer_Count++] = 'S';
									}
								else{
									ACS_Response_Buff[In_Buffer_Count++] = 'F';
									}
								}

							if(Com2_Data[15] == '0'){
								if(AGVMotionData.Drive == AGV_CHARGING || AGVMotionData.Drive == Charge_SEQ ){
									AGVMotionData.Drive = Charge_Stop_SEQ;

									PIO_Out_Port.byte.L = 0;
									ADDR16(PORTH) = PIO_Out_Port.byte.L;

									DisCharge_Seq_Num = 1;
									Charge_Seq_Num = 0;
									ACS_Response_Buff[In_Buffer_Count++] = 'S';
									}
								else{
									ACS_Response_Buff[In_Buffer_Count++] = 'F';
									//Charge_Fail_Event = 1;
									}
								}
							}

						else if(CMD_Buff[1] == 'C'){
							In_Buffer_Count = Send_Frame_HeaderOffset;
							ACS_Response_Buff[In_Buffer_Count++] = 'a';
							ACS_Response_Buff[In_Buffer_Count++] = 'c';

							if ( Last_JobID != 0 ){
								if ( AGVMotionData.Drive == AGV_Wait ){

									Job_Status_Clear();

									AGV_Stop_Function();

									ACS_Response_Buff[In_Buffer_Count++] = 'S';
									}
								else {
									Job_Cancel_Stop_Flag = 1;

									ACS_Response_Buff[In_Buffer_Count++] = 'S';
									}
								}
							else ACS_Response_Buff[In_Buffer_Count++] = 'F';
							}

						else if(CMD_Buff[1] == 'V'){
							In_Buffer_Count = Send_Frame_HeaderOffset;
							ACS_Response_Buff[In_Buffer_Count++] = 'a';
							ACS_Response_Buff[In_Buffer_Count++] = 'v';
							ACS_Response_Buff[In_Buffer_Count++] = 'S';

							}

						else if(CMD_Buff[1] == 'I'){    //LED
							In_Buffer_Count = Send_Frame_HeaderOffset;
							ACS_Response_Buff[In_Buffer_Count++] = 'a';
							ACS_Response_Buff[In_Buffer_Count++] = 'i';
							ACS_Response_Buff[In_Buffer_Count++] = 'S';

							}

						// SCIC_RecBuff_Clear();

						Int_To_HexAscii_Num( ACS_Response_Buff, Send_Frame_BodyCountOffset, ( In_Buffer_Count - Send_Frame_HeaderOffset ), 4 );

						CheckSum_Generation_Func((unsigned char *)&ACS_Response_Buff, In_Buffer_Count);

						SCIC_TX_String(ACS_Response_Buff);

						}

					for(i=0; i<20; i++) Com2_Data[Receive_Frame_HeaderOffset+i] = 0;

					Com2Data_Count = 0;
					return 2;
					}
				break;

			default :
				Com2_Data[Com2Data_Count++] = com_char;
				if(Com2Data_Count >= Route_REC_Buff_Size)Com2Data_Count = 0;
				break;
		}
		com_char = SCIC_Read_Char();
	}

	return 0;
}

void SCIB_Communication_Control(void)
{
	unsigned int com_char = 0;
	unsigned char i = 0;
	unsigned char CheckSum_Data = 0;
	unsigned int Err_Buff = 0;
	int Angle_Caluration_Buff = 0;
	int X_Axis_Caluration_Buff = 0;
	int Y_Axis_Caluration_Buff = 0;
	unsigned long TAG_Caluration_Buff = 0;

	int G_Buff = 0, guide = 0;

	int  PGV_Angle_Data_Cal = 0;
	unsigned long PGV_Tag_Num_Data_Buff = 0;
	unsigned char Safety_buff = 0, Speed_Buff = 0;
	unsigned int Node_Count = 0, Dir_Err_Flag = 1;
	unsigned int PGV_Tag_Num_Data_L = 0;
	unsigned int PGV_Tag_Num_Data_H = 0;
	unsigned char Node_Action = 0;

	if(PGV_Data_Send_Flag == 1){
		if( Read_Rwi_Position() >= PGV_Data_Rec_Buff_Size ){
			PGV_Data_Send_Flag = Com1Data_Count = 0;

			if(PGV_Data_Set_Flag == 1){

				for(i = 0; i < 3; i++){
					Com1_Data[Com1Data_Count++] = SCIB_Read_Char();
					}

				if(((Com1_Data[0] >> 4) & 0x03) == 0){
					if(Com1_Data[0] & 0x01){
						DELAY_MS(10);
						//PGV_Line_Select(Line_Select_Buff, PGV_ID);
						PGV_Data_Set_Flag = 1;
						}
					if(~Com1_Data[0] & 0x01){
						PGV_ID = 0;
						//PGV_Color_Select(Color_Select_Buff, PGV_ID);
						DELAY_MS(10);
						PGV_Data_Set_Flag = 2;
						}
					}

				}

			else if(PGV_Data_Set_Flag == 2){
				for(i = 0; i < 2; i++){
					Com1_Data[Com1Data_Count++] = SCIB_Read_Char();
					}

				if(((Com1_Data[0] >> 4) & 0x03) == 0){
					Setting_Flag = 0;
					PGV_Data_Set_Flag = 3;
					PGV_Structs_Send_Fram.PGV_Command = Sel_Blue;
					PGV_Structs_Send_Fram.PGV_Structs = Color_Green;
					PGV_Structs_Send_Fram.PGV_ID = 0;
					PGV_Com_Send(PGV_Data_Set_Flag);
					if(PGV_Start_Flag == 1)PGV_Start_Flag = 0;
					}
				}

			else if(PGV_Data_Set_Flag == 3){
				for(i = 0; i < 21; i++){
					Com1_Data[Com1Data_Count++] = SCIB_Read_Char();
					}
				}

			com_char = 1;

			if(Setting_Flag == 1){
				DELAY_MS(50);
				PGV_Line_Select(Line_Select_Buff, PGV_ID);
				//PGV_Data_Set_Flag = 1;
				Setting_Flag = 2;
				}
			}
		}

	if(com_char == 1){

		if(PGV_Data_Set_Flag == 3){

			SpinTurn_Code_Detect = Com1_Data[0] & 0x01;
			//Line_Err_Detect = Com1_Data[0] & 0x01;
			Tag_Detect = (Com1_Data[1] >> 6) & 0x01;

			Line_Err_Detect = (Com1_Data[1] >> 4) & 0x03;

			//if(Spot_Line_State == 0){
				// if(Line_Err_Detect != 0){
				// 	Distance_Sum_Buff = 0;
				// 	if(Spot_Line_Check < 15)Spot_Line_Check++;
				// 	if(Spot_Line_Check >= 14){
				// 		Spot_Line_State = Line;
				// 		Spot_Line_Check = 15;
				// 		}
				// 	}
				//}

			//if(Spot_Line_State == 1){
				// if(Line_Err_Detect == 0){
				// 	if(Spot_Line_Check >= 3)Spot_Line_Check--;
				// 	if((Spot_Line_Check < 3) && (Distance_Sum_Buff > 1000)){
				// 		Spot_Line_State = QR_Tag;
				// 		Spot_Line_Check = 0;
				// 		}
				// 	}
				//}

			//if((Com1_Data[0] & 0x01) == 0)Distance_Sum_Buff = 0;

			CheckSum_Data = 0;

			for(i = 0; i < 20; i++)CheckSum_Data ^= (Com1_Data[i] & 0x7F);


			Err_Buff = (Com1_Data[18] & 0x7F) << 7;
			Err_Buff |= (Com1_Data[19] & 0x7F);

			if((Err_Buff == 0) && (CheckSum_Data == Com1_Data[20])){

				On_Tag_Falg = Com1_Data[1] >> 6;

				X_Axis_Caluration_Buff = (long)(Com1_Data[2] & 0x7F) << 21;
				X_Axis_Caluration_Buff |= (long)(Com1_Data[3] & 0x7F) << 14;
				X_Axis_Caluration_Buff |= (long)(Com1_Data[4] & 0x7F) << 7;
				X_Axis_Caluration_Buff |= (long)(Com1_Data[5] & 0x7F);

				if(X_Axis_Caluration_Buff & 0x400000){
					X_Axis_Caluration_Buff |= 0xFF800000;
					}
				X_Axis_Data = X_Axis_Caluration_Buff;

				if(Spot_Line_State == QR_Tag){
					if(On_Tag_Falg == 1){
						Register_Buff[72] = (long)(X_Axis_Caluration_Buff >> 16) & 0xFFFF;
						Register_Buff[71] = X_Axis_Caluration_Buff & 0xFFFF;
						}
					}
				else{
					Register_Buff[72] = (long)(X_Axis_Caluration_Buff >> 16) & 0xFFFF;
					Register_Buff[71] = X_Axis_Caluration_Buff & 0xFFFF;
					}

				Y_Axis_Caluration_Buff = (Com1_Data[6] & 0x7F) << 7;
				Y_Axis_Caluration_Buff |= Com1_Data[7] & 0x7F;
				if(Y_Axis_Caluration_Buff & 0x2000)Y_Axis_Caluration_Buff |= 0xC000;
				G_Buff = Y_Axis_Data = Y_Axis_Caluration_Buff;
				Global_Guide = Y_Axis_Caluration_Buff;

				if(Y_Axis_Caluration_Buff & 0x2000)Y_Axis_Caluration_Buff |= 0xFFFF0000;

				if(Spot_Line_State == QR_Tag){
					if(On_Tag_Falg == 1){
						Register_Buff[70] = (long)(Y_Axis_Caluration_Buff >> 16) & 0xFFFF;
						Register_Buff[69] = Y_Axis_Caluration_Buff & 0xFFFF;
						}
					}
				else{
					Register_Buff[70] = (long)(Y_Axis_Caluration_Buff >> 16) & 0xFFFF;
					Register_Buff[69] = Y_Axis_Caluration_Buff & 0xFFFF;
					}

				Angle_Caluration_Buff = (Com1_Data[10] & 0x7F) << 7;
				Angle_Caluration_Buff |= (Com1_Data[11] & 0x7F);

				Angle_Caluration_Buff += PGV_Angle_Offset;

				if(Angle_Caluration_Buff < 0){
					Angle_Caluration_Buff = 3600 - Angle_Caluration_Buff;
					}
				if(Angle_Caluration_Buff > 3600){
					Angle_Caluration_Buff = Angle_Caluration_Buff - 3600;
					}

				PGV_Guide_Angle = PGV_Angle_Data = Angle_Caluration_Buff;

				if(Spot_Line_State == QR_Tag){
					if(On_Tag_Falg == 1){
						Register_Buff[73] = Angle_Caluration_Buff;
						}
					}
				// else{
				// 	Register_Buff[73] = Angle_Caluration_Buff;
				// 	}

				TAG_Caluration_Buff = (long)(Com1_Data[14] & 0x7F) << 21;
				TAG_Caluration_Buff |= (long)(Com1_Data[15] & 0x7F) << 14;
				TAG_Caluration_Buff |= (long)(Com1_Data[16] & 0x7F) << 7;
				TAG_Caluration_Buff |= (long)(Com1_Data[17] & 0x7F);

				PGV_Tag_Num_Data_Buff = PGV_Tag_Num_Data = TAG_Caluration_Buff;
				Register_Buff[76] = (long)(PGV_Tag_Num_Data >> 16) & 0xFFFF;
				Register_Buff[75] = PGV_Tag_Num_Data & 0xFFFF;

				if(Spot_Line_State == Line){
					if(Line_Err_Detect != 0)Distance_Sum_Buff = 0;

					if((Angle_Caluration_Buff > 1800) && (Angle_Caluration_Buff <= 3600)){
						PGV_Guide_Angle = Angle_Caluration_Buff -= 3600;
						}

					if(PGV_Tag_Num_Data != 0){
						if((PGV_Angle_Data >= 3150) || (PGV_Angle_Data < 450))Surface_Input = 1;
						else if((PGV_Angle_Data >= 450) && (PGV_Angle_Data < 1350))Surface_Input = 2;
						else if((PGV_Angle_Data >= 1350) && (PGV_Angle_Data < 2250))Surface_Input = 3;
						else if((PGV_Angle_Data >= 2250) && (PGV_Angle_Data < 3150))Surface_Input = 4;

						if(Surface_Input == 1){
							Global_Guide = G_Buff = (Y_Axis_Caluration_Buff * -1)  + PGV_X_Axis_Offset;
							if((PGV_Angle_Data > 1800) && (PGV_Angle_Data <= 3600)){
								PGV_Angle_Data -= 3600;
								}
							PGV_Guide_Angle = PGV_Angle_Data_Cal = PGV_Angle_Data;
							}

						if(Surface_Input == 2){

							Global_Guide = G_Buff = (X_Axis_Caluration_Buff / 10) + PGV_X_Axis_Offset;
							PGV_Guide_Angle = PGV_Angle_Data_Cal = PGV_Angle_Data - 900;
							}

						if(Surface_Input == 3){
							Global_Guide = G_Buff = Y_Axis_Caluration_Buff + PGV_X_Axis_Offset;
							PGV_Guide_Angle = PGV_Angle_Data_Cal = PGV_Angle_Data - 1800;
							}

						if(Surface_Input == 4){
							Global_Guide = G_Buff = (X_Axis_Caluration_Buff / 10) + PGV_X_Axis_Offset;
							PGV_Guide_Angle = PGV_Angle_Data_Cal = PGV_Angle_Data - 2700;
							}

						}
					}

				if((PGV_Tag_Num_Data_Buff != Old_Tag_Num) && (PGV_Tag_Num_Data_Buff != 0)){

					Distance_Sum_Buff = 0;

					if(Spot_Line_State == QR_Tag){
						if((Angle_Caluration_Buff >= 3150) || (Angle_Caluration_Buff < 450))Surface_Input = 1;
						else if((Angle_Caluration_Buff >= 450) && (Angle_Caluration_Buff < 1350))Surface_Input = 2;
						else if((Angle_Caluration_Buff >= 1350) && (Angle_Caluration_Buff < 2250))Surface_Input = 3;
						else if((Angle_Caluration_Buff >= 2250) && (Angle_Caluration_Buff < 3150))Surface_Input = 4;

						Register_Buff[114] = Surface_Input;
						}

					if( AGVMotionData.Drive == Forward_DRV ){

						// Old_Tag_Num = PGV_Tag_Num_Data;

						if(Surface_Input == 1){
							Y_Axis_Caluration_Buff += PGV_X_Axis_Offset;
							Register_Buff[110] = (Y_Axis_Caluration_Buff >> 16) & 0xFFFF;
							Register_Buff[109] = Y_Axis_Caluration_Buff & 0xFFFF;
							Register_Buff[112] = (X_Axis_Caluration_Buff >> 16) & 0xFFFF;
							Register_Buff[111] = X_Axis_Caluration_Buff & 0xFFFF;

							G_Buff = Y_Axis_Caluration_Buff;

							if((Angle_Caluration_Buff > 1800) && (Angle_Caluration_Buff <= 3600)){
								Angle_Caluration_Buff -= 3600;
								}
							PGV_Angle_Data_Cal = Angle_Caluration_Buff;
							}

						if(Surface_Input == 2){
							X_Axis_Caluration_Buff += PGV_X_Axis_Offset;
							Register_Buff[110] = (X_Axis_Caluration_Buff >> 16) & 0xFFFF;
							Register_Buff[109] = X_Axis_Caluration_Buff & 0xFFFF;
							Register_Buff[112] = (Y_Axis_Caluration_Buff >> 16) & 0xFFFF;
							Register_Buff[111] = Y_Axis_Caluration_Buff & 0xFFFF;

							G_Buff = (X_Axis_Caluration_Buff * 1);
							PGV_Angle_Data_Cal = (Angle_Caluration_Buff - 900);
							}

						if(Surface_Input == 3){
							Y_Axis_Caluration_Buff -= PGV_X_Axis_Offset;
							Register_Buff[110] = (Y_Axis_Caluration_Buff >> 16) & 0xFFFF;
							Register_Buff[109] = Y_Axis_Caluration_Buff & 0xFFFF;
							Register_Buff[112] = (X_Axis_Caluration_Buff >> 16) & 0xFFFF;
							Register_Buff[111] = X_Axis_Caluration_Buff & 0xFFFF;

							G_Buff = (Y_Axis_Caluration_Buff * -1);
							PGV_Angle_Data_Cal = (Angle_Caluration_Buff - 1800);
							}

						if(Surface_Input == 4){
							X_Axis_Caluration_Buff -= PGV_X_Axis_Offset;
							Register_Buff[110] = (X_Axis_Caluration_Buff >> 16) & 0xFFFF;
							Register_Buff[109] = X_Axis_Caluration_Buff & 0xFFFF;
							Register_Buff[112] = (Y_Axis_Caluration_Buff >> 16) & 0xFFFF;
							Register_Buff[111] = Y_Axis_Caluration_Buff & 0xFFFF;

							G_Buff = (X_Axis_Caluration_Buff * -1);
							PGV_Angle_Data_Cal = (Angle_Caluration_Buff - 2700);
							}

						guide = ((G_Buff - 0) * Y_Axis_Cal_Data) + (PGV_Angle_Data_Cal * Distance_Angle);

						Check_SEQ_Num_F = guide;

						if(guide < 0){
							Left_Mag_Count = (guide * -1) >> 2;
							//Check_SEQ_Num_F = 1;
							}

						if(guide > 0){
							Right_Mag_Count = guide >> 2;
							//Check_SEQ_Num_F = 2;
							}

						if(guide == 0){
							Right_Mag_Count = Left_Mag_Count = 0;
							//Check_SEQ_Num_F = 3;
							}

						if((Course_Driving_Status == 0) && (PGV_Tag_Num_Data_Buff != 9000000)){  // && (Last_Node != PGV_Tag_Num_Data_Buff)){
							if(Register_Buff[59] == Manual_Operate){
								PGV_Tag_Num_Data_L = (unsigned int)(PGV_Tag_Num_Data_Buff % 1000);
								PGV_Tag_Num_Data_H = (unsigned int)(PGV_Tag_Num_Data_Buff / 1000);
								// Last_Node_L = (unsigned int)(Last_Node % 1000);
								// Last_Node_H = (unsigned int)(Last_Node / 1000);

								if ( PGV_Tag_Num_Data_Buff == 431480 || PGV_Tag_Num_Data_Buff == 431580 ) {
									Forward_Mark_Stop_Status = 1;
									StopToLift_Flag = Normal_Stop;
									During_Stop_Action = 1;
									}

								else if ( ( PGV_Tag_Num_Data_H >= 405 ) && ( PGV_Tag_Num_Data_H <= 429 ) ) {
									if ( ( PGV_Tag_Num_Data_L == 305 ) || ( PGV_Tag_Num_Data_L == 310 ) || ( PGV_Tag_Num_Data_L == 370 ) ||
										( PGV_Tag_Num_Data_L == 380 ) || ( PGV_Tag_Num_Data_L == 440 ) || ( PGV_Tag_Num_Data_L == 450 ) ||
										( PGV_Tag_Num_Data_L == 510 ) || ( PGV_Tag_Num_Data_L == 520 ) || ( PGV_Tag_Num_Data_L == 580 ) ){
										Forward_Mark_Stop_Status = 1;
										StopToLift_Flag = Normal_Stop;
										During_Stop_Action = 1;
										}
									}

								else if ( ( PGV_Tag_Num_Data == 303000 ) || ( PGV_Tag_Num_Data == 305010 ) || ( PGV_Tag_Num_Data_L == 307010 ) ||
										( PGV_Tag_Num_Data == 307000 ) || ( PGV_Tag_Num_Data == 312010 ) ){
									Forward_Mark_Stop_Status = 1;
									StopToLift_Flag = Normal_Stop;
									During_Stop_Action = 1;
									}

								else if ( ( PGV_Tag_Num_Data_H >= 601 ) && ( PGV_Tag_Num_Data_H <= 617 ) && ( PGV_Tag_Num_Data_L == 110 )  ) {
									Forward_Mark_Stop_Status = 1;
									StopToLift_Flag = Normal_Stop;
									During_Stop_Action = 1;
									}
								}

							else if( OPERATE_MODE == Auto_Operate){
								for(i = 0; i < Node_Check_Count; i++){
									if(PGV_Tag_Num_Data_Buff == Auto_Dir_Information_Buff[i].Data.Node_ID){
										Line_Err_Include_Flag = Dir_Err_Flag = 0;
										Node_Count = i;
										}
									}

								if((Dir_Err_Flag == 1)&&(Old_Tag_Num != 0)&&(Old_Tag_Num != PGV_Tag_Num_Data)&&( PGV_Tag_Num_Data != 299 ))Line_Err_Include_Flag = 1;

								if(Dir_Err_Flag == 0){
									Register_Buff[14] = (long)(Last_JobID >> 16) & 0xFFFF;
									Register_Buff[13] = Last_JobID & 0xFFFF;
									Register_Buff[16] = Speed_Buff = Auto_Dir_Information_Buff[Node_Count].Data.Driving_Speed;
									Register_Buff[45] = (long)(PGV_Tag_Num_Data_Buff >> 16) & 0xFFFF;
									Register_Buff[44] = PGV_Tag_Num_Data_Buff & 0xFFFF;
									Register_Buff[17] = Auto_Dir_Information_Buff[Node_Count].Data.AGV_Direction;
									Register_Buff[18] = Auto_Dir_Information_Buff[Node_Count].Data.Turn_Angle;
									Register_Buff[48] = (long)(Auto_Dir_Information_Buff[Node_Count + 1].Data.Node_ID >> 16) & 0xFFFF;
									Register_Buff[47] = Auto_Dir_Information_Buff[Node_Count + 1].Data.Node_ID  & 0xFFFF;
									Register_Buff[21] = Node_Count;
									//Register_Buff[19] = Node_Check_Count;
									Register_Buff[20] = Safety_buff = Auto_Dir_Information_Buff[Node_Count].Data.Safety_CMD;

									TIM_Out_Port.bit4.T1 = Safety_buff;
									TIM_Out_Port.bit4.T2 = 0; //Safety_buff;
									TIM_OUT_DATA_WRITE(TIM_Out_Port.all);

									Node_Action = Auto_Dir_Information_Buff[Node_Count].Data.AGV_Direction;

									if ( Node_Count < ( Node_Check_Count - 1) ) Next_Node_Action = Auto_Dir_Information_Buff[Node_Count+1].Data.AGV_Direction;
									else Next_Node_Action = 0;

									if ( (Speed_Buff < 1 ) || ( Speed_Buff > 4 ) ) Speed_Buff = 0;
									else Speed_Buff = Speed_Buff - 1;


									if ( Node_Action == 0x01 || Node_Action == 0x08 ){
										if (Tim_2Area_Sensing == 0) Moving_Speed_Set(AGV_Speed_Buff[Speed_Buff]);
										else if ( Target_Speed > CAL_TAGET_SPEED(AGV_Speed_Buff[Speed_Buff]) )Moving_Speed_Set(AGV_Speed_Buff[Speed_Buff]);
										}
									else Moving_Speed_Set(AGV_Speed_Buff[0]);

									// if ( AGVMotionData.Drive == Forward_DRV ){
									switch(Node_Action){
										case 0x01 :     // 	Forward
											break;

										case 0x02 :     //  Right Spin to Forward
											Spin_Start(Right_Spin, Auto_Dir_Information_Buff[Node_Count].Data.Driving_Speed - 1, SpinToForward, 90, 1, 0);
											break;

										case 0x03 :		// Right Spin to Backward
											Spin_Start(Right_Spin, Auto_Dir_Information_Buff[Node_Count].Data.Driving_Speed - 1, SpinToBackward, 90, 1, 0);
											break;

										case 0x04 :		// Right Spin to Stop
											Spin_Start(Right_Spin, Auto_Dir_Information_Buff[Node_Count].Data.Driving_Speed - 1, SpinToStop, 90, 1, 1);
											break;

										case 0x05 :     // Left Spin to Forward
											Spin_Start(Left_Spin, Auto_Dir_Information_Buff[Node_Count].Data.Driving_Speed - 1, SpinToForward, 90, 1, 0);
											break;

										case 0x06 :     // Left Spin to Backward
											Spin_Start(Left_Spin, Auto_Dir_Information_Buff[Node_Count].Data.Driving_Speed - 1, SpinToBackward, 90, 1, 0);
											break;

										case 0x07 :     // Left Spin to Stop
											Spin_Start(Left_Spin, Auto_Dir_Information_Buff[Node_Count].Data.Driving_Speed - 1, SpinToStop, 90, 1, 1);
											break;

										case 0x08 :     // Backward
											break;

										case 0x09 :     // Mark Enable Stop
											Forward_Mark_Stop_Status = 1;
											StopToLift_Flag = Normal_Stop;
											During_Stop_Action = 1;
											DRV_Complete_Flag = 1;
											break;

										case 0x0A :     // Right Spin to Loading
											Spin_Start(Right_Spin, Auto_Dir_Information_Buff[Node_Count].Data.Driving_Speed - 1, SpinToLoading, 90, 1, 1);
											break;

										case 0x0B :     // Right Spin to Unloading
											Spin_Start(Right_Spin, Auto_Dir_Information_Buff[Node_Count].Data.Driving_Speed - 1, SpinToUnloading, 90, 1, 1);
											break;

										case 0x0C :     // Left Spin to Loading
											Spin_Start(Left_Spin, Auto_Dir_Information_Buff[Node_Count].Data.Driving_Speed - 1, SpinToLoading, 90, 1, 1);
											break;

										case 0x0D :     // Left Sping to Unloading
											Spin_Start(Left_Spin, Auto_Dir_Information_Buff[Node_Count].Data.Driving_Speed - 1, SpinToUnloading, 90, 1, 1);
											break;

										case 0x10 :     // Stop to Loading
											Forward_Mark_Stop_Status = 1;
											StopToLift_Flag = Lift_Loading;
											DRV_Complete_Flag = 1;
											break;

										case 0x11 :     // Stop to Unloading
											Forward_Mark_Stop_Status = 1;
											StopToLift_Flag = Lift_Unloading;
											DRV_Complete_Flag = 1;
											break;
										}
										// }
									}
								}
							}
						}

					else if(AGVMotionData.Drive == Backward_DRV  ){

						// Old_Tag_Num = PGV_Tag_Num_Data;

						Register_Buff[114] = Surface_Input;

						if(Surface_Input == 1){
							Y_Axis_Caluration_Buff += PGV_X_Axis_Offset;
							Register_Buff[110] = (Y_Axis_Caluration_Buff >> 16) & 0xFFFF;
							Register_Buff[109] = Y_Axis_Caluration_Buff & 0xFFFF;
							Register_Buff[112] = (X_Axis_Caluration_Buff >> 16) & 0xFFFF;
							Register_Buff[111] = X_Axis_Caluration_Buff & 0xFFFF;

							G_Buff = Y_Axis_Caluration_Buff;
							if((Angle_Caluration_Buff > 1800) && (Angle_Caluration_Buff <= 3600)){
								Angle_Caluration_Buff -= 3600;
								}
							PGV_Angle_Data_Cal = Angle_Caluration_Buff;
							}

						if(Surface_Input == 2){
							X_Axis_Caluration_Buff -= PGV_X_Axis_Offset;
							Register_Buff[110] = (X_Axis_Caluration_Buff >> 16) & 0xFFFF;
							Register_Buff[109] = X_Axis_Caluration_Buff & 0xFFFF;
							Register_Buff[112] = (Y_Axis_Caluration_Buff >> 16) & 0xFFFF;
							Register_Buff[111] = Y_Axis_Caluration_Buff & 0xFFFF;

							G_Buff = (X_Axis_Caluration_Buff * 1);
							PGV_Angle_Data_Cal = (Angle_Caluration_Buff - 900);
							}

						if(Surface_Input == 3){
							Y_Axis_Caluration_Buff += PGV_X_Axis_Offset;
							Register_Buff[110] = (Y_Axis_Caluration_Buff >> 16) & 0xFFFF;
							Register_Buff[109] = Y_Axis_Caluration_Buff & 0xFFFF;
							Register_Buff[112] = (X_Axis_Caluration_Buff >> 16) & 0xFFFF;
							Register_Buff[111] = X_Axis_Caluration_Buff & 0xFFFF;

							G_Buff = (Y_Axis_Caluration_Buff * -1);
							PGV_Angle_Data_Cal = (Angle_Caluration_Buff - 1800);
							}

						if(Surface_Input == 4){
							X_Axis_Caluration_Buff += PGV_X_Axis_Offset;
							Register_Buff[110] = (X_Axis_Caluration_Buff >> 16) & 0xFFFF;
							Register_Buff[109] = X_Axis_Caluration_Buff & 0xFFFF;
							Register_Buff[112] = (Y_Axis_Caluration_Buff >> 16) & 0xFFFF;
							Register_Buff[111] = Y_Axis_Caluration_Buff & 0xFFFF;

							G_Buff = (X_Axis_Caluration_Buff * -1);
							PGV_Angle_Data_Cal = (Angle_Caluration_Buff - 2700);
							}

						guide = ((G_Buff - 0) * Y_Axis_Cal_Data) + ((PGV_Angle_Data_Cal * Distance_Angle) * -1);

						Check_SEQ_Num_B = guide;
						if(guide < 0){
							Left_Mag_Count = (guide * -1) >> 2;
							//Check_SEQ_Num_B = 1;
							}

						if(guide > 0){
							Right_Mag_Count = guide >> 2;
							//Check_SEQ_Num_B = 2;
							}
						if(guide == 0){
							Right_Mag_Count = Left_Mag_Count = 0;
							//Check_SEQ_Num_B = 3;
							}

						if( (Course_Driving_Status == 0) && (PGV_Tag_Num_Data_Buff != 9000000) ){

							if( OPERATE_MODE == Manual_Operate){
								PGV_Tag_Num_Data_L = (unsigned int)(PGV_Tag_Num_Data_Buff % 1000);
								PGV_Tag_Num_Data_H = (unsigned int)(PGV_Tag_Num_Data_Buff / 1000);

								if ( PGV_Tag_Num_Data_Buff == 440011 || PGV_Tag_Num_Data_Buff == 615110
									||  PGV_Tag_Num_Data_Buff == 617110 || PGV_Tag_Num_Data_Buff == 312010 ){
									Forward_Mark_Stop_Status = 1;
									StopToLift_Flag = Normal_Stop;
									During_Stop_Action = 1;
									}

								else if ( ( PGV_Tag_Num_Data_H >= 405 ) && ( PGV_Tag_Num_Data_H <= 429 ) ) {
									if ( ( PGV_Tag_Num_Data_L == 305 ) || ( PGV_Tag_Num_Data_L == 310 ) || ( PGV_Tag_Num_Data_L == 370 ) ||
										( PGV_Tag_Num_Data_L == 380 ) || ( PGV_Tag_Num_Data_L == 440 ) || ( PGV_Tag_Num_Data_L == 450 ) ||
										( PGV_Tag_Num_Data_L == 510 ) || ( PGV_Tag_Num_Data_L == 520 ) || ( PGV_Tag_Num_Data_L == 580 ) ){
										Forward_Mark_Stop_Status = 1;
										StopToLift_Flag = Normal_Stop;
										During_Stop_Action = 1;
										}
									}
								}

							else if( OPERATE_MODE == Auto_Operate){
								for(i = 0; i < Node_Check_Count; i++){
									if(PGV_Tag_Num_Data_Buff == Auto_Dir_Information_Buff[i].Data.Node_ID){
										Line_Err_Include_Flag = Dir_Err_Flag = 0;
										Node_Count = i;
										}
									}

								if((Dir_Err_Flag == 1)&&(Old_Tag_Num != 0)&&(Old_Tag_Num != PGV_Tag_Num_Data)&&( PGV_Tag_Num_Data != 299 ))Line_Err_Include_Flag = 1;

								if(Dir_Err_Flag == 0){

									Register_Buff[14] = (long)(Last_JobID >> 16) & 0xFFFF;
									Register_Buff[13] = Last_JobID & 0xFFFF;
									Register_Buff[16] = Speed_Buff = Auto_Dir_Information_Buff[Node_Count].Data.Driving_Speed;
									Register_Buff[45] = (long)(PGV_Tag_Num_Data_Buff >> 16) & 0xFFFF;
									Register_Buff[44] = PGV_Tag_Num_Data_Buff & 0xFFFF;
									Register_Buff[17] = Auto_Dir_Information_Buff[Node_Count].Data.AGV_Direction;
									Register_Buff[18] = Auto_Dir_Information_Buff[Node_Count].Data.Turn_Angle;
									Register_Buff[48] = (long)(Auto_Dir_Information_Buff[Node_Count + 1].Data.Node_ID >> 16) & 0xFFFF;
									Register_Buff[47] = Auto_Dir_Information_Buff[Node_Count + 1].Data.Node_ID  & 0xFFFF;
									//Register_Buff[19] = Node_Check_Count;
									Register_Buff[20] = Safety_buff = Auto_Dir_Information_Buff[Node_Count].Data.Safety_CMD;

									TIM_Out_Port.bit4.T1 = 0; //Safety_buff;
									TIM_Out_Port.bit4.T2 = Safety_buff;
									TIM_OUT_DATA_WRITE(TIM_Out_Port.all);

									Node_Action = Auto_Dir_Information_Buff[Node_Count].Data.AGV_Direction;

									if ( Node_Count < ( Node_Check_Count - 1) ) Next_Node_Action = Auto_Dir_Information_Buff[Node_Count+1].Data.AGV_Direction;
									else Next_Node_Action = 0;

									if ( (Speed_Buff < 1 ) || ( Speed_Buff > 4 ) ) Speed_Buff = 0;
									else Speed_Buff = Speed_Buff - 1;

									if ( Node_Action == 0x01 || Node_Action == 0x08 ){
										if (Back_Tim_2Area_Sensing == 0) Moving_Speed_Set(AGV_Speed_Buff[Speed_Buff]);
										else if ( Target_Speed > CAL_TAGET_SPEED(AGV_Speed_Buff[Speed_Buff]) ) Moving_Speed_Set(AGV_Speed_Buff[Speed_Buff]);
										}
									else Moving_Speed_Set(AGV_Speed_Buff[0]);

									// if ( AGVMotionData.Drive == Backward_DRV ){
									switch(Node_Action){

										case 0x01 :     // 	Forward
											break;

										case 0x02 :     //  Right Spin to Forward
											Spin_Start(Right_Spin, Auto_Dir_Information_Buff[Node_Count].Data.Driving_Speed - 1, SpinToForward, 90, 1, 0);
											break;

										case 0x03 :		// Right Spin to Backward
											Spin_Start(Right_Spin, Auto_Dir_Information_Buff[Node_Count].Data.Driving_Speed - 1, SpinToBackward, 90, 1, 0);
											break;

										case 0x04 :		// Right Spin to Stop
											Spin_Start(Right_Spin, Auto_Dir_Information_Buff[Node_Count].Data.Driving_Speed - 1, SpinToStop, 90, 1, 1);
											break;

										case 0x05 :     // Left Spin to Forward
											Spin_Start(Left_Spin, Auto_Dir_Information_Buff[Node_Count].Data.Driving_Speed - 1, SpinToForward, 90, 1, 0);
											break;

										case 0x06 :     // Left Spin to Backward
											Spin_Start(Left_Spin, Auto_Dir_Information_Buff[Node_Count].Data.Driving_Speed - 1, SpinToBackward, 90, 1, 0);
											break;

										case 0x07 :     // Left Spin to Stop
											Spin_Start(Left_Spin, Auto_Dir_Information_Buff[Node_Count].Data.Driving_Speed - 1, SpinToStop, 90, 1, 1);
											break;

										case 0x08 :     // Backward
											break;

										case 0x09 :     // Mark Enable Stop
											Forward_Mark_Stop_Status = 1;
											StopToLift_Flag = Normal_Stop;
											During_Stop_Action = 1;
											DRV_Complete_Flag = 1;
											break;

										case 0x0A :     // Right Spin to Loading
											Spin_Start(Right_Spin, Auto_Dir_Information_Buff[Node_Count].Data.Driving_Speed - 1, SpinToLoading, 90, 1, 1);
											break;

										case 0x0B :     // Right Spin to Unloading
											Spin_Start(Right_Spin, Auto_Dir_Information_Buff[Node_Count].Data.Driving_Speed - 1, SpinToUnloading, 90, 1, 1);
											break;

										case 0x0C :     // Left Spin to Loading
											Spin_Start(Left_Spin, Auto_Dir_Information_Buff[Node_Count].Data.Driving_Speed - 1, SpinToLoading, 90, 1, 1);
											break;

										case 0x0D :     // Left Sping to Unloading
											Spin_Start(Left_Spin, Auto_Dir_Information_Buff[Node_Count].Data.Driving_Speed - 1, SpinToUnloading, 90, 1, 1);
											break;

										case 0x10 :     // Stop to Loading
											Forward_Mark_Stop_Status = 1;
											StopToLift_Flag = Lift_Loading;
											DRV_Complete_Flag = 1;
											break;

										case 0x11 :     // Stop to Unloading
											Forward_Mark_Stop_Status = 1;
											StopToLift_Flag = Lift_Unloading;
											DRV_Complete_Flag = 1;
											break;
										}
										// }
									}
								}
							}
						}
					else if ( Old_DRV_Buff == Forward_DRV || Old_DRV_Buff == Backward_DRV ){
						Node_Action = Auto_Dir_Information_Buff[Node_Count].Data.AGV_Direction;

						if ( Node_Action >= 2 && Node_Action <= 7 ) Sensor_Spin_Stop_Flag = Node_Action;
						else Sensor_Spin_Stop_Flag = 0;
						}

					if((Last_Node == 0) && (PGV_Tag_Num_Data_Buff != 9000000) ){
						Last_Node = PGV_Tag_Num_Data_Buff;
						Old_Tag_Num = PGV_Tag_Num_Data_Buff;
						AGV_Stop_Function();
						// AGV_Start_Place_Event_Send(Last_Node);
						Node_Chagne_Event_Flag = 1;
						}

					// Old_Tag_Num = PGV_Tag_Num_Data;

					if((Last_Node != 0) && (Last_Node != PGV_Tag_Num_Data_Buff) && (PGV_Tag_Num_Data_Buff != 9000000) ){
						Last_Node = PGV_Tag_Num_Data_Buff;
						Old_Tag_Num = PGV_Tag_Num_Data_Buff;
						// AGV_NodeChange_Event_Send(Last_Node);
						Node_Chagne_Event_Flag = 2;
						}

					Register_Buff[5] = (long)(Last_Node >> 16) & 0xFFFF;
					Register_Buff[4] = Last_Node & 0xFFFF;

					Left_Sample_Count = Right_Sample_Count = 0;

					}
				}
			}

		//if(PGV_Data_Set_Flag == 1)PGV_Data_Set_Flag = 2;

		PGV_Data_Send_Flag = Com1Data_Count = 0;
		}

}

void Charge_Sequence(void)
{
	if(Charge_Seq_Num != 0){
		if(Charge_Seq_Num == 1){

			// if ( ( PIO_IN_PORT.bit.IO1 == 1 ) && ( PIO_IN_PORT.bit.IO2 == 0 ) ) {

					AGVMotionData.Drive = Charge_SEQ;

					DELAY_MS(500);

					PIO_Out_Port.bit.OUT1 = 1;
					ADDR16(PORTH) = PIO_Out_Port.byte.L;

					Charge_Seq_Num = 2;

				// }
			}

		if(Charge_Seq_Num == 2){
			if ( ( PIO_IN_PORT.bit.IO1 == 0 ) && ( PIO_IN_PORT.bit.IO2 == 1 ) ){

				DELAY_MS(200);

				CHARGE_MC_ON();

				Charge_Seq_Num = 0;

				AGVMotionData.Drive = AGV_CHARGING;

				Charge_Check_Count = Charge_Error_Count = 0;

				if(Register_Buff[59] == Auto_Driving)AGV_Charge_Complete_Event_Send();
				}
			}
		}
}


void DisCharge_Sequence(void)
{
	if(DisCharge_Seq_Num != 0){
		if(DisCharge_Seq_Num == 1){

			// if ( ( PIO_IN_PORT.bit.IO1 == 0 ) && ( PIO_IN_PORT.bit.IO2 == 1 ) ){
				AGVMotionData.Drive = Charge_Stop_SEQ;

				CHARGE_MC_OFF();

				DELAY_MS(200);

				PIO_Out_Port.bit.OUT1 = 0;
				ADDR16(PORTH) = PIO_Out_Port.byte.L;

				DisCharge_Seq_Num = 2;
				// }
			}

		if(DisCharge_Seq_Num == 2){
			if ( ( PIO_IN_PORT.bit.IO1 == 1 ) && ( PIO_IN_PORT.bit.IO2 == 0 ) ){

				DELAY_MS(500);

				DisCharge_Seq_Num = 0;

				AGVMotionData.Drive = AGV_Wait;

				if(Register_Buff[59] == Auto_Operate)AGV_Charge_Complete_Event_Send();
				}
			}
		}
}

void Charge_Status_Check ( void )
{
	if ( AGVMotionData.Drive == AGV_CHARGING )
	{
		if ( Charge_Check_Count > 6000 ){

			if ( ( BMS_Summary_Data.BAT_SOC.all < 70 ) && ( BMS_Summary_Data.BAT_Current.all < 0 ) ){
				Charge_Error_Count++;
				}
			else Charge_Error_Count = 0;
			
			Charge_Check_Count = 0;
			}
		}
}

/* ?ž?™ ëª¨ë“œ ?¼?•Œ ë°°í„°ë¦¬ê? 40 ë¯¸ë§Œ?œ¼ë¡? 5ë¶? ?™?•ˆ ?œ ì§??  ê²½ìš° ? „?› OFF */
void Battery_Low_Check ( void )
{
	if ( OPERATE_MODE == Auto_Operate ){
		if ( Battery_Low_Check_Count > 6000 ){   // 60ì´? 
			Battery_Low_Check_Count = 0; 	
			if ( BMS_Summary_Data.BAT_SOC.all < 40 ) Battery_Low_Error_Count++;
			else Battery_Low_Error_Count = 0;

			if ( Battery_Low_Error_Count > 5 ) BAT_POW_Off_Flag = 1; 
			} 
		}
}

/* ?•Œ?žŒ?´ 60ë¶? ?™?•ˆ ?œ ì§? ?  ê²½ìš° ? „?› OFF */
void Alarm_Continue_Check ( void )
{
	if ( Error_Continue_Flag == 1 ) {
		if ( Error_Continue_Count > 6000 ){	// 60ì´?
			Error_Continue_Count = 0;

			Error_1_min_Check_Count++;
			if ( Error_1_min_Check_Count > 60 )  BAT_POW_Off_Flag = 1; 
			}
		}
}


void Forward_Mark_Enable_Stop(void)
{
	int16_t Stop_Signed_Buff = 0, Stop_Unsigned_Buff = 0;
	long X_Buff = 0, Y_Buff = 0;

	Stop_Signed_Buff = Mark_Stop_Dimension * -1;
	Stop_Unsigned_Buff = Mark_Stop_Dimension;

	if(Forward_Mark_Stop_Status > 0){
		// Slow
		if(Forward_Mark_Stop_Status == 1){
			Distance_Sum_Buff = 0;

			Course_Driving_Status = 1;
			Moving_Speed_Set(10);

			Forward_Mark_Stop_Status = 2;
			}

		else if(Forward_Mark_Stop_Status == 2){
			//Mark2 Sensor
			if(AGVMotionData.Drive == Forward_DRV){
				X_Buff = X_Axis_Data + PGV_Y_Axis_Offset;
				Y_Buff = Y_Axis_Data + PGV_Y_Axis_Offset;

				if(Surface_Input == 1){
					if(X_Buff <= Stop_Unsigned_Buff){
						Forward_Mark_Stop_Status = 3;
						// Register_Buff[49] = Distance_Sum_Buff;
						Direct_Stop();
						}
					}
				if(Surface_Input == 3){
					if(X_Buff >= Stop_Signed_Buff){
						Forward_Mark_Stop_Status = 3;
						// Register_Buff[49] = Distance_Sum_Buff;
						Direct_Stop();
						}
					}
				if(Surface_Input == 2){
					if(Y_Buff >= Stop_Signed_Buff){
						Forward_Mark_Stop_Status = 3;
						// Register_Buff[49] = Distance_Sum_Buff;
						Direct_Stop();
						}
					}
				if(Surface_Input == 4){
					if(Y_Buff <= Stop_Unsigned_Buff){
						Forward_Mark_Stop_Status = 3;
						// Register_Buff[49] = Distance_Sum_Buff;
						Direct_Stop();
						}
					}
				}

			if(AGVMotionData.Drive == Backward_DRV){
				X_Buff = X_Axis_Data - PGV_Y_Axis_Offset;
				Y_Buff = Y_Axis_Data - PGV_Y_Axis_Offset;
				if(Surface_Input == 1){
					if(X_Buff >= Stop_Signed_Buff){
						Forward_Mark_Stop_Status = 3;
						// Register_Buff[49] = Distance_Sum_Buff;
						Direct_Stop();
						}
					}
				if(Surface_Input == 3){
					if(X_Buff <= Stop_Unsigned_Buff){
						Forward_Mark_Stop_Status = 3;
						// Register_Buff[49] = Distance_Sum_Buff;
						Direct_Stop();
						}
					}
				if(Surface_Input == 2){
					if(Y_Buff <= Stop_Unsigned_Buff){
						Forward_Mark_Stop_Status = 3;
						// Register_Buff[49] = Distance_Sum_Buff;
						Direct_Stop();
						}
					}
				if(Surface_Input == 4){
					if(Y_Buff >= Stop_Signed_Buff){
						Forward_Mark_Stop_Status = 3;
						// Register_Buff[49] = Distance_Sum_Buff;
						Direct_Stop();
						}
					}
				}
			}

		// Stop -> Spin
		else if(Forward_Mark_Stop_Status == 3){
			if((AGVMotionData.Drive == AGV_Wait) || (AGVMotionData.Drive == Pause_Wait)){

				Course_Reset();
				Forward_Mark_Stop_Status = 0;

				Distance_Sum_Buff = Right_Mag_Count = Left_Mag_Count = 0;

				Course_Driving_Status = 0;
				During_Stop_Action = 0;

				if(StopToLift_Flag == StopToLoading){
					DELAY_MS(500);
					Lift_Job_Comp = 1;
					Lift_Up_GPIO();
					}

				else if(StopToLift_Flag == StopToUnloading){
					DELAY_MS(500);
					Lift_Job_Comp = 1;
					Lift_Down_GPIO();
					}
				else if(StopToLift_Flag == StopToDelayStart ){
					DELAY_MS(500);
					Lift_Job_Comp = 1;
					Forward_Mark_Stop_Status = 4;
					Forward_Mark_Delay = 0;
					Lift_Down_GPIO();
				}

				if((StopToLift_Flag == Normal_Stop) && (DRV_Complete_Flag == 1))AGV_DRV_Complete_Event_Send(Last_JobID);
				else if ( Job_Cancel_Stop_Flag == 3 ){
					Job_Status_Clear();
					Old_JobID = 0;
					}
				}
			}

		else if ( Forward_Mark_Stop_Status == 4){
			if ( Forward_Mark_Delay > 200 ) {
				Forward_Mark_Stop_Status = 5;
				Lift_Up_GPIO();
				}
			}

		else if ( Forward_Mark_Stop_Status == 5 ){
			if ( AGVMotionData.Drive == AGV_Wait ){
				DELAY_MS(1000);
				Moving_Speed_Set(AGV_Speed_Buff[0]);
				Forward_GPIO();
				Forward_Mark_Stop_Status = 0;
				}
			}
		}
}

void Lift_Action_To_Moving(void)
{
	if(Lift_Action_To_Moving_SEQ_Num > 0){
		if(Lift_Action_To_Moving_SEQ_Num == 1){
			Course_Driving_Status = 1;

			Lift_Job_Comp = 1;
			Lift_Up_GPIO();

			Lift_Action_To_Moving_SEQ_Num = 4;
			}

		else if(Lift_Action_To_Moving_SEQ_Num == 2){
			Course_Driving_Status = 1;

			Lift_Job_Comp = 1;
			Lift_Down_GPIO();

			Lift_Action_To_Moving_SEQ_Num = 4;
			}

		else if(Lift_Action_To_Moving_SEQ_Num == 4){
			if((AGVMotionData.Drive == AGV_Wait) && (Lift_Action == Lift_Wait)){
				Lift_Action_To_Moving_SEQ_Num = 0;

				if(Start_Direction == Forward_DRV){
					Moving_Speed_Set(AGV_Speed_Buff[0]);
					Forward_GPIO();
					}

				else if(Start_Direction == Backward_DRV){
					Moving_Speed_Set(AGV_Speed_Buff[0]);
					Backward_GPIO();
					}

				Course_Driving_Status = 0;
				Start_Direction = 0;

				}
			}
		}
}


void Left_Spin_Function(void)
{
	//unsigned int Delay_Buff = 0;
	int16_t Stop_Signed_Buff = 0, Stop_Unsigned_Buff = 0;
	long X_Buff = 0, Y_Buff = 0;
	int16_t Stop_Degree_Cal_Buff = 0;

	Stop_Signed_Buff = Mark_Stop_Dimension * -1;
	Stop_Unsigned_Buff = Mark_Stop_Dimension;

	if(Left_Spin_Function_Sequence > 0){

		// Slow
		if(Left_Spin_Function_Sequence == 1){
			Distance_Sum_Buff = 0;

			Course_Driving_Status = 1;
			Moving_Speed_Set(10);

			Left_Spin_Function_Sequence = 2;
			}

		// Stop Position Check
		else if(Left_Spin_Function_Sequence == 2){
			//Mark2 Sensor
			if((AGVMotionData.Drive == Forward_DRV) && (Error_Flag == 0)){
				X_Buff = X_Axis_Data + PGV_Y_Axis_Offset;
				Y_Buff = Y_Axis_Data + PGV_Y_Axis_Offset;
				if(Surface_Input == 1){
					if(X_Buff <= Stop_Unsigned_Buff){
						Left_Spin_Function_Sequence = 3;
						// Register_Buff[49] = Distance_Sum_Buff;
						Direct_Stop();
						}
					}
				if(Surface_Input == 3){
					if(X_Buff >= Stop_Signed_Buff){
						Left_Spin_Function_Sequence = 3;
						// Register_Buff[49] = Distance_Sum_Buff;
						Direct_Stop();
						}
					}
				if(Surface_Input == 2){
					if(Y_Buff >= Stop_Signed_Buff){
						Left_Spin_Function_Sequence = 3;
						// Register_Buff[49] = Distance_Sum_Buff;
						Direct_Stop();
						}
					}
				if(Surface_Input == 4){
					if(Y_Buff <= Stop_Unsigned_Buff){
						Left_Spin_Function_Sequence = 3;
						// Register_Buff[49] = Distance_Sum_Buff;
						Direct_Stop();
						}
					}
				}
			if((AGVMotionData.Drive == Backward_DRV) && (Error_Flag == 0)){
				X_Buff = X_Axis_Data - PGV_Y_Axis_Offset;
				Y_Buff = Y_Axis_Data - PGV_Y_Axis_Offset;
				if(Surface_Input == 1){
					if(X_Buff >= Stop_Signed_Buff){
						Left_Spin_Function_Sequence = 3;
						// Register_Buff[49] = Distance_Sum_Buff;
						Direct_Stop();
						}
					}
				if(Surface_Input == 3){
					if(X_Buff <= Stop_Unsigned_Buff){
						Left_Spin_Function_Sequence = 3;
						// Register_Buff[49] = Distance_Sum_Buff;
						Direct_Stop();
						}
					}
				if(Surface_Input == 2){
					if(Y_Buff <= Stop_Unsigned_Buff){
						Left_Spin_Function_Sequence = 3;
						// Register_Buff[49] = Distance_Sum_Buff;
						Direct_Stop();
						}
					}
				if(Surface_Input == 4){
					if(Y_Buff >= Stop_Signed_Buff){
						Left_Spin_Function_Sequence = 3;
						// Register_Buff[49] = Distance_Sum_Buff;
						Direct_Stop();
						}
					}
				}
			}

		// Stop -> Spin
		else if(Left_Spin_Function_Sequence == 3){
			if(AGVMotionData.Drive == AGV_Wait){
				DELAY_MS(200);
				Manual_Spin_Flag = Distance_Sum_Buff = 0;

				Moving_Speed_Set(Manual_Spin_Speed);

				Left_Spin_GPIO();

				Left_Spin_Function_Sequence = 4;

				Right_Mag_Count = Left_Mag_Count = 0;
				}
			}

		else if(Left_Spin_Function_Sequence == 4){
			if(On_Tag_Falg == 0)Spinturn_Err_Flag = 1;
			if(Spin_Stop_Scan_Flag){

				Stop_Degree_Cal_Buff = Spin_Stop_Degree;

				if(Spin_Max_Degrees == 0){
					if(((PGV_Angle_Data <= (Spin_Max_Degrees + Stop_Degree_Cal_Buff)) || (PGV_Angle_Data >= (3600 - ( Stop_Degree_Cal_Buff + 200 )))) && (PGV_Angle_Data != 0) ){
						if(Pause_Num == 1)Pause_Num = 2;
						Distance_Sum_Buff = 0;
						Left_Spin_Function_Sequence = 5;
						Direct_Stop();
						Spin_Stop_Scan_Flag = Mark_Stop = 0;
						Sensor_Spin_Stop_Flag = 0;
						}
					}
				else{
					if(PGV_Angle_Data <= (Spin_Max_Degrees + Stop_Degree_Cal_Buff)){
						if(Pause_Num == 1)Pause_Num = 2;
						Distance_Sum_Buff = 0;
						Left_Spin_Function_Sequence = 5;
						Direct_Stop();
						Spin_Stop_Scan_Flag = Mark_Stop = 0;
						Sensor_Spin_Stop_Flag = 0;
						}
					}
				}
			}

		else if(Left_Spin_Function_Sequence == 5){

			if(AGVMotionData.Drive == AGV_Wait){

				if(Forward_Flag == SpinToBackward){

					DELAY_MS(20);

					//Moving_Speed_Set(200);
					Moving_Speed_Set(AGV_Speed_Buff[0]);

					Course_Driving_Status = 0;
					Distance_Sum_Buff = 0;
					Forward_Flag = 0;
					
					Backward_GPIO();
					
					//Left_Spin_Function_Sequence = 6;
					if ( ( Next_Node_Action == 0x01 || Next_Node_Action == 0x08 ) ) Left_Spin_Function_Sequence = 6;
					else Left_Spin_Function_Sequence = 0;
					}

				else if(Forward_Flag == SpinToForward){
					
					DELAY_MS(20);

					//Moving_Speed_Set(200);
					Moving_Speed_Set(AGV_Speed_Buff[0]);

					Course_Driving_Status = 0;
					Distance_Sum_Buff = 0;
					Forward_Flag = 0;

					Forward_GPIO();
					
					//Left_Spin_Function_Sequence = 6;
					if ( ( Next_Node_Action == 0x01 || Next_Node_Action == 0x08 ) ) Left_Spin_Function_Sequence = 6;
					else Left_Spin_Function_Sequence = 0;
					}

				else if(Forward_Flag == SpinToStop){

					DELAY_MS(20);

					if(DRV_Complete_Flag == 1) AGV_DRV_Complete_Event_Send(Last_JobID);
					else if ( Job_Cancel_Stop_Flag == 3 ){
						Job_Status_Clear();
						Old_JobID = 0;
						}

					Course_Driving_Status = 0;
					Distance_Sum_Buff = 0;
					Forward_Flag = 0;

					Spin_Stop_Max_Count = 3;
					Spin_Stop_Max_Count_Buff = 3;

					Left_Spin_Function_Sequence = 0;
					}
				}
			}

		else if(Left_Spin_Function_Sequence == 6){
		 	if(Distance_Sum_Buff >= 1000){
		 		// Moving_Speed_Set(AGV_Speed_Buff[Action_After_Speed]);
		 		
		 		// if ( ( Next_Node_Action == 0x01 || Next_Node_Action == 0x08 ) )  Moving_Speed_Set(AGV_Speed_Buff[1]);
		 		//else Moving_Speed_Set(AGV_Speed_Buff[0]);
		 		Moving_Speed_Set(AGV_Speed_Buff[1]);
		 		Left_Spin_Function_Sequence = 0;
		 		}
		 	}
		}
}

void Right_Spin_Function(void)
{
	//unsigned int Seqeunce_End_Flag = 0;
	//unsigned int Delay_Buff = 0;
	int16_t Stop_Signed_Buff = 0, Stop_Unsigned_Buff = 0;
	long X_Buff = 0, Y_Buff = 0;
	int16_t Stop_Degree_Cal_Buff = 0;

	Stop_Signed_Buff = Mark_Stop_Dimension * -1;
	Stop_Unsigned_Buff = Mark_Stop_Dimension;

	if(Right_Spin_Function_Sequence > 0){

		// Slow Speed
		if(Right_Spin_Function_Sequence == 1){
			Distance_Sum_Buff = 0;

			Course_Driving_Status = 1;
			Moving_Speed_Set(10);
			Right_Spin_Function_Sequence = 2;
			}

		// Stop Position Check
		else if(Right_Spin_Function_Sequence == 2){

			if((AGVMotionData.Drive == Forward_DRV) && (Error_Flag == 0)){
				X_Buff = X_Axis_Data + PGV_Y_Axis_Offset;
				Y_Buff = Y_Axis_Data + PGV_Y_Axis_Offset;
				if(Surface_Input == 1){
					if(X_Buff <= Stop_Unsigned_Buff){
						Right_Spin_Function_Sequence = 3;
						// Register_Buff[49] = Distance_Sum_Buff;
						Direct_Stop();
						}
					}
				if(Surface_Input == 2){
					if(Y_Buff >= Stop_Signed_Buff){
						Right_Spin_Function_Sequence = 3;
						// Register_Buff[49] = Distance_Sum_Buff;
						Direct_Stop();
						}
					}
				if(Surface_Input == 3){
					if(X_Buff >= Stop_Signed_Buff){
						Right_Spin_Function_Sequence = 3;
						// Register_Buff[49] = Distance_Sum_Buff;
						Direct_Stop();
						}
					}
				if(Surface_Input == 4){
					if(Y_Buff <= Stop_Unsigned_Buff){
						Right_Spin_Function_Sequence = 3;
						// Register_Buff[49] = Distance_Sum_Buff;
						Direct_Stop();
						}
					}
				}

			if((AGVMotionData.Drive == Backward_DRV) && (Error_Flag == 0)){
				X_Buff = X_Axis_Data - PGV_Y_Axis_Offset;
				Y_Buff = Y_Axis_Data - PGV_Y_Axis_Offset;
				if(Surface_Input == 1){
					if(X_Buff >= Stop_Signed_Buff){
						Right_Spin_Function_Sequence = 3;
						// Register_Buff[49] = Distance_Sum_Buff;
						Direct_Stop();
						}
					}
				if(Surface_Input == 3){
					if(X_Buff <= Stop_Unsigned_Buff){
						Right_Spin_Function_Sequence = 3;
						// Register_Buff[49] = Distance_Sum_Buff;
						Direct_Stop();
						}
					}
				if(Surface_Input == 2){
					if(Y_Buff <= Stop_Unsigned_Buff){
						Right_Spin_Function_Sequence = 3;
						// Register_Buff[49] = Distance_Sum_Buff;
						Direct_Stop();
						}
					}
				if(Surface_Input == 4){
					if(Y_Buff >= Stop_Signed_Buff){
						Right_Spin_Function_Sequence = 3;
						// Register_Buff[49] = Distance_Sum_Buff;
						Direct_Stop();
						}
					}
				}
			}

		// Stop -> Spin
		else if(Right_Spin_Function_Sequence == 3){

			// if((AGVMotionData.Drive == AGV_Wait) || (AGVMotionData.Drive == Pause_Wait)){
			if(AGVMotionData.Drive == AGV_Wait){
				Manual_Spin_Flag = Distance_Sum_Buff = 0;
				DELAY_MS(200);

				Moving_Speed_Set(Manual_Spin_Speed);
				Right_Spin_GPIO();
				Right_Spin_Function_Sequence = 4;

				Right_Mag_Count = Left_Mag_Count = 0;
				}
			}

		// Spin Moving(Out of Mark)
		else if(Right_Spin_Function_Sequence == 4){
			if(On_Tag_Falg == 0)Spinturn_Err_Flag = 1;
			if(Spin_Stop_Scan_Flag){
				Stop_Degree_Cal_Buff = Spin_Stop_Degree;

				if(Spin_Max_Degrees == 0){
					if(((PGV_Angle_Data <= (Spin_Max_Degrees + ( Stop_Degree_Cal_Buff + 100 ) ) ) || (PGV_Angle_Data >= (3600 - Stop_Degree_Cal_Buff))) && (PGV_Angle_Data != 0) ){
						if(Pause_Num == 1)Pause_Num = 2;
						Distance_Sum_Buff = 0;
						Direct_Stop();
						Spin_Stop_Scan_Flag = Mark_Stop = 0;
						Sensor_Spin_Stop_Flag = 0;
						Right_Spin_Function_Sequence = 5;
						}
					}
				else{
					if(PGV_Angle_Data >= (Spin_Max_Degrees - Stop_Degree_Cal_Buff)){
						if(Pause_Num == 1)Pause_Num = 2;
						Distance_Sum_Buff = 0;
						Direct_Stop();
						Spin_Stop_Scan_Flag = Mark_Stop = 0;
						Sensor_Spin_Stop_Flag = 0;
						Right_Spin_Function_Sequence = 5;
						}
					}
				}
			}

		// Forward Start(End of Left Spin)
		else if(Right_Spin_Function_Sequence == 5){

			if(AGVMotionData.Drive == AGV_Wait){

				if(Forward_Flag == SpinToBackward){

					DELAY_MS(20);

					// Moving_Speed_Set(200);
					Moving_Speed_Set(AGV_Speed_Buff[0]);
					
					Course_Driving_Status = 0;
					Distance_Sum_Buff = 0;
					Forward_Flag = 0;

					Backward_GPIO();

					// Right_Spin_Function_Sequence = 6;
					if ( ( Next_Node_Action == 0x01 || Next_Node_Action == 0x08 ) )  Right_Spin_Function_Sequence = 6;
					else Right_Spin_Function_Sequence = 0;
					}

				else if(Forward_Flag == SpinToForward){

					DELAY_MS(20);

					//Moving_Speed_Set(200);
					Moving_Speed_Set(AGV_Speed_Buff[0]);

					Course_Driving_Status = 0;
					Distance_Sum_Buff = 0;
					Forward_Flag = 0;

					Forward_GPIO();

					// Right_Spin_Function_Sequence = 6;
					if ( ( Next_Node_Action == 0x01 || Next_Node_Action == 0x08 ) )  Right_Spin_Function_Sequence = 6;
					else Right_Spin_Function_Sequence = 0;
					}

				else if(Forward_Flag == SpinToStop){
					
					DELAY_MS(20);

					if(DRV_Complete_Flag == 1)AGV_DRV_Complete_Event_Send(Old_JobID); 		// ??ºÂÃ«Â?? Old_JobID
					else if ( Job_Cancel_Stop_Flag == 3 ){
						Job_Status_Clear();
						}

					Course_Driving_Status = 0;
					Distance_Sum_Buff = 0;
					Forward_Flag = 0;

					Spin_Stop_Max_Count = 3;
					Spin_Stop_Max_Count_Buff = 3;

					Right_Spin_Function_Sequence = 0;
					}
				}
			}

		else if(Right_Spin_Function_Sequence == 6){
			if(Distance_Sum_Buff >= 1000){

				// if ( ( Next_Node_Action == 0x01 || Next_Node_Action == 0x08 ) )  Moving_Speed_Set(AGV_Speed_Buff[1]);
		 		//else Moving_Speed_Set(AGV_Speed_Buff[0]);
		 		Moving_Speed_Set(AGV_Speed_Buff[1]);
				Right_Spin_Function_Sequence = 0;
				}
			}
		}
}

EMG_IO_FRMAE EMG_IO_Buff;
void EMR_Process(void)
{
	unsigned int AGVMotion_ErrorCode_Temp = 0;
	unsigned int Error_Flag_Temp = 0;
	unsigned int AGVMotion_Drive_EMR_Temp = 0;
	unsigned long Distance_Sum_Buff_Temp = 0;
	//unsigned int Near_Sensor_Read_Buff_Temp  = 0;
	unsigned char Lift_PIO_Control_Temp = 0;
	unsigned char Tim_2Area_Sensing_Buff;
	unsigned char Back_Tim_2Area_Sensing_Buff;
	// unsigned char EMG_IO_Buff = 0;
	// unsigned char BUM_IO_Buff = 0;
	// EMG_IO_FRMAE EMG_IO_Buff;
	PORT_IO_FRAME Motor_Err_Read_Temp = {0};
	TIM_IO_FRAME Near_Sensor_Buff, Near_Sensor_Set_Buff;

	AGVMotion_ErrorCode_Temp = AGVMotionData.Error_Code;
	Error_Flag_Temp = Error_Flag;
	AGVMotion_Drive_EMR_Temp = AGVMotionData.Drive;
	Distance_Sum_Buff_Temp = Distance_Sum_Buff;

	Lift_PIO_Control_Temp = Lift_PIO_Control;
	// EMG_Bumper_IO_Buff = (GpioDataRegs.GPBDAT.all >> 16);
	Tim_2Area_Sensing_Buff = Tim_2Area_Sensing;
	Back_Tim_2Area_Sensing_Buff = Back_Tim_2Area_Sensing;

	EMG_IO_Buff.all = EMG_Signal_Read();
	Near_Sensor_Buff.all = Near_Sensor_Read();
	Near_Sensor_Set_Buff.all = TIM_Out_Port.all;

	MOT_ALARM.bit.MT1_ALM = MT1_PORT_AL_IN; 	// Right Motor Alarm
	MOT_ALARM.bit.MT2_ALM = MT3_PORT_AL_IN; 	// Left Motor Alarm
	MOT_ALARM.bit.MT3_ALM = MT2_PORT_AL_IN;		// Lift Motor Alarm 
	// MOT_ALARM.bit.MT4_ALM = MT4_PORT_AL_IN;

	// EMR On
	if( EMG_IO_Buff.bit4.EMG != 0x0F ){
		if( Error_Flag_Temp == 0)Error_Flag_Temp = 1;      // Error Mode
		if( AGVMotion_ErrorCode_Temp != AGV_EMR){
			AGVMotion_ErrorCode_Temp = AGV_EMR;
			}
		}

	else {
		// Bumper
		if((EMG_IO_Buff.bit.BUMP1 == 1) && (Motor_Control_Status != MOTOR_STATUS_STOP)){
		  if(Error_Flag_Temp == 0)Error_Flag_Temp = 1;    // Error Mode
		  if(AGVMotion_ErrorCode_Temp != AGV_Bump_Err)AGVMotion_ErrorCode_Temp = AGV_Bump_Err;
		  }

		else {

			if(AGVMotion_ErrorCode_Temp != AGV_Bump_Err){
				if ( Motor_Control_Status == MOTOR_STATUS_RUN ){
				 	if ( MOT_ALARM.bit.MT1_ALM == 0 ) Mot_Alamr_Cnt[0]++;
				 	else Mot_Alamr_Cnt[0] = 0;

				 	if ( MOT_ALARM.bit.MT2_ALM == 0 ) Mot_Alamr_Cnt[1]++;
				 	else Mot_Alamr_Cnt[1] = 0;

				 	// if ( MOT_ALARM.bit.MT2_ALM == 0 ) Mot_Alamr_Cnt[2]++;
				 	// else Mot_Alamr_Cnt[1] = 0;

				 	if ( Mot_Alamr_Cnt[0] >= 10 || Mot_Alamr_Cnt[1] >= 10 ){
				 		if(Error_Flag_Temp == 0)Error_Flag_Temp = 1;    // Error Mode
						if(AGVMotion_ErrorCode_Temp != AGV_Motor_ALR)AGVMotion_ErrorCode_Temp = AGV_Motor_ALR;
				 		}
				 	}

				Register_Buff[42] = Near_Sensor_Buff.all;

				if ( Near_Sensor_Buff.bit4.T1 ){
					if ( Near_Sensor_Buff.bit.IO1_2 == 1 ) {
						if ( AGVMotion_Drive_EMR_Temp == Forward_DRV ){
							if( Tim_2Area_Sensing_Buff == 0 ) {
								if ( Target_Speed > CAL_TAGET_SPEED(80) ) Left_Target_Speed = Right_Target_Speed = Target_Speed = CAL_TAGET_SPEED(80);
								Tim_2Area_Sensing_Buff = 1;
								}
							Straight_Counre_Reading_Count = 0;
							}
						else if ( AGVMotion_Drive_EMR_Temp == AGV_Near_F_Err ){
							if ( ( Near_Sensor_Buff.bit.IO1_1 == 0 ) && (Course_Driving_Status == 0) ) {
								Error_Flag_Temp = 0;
								AGVMotion_ErrorCode_Temp = AGV_Wait;
								}
							}
						}	

					if ( Near_Sensor_Buff.bit.IO1_1 == 1 ) {
						if( AGVMotion_Drive_EMR_Temp == Forward_DRV ) {
							// if ( Forward_Mark_Stop_Status != 2 &&  Left_Spin_Function_Sequence != 2 && Right_Spin_Function_Sequence != 2 ){
							if(Course_Driving_Status == 0){	
								if(Error_Flag_Temp == 0) Error_Flag_Temp = 1;    // Error Mode
								if(AGVMotion_ErrorCode_Temp != AGV_Near_F_Err){
									AGVMotion_ErrorCode_Temp = AGV_Near_F_Err;
									Front_Safety_Event = 1;
									}
								Straight_Counre_Reading_Count = 0;
								}
							}
						}
					}
				else {
					if ( AGVMotion_Drive_EMR_Temp ==  AGV_Near_F_Err ) {
						Tim_2Area_Sensing_Buff = 0;
						Error_Flag_Temp = 0;
						AGVMotion_ErrorCode_Temp = AGV_Wait;
						}
					if(AGVMotion_Drive_EMR_Temp == Forward_DRV)	{
						if(Tim_2Area_Sensing_Buff == 1){
							Tim_2Area_Sensing_Buff = 0;
							if ( Old_Target_Speed > CAL_TAGET_SPEED(AGV_Speed_Buff[0]) ) Old_Target_Speed = CAL_TAGET_SPEED(AGV_Speed_Buff[0]);
							Left_Target_Speed = Right_Target_Speed = Target_Speed = Old_Target_Speed;
							}
						}
					}

				if ( Near_Sensor_Buff.bit4.T2 ){
					if ( Near_Sensor_Buff.bit.IO2_2 == 1 ) {
						if ( AGVMotion_Drive_EMR_Temp == Backward_DRV ){
							if( Back_Tim_2Area_Sensing_Buff == 0 ){
								if ( Target_Speed > CAL_TAGET_SPEED(80) ) Left_Target_Speed = Right_Target_Speed = Target_Speed =  CAL_TAGET_SPEED(80);
								Back_Tim_2Area_Sensing_Buff = 1;
								}
							Straight_Counre_Reading_Count = 0;
							}
						else if ( AGVMotion_Drive_EMR_Temp == AGV_Near_B_Err ){
							if ( ( Near_Sensor_Buff.bit.IO2_1 == 0 ) && (Course_Driving_Status == 0) ){
								Error_Flag_Temp = 0;
								AGVMotion_ErrorCode_Temp = AGV_Wait;
								}
							}
						}

					if ( Near_Sensor_Buff.bit.IO2_1 == 1 ) {
						if(AGVMotion_Drive_EMR_Temp == Backward_DRV ){
							// if ( Forward_Mark_Stop_Status != 2 &&  Left_Spin_Function_Sequence != 2 && Right_Spin_Function_Sequence != 2 ){
							if(Course_Driving_Status == 0){
								if(Error_Flag_Temp == 0) Error_Flag_Temp = 1;    // Error Mode
								if(AGVMotion_ErrorCode_Temp != AGV_Near_B_Err){
									AGVMotion_ErrorCode_Temp = AGV_Near_B_Err;
									Rear_Safety_Event = 1;
									}
								Straight_Counre_Reading_Count = 0;
								}
							}
						}
					}
				else {
					if ( AGVMotion_Drive_EMR_Temp ==  AGV_Near_B_Err ) {
						Back_Tim_2Area_Sensing_Buff = 0;
						Error_Flag_Temp = 0;
						AGVMotion_ErrorCode_Temp = AGV_Wait;
						}

					if(AGVMotion_Drive_EMR_Temp == Backward_DRV)	{
						if(Back_Tim_2Area_Sensing_Buff == 1)	{
							Back_Tim_2Area_Sensing_Buff = 0;
							if ( Old_Target_Speed > CAL_TAGET_SPEED(AGV_Speed_Buff[0]) ) Old_Target_Speed = CAL_TAGET_SPEED(AGV_Speed_Buff[0]);
							Left_Target_Speed = Right_Target_Speed = Target_Speed = Old_Target_Speed;
							}
						}
					}

				// line Error
				if((AGVMotion_Drive_EMR_Temp == Forward_DRV) || (AGVMotion_Drive_EMR_Temp == Backward_DRV)){
					if(Distance_Sum_Buff_Temp > 30000 ){//16000){ // 1.5m = 1500mm / 0.0915 ??3800
						if(Error_Flag_Temp == 0)Error_Flag_Temp = 1;    // Error Mode
						if(AGVMotion_ErrorCode_Temp != AGV_Line_Error)AGVMotion_ErrorCode_Temp = AGV_Line_Error;
						}

					if ( Forward_Mark_Stop_Status == 2 || Left_Spin_Function_Sequence == 2 || Right_Spin_Function_Sequence == 2){
						if ( Distance_Sum_Buff_Temp > 1500 ){
							if(Error_Flag_Temp == 0)Error_Flag_Temp = 1;    // Error Mode
							if(AGVMotion_ErrorCode_Temp != AGV_Line_Error)AGVMotion_ErrorCode_Temp = AGV_Line_Error;
							}
						}

					if(Line_Err_Include_Flag == 1){ // 1.5m = 1500mm / 0.0915 ??3800
						if(Error_Flag_Temp == 0)Error_Flag_Temp = 1;    // Error Mode
						if(AGVMotion_ErrorCode_Temp != AGV_Direction_Err)AGVMotion_ErrorCode_Temp = AGV_Direction_Err;
						}
					}

				if((AGVMotion_Drive_EMR_Temp == Right_Spin_DRV) || (AGVMotion_Drive_EMR_Temp == Left_Spin_DRV)){
					if(Spinturn_Err_Flag == 1){ // 1.5m = 1500mm / 0.0915 ??3800
						if(Error_Flag_Temp == 0)Error_Flag_Temp = 1;    // Error Mode
						if(AGVMotion_ErrorCode_Temp != AGV_Line_Error)AGVMotion_ErrorCode_Temp = AGV_Line_Error;
						}
					}

				if ( AGVMotion_Drive_EMR_Temp == PIN_Moving ){
					if ( Lift_Error_Cnt > 2500 ) {
						if(Error_Flag_Temp == 0)Error_Flag_Temp = 1;    // Error Mode
						if(AGVMotion_ErrorCode_Temp != Lift_Timeout_Err)AGVMotion_ErrorCode_Temp = Lift_Timeout_Err;
						}

					if ( MOT_ALARM.bit.MT3_ALM == 0 ) Mot_Alamr_Cnt[2]++;
				 	else Mot_Alamr_Cnt[2] = 0;

				 	if ( Mot_Alamr_Cnt[2] >= 10 ){
				 		if(Error_Flag_Temp == 0)Error_Flag_Temp = 1;    // Error Mode
						if(AGVMotion_ErrorCode_Temp != Lift_Motor_ALR)AGVMotion_ErrorCode_Temp = Lift_Motor_ALR;
				 		}
					}

				if ( AGVMotion_Drive_EMR_Temp == AGV_CHARGING ){
					if ( Charge_Error_Count > 30 ){
						if(Error_Flag_Temp == 0)Error_Flag_Temp = 1;
						if ( AGVMotion_ErrorCode_Temp != AGV_Chargine_Err )AGVMotion_ErrorCode_Temp = AGV_Chargine_Err; 
						}
					}

				if(Pallet_Detect_Err_Flag == 1){
					if(Error_Flag_Temp == 0)Error_Flag_Temp = 1;    // Error Mode
					if(AGVMotion_ErrorCode_Temp != Pallet_Detect_Err)AGVMotion_ErrorCode_Temp = Pallet_Detect_Err;
					}

				if ( Brake_Release_Err_Flag == 1 ){
					if(Error_Flag_Temp == 0)Error_Flag_Temp = 1;    // Error Mode
					if(AGVMotion_ErrorCode_Temp != Brake_Release_Err)AGVMotion_ErrorCode_Temp = Brake_Release_Err;
				 	}
				}
			}
		}

	Lift_PIO_Control = Lift_PIO_Control_Temp;
	Error_Flag = Error_Flag_Temp;
	AGVMotionData.Error_Code = AGVMotion_ErrorCode_Temp;
	Tim_2Area_Sensing = Tim_2Area_Sensing_Buff;
	Back_Tim_2Area_Sensing = Back_Tim_2Area_Sensing_Buff;
	Motor_Err_Read = Motor_Err_Read_Temp.all;
}

void Error_Event_Debug ( void )
{
	SEQ_NUM.bit.Seq1 = Forward_Mark_Stop_Status; 
	SEQ_NUM.bit.Seq2 = Right_Spin_Function_Sequence; 
	SEQ_NUM.bit.Seq3 = Left_Spin_Function_Sequence;
	SEQ_NUM.bit.Seq4 = Lift_Action_To_Moving_SEQ_Num;
	Register_Buff[151] = SEQ_NUM.all;

	EVENT_FLAG.bit.E_Course_Status = Course_Driving_Status;
	EVENT_FLAG.bit.E_Pause_Flag = Pause_Flag;
	EVENT_FLAG.bit.E_Pause_Num = Pause_Num;
	EVENT_FLAG.bit.E_F_Tim_Sensing = Tim_2Area_Sensing; 
	EVENT_FLAG.bit.E_R_Tim_Sensing = Back_Tim_2Area_Sensing;
	EVENT_FLAG.bit.E_On_Tag_Flag = On_Tag_Falg;
	EVENT_FLAG.bit.E_Direct_Stop = Direct_Stop_Flag;
	EVENT_FLAG.bit.E_Last_Drive = AGVMotionData.Drive;
	Register_Buff[152] = EVENT_FLAG.all;

	Err_Event_Debug_Flag = 1;
}

void EMR_CODE_Process(void)
{
	if((AGVMotionData.Error_Code == AGV_Bump_Err)&&(AGVMotionData.Drive != AGV_Bump_Err)){

		AGVMotionData.Drive = AGV_Bump_Err;

		PIO_OUT_PIN_ALL = 0;
		PIO_OUT_ADDR_L_Write;
		PIO_OUT_ADDR_H_Write;

		Course_Reset();
		Stop_GPIO();

		LED_Color_Select(3500,0,0);
		LED_Flicker_Flag = 0;
		Flicker_Buff = 0;
		SOUND_OUT_PIN_ALL = 0;
		SOUND_OUT_ADDR_WRITE;

		Error_Continue_Flag = 1;
		AGV_DRV_Event_Send(2);
		}

	else if((AGVMotionData.Error_Code == AGV_Line_Error)&&(AGVMotionData.Drive != AGV_Line_Error)){

		Stop_GPIO();

		LED_Color_Select(3500,0,0);
		LED_Flicker_Flag = 0;
		Flicker_Buff = 0;
		SOUND_OUT_PIN_ALL = 0;
		SOUND_OUT_ADDR_WRITE;

		DELAY_MS(500);

		Error_Continue_Flag = 1;
		AGVMotionData.Drive = AGV_Line_Error;
		AGV_DRV_Event_Send(3);
		}

	else if((AGVMotionData.Error_Code == AGV_Direction_Err)&&(AGVMotionData.Drive != AGV_Direction_Err)){

		AGVMotionData.Drive = AGV_Direction_Err;

		Stop_GPIO();

		LED_Color_Select(3500,0,0);
		LED_Flicker_Flag = 0;
		Flicker_Buff = 0;
		SOUND_OUT_PIN_ALL = 0;
		SOUND_OUT_ADDR_WRITE;

		Error_Continue_Flag = 1;
		AGV_DRV_Event_Send(6);
		}

	else if((AGVMotionData.Error_Code == AGV_Near_F_Err)&&(AGVMotionData.Drive != AGV_Near_F_Err)){
		//if(AGVMotionData.Drive != AGV_Stop){
			AGVMotionData.Drive = AGV_Near_F_Err;

			Stop_GPIO();

			LED_Color_Select(3500,0,0);
			LED_Flicker_Flag = 0;
			Flicker_Buff = 0;
			SOUND_OUT_PIN_ALL = 0;
			SOUND_OUT_ADDR_WRITE;

			AGV_DRV_Event_Send(1);

			//DELAY_MS(400);
			DELAY_MS(200);

			Error_Continue_Flag = 1;
			Front_Safety_Event = 0;
			//}
		}

	else if((AGVMotionData.Error_Code == AGV_Near_B_Err)&&(AGVMotionData.Drive != AGV_Near_B_Err)){
		//if(AGVMotionData.Drive != AGV_Stop){
			AGVMotionData.Drive = AGV_Near_B_Err;

			Stop_GPIO();

			LED_Color_Select(3500,0,0);
			LED_Flicker_Flag = 0;
			Flicker_Buff = 0;
			SOUND_OUT_PIN_ALL = 0;
			SOUND_OUT_ADDR_WRITE;

			AGV_DRV_Event_Send(1);

			DELAY_MS(200);

			Error_Continue_Flag = 1;
			Rear_Safety_Event = 0;
			//}
		}

	else if((AGVMotionData.Error_Code == AGV_EMR)&&(AGVMotionData.Drive != AGV_EMR)){
		AGVMotionData.Drive = AGV_EMR;

		SOUND_OUT_PIN_ALL = 0;
		SOUND_OUT_ADDR_WRITE;

		Stop_GPIO();

		LED_Color_Select(3500,0,0);
		LED_Flicker_Flag = 0;
		Flicker_Buff = 0;
		SOUND_OUT_PIN_ALL = 0;
		SOUND_OUT_ADDR_WRITE;

		Error_Continue_Flag = 1;
		AGV_DRV_Event_Send(5);
		}

	else if((AGVMotionData.Error_Code == AGV_Motor_ALR)&&(AGVMotionData.Drive != AGV_Motor_ALR)){
		AGVMotionData.Drive = AGV_Motor_ALR;

		Mot_Alamr_Cnt[0] = 0;
		Mot_Alamr_Cnt[1] = 0;

		Stop_GPIO();

		LED_Color_Select(3500,0,0);
		LED_Flicker_Flag = 0;
		Flicker_Buff = 0;
		SOUND_OUT_PIN_ALL = 0;
		SOUND_OUT_ADDR_WRITE;

		Error_Continue_Flag = 1;
		AGV_DRV_Event_Send(70);
		}

	else if((AGVMotionData.Error_Code == Pallet_Detect_Err)&&(AGVMotionData.Drive != Pallet_Detect_Err)){
		AGVMotionData.Drive = Pallet_Detect_Err;

		Stop_GPIO();

		LED_Color_Select(3500,0,0);
		LED_Flicker_Flag = 0;
		Flicker_Buff = 0;
		SOUND_OUT_PIN_ALL = 0;
		SOUND_OUT_ADDR_WRITE;

		Error_Continue_Flag = 1;
		AGV_DRV_Event_Send(Pallet_Detect_Err);
		}

	// else if((AGVMotionData.Error_Code == AGV_Carrier_Err)&&(AGVMotionData.Drive != AGV_Carrier_Err)){
	// 	AGVMotionData.Drive = AGV_Carrier_Err;

	// 	LED_Color_Select(3500,0,0);
	// 	LED_Flicker_Flag = 0;
	// 	Flicker_Buff = 0;
	// 	SOUND_OUT_PIN_ALL = 0;
	// 	SOUND_OUT_ADDR_WRITE;


	// 	AGV_DRV_Event_Send(20);
	// 	}

	else if((AGVMotionData.Error_Code == Brake_Release_Err)&&(AGVMotionData.Drive != Brake_Release_Err)){
		// Upper_Cart_Check = 0;
		AGVMotionData.Drive = Brake_Release_Err;

		LED_Color_Select(3500,0,0);
		LED_Flicker_Flag = 0;
		Flicker_Buff = 0;
		SOUND_OUT_PIN_ALL = 0;
		SOUND_OUT_ADDR_WRITE;

		Error_Continue_Flag = 1;
		AGV_DRV_Event_Send(Brake_Release_Err);
		}

	else if((AGVMotionData.Error_Code == Lift_Timeout_Err)&&(AGVMotionData.Drive != Lift_Timeout_Err)){
		AGVMotionData.Drive = Lift_Timeout_Err;

		Lift_Stop_GPIO();

		LED_Color_Select(3500,0,0);
		LED_Flicker_Flag = 0;
		Flicker_Buff = 0;
		SOUND_OUT_PIN_ALL = 0;
		SOUND_OUT_ADDR_WRITE;

		Error_Continue_Flag = 1;
		AGV_DRV_Event_Send(Lift_Timeout_Err);
		} 

	else if((AGVMotionData.Error_Code == Lift_Motor_ALR)&&(AGVMotionData.Drive != Lift_Motor_ALR)){
		AGVMotionData.Drive = Lift_Motor_ALR;

		Lift_Stop_GPIO();
		
		LED_Color_Select(3500,0,0);
		LED_Flicker_Flag = 0;
		Flicker_Buff = 0;
		SOUND_OUT_PIN_ALL = 0;
		SOUND_OUT_ADDR_WRITE;

		Error_Continue_Flag = 1;
		AGV_DRV_Event_Send(Lift_Motor_ALR);
		} 

	else if((AGVMotionData.Error_Code == AGV_Chargine_Err)&&(AGVMotionData.Drive != AGV_Chargine_Err)){
		AGVMotionData.Drive = AGV_Chargine_Err;

		Charge_Error_Count = Charge_Check_Count = 0;
		
		ADDR16(PORTH) = PIO_Out_Port.byte.L = 0;

		Error_Continue_Flag = 1;
		AGV_DRV_Event_Send(AGV_Chargine_Err);
		} 
}

void STOP_WAIT_Process(void)
{
	if( Motor_Control_Status == MOTOR_STATUS_STOP ){
		if(AGVMotionData.Drive == AGV_Bump_Err){
			}

		else if(AGVMotionData.Drive == AGV_Near_F_Err){
			Left_Target_Speed = 0;
			Right_Target_Speed = 0;

			LED_Color_Select(0,3500,0);
			LED_Flicker_Flag = 1;
			Flicker_Buff = 0;
			SOUND_OUT_PIN_ALL = Sound1;
			SOUND_OUT_ADDR_WRITE;

			Error_Continue_Flag = 0;
			Error_Continue_Count = 0;
			Error_1_min_Check_Count = 0;

			DELAY_MS(300);

			if ( ( On_Tag_Falg == 1 ) && (Sensor_Spin_Stop_Flag != 0 ) ){

				Moving_Speed_Set(20);
				Forward_GPIO();				
				}
			else {
				if ( Tim_2Area_Sensing == 1 ){
					Left_Target_Speed = Right_Target_Speed = Target_Speed = CAL_TAGET_SPEED(80);
					}
				else {
					if ( Old_Target_Speed > CAL_TAGET_SPEED(AGV_Speed_Buff[0]) ) Old_Target_Speed = CAL_TAGET_SPEED(AGV_Speed_Buff[0]);
					Left_Target_Speed = Right_Target_Speed = Target_Speed = Old_Target_Speed;
					}

				Forward_GPIO();
				}
			}

		else if(AGVMotionData.Drive == AGV_Near_B_Err){
			Left_Target_Speed = 0;
			Right_Target_Speed = 0;

			LED_Color_Select(0,3500,0);
			LED_Flicker_Flag = 1;
			Flicker_Buff = 0;
			SOUND_OUT_PIN_ALL = Sound1;
			SOUND_OUT_ADDR_WRITE;

			Error_Continue_Flag = 0;
			Error_Continue_Count = 0;
			Error_1_min_Check_Count = 0;

			DELAY_MS(300);

			if ( ( On_Tag_Falg == 1 ) && ( Sensor_Spin_Stop_Flag != 0 ) ){
				// switch (Sensor_Spin_Stop_Flag){
				// 	case 0x02 : Spin_Start(Right_Spin, 0, SpinToForward, 90, 3, 0);	break;
				// 	case 0x03 :	Spin_Start(Right_Spin, 0, SpinToBackward, 90, 3, 0); break;
				// 	case 0x04 : Spin_Start(Right_Spin, 0, SpinToStop, 90, 3, 1); break;
				// 	case 0x05 : Spin_Start(Left_Spin, 0, SpinToForward, 90, 3, 0); break;
				// 	case 0x06 : Spin_Start(Left_Spin, 0, SpinToBackward, 90, 3, 0); break;
				// 	case 0x07 : Spin_Start(Left_Spin, 0, SpinToStop, 90, 3, 1); break;
				// 	}
				Moving_Speed_Set(20);
				Backward_GPIO();		
				}
			else {
				if ( Back_Tim_2Area_Sensing == 1 ){
					Left_Target_Speed = Right_Target_Speed = Target_Speed = CAL_TAGET_SPEED(80);
					}
				else {
					if ( Old_Target_Speed > CAL_TAGET_SPEED(AGV_Speed_Buff[0]) ) Old_Target_Speed = CAL_TAGET_SPEED(AGV_Speed_Buff[0]);
					Left_Target_Speed = Right_Target_Speed = Target_Speed = Old_Target_Speed;
					}

				Backward_GPIO();
				}
			}

		else if(AGVMotionData.Drive == AGV_Low_Battery){

			}

		else if(AGVMotionData.Drive == AGV_CHARGING){

			}

		else if(AGVMotionData.Drive == Pause_Wait){
			if ( Pause_Num == 0 ){
				Pause_Flag = 0;
				AGVMotionData.Drive = AGV_Wait;
				}
			}

		else if((AGVMotionData.Drive != AGV_Wait) && (AGVMotionData.Drive != PIN_Moving) && (AGVMotionData.Drive != IO_Test_Mode)
			&& (AGVMotionData.Drive != Pause_Wait) && (AGVMotionData.Drive != Charge_SEQ ) && (AGVMotionData.Drive != Charge_Stop_SEQ )){

			if(Pause_Num == 2){
				AGVMotionData.Drive = Pause_Wait;
				LED_Color_Select(1500,1000,1500);
				LED_Flicker_Flag = 1;
				Flicker_Buff = 0;
				SOUND_OUT_PIN_ALL = 0;
				SOUND_OUT_ADDR_WRITE;
				}

			else { 			//if(Pause_Num == 0){
				AGVMotionData.Drive = AGV_Wait;
				}

			if(Tim_2Area_Sensing != 0) Tim_2Area_Sensing = 0;
			if(Back_Tim_2Area_Sensing != 0) Back_Tim_2Area_Sensing = 0;
			Direct_Stop_Flag = 0;
			}
		}
}

void LCD_Display_Process(void)
{
	LCD_Display_Count++;
	if(LCD_Display_Count >= 10){

		LCD_Display_Count = 0;
		}
}


unsigned int Battery_Charge_Ratio(void)
{
	unsigned int bat;

	if( ADC_Val_Buff[0] > PLow_Data ){

		bat = ADC_Val_Buff[0] - PLow_Data;

		bat = (unsigned long)(bat * 100) / ( P25V_Data - PLow_Data);
	}
	else bat = 0;

	if( bat > 100 )bat = 100;

	return (bat);
}

unsigned int Low_Battery_Count = 0;
unsigned int Low_Battery_State = 0;
void Battery_Process(void)
{
	if((AGV_RunData.Battery < 8)&&(Low_Battery_State == 0)){
		Low_Battery_Count += 1;

		if(Low_Battery_Count > 100){
			Low_Battery_Count = 0;
			Low_Battery_State = 1;
			}
		}
	else if((AGV_RunData.Battery > 30)&&(Low_Battery_State == 1)){
		Low_Battery_Count += 1;

		if(Low_Battery_Count > 100){
			Low_Battery_Count = 0;
			Low_Battery_State = 0;
			}
		}
}

unsigned int Sound_Count = 0;
unsigned int Sound_Change_Count = 0;
void Sound_LED_Process(void)
{
	Sound_Count += 1;

	if(Sound_Count > 2){
		Sound_Count = 0;

		if((AGVMotionData.Drive == Forward_DRV)||(AGVMotionData.Drive == Backward_DRV) ||
			(AGVMotionData.Drive == Right_Spin_DRV) || (AGVMotionData.Drive == Left_Spin_DRV)){
			if( SOUND_OUT_PIN_ALL != Sound1 ){
				LED_Color_Select(0,18000,0);
				LED_Flicker_Flag = 1;
				Flicker_Buff = 0;
				SOUND_OUT_PIN_ALL = Sound1;
				SOUND_OUT_ADDR_WRITE;
				}
			}

		else if(AGVMotionData.Drive == AGV_CHARGING){
			if( ( SOUND_OUT_PIN_ALL != 0 ) || ( ETC_IO.bit.LED_BLU != 1 ) ){
				LED_Color_Select(0,0,18000);
				LED_Flicker_Flag = 0;
				Flicker_Buff = 0;
				SOUND_OUT_PIN_ALL = 0;
				SOUND_OUT_ADDR_WRITE;
				}
			}

		else if(AGVMotionData.Drive == Pause_Wait){
			if ( SOUND_OUT_PIN_ALL != 0 ){
				LED_Color_Select(1500,1000,1500);
				LED_Flicker_Flag = 1;
				Flicker_Buff = 0;
				SOUND_OUT_PIN_ALL = Sound4;
				SOUND_OUT_ADDR_WRITE;
				}
			}

		else if(AGVMotionData.Drive == AGV_Wait){

			if(Low_Battery_State == 1){
				if( SOUND_OUT_PIN_ALL != Sound4 ){
					AGVMotionData.Drive = AGV_Low_Battery;
					LED_Color_Select(18000,0,0);
					LED_Flicker_Flag = 1;
					Flicker_Buff = 0;
					SOUND_OUT_PIN_ALL = Sound4;
					SOUND_OUT_ADDR_WRITE;
					}
				}

			else if( SOUND_OUT_PIN_ALL != 0 ){
				LED_Color_Select(18000,18000,0);
				LED_Flicker_Flag = 0;
				Flicker_Buff = 0;
				SOUND_OUT_PIN_ALL = 0;
				SOUND_OUT_ADDR_WRITE;
				}

			else{
				LED_Color_Select(18000,18000,0);
				}
			}

		else if(AGVMotionData.Drive == Manual_DRV){
			if( SOUND_OUT_PIN_ALL != Sound1 ){
				LED_Color_Select(0,18000,0);
				LED_Flicker_Flag = 1;
				Flicker_Buff = 0;
				SOUND_OUT_PIN_ALL = Sound1;
				SOUND_OUT_ADDR_WRITE;
				}
			}

		else if((AGVMotionData.Drive == AGV_Bump_Err) || (AGVMotionData.Drive == AGV_Near_F_Err) || (AGVMotionData.Drive == AGV_Near_B_Err)){
			if( SOUND_OUT_PIN_ALL != Sound4 ){
				LED_Color_Select(18000,0,0);
				LED_Flicker_Flag = 1;
				Flicker_Buff = 0;
				SOUND_OUT_PIN_ALL = Sound4;
				SOUND_OUT_ADDR_WRITE;
				}
			}

		else if((AGVMotionData.Drive == AGV_EMR)||(AGVMotionData.Drive == AGV_Line_Error)
			||(AGVMotionData.Drive == AGV_Motor_ALR) || (AGVMotionData.Drive == AGV_Carrier_Err)
			||(AGVMotionData.Drive == AGV_Direction_Err) ||(AGVMotionData.Drive == Brake_Release_Err)
			||(AGVMotionData.Drive == Pallet_Detect_Err) ||(AGVMotionData.Drive == Lift_Timeout_Err)
			||(AGVMotionData.Drive == Lift_Motor_ALR) || (AGVMotionData.Drive == AGV_Chargine_Err)){
			if( SOUND_OUT_PIN_ALL != Sound4 ){
				LED_Color_Select(3500,0,0);
				LED_Flicker_Flag = 0;
				Flicker_Buff = 0;
				SOUND_OUT_PIN_ALL = Sound4;
				SOUND_OUT_ADDR_WRITE;
				}
			}
		}

}



// void R2100_Data_Req_Function(unsigned char Sen_Num, unsigned char Count)
// {
// 	// can_data_to_send_1.data0 = 0x4000;
// 	// can_data_to_send_1.data1 = 0x2000;
// 	// can_data_to_send_1.data1 |= Count;

// 	can1Tx0Data[0] = 0x40;
// 	can1Tx0Data[1] = 0x00;
// 	can1Tx0Data[2] = 0x20;
// 	can1Tx0Data[3] = Count;
// 	can1Tx0Data[4] = 0;
// 	can1Tx0Data[5] = 0;
// 	can1Tx0Data[6] = 0;
// 	can1Tx0Data[7] = 0;

// 	if(Sen_Num == 0) CAN_Send_Data(&hcan1, R2100_1Send_ID, STD_Mode, 8, can1Tx0Data);
// 	else if(Sen_Num == 1) CAN_Send_Data(&hcan2, R2100_2Send_ID, STD_Mode, 8, can1Tx0Data);

// }

void AGV_DRV_Parameter_Setting(void)
{
	unsigned int i;
	uint16_t EEP_Buff[50];

	for(i = 0; i < 12; i++){
		EEP_Buff[i] = EEPROM_Read((i * 2), 2);
		DELAY_US(10);
		}

	for(i = 0; i < 6; i++){
		EEP_Buff[i + 12] = EEPROM_Read((i * 2) + 24, 2);
		DELAY_US(10);
		}

	// for(i = 0; i < 14; i++){
	// 	EEP_Buff[i + 18] = EEPROM_Read((i * 2) + 36, 2);
	// 	DELAY_US(10);
	// 	}
	for(i = 0; i < 16; i++){
		EEP_Buff[i + 18] = EEPROM_Read((i * 2) + 36, 2);
		DELAY_US(10);
		}

	EEP_Buff[0] = EEPROM_Read(0, 2);
	DELAY_US(10);

	if(EEP_Buff[0] != 0xFFFF)AGV_Number = EEP_Buff[0];

	for(i = 1; i < 5; i++){
		if(EEP_Buff[i] != 0xFFFF)AGV_Speed_Buff[i - 1] = EEP_Buff[i];
		}

	for(i = 5; i < 8; i++){
		if(EEP_Buff[i] != 0xFFFF)PID_Gain_Buff[i - 5] = EEP_Buff[i];
		}

	if(EEP_Buff[8] != 0xFFFF)Distance_Cal_Buff = EEP_Buff[8];

	if(EEP_Buff[9] != 0xFFFF)Angle_Cal_Buff = EEP_Buff[9];
	Register_Buff[31] = Angle_Cal_Temp = Angle_Cal_Buff;

	if(EEP_Buff[10] != 0xFFFF)Manual_AGV_Speed = EEP_Buff[10];

	if(EEP_Buff[11] != 0xFFFF)Manual_Spin_Speed = EEP_Buff[11];

	if(EEP_Buff[12] != 0xFFFF)PGV_X_Axis_Offset = EEP_Buff[12];
	if(EEP_Buff[13] != 0xFFFF)PGV_Y_Axis_Offset = EEP_Buff[13];
	if(EEP_Buff[14] != 0xFFFF)PGV_Angle_Offset =  EEP_Buff[14];

	if(EEP_Buff[15] != 0xFFFF)Spin_P_Factor = EEP_Buff[15];
	if(EEP_Buff[16] != 0xFFFF)Spin_I_Factor = EEP_Buff[16];
	if(EEP_Buff[17] != 0xFFFF)Spin_D_Factor = EEP_Buff[17];

	if(EEP_Buff[18] != 0xFFFF)Mark_Stop_Dimension = EEP_Buff[18];
	if(EEP_Buff[19] != 0xFFFF)Spin_Stop_Degree = EEP_Buff[19];
	if(EEP_Buff[20] != 0xFFFF)Line_X_Correction = EEP_Buff[20];
	if(EEP_Buff[21] != 0xFFFF)Motor_Speed_Sum = EEP_Buff[21];
	if(EEP_Buff[22] != 0xFFFF)Line_Deg_Correction = EEP_Buff[22];
	if(EEP_Buff[23] != 0xFFFF)Motor_Speed_Divide = EEP_Buff[23];
	if(EEP_Buff[24] != 0xFFFF)HighSpeed_Y_Axis_Cal_Data = EEP_Buff[24];
	if(EEP_Buff[25] != 0xFFFF)HighSpeed_Distance_Angle = EEP_Buff[25];
	if(EEP_Buff[26] != 0xFFFF)Motor_Balance = EEP_Buff[26];
	if(EEP_Buff[27] != 0xFFFF)HSpeed_Angle_Cal_Buff = EEP_Buff[27];
	if(EEP_Buff[28] != 0xFFFF)Torque_Cal_Buff = EEP_Buff[28];
	if(EEP_Buff[29] != 0xFFFF)AGV_Manual_Course_Num = EEP_Buff[29];
	if(EEP_Buff[30] != 0xFFFF)LowSpeed_Y_Axis_Cal_Data = EEP_Buff[30];
	if(EEP_Buff[31] != 0xFFFF)LowSpeed_Distance_Angle = EEP_Buff[31];
	if(EEP_Buff[32] != 0xFFFF)Lift_DA_Value = EEP_Buff[32];
	if(EEP_Buff[33] != 0xFFFF)Spin_Slow_Start_Buff = EEP_Buff[33];

	Spin_Slow_Start_180Count = Spin_180_Encoder_Pulse - ((Spin_180_Encoder_Pulse / 100) * Spin_Slow_Start_Buff);
	Spin_Slow_Start_90Count = Spin_90_Encoder_Pulse - ((Spin_90_Encoder_Pulse / 100) * Spin_Slow_Start_Buff);

	Register_Buff[22] = AGV_Number;
}

// unsigned char HMI_Read_Buff[40] = {0};

// MC_FRAME_STRUCT McFrame;
unsigned int tt = 0;
unsigned int Dum_Buff = 0;

unsigned char SCIA_Communication_Control(void)
{
	uint16_t CRC_Buff = 0, CRC_Check_Buff = 0, CRC_Check_Count = 0;
	uint16_t CRC_Make_Buff;
	uint16_t Coil_Bit_Addr = 0, Coil_Bit_Count = 0, Coil_Word_Addr = 0, Coil_Word_Count = 0, Coil_Bit_Buff = 0;
	uint16_t Register_Write_Data = 0; //  Coil_Force_Data = 0,
	uint16_t Register_Addr = 0, Register_Data_Count = 0;
	int8_t Send_Data_Buff[40];  //, Debug_Data_Buff[20] = {0};
	uint8_t i;

	uint16_t EEP_Addr = 0;
	// unsigned int tim_buff = 0;

	if(SCIA_Read_Check()){
		Modbus_Frame_Ack = Modbus_Frame_Rev;
		Com0_Data[Com0Data_Count++] = SCIA_Read_Char();
		Modbus_Check_Count = 0;
		}

	if(Modbus_Frame_Ack == Modbus_Rev_Complete){

		CRC_Buff = (Com0_Data[Com0Data_Count-1] << 8) | Com0_Data[Com0Data_Count-2];

		CRC_Check_Buff = GetCRC16((char *)&Com0_Data, Com0Data_Count - 2, 0);

		if(CRC_Buff == CRC_Check_Buff){

			if( Com0_Data[0] == Modbus_Slave_ID )
			{
				if( Com0_Data[1] == Single_Coil_Write ){
					Coil_Bit_Addr = (Com0_Data[2] << 8) | Com0_Data[3];
					// Coil_Force_Data = Com0_Data[4];

					Coil_Word_Addr = ( Coil_Bit_Addr - 1 ) / 16;
					Coil_Bit_Buff = ( Coil_Bit_Addr - 1 ) % 16;

					if ( Com0_Data[4] == 0xFF ) {
						Coil_Buff[Coil_Word_Addr] |= ( 1 << Coil_Bit_Buff );
						// Printf("\n%d->ON\n",Coil_Bit_Addr);
					}
					else if ( Com0_Data[4] == 0x00 ) {
						Coil_Buff[Coil_Word_Addr] &= ~( 1 << Coil_Bit_Buff );
						// Printf("\n%d->OF\n",Coil_Bit_Addr);
					}
					// else
					// {
					// 	Printf("\n%d->UN\n",Coil_Bit_Addr);
					// }

					switch(Coil_Bit_Addr){
						case 1 :
							if(AGVMotionData.Drive == AGV_Wait){
								Key_Input_Stop = 0;
								Moving_Speed_Set(AGV_Speed_Buff[0]);
								Forward_GPIO();

								Sample_Acq_Flag = 1;
								}
							break;

						case 2 :
							if(AGVMotionData.Drive == AGV_Wait){
								Key_Input_Stop = 0;
								Moving_Speed_Set(AGV_Speed_Buff[0]);
								Backward_GPIO();

								Sample_Acq_Flag = 1;
								}
							break;

						case 3 :
							if((AGVMotionData.Drive == Forward_DRV)||(AGVMotionData.Drive == Backward_DRV)){
								PreDriving_State_By_Key = AGVMotionData.Drive;
								Key_Input_Stop = 1;
								}

								Manual_Spin_Flag = 1;
								AGV_Stop_Function();
							break;

						case 4 :
							if(Com0_Data[4] == 0xFF){
								// if( OPERATE_MODE == Auto_Operate){
								// 	OPERATE_MODE = Manual_Operate;
								// 	AGV_DRV_Event_Send(30);
								// }
								// else if( OPERATE_MODE == Manual_Operate){
								// 	OPERATE_MODE = Auto_Operate;
								// 	AGV_DRV_Event_Send(31);
								// }
							}
							break;

						case 5 :
							if(Com0_Data[4] == 0xFF){
								if(AGVMotionData.Drive == AGV_Wait){
									Moving_Speed_Set(Manual_AGV_Speed);
									Forward_GPIO();
									AGVMotionData.Drive = Manual_DRV;
									Manual_Spin_Flag = 1;
									}
								}
							else{
								AGV_Stop_Function();
								}
							break;

						case 6 :
							if(Com0_Data[4] == 0xFF){
								if(AGVMotionData.Drive == AGV_Wait){
									Moving_Speed_Set(Manual_AGV_Speed);
									Backward_GPIO();
									AGVMotionData.Drive = Manual_DRV;
									Manual_Spin_Flag = 1;
									}
								}
							else{
								AGV_Stop_Function();
								}
							break;

						///Manual Backward///////
						case 7 :
							if(Com0_Data[4] == 0xFF){
								if(AGVMotionData.Drive == AGV_Wait){
									Moving_Speed_Set(Manual_Spin_Speed);
									//Left_Target_Speed = Right_Target_Speed = Target_Speed = 1024;
									Right_Spin_GPIO();
									AGVMotionData.Drive = Manual_DRV;
									Manual_Spin_Flag = 1;
									}
								}
							else{
								AGV_Stop_Function();
								}
							break;

						///Manual Forward///////
						case 8 :
							if(Com0_Data[4] == 0xFF){
								Moving_Speed_Set(Manual_Spin_Speed);
								//Left_Target_Speed = Right_Target_Speed = Target_Speed = 1024;
								Left_Spin_GPIO();
								AGVMotionData.Drive = Manual_DRV;
								Manual_Spin_Flag = 1;
								}
							else{
								AGV_Stop_Function();
								}
							break;

						////RFID Write!!//////////
						case 9 :
							/*
							SCIC_TX_String(">3\r\n");
							Rfid_Write_Flag = 1;
							sprintf((char *)&RFID_TX_Data,">w 1 2 %02X%02X%02X%02X0000000000000000\r\n", Register_Buff[Write_RFID_No1], Register_Buff[Write_RFID_No2], Register_Buff[Write_RFID_No3], Register_Buff[Write_RFID_No4],CheckSum_Data);
							SCIC_TX_String((char *)&RFID_TX_Data);
							*/
							break;

						case 10 :
							if(AGVMotionData.PIN_State == Lift_Up)Lift_Down_GPIO();
							if(AGVMotionData.PIN_State == Lift_Down)Lift_Up_GPIO();
							break;

						///Dir_EEPROM_READ///////
						case 11 :
							if(Dir_Setting_Frame.Data.Course_ID > 0){
								EEP_Addr = ((Course_Addr_Offset * (Dir_Setting_Frame.Data.Course_ID - 1)) + Parameter_Add_Offset) + (Dir_Setting_Frame.Data.Dir_RFID * Tag_Addr_Num);

								for(i = 0; i < 7; i++){
									Dir_Setting_Frame.Buff[2 + i] = AT25128_READ_EEPROM(EEP_Addr + i);
									DELAY_MS(5);
									}

								for(i = 0; i < 9; i++){
									Register_Buff[i + 13] = Dir_Setting_Frame.Buff[i];
									}
								}

							break;

						////Dir_EEPROM_WRITE////////
						case 12 :
							if(Dir_Setting_Frame.Data.Course_ID > 0){
								EEP_Addr = ((Course_Addr_Offset * (Dir_Setting_Frame.Data.Course_ID - 1)) + Parameter_Add_Offset) + (Dir_Setting_Frame.Data.Dir_RFID * Tag_Addr_Num);

								AT25128_WP_High;
								DELAY_US(10);

								for(i = 0; i < 7; i++){
									AT25128_EEPROM_Control(WREN);
									AT25128_Write_EEPROM(EEP_Addr + i, Dir_Setting_Frame.Buff[i + 2]);
									//AT25128_Write_EEPROM(i + 1, 0);
									AT25128_EEPROM_Control(WRDI);
									DELAY_MS(5);
									}

								DELAY_US(10);
								AT25128_WP_Low;
								}

							break;

						///Para_EEPROM_READ///////
						case 13 :
							Register_Buff[22] = AGV_Number;

							for(i = 0; i < 4; i++){
								Register_Buff[23+i] = AGV_Speed_Buff[i];
								}
							for(i = 0; i < 3; i++){
								Register_Buff[27+i] = PID_Gain_Buff[i];
								}

							Register_Buff[30] = Distance_Cal_Buff;

							Register_Buff[31] = Angle_Cal_Buff;

							Register_Buff[32] = Manual_AGV_Speed;

							Register_Buff[33] = Manual_Spin_Speed;

							Register_Buff[53] = PGV_X_Axis_Offset;
							Register_Buff[54] = PGV_Y_Axis_Offset;
							Register_Buff[55] = PGV_Angle_Offset;

							Register_Buff[56] = Spin_P_Factor;
							Register_Buff[57] = Spin_I_Factor;
							Register_Buff[58] = Spin_D_Factor;

							Register_Buff[87] = Mark_Stop_Dimension;
							Register_Buff[88] = Spin_Stop_Degree;

							Register_Buff[89] = Line_X_Correction;
							Register_Buff[90] = Motor_Speed_Sum;
							Register_Buff[91] = Line_Deg_Correction;
							Register_Buff[92] = Motor_Speed_Divide;
							//Register_Buff[93] = Y_Axis_Cal_Data;
							//Register_Buff[94] = Distance_Angle;
							Register_Buff[93] = HighSpeed_Y_Axis_Cal_Data;
							Register_Buff[94] = HighSpeed_Distance_Angle;

							Register_Buff[95] = Motor_Balance;
							Register_Buff[96] = HSpeed_Angle_Cal_Buff;
							Register_Buff[97] = Torque_Cal_Buff;

							Register_Buff[98] = AGV_Manual_Course_Num;

							Register_Buff[99] = LowSpeed_Y_Axis_Cal_Data;
							Register_Buff[100] = LowSpeed_Distance_Angle;
							Register_Buff[101] = Lift_DA_Value;
							Register_Buff[102] = Spin_Slow_Start_Buff;
							break;

						////Para_EEPROM_WRITE////////
						case 14 :

							AT25128_WP_High;
							DELAY_US(10);

							for(i = 0; i < 12; i++){
								EEPROM_Write((i*2), Register_Buff[22 + i], 2);
								DELAY_US(10);
								}

							for(i = 0; i < 6; i++){
								EEPROM_Write((i*2)+24, Register_Buff[53 + i], 2);
								DELAY_US(10);
								}

							for(i = 0; i < 16; i++){
								EEPROM_Write((i*2)+36, Register_Buff[87 + i], 2);
								DELAY_US(10);
								}

							DELAY_US(10);
							AT25128_WP_Low;

							AGV_Number = Register_Buff[22];

							for(i = 0; i < 4; i++){
								AGV_Speed_Buff[i] = Register_Buff[23+i];
								}
							for(i = 0; i < 3; i++){
								PID_Gain_Buff[i] = Register_Buff[27+i];
								}


							P_Factor = ((float)PID_Gain_Buff[P_Gain] / 10) * SCALING_FACTOR;
							I_Factor = ((float)PID_Gain_Buff[I_Gain] / 10) * SCALING_FACTOR;
							D_Factor = ((float)PID_Gain_Buff[D_Gain] / 10) * SCALING_FACTOR;

							Distance_Cal_Buff = Register_Buff[30];

							Angle_Cal_Buff = Register_Buff[31];

							Manual_AGV_Speed = Register_Buff[32];

							Manual_Spin_Speed = Register_Buff[33];

							PGV_X_Axis_Offset = Register_Buff[53];
							PGV_Y_Axis_Offset = Register_Buff[54];
							PGV_Angle_Offset = Register_Buff[55];

							Spin_P_Factor = Register_Buff[56];
							Spin_I_Factor = Register_Buff[57];
							Spin_D_Factor = Register_Buff[58];

							Mark_Stop_Dimension = Register_Buff[87];
							Spin_Stop_Degree = Register_Buff[88];
							Line_X_Correction = Register_Buff[89];
							Motor_Speed_Sum = Register_Buff[90];
							Line_Deg_Correction = Register_Buff[91];
							Motor_Speed_Divide = Register_Buff[92];
							//Y_Axis_Cal_Data = Register_Buff[93];
							//Distance_Angle = Register_Buff[94];
							HighSpeed_Y_Axis_Cal_Data = Register_Buff[93];
							HighSpeed_Distance_Angle = Register_Buff[94];

							Motor_Balance = Register_Buff[95];
							HSpeed_Angle_Cal_Buff = Register_Buff[96];
							Torque_Cal_Buff = Register_Buff[97];

							AGV_Manual_Course_Num = Register_Buff[98];

							LowSpeed_Y_Axis_Cal_Data = Register_Buff[99];
							LowSpeed_Distance_Angle = Register_Buff[100];
							Lift_DA_Value = Register_Buff[101];
							Spin_Slow_Start_Buff = Register_Buff[102];

							Spin_Slow_Start_180Count = Spin_180_Encoder_Pulse - ((Spin_180_Encoder_Pulse / 100) * Spin_Slow_Start_Buff);
							Spin_Slow_Start_90Count = Spin_90_Encoder_Pulse - ((Spin_90_Encoder_Pulse / 100) * Spin_Slow_Start_Buff);
							break;

						case 15 :
							for(i = 0; i < 12; i++){
								Register_Buff[22 + i] =  EEPROM_Read((i * 2), 2);//AT25128_READ_EEPROM(i);
								DELAY_US(10);
								}
							for(i = 0; i < 6; i++){
								Register_Buff[53 + i] =  EEPROM_Read((i * 2) + 24, 2);//AT25128_READ_EEPROM(i);
								DELAY_US(10);
								}
							for(i = 0; i < 16; i++){
								Register_Buff[87 + i] =  EEPROM_Read((i * 2) + 36, 2);//AT25128_READ_EEPROM(i);
								DELAY_US(10);
								}

							break;

						case 16 :
							if(Dir_Setting_Frame.Data.Course_ID > 0){
								EEP_Addr = ((Course_Addr_Offset * (Dir_Setting_Frame.Data.Course_ID - 1)) + Parameter_Add_Offset) + (Dir_Setting_Frame.Data.Dir_RFID * Tag_Addr_Num);

								for(i = 0; i < 7; i++){
									Dir_Setting_Frame.Buff[2 + i] = AT25128_READ_EEPROM(EEP_Addr + i);
									DELAY_MS(5);
									}

								for(i = 0; i < 9; i++){
									Register_Buff[i + 13] = Dir_Setting_Frame.Buff[i];
									}
								}
							break;

						case 18 :
							break;

						case 19 :
							if(Com0_Data[4] == 0xFF){
									Motor_Relay_ON();

								 // MT1_PORT_START_PIN = 1;
									MT1_PORT_BREAK_PIN = 1;
									MT1_PORT_ADDR_WRITE;

									//MT3_PORT_START_PIN = 1;
									MT3_PORT_BREAK_PIN = 1;
									MT3_PORT_ADDR_WRITE;
								}
							break;

						case 20 :
							//Motor_Relay_OFF;
							if(Com0_Data[4] == 0xFF){
									Motor_Relay_OFF();

									MT1_PORT_START_PIN = 0;
									MT1_PORT_BREAK_PIN = 0;
									MT1_PORT_ADDR_WRITE;

									MT3_PORT_START_PIN = 0;
									MT3_PORT_BREAK_PIN = 0;
									MT3_PORT_ADDR_WRITE;
								}
							break;

						case 21 :
							if(Com0_Data[4] == 0xFF){
								TIM_Out_Port.bit4.T1 = Register_Buff[40] & 0x0F;

								TIM_OUT_DATA_WRITE(TIM_Out_Port.all);
								//ADDR16(PORTF) = TIM_Out_Port.all ;
							}
							break;

						case 22 :
							if(Com0_Data[4] == 0xFF){
								TIM_Out_Port.bit4.T2 = Register_Buff[41] & 0x0F;

								TIM_OUT_DATA_WRITE(TIM_Out_Port.all);
								//ADDR16(PORTF) = TIM_Out_Port.all ;
							}
							break;

						case 23 :
							if(Com0_Data[4] == 0xFF){

								// tim_buff = Register_Buff[42] & 0x0F;

								// REAR_TIM_OUT_Data.bit4.LEFT = Register_Buff[42] & 0x0F;

								// PORTI = REAR_TIM_OUT_Data.all;
							}
							break;

						case 24 :
							if(Com0_Data[4] == 0xFF){

								// tim_buff = Register_Buff[43] & 0x0F;

								// REAR_TIM_OUT_Data.bit4.RIGHT = Register_Buff[43] & 0x0F;

								// PORTI = REAR_TIM_OUT_Data.all;
							}
							break;

						case 25 :
							if(Com0_Data[4] == 0xFF){

								PIO_Out_Port.bit4.LL = Register_Buff[38] & 0x0F;

								ADDR16(PORTH) = PIO_Out_Port.byte.L;
							}
							break;

						case 26 :
							if(Com0_Data[4] == 0xFF){

								PIO_Out_Port.bit4.LH = Register_Buff[39] & 0x0F;

								ADDR16(PORTH) = PIO_Out_Port.byte.L;
							}
							break;

						case 30 :
							if(Com0_Data[4] == 0xFF){
								Pallet_Detect_Enable ^= 1;
							}
							break;

						case 35 :           // Lift Up
								if(Lift_Action == Lift_Wait)Lift_Up_GPIO();
							break;

						case 36 :           // Lift Down
								if(Lift_Action == Lift_Wait)Lift_Down_GPIO();
							break;

						case 37 :           // Lift Up
							if(Com0_Data[4] == 0xFF){
								// RFID_Read(ON);
								//C550_RX_FIFO_Reset(2);
								//DELAY_US(10);
								//C550_RX_Enable(2,1);
								}
							break;

						case 38 :           // Lift Down
							if(Com0_Data[4] == 0xFF){
								//RFID_Read(OFF);

								}
							break;

						case 39 :
							if(Com0_Data[4] == 0xFF){
								//Lift_Reset_GPIO();
							}
							break;

						case 46 :           // LED RED
							if(Com0_Data[4] == 0xFF)LED_Color_Select(10000,0,0);
							break;

						case 47 :           // LED Yellow
							if(Com0_Data[4] == 0xFF)LED_Color_Select(0,10000,0);
							break;

						case 48 :           // LED Green
							if(Com0_Data[4] == 0xFF)LED_Color_Select(0,0,10000);
							break;

						case 49 :           // Sound1-out
							if(Com0_Data[4] == 0xFF){
								SOUND_OUT_PIN1 ^= 1;
								SOUND_OUT_ADDR_WRITE;
								}
							break;

						case 50 :
							if(Com0_Data[4] == 0xFF){
								SOUND_OUT_PIN2 ^= 1;
								SOUND_OUT_ADDR_WRITE;
								}
							break;

						case 51 :
							if(Com0_Data[4] == 0xFF){
								SOUND_OUT_PIN3 ^= 1;
								SOUND_OUT_ADDR_WRITE;
								}
							break;

						case 52 :
							if(Com0_Data[4] == 0xFF){
								SOUND_OUT_PIN4 ^= 1;
								SOUND_OUT_ADDR_WRITE;
								}
							break;

						case 53 :
							if(Com0_Data[4] == 0xFF){
								SOUND_OUT_PIN5 ^= 1;
								SOUND_OUT_ADDR_WRITE;
								}
							break;

						case 54 :
							if(Com0_Data[4] == 0xFF){
								SOUND_OUT_PIN6 ^= 1;
								SOUND_OUT_ADDR_WRITE;
								}
							break;

						case 55 :           // Charge_MC_On,
							if(Com0_Data[4] == 0xFF){
								if ( ETC_IO.bit.MC_ON == 1) CHARGE_MC_OFF();
								else CHARGE_MC_ON();
								}
							break;

						case 56 :           // Motor_Relay_On
							if(Com0_Data[4] == 0xFF){
								if ( ETC_IO.bit.MT_ON == 1 ) Motor_Relay_OFF();
								else Motor_Relay_ON();
								}
							break;

						case 57 :			// Lift Power On/Off
							if(Com0_Data[4] == 0xFF){
								if ( ETC_IO.bit.LF_ON == 1 ) LIFT_POW_OFF();
								else LIFT_POW_ON();
								}
							break;

						case 65 :
							if(Com0_Data[4] == 0xFF){
								if(Register_Buff[121] == 0){
									Route_End_Complete_Data = 0;
									AGV_JobCancle_Event_Send(Last_Node);
									}
								}
							break;

						case 66 :
							if(Com0_Data[4] == 0xFF){
								if(Register_Buff[122] == 0)AGV_DRV_Complete_Event_Send(Last_JobID);
								}
							break;

						case 70 :
							if ( AGVMotionData.Drive == AGV_Wait ) Charge_Seq_Num = 1;
							break;

						case 71 :
							if ( AGVMotionData.Drive == AGV_CHARGING ) DisCharge_Seq_Num = 1;
							break;

						case 79 :
							if ( AGVMotionData.Drive == AGV_Wait ) AGVMotionData.Drive = IO_Test_Mode;
							break;

						case 80 :
							if ( AGVMotionData.Drive == IO_Test_Mode) AGVMotionData.Drive = AGV_Wait;
							break;

						case 90 :  /* Battery OFF */
							BAT_POW_Off_Flag = 1;
							break;
					}

					Send_Data_Buff[CRC_Check_Count++] = Modbus_Slave_ID;
					Send_Data_Buff[CRC_Check_Count++] = Single_Coil_Write;
					Send_Data_Buff[CRC_Check_Count++] = Com0_Data[2];
					Send_Data_Buff[CRC_Check_Count++] = Com0_Data[3];
					Send_Data_Buff[CRC_Check_Count++] = Com0_Data[4];
					Send_Data_Buff[CRC_Check_Count++] = Com0_Data[5];
					Send_Data_Buff[CRC_Check_Count++] = Com0_Data[6];
					Send_Data_Buff[CRC_Check_Count++] = Com0_Data[7];

					// SCIC_TX_String_Num(Send_Data_Buff, CRC_Check_Count);
					SCIA_TX_String_Num(Send_Data_Buff, CRC_Check_Count);
					}

				if( Com0_Data[1] == Single_Coil_Read ){
					// SCIF_TX_String_Num(Com0_Data, Com0Data_Count);
					Coil_Bit_Addr = (Com0_Data[2] << 8) | Com0_Data[3];
					Coil_Bit_Count = (Com0_Data[4] << 8) | Com0_Data[5];

					Coil_Word_Addr = Coil_Bit_Addr / 16;
					Coil_Word_Count = Coil_Bit_Count / 16;

					Send_Data_Buff[CRC_Check_Count++] = Modbus_Slave_ID;
					Send_Data_Buff[CRC_Check_Count++] = Single_Coil_Read;

					Send_Data_Buff[CRC_Check_Count++] = Coil_Word_Count;

					for(i = 0; i < Coil_Word_Count; i++ ){
						Send_Data_Buff[CRC_Check_Count++] = (Coil_Buff[Coil_Word_Addr + i] >> 8) & 0x00FF;
						Send_Data_Buff[CRC_Check_Count++] = Coil_Buff[Coil_Word_Addr + i] & 0x00FF;
						}

					CRC_Make_Buff = GetCRC16((char *)&Send_Data_Buff, CRC_Check_Count, 0);

					Send_Data_Buff[CRC_Check_Count++] = CRC_Make_Buff & 0x00FF;
					Send_Data_Buff[CRC_Check_Count++] = (CRC_Make_Buff >> 8) & 0x00FF;

					SCIA_TX_String_Num(Send_Data_Buff, CRC_Check_Count);
					// SCIF_TX_String_Num("->", 2);
					// SCIF_TX_String_Num(Send_Data_Buff, CRC_Check_Count);
					}

				if( Com0_Data[1] == Single_Register_Read ){
					// SCIC_TX_String("11->S_R_RD\r\n");
					Register_Addr = (Com0_Data[2] << 8) | Com0_Data[3];
					Register_Data_Count = (Com0_Data[4] << 8) | Com0_Data[5];

					// Register_Buff[Register_Data_Count-1]= tt++;

					Send_Data_Buff[CRC_Check_Count++] = Modbus_Slave_ID;
					Send_Data_Buff[CRC_Check_Count++] = Single_Register_Read;
					Send_Data_Buff[CRC_Check_Count++] = Register_Data_Count * 2;

					for(i = 0; i < Register_Data_Count; i++){
						Send_Data_Buff[CRC_Check_Count++] = (Register_Buff[Register_Addr + i] >> 8) & 0x00FF;
						Send_Data_Buff[CRC_Check_Count++] = Register_Buff[Register_Addr + i] & 0x00FF;
					}

					CRC_Make_Buff = GetCRC16((char *)&Send_Data_Buff, CRC_Check_Count, 0);

					Send_Data_Buff[CRC_Check_Count++] = CRC_Make_Buff & 0x00FF;
					Send_Data_Buff[CRC_Check_Count++] = (CRC_Make_Buff >> 8) & 0x00FF;

					SCIA_TX_String_Num(Send_Data_Buff, CRC_Check_Count);
					}

				if( Com0_Data[1] == Single_Register_Write ){
					Register_Addr = (Com0_Data[2] << 8) | Com0_Data[3];
					Register_Write_Data = (Com0_Data[4] << 8) | Com0_Data[5];

					Register_Buff[Register_Addr] = Register_Write_Data;

					Send_Data_Buff[CRC_Check_Count++] = Modbus_Slave_ID;
					Send_Data_Buff[CRC_Check_Count++] = Single_Register_Write;
					Send_Data_Buff[CRC_Check_Count++] = Com0_Data[2];
					Send_Data_Buff[CRC_Check_Count++] = Com0_Data[3];
					Send_Data_Buff[CRC_Check_Count++] = Com0_Data[4];
					Send_Data_Buff[CRC_Check_Count++] = Com0_Data[5];
					Send_Data_Buff[CRC_Check_Count++] = Com0_Data[6];
					Send_Data_Buff[CRC_Check_Count++] = Com0_Data[7];

					SCIA_TX_String_Num(Send_Data_Buff, CRC_Check_Count);
					}
				}
			}
		// else
		// {
		//   SCIC_TX_String("CK->");
		//   SCIC_TX_Char(Com0_Data[1]+0x30);
		//   SCIC_TX_Char('\r');
		//   SCIC_TX_Char('\n');
		//   }

		Modbus_Frame_Ack = Modbus_Frame_Wait;
		Com0Data_Count = 0;
		}

	return 0;
}

void SCID_Communication_Control(void)
{
	unsigned char com_char; //, i;
	unsigned char Switch_Num = 0;
	// unsigned char Debug_Buff[20] = {0};

	com_char = SCID_Read_Char();

	while( com_char ){

		switch(com_char){
			case STX :
				Com3Data_Count = 0;
				break;

			case ETX :
				Switch_Num = (Com3_Data[0] - 0x30) * 10;
				Switch_Num |= Com3_Data[1] - 0x30;

				switch(Switch_Num){
					case 1 :
						if ( AGVMotionData.Drive == IO_Test_Mode ){
							if(Com3_Data[2] == '1') Remote_In.all = 0x0001;
							else Remote_In.all = 0x0000;
						}
						else if(OPERATE_MODE == Manual_Operate){
							if(Com3_Data[2] == '1'){
								if(AGVMotionData.Drive == AGV_Wait){
									Moving_Speed_Set(Manual_AGV_Speed);
									Forward_GPIO();
									AGVMotionData.Drive = Manual_DRV;
									Manual_Spin_Flag = 1;
									}
								}
							else{
								AGV_Stop_Function();
								}
							}

						break;

					case 2 :
						if ( AGVMotionData.Drive == IO_Test_Mode ){
							if(Com3_Data[2] == '1') Remote_In.all = 0x0002;
							else Remote_In.all = 0x0000;
						}
						else if(OPERATE_MODE == Manual_Operate){
							if(Com3_Data[2] == '1'){
								if(AGVMotionData.Drive == AGV_Wait){
									Moving_Speed_Set(Manual_AGV_Speed);
									Backward_GPIO();
									AGVMotionData.Drive = Manual_DRV;
									Manual_Spin_Flag = 1;
									}
								}
							else{
								AGV_Stop_Function();
								}
							}
						break;

					case 3 :
						if ( AGVMotionData.Drive == IO_Test_Mode ){
							if(Com3_Data[2] == '1') Remote_In.all = 0x0004;
							else Remote_In.all = 0x0000;
						}
						else if(OPERATE_MODE == Manual_Operate){
							if(Com3_Data[2] == '1'){
								if(AGVMotionData.Drive == AGV_Wait){
									Moving_Speed_Set(Manual_Spin_Speed);
									Left_Spin_GPIO();
									//AGVMotionData.Drive = Manual_DRV;
									Manual_Spin_Flag = 1;
									}
								}
							else{
								AGV_Stop_Function();
								//Manual_Spin_Flag = 0;
								}
							}
						break;

					case 4 :
						if ( AGVMotionData.Drive == IO_Test_Mode ){
							if(Com3_Data[2] == '1') Remote_In.all = 0x0008;
							else Remote_In.all = 0x0000;
						}
						else if(OPERATE_MODE == Manual_Operate){
							if(Com3_Data[2] == '1'){
								if(AGVMotionData.Drive == AGV_Wait){
									Moving_Speed_Set(Manual_Spin_Speed);
									Right_Spin_GPIO();
									//AGVMotionData.Drive = Manual_DRV;
									Manual_Spin_Flag = 1;
									}
								}
							else{
								AGV_Stop_Function();
								//Manual_Spin_Flag = 0;
								}
							}
						break;

					case 5 :
						if ( AGVMotionData.Drive == IO_Test_Mode ){
							if(Com3_Data[2] == '1') Remote_In.all = 0x0010;
							else Remote_In.all = 0x0000;
						}
						else if(Com3_Data[2] == '1'){
							if(AGVMotionData.Drive == AGV_Wait)
							{
								Moving_Speed_Set(AGV_Speed_Buff[0]);
								Distance_Sum_Buff = 0;

								Forward_GPIO();
								}
							}
						break;

					case 6 :
						if ( AGVMotionData.Drive == IO_Test_Mode ){
							if(Com3_Data[2] == '1') Remote_In.all = 0x0020;
							else Remote_In.all = 0x0000;
						}
						else if(Com3_Data[2] == '1'){
							Manual_Spin_Flag = 0;
							AGV_Stop_Function();
							// OPERATE_MODE = Manual_Operate;
							// AGV_DRV_Event_Send(30);
							}
						break;

					case 7 :
						if ( AGVMotionData.Drive == IO_Test_Mode ){
							if(Com3_Data[2] == '1') Remote_In.all = 0x0040;
							else Remote_In.all = 0x0000;
						}
						else if(Com3_Data[2] == '1'){
							if(AGVMotionData.Drive == AGV_Wait){
								Moving_Speed_Set(AGV_Speed_Buff[0]);
								Distance_Sum_Buff = 0;
								Backward_GPIO();
								}
							}
						break;

					case 8 :
						if ( AGVMotionData.Drive == IO_Test_Mode ){
							if(Com3_Data[2] == '1') Remote_In.all = 0x0080;
							else Remote_In.all = 0x0000;
						}
						else if(Com3_Data[2] == '1'){
							if(OPERATE_MODE == Manual_Operate){
								if(AGVMotionData.PIN_State == Lift_Up){
									if(Lift_Action == Lift_Wait)Lift_Down_GPIO();
									}
								if(AGVMotionData.PIN_State == Lift_Down){
									if(Lift_Action == Lift_Wait)Lift_Up_GPIO();
									}
								}
							}
						break;

					case 9 :
						if ( AGVMotionData.Drive == IO_Test_Mode ){
							if(Com3_Data[2] == '1') Remote_In.all = 0x0100;
							else Remote_In.all = 0x0000;
						}
						else if(Com3_Data[2] == '1'){
							if(AGVMotionData.Drive == AGV_Wait){
								// if(OPERATE_MODE == Auto_Operate){
								// 	OPERATE_MODE = Manual_Operate;
								// 	AGV_DRV_Event_Send(30);
								// 	}

								// else if(OPERATE_MODE == Manual_Operate){
								// 	OPERATE_MODE = Auto_Operate;
								// 	AGV_DRV_Event_Send(31);
								// 	}
								}
							}
						break;

					case 10 :
						if ( AGVMotionData.Drive == IO_Test_Mode ){
							if(Com3_Data[2] == '1') Remote_In.all = 0x0200;
							else Remote_In.all = 0x0000;
						}
						break;
					}
				Com3Data_Count = 0;
				break;

			default :
				Com3_Data[Com3Data_Count++] = com_char;
				if ( Com3Data_Count >= Com3buffsize ) Com3Data_Count = 0;
				break;

			}
			com_char = SCID_Read_Char();
		}

}

unsigned char t1 = 0;
void Ex_PORT_Control_Can_Data_Send ( int ID )
{
	if ( ID == 0 ){
		can2Tx0Data[0] = Ex_Board1_Out_Port.byte[1];// = t1;// Extended Board POUT  8 ~ 15
		can2Tx0Data[1] = Ex_Board1_Out_Port.byte[0];// = t1+1;// Extended Board POUT  0 ~ 8
		can2Tx0Data[2] = Ex_Board1_Out_Port.byte[2];//  = t1+2;// Extended Board POUT  16 ~ 23
		can2Tx0Data[3] = 0;
		can2Tx0Data[4] = 0;
		can2Tx0Data[5] = 0;
		can2Tx0Data[6] = 0;
		can2Tx0Data[7] = 0;
		t1++;

		CAN_Send_Data(&hcan2, Can_Input_GPIO_ID, STD_Mode, 8, can2Tx0Data);
		}

	else if ( ID == 1 ){
		can2Tx1Data[0] = Ex_Board2_Out_Port.byte[1];// Extended Board2 POUT  8 ~ 15
		can2Tx1Data[1] = Ex_Board2_Out_Port.byte[0];// Extended Board2 POUT  0 ~ 8
		can2Tx1Data[2] = Ex_Board2_Out_Port.byte[2];// Extended Board2 POUT  16 ~ 23
		can2Tx1Data[3] = 0;
		can2Tx1Data[4] = 0;
		can2Tx1Data[5] = 0;
		can2Tx1Data[6] = 0;
		can2Tx1Data[7] = 0;

		CAN_Send_Data(&hcan2, Can_Output_GPIO_2_ID, STD_Mode, 8, can2Tx1Data);
		}
}

unsigned char BMS_Data_Cnt = 0;
Uint16_Frame ID_Buff;

void BMS_Data_Request ( void )
{
	uint8_t TxData[8] = {0};
	// Uint16_Frame ID_Buff;

	ID_Buff.all = BMS_ID_Buff[BMS_Data_Cnt++];
	TxData[0] = ID_Buff.byte.L;
	TxData[1] = ID_Buff.byte.H;

	CAN_Send_Data(&hcan1, BMS_REQUEST_ID, STD_Mode, 8, TxData);

	if ( BMS_Data_Cnt >= 8 ) BMS_Data_Cnt = 0;
}

void BMS_Power_OFF (void)
{
	uint8_t TxData[8] = {0};
	// Uint16_Frame ID_Buff;

	TxData[0] = 0x5E;
	TxData[1] = 0x73;

	CAN_Send_Data(&hcan1, BMS_REQUEST_ID, STD_Mode, 8, TxData);
}

//PORT_BYTE_FRAME DEBUG_Buff[4];
//STOP_DEBUG_DATA DEBUG_DATA;
// int TEST_Target_Value, TEST_processValue, TEST_Motor_num;
// long TEST_error, TEST_p_term, TEST_d_term;
// long TEST_i_term, TEST_ret, TEST_temp;
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	// unsigned char i, k = 0;
	// unsigned int Test_Buff = 0;
	// unsigned char Test_Cnt = 0, tt = 0, i;
	// unsigned char tp1 = 0, tp2 = 0;
	// int8_t SD_WR_BUF[30] = {"AGV LOG WRITE...\r\n"};
	// int8_t SD_RD_BUF[100] = {0};
	// unsigned char BUF[20] = {0};
	// PORT_IO_FRAME IO_8Buff;
	// PORT_BYTE_FRAME IO_16Buff;

	// unsigned int wr_pos, rd_pos;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

	DWT_Init();

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_FMC_Init();
  MX_SPI1_Init();
  MX_UART7_Init();
  MX_UART8_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
  MX_SPI2_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_ADC1_Init();
  MX_CAN2_Init();
  /* USER CODE BEGIN 2 */

	ADDR16(PORTA) = MT1_Port.all = 0;
	ADDR16(PORTB) = MT2_Port.all = 0;
	ADDR16(PORTC) = MT3_Port.all = 0;
	ADDR16(PORTD) = MT4_Port.all = 0;
	ADDR16(PORTE) = Sound_Out_Port.all = 0;
	ADDR16(PORTF) = TIM_Out_Port.all = 0;
	ADDR16(PORTG) = Lift_Out_Port.all = 0;
	ADDR16(PORTH) = PIO_Out_Port.byte.L = 0;
	ADDR16(PORTI) = PIO_Out_Port.byte.H = 0;
	ADDR16(PORTJ) = SPARE_Out_Port.all = 0;

	HAL_TIM_Base_Start_IT(&htim7);
	HAL_TIM_Base_Start_IT(&htim6);
	UART_REC_IT_Set(1);
	UART_REC_IT_Set(2);
	UART_REC_IT_Set(3);
	UART_REC_IT_Set(4);
	UART_REC_IT_Set(5);
	UART_REC_IT_Set(6);

	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);

	AT25128_WP_Low;
	AT93C56_Disable;

	//HAL_GPIO_WritePin(GPIOH, ENC_SEL1_Pin|ENC_SEL2_Pin, GPIO_PIN_SET);	// Encoder Input - TTL Signal
	HAL_GPIO_WritePin(GPIOH, ENC_SEL1_Pin|ENC_SEL2_Pin, GPIO_PIN_RESET);	// Encoder Input - Line Drive Signal

	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

	Ex_Board1_Out_Port.all = 0;
	Ex_Board2_Out_Port.all = 0;
	Ex_Board1_In_Port.all = 0;
	Ex_Board2_In_Port.all = 0;

	Remote_In.all = 0;

	MaxError = MAX_INT / (P_Factor+ 1);
	MaxSumError = MAX_I_TERM / (I_Factor+ 1);

	CAN_Mask_Init(&hcan1, 0x000, 0x000);
	CAN_Mask_Init(&hcan2, 0x000, 0x000);

	HAL_CAN_Start(&hcan1);
	HAL_CAN_Start(&hcan2);
	/*
	canTXHeader.StdId = 0x501;
	canTXHeader.RTR = CAN_RTR_DATA;
	canTXHeader.IDE = CAN_ID_STD;
	canTXHeader.DLC = 8;
	*/
	//for(i = 0; i < 8; i++)can1Tx0Data[i] = 0;

	// UART1_TX_String_Buff[17] = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_Val_Buff, 5);

	LED_Color_Select(18000,18000,0);

	HAL_Delay(1);

	AD5754_Init();

	HAL_Delay(100);

	AGVMotionData.Drive = AGV_Wait;
	AGVMotionData.Error_Code = AGV_Wait;

	Present_Course_Number = 0;
	Course_Number = 1;

	LCD_Buff_Clear();

	AGV_DRV_Parameter_Setting();

	// IO_16Buff.all = GPIO_Read_Port(KEY_IN_GPIO_Port);

	//Old_LiftIN =
	Old_KeyIN.all = 0;

	#ifdef SCIA_485
	SCIA_485_RX;
	#endif

	Printf("PROGRAM START!!\r\n");

	AGVMotionData.Drive = AGV_Wait;
	OPERATE_MODE = Auto_Operate;

	DELAY_MS(1000);

	// if((retSD = f_mount(&SDFatFS, &SDPath[0], 1)) == FR_OK )
	// {
	// 	Printf("f_mount OK!\r\n");

	// 	if((retSD = f_open(&SDFile, "0:/AGV_log.txt", ( FA_OPEN_EXISTING | FA_WRITE | FA_READ ) )) == FR_OK )
	// 	{
	// 		f_write(&SDFile, SD_WR_BUF, StrLen(SD_WR_BUF), &wr_pos );

	// 		Printf("WR_POS->%d\r\n", wr_pos);

	// 		f_close(&SDFile);

	// 		if((retSD = f_open(&SDFile, "0:/AGV_log.txt", ( FA_OPEN_EXISTING | FA_WRITE | FA_READ ) )) == FR_OK ){
	// 			f_read(&SDFile, SD_RD_BUF, 100, &rd_pos);

	// 			Printf("RD_POS->%d, %s", rd_pos, SD_RD_BUF);
	// 		}
	// 	}
	// 	else
	// 	{
	// 		Printf("File Open failed.\r\n", wr_pos);
	// 	}
	// }
	// else
	// {
	// 	Printf("f_mount failed!\r\n");
	// }

	Motor_DA_Buff[0] = 0;
	Motor_DA_Buff[1] = 0;
	Motor_DA_Buff[2] = 0;
	Motor_DA_Buff[3] = 0;
	AD5754_DA_Out_Function((unsigned int *)Motor_DA_Buff, 4);

	// HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC_Val_Buff, 5);
	Motor_Relay_ON();

	DELAY_MS(500);

	LIFT_POW_ON();

	LiftSW_IN.all = LiftSW_Old.all = 0;



	TIM_Out_Port.all = 0;
	TIM_OUT_DATA_WRITE(TIM_Out_Port.all);

	Key_Inp.bit.KeyLiftUP_IN = HAL_GPIO_ReadPin ( ATTACH1_DOWN_IN_GPIO_Port, ATTACH1_DOWN_IN_Pin ); // HAL_GPIO_ReadPin ( ATTACH2_UP_IN_GPIO_Port, ATTACH2_UP_IN_Pin );
	Key_Inp.bit.KeyLiftDN_IN = HAL_GPIO_ReadPin ( ATTACH2_DOWN_IN_GPIO_Port, ATTACH2_DOWN_IN_Pin );

	if ( ( Key_Inp.bit.KeyLiftUP_IN == 0 ) && ( Key_Inp.bit.KeyLiftDN_IN == 1 ) ) {
		AGVMotionData.PIN_State = Lift_Up;
		Lift_Enc_Count = TIM2->CNT = LIFT_TOP_POS;
		}
	else if ( ( Key_Inp.bit.KeyLiftUP_IN == 1 ) && ( Key_Inp.bit.KeyLiftDN_IN == 0 ) ) {
		AGVMotionData.PIN_State = Lift_Down;
		Lift_Enc_Count = TIM2->CNT = LIFT_BOTTOM_POS;
		}
	else {
		Lift_Init_Flag = 1;
		}

	MOT_ALARM.all = 0;

	SEQ_NUM.all = 0;

	while (1)
	{
		SCIA_Communication_Control();
		//SCIB_Communication_Control();   // PGV_RS-485 Interface
		SCIC_Communication_Control();
		SCID_Communication_Control();
		// SCIE_Communication_Control();

		// DEBUG_Buff[0].port.L = AGVMotionData.Drive;
		// DEBUG_Buff[0].port.H = AGVMotionData.Error_Code;

		///// EMR /////
		if(Error_Flag == 2)EMR_CODE_Process();

		///// Motor Stop check(Stop -> Wait) /////
		if(Error_Flag == 0)STOP_WAIT_Process();

		// DEBUG_DATA.bit.Error_Code = AGVMotionData.Error_Code;
		// DEBUG_DATA.bit.Error_Flag = Error_Flag;
		// DEBUG_DATA.bit.Stop_Status = Motor_Control_Status;

		// Register_Buff[50] = DEBUG_DATA.all;

		if ( Node_Chagne_Event_Flag != 0 ){
			if ( Node_Chagne_Event_Flag == 1 ) AGV_Start_Place_Event_Send(Last_Node);
			else if ( Node_Chagne_Event_Flag == 2) AGV_NodeChange_Event_Send(Last_Node);
			}

		///// TIM Sensor /////
		Forward_Mark_Enable_Stop();

		// TEST_Route_Seq();

		Left_Spin_Function();

		Right_Spin_Function();

		// Stop_Action_Sequence();

		Lift_Action_To_Moving();

		// StopToStart_SEQ();

		///// Commend Driving /////
		LCD_Display_Process();

		LCD_Display_Var_Assembly();

		if ( Key_Event_Flag == 0 ) Lift_Action_Start();

		Charge_Sequence();

		DisCharge_Sequence();

		if(Timer7_25mS_Count >= 4){

			Register_Buff[35] = BMS_Summary_Data.BAT_SOC.all; //ADC_Val_Buff[0]; // Battery_Charge_Ratio(); //

			if(Event_Send_Flag == 1){
				DRV_Complete_Count++;
				if(DRV_Complete_Count >= 10){
					if(Lift_Complete_Data == 1)AGV_Lift_Event_Send(AGVMotionData.PIN_State);
					if(Route_End_Complete_Data == 1)AGV_DRV_Complete_Event_Send(Last_JobID);
					if(Charge_Action_Complete_Data == 1) AGV_Charge_Complete_Event_Send();
					if(Job_Cancel_Data == 1) AGV_JobCancle_Event_Send(Last_Node);
					}
				}

			Sound_LED_Process();

			Key_Input();

			Lift_Manual_Action();

			Charge_Status_Check();

			Battery_Low_Check();

			Alarm_Continue_Check();

			Timer7_25mS_Count = 0;
			}

		if(PGV_Send_Timmer_Count >= 1){

			SCIB_Communication_Control();   // PGV_RS-485 Interface

			if(PGV_Start_Flag != 0){
				Tim_Reset_Count++;

				if(Tim_Reset_Count >= 500){
					PGV_Data_Set_Flag = 1;
					Line_Select_Buff = Dir_Left;
					Color_Select_Buff = Color_Blue;
					PGV_ID = 0;
					PGV_Start_Flag = Tim_Reset_Count = 0;
					}
				}

			if((PGV_Data_Set_Flag != 0) && (PGV_Start_Flag == 0)){
				Tim_Reset_Count++;

				if(Tim_Reset_Count >= 10){

					if(PGV_Data_Set_Flag == 3){
						PGV_Structs_Send_Fram.PGV_ID = 0;
						PGV_Com_Send(PGV_Data_Set_Flag);
						SCIB_485_RX;
						}

					if(PGV_Data_Set_Flag == 2){
						PGV_Color_Select(Color_Select_Buff, PGV_ID);
						}

					if(PGV_Data_Set_Flag == 1){
						PGV_Line_Select(Line_Select_Buff, PGV_ID);
						}

						Com1Data_Count = Tim_Reset_Count = 0;
						PGV_Data_Send_Flag = 1;
						Reset_Rwi_Position();
					}
				}

			PGV_Send_Timmer_Count = 0;
		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 216;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 8;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
unsigned char Send_ID = 0;
unsigned int Left_Count = 0, Right_Count = 0;
unsigned int R_ENC = 0, L_ENC = 0;
unsigned int Can2_Send_Cnt = 0;
unsigned char Ex_Board_Num = 0;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	/* 50mS ****************************************************/
	if(htim->Instance == TIM7){

		int Motor_Speed_R = 10;
		int Motor_Speed_L = 10;
		unsigned int R_PID_BUFF = 0;
		unsigned int L_PID_BUFF = 0;

		int Right_PID_Count_tim1 = 0, Left_PID_Count_tim1 = 0;

		unsigned long Distance_Temp = 0;

		unsigned int Right_Mag_Count_Temp = 0;
		unsigned int Left_Mag_Count_Temp = 0;

		int Motor_DA_Data[4] = {0};
		unsigned int AGVMotion_Drive_Temp = 0;

		int Slow_Speed_Ctrl_R = 0;
		int Slow_Speed_Ctrl_L = 0;

		// Line type
		unsigned int Spot_Line_State_Temp = 0;
		// int Guide_new = 0, T_Guide = 0;
		// int Motor_Speed = 10;
		// int R_Sp = 0, L_Sp = 0;
		//int Present_Angle = 0, Right_PID_Count_tim1 = 0, Left_PID_Count_tim1 = 0;

		//unsigned int Mark_On_Temp = 0;
		unsigned int R_Turn_Speed_Temp = 0;
		unsigned int L_Turn_Speed_Temp = 0;
		char Start_Speed_Flag = 0, UP_Complete = 0;
		//int Diff_Speed_Buff = 0;
		//unsigned int Slow_Flag = 0;
		// int Torque_Compare = 0, L_Torque_Buff = 0, R_Torque_Buff = 0; // 
		int Corecction_Angle = 0;

		Timer7_25mS_Count++;
		Timer7_100mS_Cnt++;

		PID_Control_Count += 1;

	 	if (PID_Control_Count == 1)  EMR_Process();   // EMR Switch

		///// Motor Moving Process loop /////
		if(PID_Control_Count >= 2){
			PID_Control_Count = 0;

			Motor_Speed_R = Right_Target_Speed;
			Motor_Speed_L = Left_Target_Speed;
			Motor_DA_Data[DACA] = Motor_DA_Buff[DACA];
			Motor_DA_Data[DACC] = Motor_DA_Buff[DACC];
			Start_Speed_Flag = Start_Speed_UpFlag;
			UP_Complete = Speed_Up_Complete_Flag;

			R_PID_BUFF = TIM1->CNT;
			L_PID_BUFF = TIM3->CNT;

			TIM1->CNT = 0x8000;
			TIM3->CNT = 0x8000;

			if ( R_PID_BUFF >= 0x8000 ) Right_PID_Count = R_PID_BUFF - 0x8000;
			else Right_PID_Count = 0x8000 - R_PID_BUFF;

			if ( L_PID_BUFF >= 0x8000 ) Left_PID_Count = L_PID_BUFF - 0x8000;
			else Left_PID_Count = 0x8000 - L_PID_BUFF;

			R_ENC = Right_PID_Count;
			L_ENC = Left_PID_Count;

			///// Error Prrocessing(Stop) /////

			if(Error_Flag == 1){

				if ( Err_Event_Debug_Flag == 0)  Error_Event_Debug();

				Right_PID_Count_tim1 = Right_PID_Count;
				Left_PID_Count_tim1 = Left_PID_Count;
				Register_Buff[36] = Right_PID_Count;
				Register_Buff[37] = Left_PID_Count;

				// Error_Stop_Count += 1;

				if(Motor_DA_Data[DACA] > 5000)Motor_DA_Data[DACA] -= 5000;
				if(Motor_DA_Data[DACC] > 5000)Motor_DA_Data[DACC] -= 5000;

				if(Motor_DA_Data[DACA] < 0)Motor_DA_Data[DACA] = 0;
				if(Motor_DA_Data[DACC] < 0)Motor_DA_Data[DACC] = 0;

				if((Motor_DA_Data[DACA] <= 5000) && (Motor_DA_Data[DACC] <= 5000)){
					Motor_DA_Data[DACA] = Motor_DA_Data[DACC] = 0;
					}

				AD5754_DA_Out_Function((unsigned int *)Motor_DA_Data, 4);

				if((Motor_DA_Data[DACA] <= 1000) && (Motor_DA_Data[DACC] <= 1000)){
					if( (Right_PID_Count_tim1 < 60) && (Left_PID_Count_tim1 < 60) ){

						Error_Flag = 2;
						// Error_Stop_Count = 0;
						}
					}
				/*
				if(Error_Stop_Count > (1*20)){
					Error_Flag = 2;
					Error_Stop_Count = 0;
					}
				*/
				}

			///// Error Prrocessing(Stop) /////

			//else if(Error_Flag == 0){
			if(Error_Flag == 0){
				Right_PID_Count_tim1 = Right_PID_Count;
				Left_PID_Count_tim1 = Left_PID_Count;
				Right_Count = Register_Buff[36] = Right_PID_Count;
				Left_Count = Register_Buff[37] = Left_PID_Count;

				AGVMotion_Drive_Temp = AGVMotionData.Drive;
				//Mark_On_Temp = Mark_On;

				// L_Torque_Buff = Left_Motor_Torque;
				// R_Torque_Buff = Right_Motor_Torque;

				// Torque_Compare = R_Torque_Buff - L_Torque_Buff;

				Register_Buff[84] = Left_PID_Count_tim1;
				Register_Buff[85] = Right_PID_Count_tim1;

				if((AGVMotion_Drive_Temp == AGV_Stop)||(Mark_Stop == 1)){

					if ( Motor_Control_Status == MOTOR_STATUS_RUN ) Start_Slope_Count += 1;

					if(Motor_Speed_R < Slow_Speed_Ctrl_R){
						Slow_Speed_Ctrl_R -= 1000;
						if(Right_Speed_Slope == 1){
							if(Motor_Speed_R > Slow_Speed_Ctrl_R){
								Slow_Speed_Ctrl_R = Motor_Speed_R;
								Right_Speed_Slope = 0;
								}
							}
						}
					if(Motor_Speed_R == Slow_Speed_Ctrl_R){
						Slow_Speed_Ctrl_R = Motor_Speed_R;
						}
					if(Slow_Speed_Ctrl_R < 0)Slow_Speed_Ctrl_R = 0;


					if(Motor_Speed_L < Slow_Speed_Ctrl_L){
						Slow_Speed_Ctrl_L -= 1000;
						if(Left_Speed_Slope == 1){
							if(Motor_Speed_L > Slow_Speed_Ctrl_L){
								Slow_Speed_Ctrl_L = Motor_Speed_L;
								Left_Speed_Slope = 0;
								}
							}
						}
					if(Motor_Speed_L == Slow_Speed_Ctrl_L){
						Slow_Speed_Ctrl_L = Motor_Speed_L;
						}
					if(Slow_Speed_Ctrl_L < 0)Slow_Speed_Ctrl_L = 0;

					Motor_DA_Data[DACA] = PID_Control((Slow_Speed_Ctrl_R + R_Turn_Speed_Temp), Right_PID_Count_tim1, 0);
					Motor_DA_Data[DACC] = PID_Control((Slow_Speed_Ctrl_L + L_Turn_Speed_Temp), Left_PID_Count_tim1, 2);

					if(Motor_DA_Data[DACA] < 0)Motor_DA_Data[DACA] = 0;
					else if(Motor_DA_Data[DACA] >= 27000)Motor_DA_Data[DACA] = 27000;

					if(Motor_DA_Data[DACC] < 0)Motor_DA_Data[DACC] = 0;
					else if(Motor_DA_Data[DACC] >= 27000)Motor_DA_Data[DACC] = 27000;

					if((Motor_DA_Data[DACA] <= 1000) && (Motor_DA_Data[DACC] <= 1000)){

						if((Right_PID_Count_tim1 < 60) && (Left_PID_Count_tim1 < 60)){

							Solw_Stop_Flag = Manual_Spin_Flag = Start_Slope_Count = 0;
							Stop_GPIO();
							//AGVMotionData.Drive = AGV_Wait;
							// Error_Stop_Count = 0;
							}
						}

					AD5754_DA_Out_Function((unsigned int *)Motor_DA_Data, 4);
					}

				else if(AGVMotion_Drive_Temp == Manual_DRV){

					Slow_Speed_Ctrl_R = Slow_Speed_UpDown_R;
					Slow_Speed_Ctrl_L = Slow_Speed_UpDown_L;

					Register_Buff[Guide_Sensor_State_Addr] = Right_PID_Count_tim1;

					if(Motor_Speed_R > Slow_Speed_Ctrl_R){
						Slow_Speed_Ctrl_R += 10;
						if(Right_Speed_Slope == 2){
							if(Motor_Speed_R < Slow_Speed_Ctrl_R){
								Slow_Speed_Ctrl_R = Motor_Speed_R;
								Right_Speed_Slope = 0;
								}
							}
						//Slow_Speed_Ctrl_R += 2;
						}
					else if(Motor_Speed_R < Slow_Speed_Ctrl_R){
						Slow_Speed_Ctrl_R -= 10;
						if(Right_Speed_Slope == 1){
							if(Motor_Speed_R > Slow_Speed_Ctrl_R){
								Slow_Speed_Ctrl_R = Motor_Speed_R;
								Right_Speed_Slope = 0;
								}
							}
						//Slow_Speed_Ctrl_R -= 10;
						}
					else {
						Slow_Speed_Ctrl_R = Motor_Speed_R;
						}
					if(Slow_Speed_Ctrl_R < 0)Slow_Speed_Ctrl_R = 0;

					if(Motor_Speed_L > Slow_Speed_Ctrl_L){
						Slow_Speed_Ctrl_L += 10;
						if(Left_Speed_Slope == 2){
							if(Motor_Speed_L < Slow_Speed_Ctrl_L){
								Slow_Speed_Ctrl_L = Motor_Speed_L;
								Left_Speed_Slope = 0;
								}
							}
						//Slow_Speed_Ctrl_L += 2;
						}
					else if(Motor_Speed_L < Slow_Speed_Ctrl_L){
						Slow_Speed_Ctrl_L -= 10;
						if(Left_Speed_Slope == 1){
							if(Motor_Speed_L > Slow_Speed_Ctrl_L){
								Slow_Speed_Ctrl_L = Motor_Speed_L;
								Left_Speed_Slope = 0;
								}
							}
						//Slow_Speed_Ctrl_L -= 10;
						}
					else {
						Slow_Speed_Ctrl_L = Motor_Speed_L;
						}
					if(Slow_Speed_Ctrl_L < 0)Slow_Speed_Ctrl_L = 0;

					Motor_DA_Data[DACA] = PID_Control((Slow_Speed_Ctrl_R + R_Turn_Speed_Temp), Right_PID_Count_tim1, 0);
					Motor_DA_Data[DACC] = PID_Control((Slow_Speed_Ctrl_L + L_Turn_Speed_Temp), Left_PID_Count_tim1, 2);

					if(Motor_DA_Data[DACA] < 0)Motor_DA_Data[DACA] = 0;
					if(Motor_DA_Data[DACC] < 0)Motor_DA_Data[DACC] = 0;

					if(Motor_DA_Data[DACA] >= 27000)Motor_DA_Data[DACA] = 27000;
					if(Motor_DA_Data[DACC] >= 27000)Motor_DA_Data[DACC] = 27000;

					AD5754_DA_Out_Function((unsigned int *)&Motor_DA_Data, 4);
					}

				else if( (AGVMotion_Drive_Temp == Right_Spin_DRV) || (AGVMotion_Drive_Temp == Left_Spin_DRV) ){
					//240, 8 // 230, 6
					Slow_Speed_Ctrl_R = Slow_Speed_UpDown_R;
					Slow_Speed_Ctrl_L = Slow_Speed_UpDown_L;

					if(Motor_Speed_R > Slow_Speed_Ctrl_R){
						Slow_Speed_Ctrl_R += 15;
						if(Right_Speed_Slope == 2){
							if(Motor_Speed_R < Slow_Speed_Ctrl_R){
								Slow_Speed_Ctrl_R = Motor_Speed_R;
								Right_Speed_Slope = 0;
								}
							}
						}
					else if(Motor_Speed_R < Slow_Speed_Ctrl_R){
						Slow_Speed_Ctrl_R -= 40;
						if(Right_Speed_Slope == 1){
							if(Motor_Speed_R > Slow_Speed_Ctrl_R){
								Slow_Speed_Ctrl_R = Motor_Speed_R;
								Right_Speed_Slope = 0;
								}
							}
						}
					else {
						Slow_Speed_Ctrl_R = Motor_Speed_R;
						}
					if(Slow_Speed_Ctrl_R < 0)Slow_Speed_Ctrl_R = 0;

					if(Motor_Speed_L > Slow_Speed_Ctrl_L){
						Slow_Speed_Ctrl_L += 15;
						if(Left_Speed_Slope == 2){
							if(Motor_Speed_L < Slow_Speed_Ctrl_L){
								Slow_Speed_Ctrl_L = Motor_Speed_L;
								Left_Speed_Slope = 0;
								}
							}
						}
					else if(Motor_Speed_L < Slow_Speed_Ctrl_L){
						Slow_Speed_Ctrl_L -= 40;
						if(Left_Speed_Slope == 1){
							if(Motor_Speed_L > Slow_Speed_Ctrl_L){
								Slow_Speed_Ctrl_L = Motor_Speed_L;
								Left_Speed_Slope = 0;
								}
							}
						}
					else {
						Slow_Speed_Ctrl_L = Motor_Speed_L;
						}
					if(Slow_Speed_Ctrl_L < 0)Slow_Speed_Ctrl_L = 0;


					Motor_DA_Data[DACA] = PID_Control((Slow_Speed_Ctrl_R + R_Turn_Speed_Temp), Right_PID_Count_tim1, 0);
					Motor_DA_Data[DACC] = PID_Control((Slow_Speed_Ctrl_L + L_Turn_Speed_Temp), Left_PID_Count_tim1, 2);

					if(Motor_DA_Data[DACA] < 0)Motor_DA_Data[DACA] = 0;
					if(Motor_DA_Data[DACC] < 0)Motor_DA_Data[DACC] = 0;

					if(Motor_DA_Data[DACA] >= 27000)Motor_DA_Data[DACA] = 27000;
					if(Motor_DA_Data[DACC] >= 27000)Motor_DA_Data[DACC] = 27000;

					AD5754_DA_Out_Function((unsigned int *)&Motor_DA_Data, 4);

					Distance_Temp = Distance_Sum_Buff;
					Distance_Sum_Buff = Distance_Temp + ((unsigned long)(Left_PID_Count_tim1 + Right_PID_Count_tim1) / 2); // 2500 * 2 = 1.5m

					if(Manual_Spin_Flag == 0){
						if((SpinTurn_Deg == 90) && ( Spin_Slow_Flag == 1) && (Spin_Max_Degrees == 0)){
							if(Distance_Sum_Buff >= Spin_Slow_Start_90Count){
								Moving_Speed_Set(10);
								Spin_Stop_Scan_Flag = 1;
								// Register_Buff[78] = Distance_Sum_Buff;
								Spin_Slow_Flag = 0;
								}
							}
						else if((SpinTurn_Deg == 90) && ( Spin_Slow_Flag == 1)){
							if(Distance_Sum_Buff >= Spin_Slow_Start_90Count){
								Moving_Speed_Set(10);
								Spin_Stop_Scan_Flag = 1;
								Spin_Slow_Flag = 0;
								}
							}
						else if((SpinTurn_Deg == 180) && ( Spin_Slow_Flag == 1)){
							if(Distance_Sum_Buff >= Spin_Slow_Start_180Count){
								Moving_Speed_Set(10);
								Spin_Stop_Scan_Flag = 1;
								// Register_Buff[78] = Distance_Sum_Buff;
								Spin_Slow_Flag = 0;
								}
							}
						else if((SpinTurn_Deg == 180)  && ( Spin_Slow_Flag == 1) && (Spin_Max_Degrees == 0)){
							if(Distance_Sum_Buff >= Spin_Slow_Start_180Count){
								Moving_Speed_Set(10);
								Spin_Stop_Scan_Flag = 1;
								// Register_Buff[78] = Distance_Sum_Buff;
								Spin_Slow_Flag = 0;
								}
							}

						// Register_Buff[77] = Distance_Sum_Buff;
						}
					}


				else if( ( AGVMotion_Drive_Temp == Forward_DRV ) || ( AGVMotion_Drive_Temp == Backward_DRV ) ){

					Right_Mag_Count_Temp = Right_Mag_Count;
					Left_Mag_Count_Temp = Left_Mag_Count;

					Slow_Speed_Ctrl_R = Slow_Speed_UpDown_R;
					Slow_Speed_Ctrl_L = Slow_Speed_UpDown_L;

					Register_Buff[Guide_Sensor_State_Addr] = Right_PID_Count_tim1;

					Slow_Speed_Ctrl_R = Slow_Speed_UpDown_R;
					Slow_Speed_Ctrl_L = Slow_Speed_UpDown_L;

					// Line Type
					Spot_Line_State_Temp = Spot_Line_State;

					if(Motor_Speed_R > Slow_Speed_Ctrl_R){
						Slow_Speed_Ctrl_R += 10;//30;
						}
					else if(Motor_Speed_R < Slow_Speed_Ctrl_R){
						Slow_Speed_Ctrl_R -= 45;//55;
						//Slow_Speed_Ctrl_R -= 10;
						//Slow_Flag = 1;
						}

					else {
						Slow_Speed_Ctrl_R = Motor_Speed_R;
						}

					if(Slow_Speed_Ctrl_R < 0)Slow_Speed_Ctrl_R = 0;

					if(Motor_Speed_L > Slow_Speed_Ctrl_L){
						Slow_Speed_Ctrl_L += 10;//30;
						}
					else if(Motor_Speed_L < Slow_Speed_Ctrl_L){
						Slow_Speed_Ctrl_L -= 45;//55;
						//Slow_Speed_Ctrl_L -= 10;
						//Slow_Flag = 1;
						}
					else {
						Slow_Speed_Ctrl_L = Motor_Speed_L;
						}
					if(Slow_Speed_Ctrl_L < 0)Slow_Speed_Ctrl_L = 0;


					if(Spot_Line_State_Temp == QR_Tag){

						if(AGV_VEL <= 200)Corecction_Angle = Angle_Cal_Buff;
						else Corecction_Angle = HSpeed_Angle_Cal_Buff;

						if(Right_Mag_Count_Temp > 0){
							Right_Mag_Count_Temp -= Distance_Cal_Buff;
							Left_PID_Count_tim1 += Corecction_Angle;
							}
						if(Left_Mag_Count_Temp > 0){
							Left_Mag_Count_Temp -= Distance_Cal_Buff;
							Right_PID_Count_tim1 += Corecction_Angle;
							}
						/*
						if(AGV_VEL <= 120){
							if(Torque_Compare > 12){
								Right_PID_Count_tim1 += AGV_VEL / Torque_Cal_Buff;
								}

							if(Torque_Compare < -12){
								Left_PID_Count_tim1 += AGV_VEL / Torque_Cal_Buff;
								}
							}
						*/
						Motor_DA_Data[DACA] = PID_Control( Slow_Speed_Ctrl_R, Right_PID_Count_tim1, 0 );
						Motor_DA_Data[DACC] = PID_Control( Slow_Speed_Ctrl_L, Left_PID_Count_tim1, 2 );

						if(Motor_DA_Data[DACA] < 0) Motor_DA_Data[DACA] = 0;
						if(Motor_DA_Data[DACC] < 0) Motor_DA_Data[DACC] = 0;

						if(Motor_DA_Data[DACA] >= 27000) Motor_DA_Data[DACA] = 27000;
						if(Motor_DA_Data[DACC] >= 27000) Motor_DA_Data[DACC] = 27000;

						AD5754_DA_Out_Function((unsigned int *)Motor_DA_Data, 4);

					}

					Distance_Temp = Distance_Sum_Buff;
					Distance_Sum_Buff = Distance_Temp + ((Left_PID_Count_tim1 + Right_PID_Count_tim1) / 2); // 2500 * 2 = 1.5m

					//Left_Sample_Count += Left_PID_Count_tim1;
					//Right_Sample_Count += Right_PID_Count_tim1;

					Right_Mag_Count = Right_Mag_Count_Temp;
					Left_Mag_Count = Left_Mag_Count_Temp;
					}
				}

			Slow_Speed_UpDown_R = Slow_Speed_Ctrl_R;
			Slow_Speed_UpDown_L = Slow_Speed_Ctrl_L;
			Motor_DA_Buff[DACA] = Motor_DA_Data[DACA];
			Motor_DA_Buff[DACC] = Motor_DA_Data[DACC];
			Start_Speed_UpFlag = Start_Speed_Flag;
			Speed_Up_Complete_Flag = UP_Complete;

			Register_Buff[77] = Motor_DA_Buff[DACC];  // Slow_Speed_Ctrl_L;
			Register_Buff[78] = Motor_DA_Buff[DACA];  // Slow_Speed_Ctrl_R;

			Right_PID_Count = Left_PID_Count = 0;
			R_Count_flag = L_Count_flag = 1;
			}

		}

	else if(htim->Instance == TIM6){

		if(Modbus_Frame_Ack == Modbus_Frame_Rev){
			Modbus_Check_Count++;
			if(Modbus_Check_Count >= 4){
				if(Com0Data_Count > 0)Modbus_Frame_Ack = Modbus_Rev_Complete;
				Modbus_Check_Count = 0;
				}
			}

		if(PORT_Can1_RECV == 1){
			PORT_Can1_RECV = 0;
			}

		Register_Buff[Driving_Status_Addr] = AGVMotionData.Drive;

		if(Stop_Delay_Flag == 1){
			Stop_Delay_Count++;
			if(Stop_Delay_Count >= 60){
				AGV_Stop_Function();
				Stop_Delay_Flag = Stop_Delay_Count = 0;
				}
			}

		if ( AGVMotionData.Drive == AGV_CHARGING ) Charge_Check_Count++;
		if ( OPERATE_MODE == Auto_Operate ) Battery_Low_Check_Count++;
		if ( Error_Continue_Flag == 1 ) Error_Continue_Count++;

		/*
		if ( Forward_Mark_Stop_Status == 4 && AGVMotionData.Drive == AGV_Wait ) Forward_Mark_Delay++;
		*/
		if ( Lift_Action == Lift_Reset_Action ) Lift_Reset_Cnt++;

		Carriage_Status_Check(5);

		if ( Can2_Send_Cnt >= 50 ) {
			if ( BAT_POW_Off_Flag == 0 ) BMS_Data_Request();
			else BMS_Power_OFF();
			Can2_Send_Cnt = 0;
			}
		else {
			if ( Ex_Board_Num == 0 ) {
				Ex_PORT_Control_Can_Data_Send(Ex_Board_Num);
				Ex_Board_Num = 1;
				}
			else if ( Ex_Board_Num == 1 ) {
				Ex_PORT_Control_Can_Data_Send(Ex_Board_Num);
				Ex_Board_Num = 0;
				}
			Can2_Send_Cnt++;
			}

		PGV_Send_Timmer_Count++;

		// if ( Lift_Action == Lift_Up_Action ) {
		// 	Lift_Enc_Count = TIM2->CNT;

		// 	if ( Lift_Enc_Count > LIFT_UP_SOLW_POS ) {
		// 		Motor_DA_Buff[DACB] = LIFT_SPEED_MIN_DA_VAL;
		// 		AD5754_DA_Out_Function((unsigned int *)Motor_DA_Buff, 4);
		// 		}
		// 	}
		// else if ( Lift_Action == Lift_Down_Action){
		// 	Lift_Enc_Count = TIM2->CNT;

		// 	if ( Lift_Enc_Count < LIFT_DOWN_SLOW_POS) {
		// 		Motor_DA_Buff[DACB] = LIFT_SPEED_MIN_DA_VAL;
		// 		AD5754_DA_Out_Function((unsigned int *)Motor_DA_Buff, 4);
		// 		}
		// 	}

		}



}

unsigned int GPIO_Read_Port(GPIO_TypeDef* GPIOx)
{
	unsigned int port = 0;

	port = GPIOx->IDR;

	return port;
}

void GPIO_Write_Port(GPIO_TypeDef* GPIOx, uint8_t type, uint16_t Data )
{
	PORT_BYTE_FRAME IO_Buff;

	if ( type == DATA_LOW ){
		IO_Buff.all = GPIOx->ODR;

		IO_Buff.byte.L = Data & 0xFF;

		GPIOx->ODR = IO_Buff.all;
	}
	else if ( type == DATA_HIGH ){
		IO_Buff.all = GPIOx->ODR;

		IO_Buff.byte.H = Data & 0xFF;

		GPIOx->ODR = IO_Buff.all;
	}
	else {
		GPIOx->ODR = Data & 0xFFFF;
	}
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
		 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
