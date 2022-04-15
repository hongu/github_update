/* USER CODE BEGIN Header */
/**
	******************************************************************************
	* @file           : main.h
	* @brief          : Header for main.c file.
	*                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
typedef struct{
	unsigned char AGV_No;       //ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½È£
	// unsigned char Dirving_Mode;   //ï¿½ï¿½ï¿½ï¿½ï¿½ï¿½
	// unsigned char Position;     //RFIDï¿½ï¿½ï¿½ï¿½
	// unsigned int  Moving_Distance;  //ï¿½ï¿½ï¿½ï¿½Å¸ï¿?
	// unsigned char Guide_No;     //ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½
	// unsigned int  RFID;         //ï¿½Î½ï¿½ RFIDï¿½ï¿½ï¿½ï¿½
	// unsigned int  Motor_Speed;    //ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ ï¿½Óµï¿½
	// int       Gyro_Yaw;
	unsigned int  Battery;        //ï¿½ï¿½ï¿½Í¸ï¿½ ï¿½ï¿½ï¿½ï¿½
	// unsigned char tcar;
}AGVTX_Struct;

extern AGVTX_Struct AGV_RunData;

typedef struct{
	unsigned int  Drive;
	unsigned int  Speed;
	unsigned int  Line_State;
	unsigned int  EndDist;
	unsigned int  DSpeed;
	unsigned int  Course_No;
	unsigned int  P_Course_No;
	unsigned int  PIN_State;
	unsigned int  Error_Code;
	unsigned char   Tim;
}Motion_Struct;

extern Motion_Struct AGVMotionData;

typedef struct{
	uint8_t IO1:1;
	uint8_t IO2:1;
	uint8_t IO3:1;
	uint8_t IO4:1;
	uint8_t IO5:1;
	uint8_t IO6:1;
	uint8_t IO7:1;
	uint8_t IO8:1;
}PORT_STRUCT;

typedef union{
	uint8_t all;
	PORT_STRUCT bit;
}PORT_IO_FRAME;

typedef struct{
	uint16_t IO1:1;
	uint16_t IO2:1;
	uint16_t IO3:1;
	uint16_t IO4:1;
	uint16_t IO5:1;
	uint16_t IO6:1;
	uint16_t IO7:1;
	uint16_t IO8:1;
	uint16_t IO9:1;
	uint16_t IO10:1;
	uint16_t IO11:1;
	uint16_t IO12:1;
	uint16_t IO13:1;
	uint16_t IO14:1;
	uint16_t IO15:1;
	uint16_t IO16:1;
}IO_BIT16_STRUCT;

typedef struct{
	uint16_t L:8;
	uint16_t H:8;
}IO_BYTE_STRUCT;

typedef struct{
	uint16_t LL:4;
	uint16_t LH:4;
	uint16_t HL:4;
	uint16_t HH:4;
}IO_BIT4_STRUCT;

typedef union{
	IO_BIT16_STRUCT bit;
	IO_BYTE_STRUCT byte;
	IO_BIT4_STRUCT bit4;
	uint16_t all;
}PORT_BYTE_FRAME;

#define DATA_ALL 	0
#define DATA_LOW 	1
#define DATA_HIGH 	2

typedef struct{
	uint32_t IO1:1;
	uint32_t IO2:1;
	uint32_t IO3:1;
	uint32_t IO4:1;
	uint32_t IO5:1;
	uint32_t IO6:1;
	uint32_t IO7:1;
	uint32_t IO8:1;
	uint32_t IO9:1;
	uint32_t IO10:1;
	uint32_t IO11:1;
	uint32_t IO12:1;
	uint32_t IO13:1;
	uint32_t IO14:1;
	uint32_t IO15:1;
	uint32_t IO16:1;
	uint32_t IO17:1;
	uint32_t IO18:1;
	uint32_t IO19:1;
	uint32_t IO20:1;
	uint32_t IO21:1;
	uint32_t IO22:1;
	uint32_t IO23:1;
	uint32_t IO24:1;
	uint32_t IO25:1;
	uint32_t IO26:1;
	uint32_t IO27:1;
	uint32_t IO28:1;
	uint32_t IO29:1;
	uint32_t IO30:1;
	uint32_t IO31:1;
	uint32_t IO32:1;
}EX_IO_STRUCT;

typedef union{
	EX_IO_STRUCT bit;
	uint8_t byte[4];
	uint32_t all;
}EX_IO_FRAME;

extern EX_IO_FRAME Ex_Board1_Out_Port, Ex_Board1_In_Port;
extern EX_IO_FRAME Ex_Board2_Out_Port, Ex_Board2_In_Port;
extern PORT_IO_FRAME PIO_IN_PORT;

// extern PORT_IO_FRAME TIM_IN_PORT;

// typedef struct{
//   unsigned int OUT1:1;
//   unsigned int OUT2:1;
//   unsigned int OUT3:1;
//   unsigned int OUT4:1;
//   unsigned int OUT5:1;
//   unsigned int OUT6:1;
//   unsigned int OUT7:1;
//   unsigned int OUT8:1;
//   unsigned int OUT9:1;
//   unsigned int OUT10:1;
//   unsigned int RUN:1;
//   unsigned int RES:5;
// }MOT_OUT_BIT_STRUCT;

// typedef struct{
//   unsigned char L;
//   unsigned char H;
// }MOT_OUT_BYTE_STRUCT;

// typedef union{
//   MOT_OUT_BIT_STRUCT bit;
//   MOT_OUT_BYTE_STRUCT byte;
//   unsigned int all;
// }MOT_OUT_FRAME;
typedef struct{
	unsigned char MT1_ALM:1;
	unsigned char MT2_ALM:1;
	unsigned char MT3_ALM:1;
	unsigned char MT4_ALM:1;
	unsigned char MT5_ALM:1;
	unsigned char MT6_ALM:1;
	unsigned char MT7_ALM:1;
	unsigned char MT8_ALM:1;
}MOTOR_ALARM_STRUCT;

typedef union{
	MOTOR_ALARM_STRUCT bit;
	unsigned char all;
}MOTOR_ALARM_FRAME;

extern MOTOR_ALARM_FRAME MOT_ALARM;

typedef struct{
	uint8_t OUT1:1;
	uint8_t OUT2:1;
	uint8_t OUT3:1;
	uint8_t OUT4:1;
	uint8_t OUT5:1;
	uint8_t OUT6:1;
	uint8_t OUT7:1;
	uint8_t OUT8:1;
}BIT_OUT_STRUCT;

typedef union{
	BIT_OUT_STRUCT bit;
	uint8_t all;
}MOT_OUT_FRAME;

extern MOT_OUT_FRAME MT1_Port, MT2_Port, MT3_Port, MT4_Port;

typedef struct{
	uint8_t SOUND1:1;
	uint8_t SOUND2:1;
	uint8_t SOUND3:1;
	uint8_t SOUND4:1;
	uint8_t SOUND5:1;
	uint8_t SOUND6:1;
	uint8_t RES7:1;
	uint8_t RES8:1;
}SOUND_OUT_BIT_STRUCT;

typedef union{
	SOUND_OUT_BIT_STRUCT bit;
	uint8_t all;
}SOUND_OUT_FRAME;

extern SOUND_OUT_FRAME Sound_Out_Port;

typedef struct{
	uint8_t IO1_1:1;
	uint8_t IO1_2:1;
	uint8_t IO1_3:1;
	uint8_t IO1_4:1;
	uint8_t IO2_1:1;
	uint8_t IO2_2:1;
	uint8_t IO2_3:1;
	uint8_t IO2_4:1;
}TIM_BIT_STRUCT;

typedef struct{
	uint8_t T1:4;
	uint8_t T2:4;
}TIM_PORT_STRUCT;

typedef union{
	TIM_BIT_STRUCT bit;
	TIM_PORT_STRUCT bit4;
	uint8_t all;
}TIM_IO_FRAME;

extern TIM_IO_FRAME TIM_Out_Port;
extern TIM_IO_FRAME TIM_IN_PORT;

//#define TIM_OUT_DATA_WRITE(DATA)	 GPIO_Write_Port(GPIOJ, DATA_LOW, (~DATA))
#define TIM_OUT_DATA_WRITE(DATA) 	ADDR16(PORTF) = TIM_Out_Port.all = ( DATA & 0xFF );

typedef struct{
	uint8_t PIN_UP:1;
	uint8_t PIN_DOWN:1;
	uint8_t RES:6;
}LIFT_OUT_BIT_STRUCT;

typedef union{
	LIFT_OUT_BIT_STRUCT bit;
	uint8_t all;
}LIFT_OUT_FRAME;

extern LIFT_OUT_FRAME Lift_Out_Port;

#define LIFT_TOP_POS				13500
#define LIFT_BOTTOM_POS 			0

#define LIFT_UP_SOLW_POS			12000
#define LIFT_DOWN_SLOW_POS 			1500

#define LIFT_SPEED_MIN_DA_VAL		8000
#define LIFT_SPEED_MAX_DA_VAL 		27000

extern int16_t Lift_Enc_Count;
// Lift_Enc_Count = TIM2->CNT = 0;

typedef struct{
	uint16_t OUT1:1;
	uint16_t OUT2:1;
	uint16_t OUT3:1;
	uint16_t OUT4:1;
	uint16_t OUT5:1;
	uint16_t OUT6:1;
	uint16_t OUT7:1;
	uint16_t OUT8:1;
	uint16_t RES9:1;
	uint16_t RES10:1;
	uint16_t RES11:1;
	uint16_t RES12:1;
	uint16_t RES13:1;
	uint16_t RES14:1;
	uint16_t RES15:1;
	uint16_t RES16:1;
}PIO_OUT_BIT_STRUCT;

typedef struct{
	uint16_t LL:4;
	uint16_t LH:4;
	uint16_t HL:4;
	uint16_t HH:4;
}PIO_OUT_4BIT_STRUCT;

typedef struct{
	uint16_t L:8;
	uint16_t H:8;
}PIO_OUT_BYTE_STRUCT;

typedef union{
	PIO_OUT_BIT_STRUCT bit;
	PIO_OUT_4BIT_STRUCT bit4;
	PIO_OUT_BYTE_STRUCT byte;
	uint16_t all;
}PIO_OUT_FRAME;

extern PIO_OUT_FRAME PIO_Out_Port;

typedef struct{
	uint8_t MANUAL_UP:1;
	uint8_t MANUAL_DN:1;
	uint8_t RES:6;
}MANUAL_LIFT_STRUCT;

typedef union{
	uint8_t all;
	MANUAL_LIFT_STRUCT bit;
}MANUAL_LIFT_FRAME;

extern MANUAL_LIFT_FRAME LiftSW_IN, LiftSW_Old;
#define LIFT_KEY_NONE_EVENT		0
#define LIFT_KEY_UP_EVENT 		1
#define LIFT_KEY_DOWN_EVENT 	2
#define LIFT_KEY_STOP_EVENT 	3

extern void AGV_Stop_Function(void);

/*** MOBIS Default Parameter ****************************************************/
#define PARAM_P1_AGV_ID 				99	
#define PARAM_P1_SPEED1					110
#define PARAM_P1_SPEED2					250
#define PARAM_P1_SPEED3					400
#define PARAM_P1_SPEED4 				600
#define PARAM_P1_SPEED_MANUAL			200

#define PARAM_P1_P_GAIN					35
#define PARAM_P1_I_GAIN					55
#define PARAM_P1_D_GAIN 				10
#define PARAM_P1_DISTANCE_CORRECTION	1
#define PARAM_P1_ANGLE_CORRETION		20
#define PARAM_P1_SPEED_SPIN 			150

#define PARAM_P2_X_OFFSET 				0
#define PARAM_P2_Y_OFFSET				0
#define PARAM_P2_ANGLE_OFFSET 			0
#define PARAM_P2_STOP_OFFSET 			5
#define PARAM_P2_LINE_X_CORRECTIION 	0
#define PARAM_P2_LINE_DEG_CORRECTION 	0

#define PARAM_P2_SPIN_P_GAIN 			10
#define PARAM_P2_SPIN_I_GAIN 			30
#define PARAM_P2_SPIN_D_GAIN			20
#define PARAM_P2_SPIN_STOP_OFFSET 		12
#define PARAM_P2_LINE_SPEED_SUM 		10
#define PARAM_P2_LINE_CORRECT_DIV 		10

#define PARAM_P3_H_SPEED_X_CORRECTION 		2
#define PARAM_P3_MOTOR_BALANCE 				0
#define PARAM_P3_TORQUE_CORRETION			0
#define PARAM_P3_L_SPEED_X_CORRECTION		2
#define PARAM_P3_LIFT_SPEED					20000
#define PARAM_P3_RESERVED3					0

#define PARAM_P3_H_SPEED_DEG_CORRECTION		5
#define PARAM_P3_H_SPEED_CORRECTION 		20
#define PARAM_P3_MANUAL_COURSE_NUM			0
#define PARAM_P3_LOW_SPEED_DEG_CORRECTION	0
#define PARAM_P3_SPIN_SLOW_START			30
#define PARAM_P3_RESERVED4 					0


/*******************************************************************************/

/*** TOUCH HMI VARIABLE DEFINE ***/
#define AGV_ID				1

#define QR_Tag				0
#define Line				1

#define ON					1
#define OFF					0

#define Auto_Operate		1
#define Manual_Operate		0

#define BLUE				1
#define YELLOW				2
#define GREEN				3
#define RED					4

#define Psition_Inpuiry		0x06
#define Sel_Direction		0x07
#define Sel_Blue			0x06
#define Sel_Green			0x04
#define Sel_RED				0x04

#define Dir_Left			0x02
#define Dir_Right			0x01

#define Color_Blue			0x01
#define Color_Green			0x02
#define Color_Red			0x04

#define Motor_Control_Offset		2048

#define Normal_Stop					0
#define Lift_Loading				1
#define Lift_Unloading				2

#define Event_Send_Header_Offset		10

#define Receive_Frame_HeaderOffset		13
#define JobID_Frame_Offset				15
#define NumofNode_Offset				20


#define JobID_Len					5
#define Node_Count_Len				2

#define Index_Num_Len				3
#define NodeID_Len					6
#define Direction_Len				2
#define Speed_Len					3
#define Turn_Angle_Len				3
#define Safety_CMD_Len				1
#define Frame_Len					18

#define Index_Num_Offset			22

#define NodeID_Num_Offset			( Index_Num_Offset + Index_Num_Len )
#define Direction_Offset			( NodeID_Num_Offset + NodeID_Len )
#define Speed_Num_Offset			( Direction_Offset + Direction_Len )
#define Turn_Angle_Offset			( Speed_Num_Offset + Speed_Len )
#define Safety_CMD_Offset			( Turn_Angle_Offset + Turn_Angle_Len )

#define Send_Frame_BodyCountOffset 	10
#define Send_Frame_HeaderOffset 	14
#define AGV_Action_Num_Addr 		20

#define Modbus_Frame_Wait     0
#define Modbus_Frame_Rev      1
#define Modbus_Rev_Complete   2

#define Single_Coil_Read      0x01
#define Single_Register_Read  0x03
#define Single_Coil_Write     0x05
#define Single_Register_Write 0x06

#define Off_Area              0
#define Normal_Area           8
#define Charger_Lift_Area         9
#define Chamber_Area          10
#define Cooling_CahmberCell_Area      11

#define First_Destination     0
#define Second_Destination      1
#define Third_Destination     2
#define Fourth_Destination      3

#define Left_Spin         0
#define Right_Spin        1

#define SpinToBackward        1
#define SpinToForward       2
#define SpinToStop          3
#define SpinToLoading       4
#define SpinToUnloading       5
#define SpinToLoadingToForward    6
#define SpinToLoadingToBackward   7
#define SpinToUnloadingToForward  8
#define SpinToUnloadingToBackward 9

#define Stop_End          0
#define StopToLoading       1
#define StopToUnloading       2
#define StopToDelayStart      3

#define Forward_Restart       1
#define Backward_Restart      2


#define Right_Mark_En     1
#define Left_Mark_En        2

#define Mark_Sensor       0
#define Encoder         1

#define Encoder_Stop_Pulse    4650


/////LCD ï¿½ï¿½ï¿½ï¿½ ï¿½Ö¼ï¿½/////////////
#define Driving_Status_Addr     1
#define Guide_Sensor_State_Addr   2
#define Attach_Status       3
#define AGV_No_Buff         4
#define Write_RFID_No1        5
#define Write_RFID_No2        6
#define Write_RFID_No3        7
#define Write_RFID_No4        8
#define Read_RFID_No1       9
#define Read_RFID_No2       10
#define Read_RFID_No3       11
#define Read_RFID_No4       12


#define Radian      0
#define Curve_Speed   1
#define Distance    2
#define Flag      3
#define Curve_Sum   4
#define Target_Encoder  5
#define Turn_Direction  6

#define Auto_Driving  1
#define Manual_Driving  0

// #define M1_M2_Motor_Enable      GpioDataRegs.GPADAT.bit.GPIO2 = 1
// #define M1_M2_Motor_Disable     GpioDataRegs.GPADAT.bit.GPIO2 = 0


////ï¿½Üºï¿½ ï¿½ï¿½Ä¡ ï¿½Ö¼ï¿½/////////
// #define FPGA_ADDR   0x004000
// #define C2550_ADDR    0x100000

///FPGA Read Address/////////////
// #define Encoder1_Addr 0x004000
// #define Encoder2_Addr 0x004001
// #define Encoder3_Addr 0x004002
// #define Encoder4_Addr 0x004003

///FPGA Read Address/////////////
// #define PORTA       0x004000
// #define PORTB       0x004001
// #define PORTC       0x004002
// #define PORTD       0x004003
// #define PORTE       0x004004
// #define PORTF       0x004005
// #define PORTG       0x004006

/////Communication Control Port//////////////

/////kEY_INPUT Port//////////////
#define KEY_Auto_Start    0x01    // 0
#define KEY_Reset         0x04    // 2
#define KEY_Stop          0x02    //  1
#define KEY_EMR_EPB1      0x08    //  3
#define KEY_BUMPF         0x40    //  6

#define Backward_Detection_1    6
#define Backward_Detection_2    7

//#define KEY_BUMPR       7
#define KEY_Foward        8
#define KEY_Backward      9

/*
#define Motor1_CW       0x80      //Left_Motor
#define Motor2_CW       0x40      //Right_Motor
#define Motor3_CW       0x20
#define Motor4_CW       0x10
*/


/*
#define Motor1_CW       0x01      //Left_Motor
#define Motor2_CW       0x02      //Right_Motor
#define Motor3_CW       0x04
#define Motor4_CW       0x08
*/
#define Motor_Start_Stop  0x10
#define Motor_RUN         0x20
#define Motor_VR_EXT      0x40

#define Motor1_CW       0x01      //Left_Motor
#define Motor1_CCW        0x02      //Right_Motor
#define Motor2_CW       0x04
#define Motor2_CCW        0x08
#define Motor3_CW       0x10      //Left_Motor
#define Motor3_CCW        0x20      //Right_Motor
#define Motor4_CW       0x40
#define Motor4_CCW        0x80

#define Motor1_Break      0x01      //Left_Motor
#define Motor2_Break      0x02      //Right_Motor
#define Motor3_Break      0x04
#define Motor4_Break      0x08
#define Motor_Start       0x10
#define Motor_Stop        0x20
#define Motor_RST       0x40

/*
#define Lift1_High        0x02
#define Lift2_High        0x01
*/
#define Lift1_High        0x01
#define Lift2_High        0x02

#define Lift1_TEST        0x01
#define Lift2_TEST        0x02

#define Lift_Wait       0
#define Lift_Stop       1
#define Lift_Up_Wait      2
#define Lift_Down_Wait      3
#define Lift_Up_Error     4
#define Lift_Down_Error     5
#define Lift_Up_Action      6
#define Lift_Down_Action    7
#define Lift_Reset_Action     8


//////Guide-Driving Mode///////////////////
#define Driving_Wait        0
#define Driving           1
#define Table_IN            2
#define Table_Out         3
#define Slow_Driving        4
#define Mark_Inspection     5
#define Lift_Moving         6
#define Table_IN_PinDown      7
#define Table_Release       8
#define Table_Release_PinDown 9

/////Spot-Mode//////////////////////
#define Spot_DRV_Mode     0
#define Guide_DRV_Mode      1
#define Manual_DRV_Mode   2
#define Stop_DRV_Mode     4

////LCD_Mode/////////////////////
#define Main_Menu       0
#define RFID_Main_Manu      1
#define RFID_Select_Manu    2
#define RFID_Save_Manu      3

#define Auto_DRV          1
#define Manual_DRV        2
#define Forward_DRV       3
#define Backward_DRV        4
#define Right_Spin_DRV      5
#define Left_Spin_DRV     6
#define AGV_Rotation        7
#define Right_Turn_DRV      8
#define Left_Turn_DRV     9

#define Vertical_Right      10
#define Vertical_Left     11
#define Motor_Steering      12

#define Motor_Origin    'O'

#define Lift_Timeout      	2
#define Lift_Up         	1
#define Lift_Down         	0

#define AGV_Wait          	0
#define AGV_Stop          	13
#define PIN_Moving        	14

#define AGV_EMR         	20
#define AGV_Line_Error      21
#define AGV_Bump_Err        22
#define AGV_Near_F_Err      23
#define AGV_Near_B_Err      24

#define AGV_Low_Battery     25
#define AGV_Reset_REQ     	26
#define AGV_Initializing    27

#define AGV_Motor_ALR     	29
#define AGV_Carrier_Err     30
#define AGV_R2100_F_Err     31
#define AGV_R2100_B_Err     32

#define AGV_CHARGING        33

#define AGV_Direction_Err   34

#define Route_Error       	35
#define Double_Insert       36

#define IO_Test_Mode    	37
#define Pause_Wait      	38
#define Car_Tagging_Err 	39


#define	Charge_SEQ			42
#define Charge_Stop_SEQ 	43

#define Brake_Release_Err 	44
#define Pallet_Detect_Err  	45

#define Lift_Motor_ALR		46			// 리프트 알람
#define Lift_Timeout_Err 	47			// 리프트 타임 아웃
#define AGV_Chargine_Err 	48 			// 충전 이상 

#define Before_Spin      	1
#define After_Spin        	2

#define Door_Open         	1
#define Door_Close        	2

#define AUTO_MODE       	0
#define MANUAL_MODE       	1

////AGVC_ï¿½Û½Å¿ï¿½ ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½////
#define ONLINE           '4'
#define OFFLINE         '0'

#define AUTO            '1'
#define MANUAL          '2'

#define AGV_RUN         '3'
#define AGV_IDLE          '4'
#define AGV_CHARGE        '5'
#define AGV_DOWN          '1'

#define ACTIVE          '1'
#define NON_ACTIVE        '2'

#define ENTER_MAP       '1'
#define EXIT_MAP          '2'

#define BAT_NOMAL       '2'
#define BAT_LOW_1       '3'
#define BAT_LOW_2       '4'

#define STOP_NOMAL        '0'
#define STOP_STOP       '1'
#define STOP_HOST_PUASE   '4'
#define STOP_VEHICLE_PUASE  '3'

#define AGV_INSTALL       '1'
#define AGV_REMOVE        '2'
#define AGV_UNKNOWN       '3'
#define AGV_ERROR       '4'

///AGVC EXECUTE ï¿½ï¿½ï¿½ï¿½ ï¿½ï¿½ï¿½ï¿½ Addr/////////
#define EXE_Car_ID        0
#define EXE_Source_ND     1
#define EXE_Destination_ND  2
#define EXE_Job_ID        4
#define EXE_Job_S       5
#define EXE_Job_D       6
#define EXE_State_No      7

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define MT3_ABS_SCK_Pin GPIO_PIN_2
#define MT3_ABS_SCK_GPIO_Port GPIOE
#define MT3_ABS_DATA_Pin GPIO_PIN_5
#define MT3_ABS_DATA_GPIO_Port GPIOE
#define KEY_IN1_Pin GPIO_PIN_8
#define KEY_IN1_GPIO_Port GPIOI
#define KEY_IN2_Pin GPIO_PIN_9
#define KEY_IN2_GPIO_Port GPIOI
#define KEY_IN3_Pin GPIO_PIN_10
#define KEY_IN3_GPIO_Port GPIOI
#define KEY_IN4_Pin GPIO_PIN_11
#define KEY_IN4_GPIO_Port GPIOI
#define KEY_IN5_Pin GPIO_PIN_12
#define KEY_IN5_GPIO_Port GPIOI
#define KEY_IN6_Pin GPIO_PIN_13
#define KEY_IN6_GPIO_Port GPIOI
#define KEY_IN7_Pin GPIO_PIN_14
#define KEY_IN7_GPIO_Port GPIOI
#define SCIF_RX_Pin GPIO_PIN_6
#define SCIF_RX_GPIO_Port GPIOF
#define SCIF_TX_Pin GPIO_PIN_7
#define SCIF_TX_GPIO_Port GPIOF
#define MT1_ERR_Pin GPIO_PIN_8
#define MT1_ERR_GPIO_Port GPIOF
#define MT2_ERR_Pin GPIO_PIN_9
#define MT2_ERR_GPIO_Port GPIOF
#define MT3_ERR_Pin GPIO_PIN_10
#define MT3_ERR_GPIO_Port GPIOF
#define BAT_CHECK_Pin GPIO_PIN_0
#define BAT_CHECK_GPIO_Port GPIOC
#define TORQUE_AD_MT1_Pin GPIO_PIN_1
#define TORQUE_AD_MT1_GPIO_Port GPIOC
#define TORQUE_AD_MT2_Pin GPIO_PIN_2
#define TORQUE_AD_MT2_GPIO_Port GPIOC
#define TORQUE_AD_MT3_Pin GPIO_PIN_3
#define TORQUE_AD_MT3_GPIO_Port GPIOC
#define EQEP3A_Pin GPIO_PIN_0
#define EQEP3A_GPIO_Port GPIOA
#define EQEP3B_Pin GPIO_PIN_1
#define EQEP3B_GPIO_Port GPIOA
#define SCIB_TX_Pin GPIO_PIN_2
#define SCIB_TX_GPIO_Port GPIOA
#define DAC_CLR_Pin GPIO_PIN_2
#define DAC_CLR_GPIO_Port GPIOH
#define DAC_BIN_Pin GPIO_PIN_3
#define DAC_BIN_GPIO_Port GPIOH
#define SCIB_RX_Pin GPIO_PIN_3
#define SCIB_RX_GPIO_Port GPIOA
#define ROM_CS_Pin GPIO_PIN_4
#define ROM_CS_GPIO_Port GPIOA
#define ROM_SCK_Pin GPIO_PIN_5
#define ROM_SCK_GPIO_Port GPIOA
#define ROM_MISO_Pin GPIO_PIN_6
#define ROM_MISO_GPIO_Port GPIOA
#define ROM_MOSI_Pin GPIO_PIN_7
#define ROM_MOSI_GPIO_Port GPIOA
#define TORQUE_AD_MT4_Pin GPIO_PIN_4
#define TORQUE_AD_MT4_GPIO_Port GPIOC
#define ROM_WP_Pin GPIO_PIN_0
#define ROM_WP_GPIO_Port GPIOB
#define KEY_IN8_Pin GPIO_PIN_15
#define KEY_IN8_GPIO_Port GPIOI
#define TIM1_OUT1_Pin GPIO_PIN_0
#define TIM1_OUT1_GPIO_Port GPIOJ
#define TIM1_OUT2_Pin GPIO_PIN_1
#define TIM1_OUT2_GPIO_Port GPIOJ
#define TIM1_OUT3_Pin GPIO_PIN_2
#define TIM1_OUT3_GPIO_Port GPIOJ
#define TIM1_OUT4_Pin GPIO_PIN_3
#define TIM1_OUT4_GPIO_Port GPIOJ
#define TIM2_OUT1_Pin GPIO_PIN_4
#define TIM2_OUT1_GPIO_Port GPIOJ
#define MT4_ERR_Pin GPIO_PIN_11
#define MT4_ERR_GPIO_Port GPIOF
#define SPARE_IN1_Pin GPIO_PIN_12
#define SPARE_IN1_GPIO_Port GPIOF
#define SPARE_IN2_Pin GPIO_PIN_13
#define SPARE_IN2_GPIO_Port GPIOF
#define SPARE_IN3_Pin GPIO_PIN_14
#define SPARE_IN3_GPIO_Port GPIOF
#define SPARE_IN4_Pin GPIO_PIN_15
#define SPARE_IN4_GPIO_Port GPIOF
#define SPARE_IN5_Pin GPIO_PIN_0
#define SPARE_IN5_GPIO_Port GPIOG
#define SPARE_IN6_Pin GPIO_PIN_1
#define SPARE_IN6_GPIO_Port GPIOG
#define SCIC_TX_Pin GPIO_PIN_10
#define SCIC_TX_GPIO_Port GPIOB
#define SCIC_RX_Pin GPIO_PIN_11
#define SCIC_RX_GPIO_Port GPIOB
#define TIM2_OUT2_Pin GPIO_PIN_5
#define TIM2_OUT2_GPIO_Port GPIOJ
#define MT1_ABS_SCK_Pin GPIO_PIN_6
#define MT1_ABS_SCK_GPIO_Port GPIOH
#define MT1_ABS_DATA_Pin GPIO_PIN_7
#define MT1_ABS_DATA_GPIO_Port GPIOH
#define ENC_SEL1_Pin GPIO_PIN_8
#define ENC_SEL1_GPIO_Port GPIOH
#define ENC_SEL2_Pin GPIO_PIN_9
#define ENC_SEL2_GPIO_Port GPIOH
#define DIRECT_OUT5_Pin GPIO_PIN_12
#define DIRECT_OUT5_GPIO_Port GPIOH
#define MT_DAC_EN_Pin GPIO_PIN_8
#define MT_DAC_EN_GPIO_Port GPIOD
#define EQEP4A_Pin GPIO_PIN_12
#define EQEP4A_GPIO_Port GPIOD
#define EQEP4B_Pin GPIO_PIN_13
#define EQEP4B_GPIO_Port GPIOD
#define TIM2_OUT3_Pin GPIO_PIN_6
#define TIM2_OUT3_GPIO_Port GPIOJ
#define TIM2_OUT4_Pin GPIO_PIN_7
#define TIM2_OUT4_GPIO_Port GPIOJ
#define PIO_IN1_Pin GPIO_PIN_8
#define PIO_IN1_GPIO_Port GPIOJ
#define PIO_IN2_Pin GPIO_PIN_9
#define PIO_IN2_GPIO_Port GPIOJ
#define PIO_IN3_Pin GPIO_PIN_10
#define PIO_IN3_GPIO_Port GPIOJ
#define PIO_IN4_Pin GPIO_PIN_11
#define PIO_IN4_GPIO_Port GPIOJ
#define ATTACH1_UP_IN_Pin GPIO_PIN_0
#define ATTACH1_UP_IN_GPIO_Port GPIOK
#define ATTACH1_DOWN_IN_Pin GPIO_PIN_1
#define ATTACH1_DOWN_IN_GPIO_Port GPIOK
#define ATTACH2_UP_IN_Pin GPIO_PIN_2
#define ATTACH2_UP_IN_GPIO_Port GPIOK
#define SPARE_IN7_Pin GPIO_PIN_2
#define SPARE_IN7_GPIO_Port GPIOG
#define SPARE_IN8_Pin GPIO_PIN_3
#define SPARE_IN8_GPIO_Port GPIOG
#define SPARE_IN9_Pin GPIO_PIN_4
#define SPARE_IN9_GPIO_Port GPIOG
#define PIO_GO_Pin GPIO_PIN_6
#define PIO_GO_GPIO_Port GPIOG
#define EQEP2A_Pin GPIO_PIN_6
#define EQEP2A_GPIO_Port GPIOC
#define EQEP2B_Pin GPIO_PIN_7
#define EQEP2B_GPIO_Port GPIOC
#define EQEP1A_Pin GPIO_PIN_8
#define EQEP1A_GPIO_Port GPIOA
#define EQEP1B_Pin GPIO_PIN_9
#define EQEP1B_GPIO_Port GPIOA
#define SCIA_RE_Pin GPIO_PIN_11
#define SCIA_RE_GPIO_Port GPIOA
#define SCIB_RE_Pin GPIO_PIN_12
#define SCIB_RE_GPIO_Port GPIOA
#define DIRECT_OUT6_Pin GPIO_PIN_13
#define DIRECT_OUT6_GPIO_Port GPIOH
#define DIRECT_OUT7_Pin GPIO_PIN_14
#define DIRECT_OUT7_GPIO_Port GPIOH
#define DIRECT_OUT8_Pin GPIO_PIN_15
#define DIRECT_OUT8_GPIO_Port GPIOH
#define DAC_NSS_Pin GPIO_PIN_0
#define DAC_NSS_GPIO_Port GPIOI
#define SC_CD_Pin GPIO_PIN_3
#define SC_CD_GPIO_Port GPIOD
#define PIO_IN5_Pin GPIO_PIN_12
#define PIO_IN5_GPIO_Port GPIOJ
#define PIO_IN6_Pin GPIO_PIN_13
#define PIO_IN6_GPIO_Port GPIOJ
#define PIO_IN7_Pin GPIO_PIN_14
#define PIO_IN7_GPIO_Port GPIOJ
#define PIO_IN8_Pin GPIO_PIN_15
#define PIO_IN8_GPIO_Port GPIOJ
#define SCIE_TX_Pin GPIO_PIN_9
#define SCIE_TX_GPIO_Port GPIOG
#define DIRECT_OUT1_Pin GPIO_PIN_10
#define DIRECT_OUT1_GPIO_Port GPIOG
#define DIRECT_OUT2_Pin GPIO_PIN_11
#define DIRECT_OUT2_GPIO_Port GPIOG
#define DIRECT_OUT3_Pin GPIO_PIN_12
#define DIRECT_OUT3_GPIO_Port GPIOG
#define DIRECT_OUT4_Pin GPIO_PIN_13
#define DIRECT_OUT4_GPIO_Port GPIOG
#define SCIE_RX_Pin GPIO_PIN_14
#define SCIE_RX_GPIO_Port GPIOG
#define ATTACH2_DOWN_IN_Pin GPIO_PIN_3
#define ATTACH2_DOWN_IN_GPIO_Port GPIOK
#define MT1_ABS_RESET_Pin GPIO_PIN_4
#define MT1_ABS_RESET_GPIO_Port GPIOK
#define MT3_ABS_RESET_Pin GPIO_PIN_5
#define MT3_ABS_RESET_GPIO_Port GPIOK
#define MARK1_IN_Pin GPIO_PIN_6
#define MARK1_IN_GPIO_Port GPIOK
#define MARK2_IN_Pin GPIO_PIN_7
#define MARK2_IN_GPIO_Port GPIOK
#define MOTOR_ON_Pin GPIO_PIN_15
#define MOTOR_ON_GPIO_Port GPIOG
#define SCIA_TX_Pin GPIO_PIN_6
#define SCIA_TX_GPIO_Port GPIOB
#define SCIA_RX_Pin GPIO_PIN_7
#define SCIA_RX_GPIO_Port GPIOB
#define SCID_RX_Pin GPIO_PIN_0
#define SCID_RX_GPIO_Port GPIOE
#define SCID_TX_Pin GPIO_PIN_1
#define SCID_TX_GPIO_Port GPIOE
#define LED_BLUE_Pin GPIO_PIN_5
#define LED_BLUE_GPIO_Port GPIOI
#define LED_RED_Pin GPIO_PIN_6
#define LED_RED_GPIO_Port GPIOI
#define LED_GREEN_Pin GPIO_PIN_7
#define LED_GREEN_GPIO_Port GPIOI
/* USER CODE BEGIN Private defines */

#define HIGH	GPIO_PIN_SET    // 1
#define LOW		GPIO_PIN_RESET  // 0

#define DELAY_US(delay)     DWT_Delay(delay)
#define DELAY_MS(delay)     HAL_Delay(delay)

#define ADDR16(A)     (*((volatile unsigned char *)(A)))

#define PORTA       0x60000000
#define PORTB       0x60000001
#define PORTC       0x60000002
#define PORTD       0x60000003
#define PORTE       0x60000004
#define PORTF       0x60000005
#define PORTG       0x60000006
#define PORTH       0x60000007
#define PORTI       0x60000008
#define PORTJ       0x60000009
#define PORTK       0x6000000A
#define PORTL       0x6000000B

#define DRV_Complete        0
#define CV_Complete         1
#define Charge_Complete     2
#define ET_Complete         3
#define Job_Cancel          4
#define DRV_Job_Start 		10

#define SCIA_485      1     // SCIA RS-485 USE
#define SCIB_485      2     // SCIB RS-485 USE

#define SCIA_485_TX HAL_GPIO_WritePin(SCIA_RE_GPIO_Port, SCIA_RE_Pin, GPIO_PIN_SET)
#define SCIA_485_RX HAL_GPIO_WritePin(SCIA_RE_GPIO_Port, SCIA_RE_Pin, GPIO_PIN_RESET)

#define SCIB_485_TX HAL_GPIO_WritePin(SCIB_RE_GPIO_Port, SCIB_RE_Pin, GPIO_PIN_SET)
#define SCIB_485_RX HAL_GPIO_WritePin(SCIB_RE_GPIO_Port, SCIB_RE_Pin, GPIO_PIN_RESET)

#define Direction_Encoder1_SCK_High HAL_GPIO_WritePin(MT1_ABS_SCK_GPIO_Port, MT1_ABS_SCK_Pin, GPIO_PIN_SET)
#define Direction_Encoder1_SCK_Low  HAL_GPIO_WritePin(MT1_ABS_SCK_GPIO_Port, MT1_ABS_SCK_Pin, GPIO_PIN_RESET)

#define Direction_Encoder2_SCK_High HAL_GPIO_WritePin(MT3_ABS_SCK_GPIO_Port, MT3_ABS_SCK_Pin, GPIO_PIN_SET)
#define Direction_Encoder2_SCK_Low  HAL_GPIO_WritePin(MT3_ABS_SCK_GPIO_Port, MT3_ABS_SCK_Pin, GPIO_PIN_RESET)

#define CPLD_GPIO_Write(addr, port, data)   ADDR16(addr) = port = data

#define MT1_PORT_AL_IN 		HAL_GPIO_ReadPin(MT1_ERR_GPIO_Port, MT1_ERR_Pin)
#define MT2_PORT_AL_IN 		HAL_GPIO_ReadPin(MT2_ERR_GPIO_Port, MT2_ERR_Pin)
#define MT3_PORT_AL_IN 		HAL_GPIO_ReadPin(MT3_ERR_GPIO_Port, MT3_ERR_Pin)
#define MT4_PORT_AL_IN 		HAL_GPIO_ReadPin(MT4_ERR_GPIO_Port, MT4_ERR_Pin)


#define FRONT_CART_IN 		HAL_GPIO_ReadPin ( ATTACH1_UP_IN_GPIO_Port, ATTACH1_UP_IN_Pin )
#define REAR_CART_IN 		HAL_GPIO_ReadPin ( ATTACH2_UP_IN_GPIO_Port, ATTACH2_UP_IN_Pin )

#define LIFT_UP_IN 			HAL_GPIO_ReadPin ( ATTACH1_DOWN_IN_GPIO_Port, ATTACH1_DOWN_IN_Pin )
#define LIFT_DONW_IN 		HAL_GPIO_ReadPin ( ATTACH2_DOWN_IN_GPIO_Port, ATTACH2_DOWN_IN_Pin )

#define CHARGE_MC_PIN 			SPARE_Out_Port.bit.IO1
#define CHARGE_MC_ADDR_WRITE	ADDR16(PORTJ) = SPARE_Out_Port.all



// #define MT1_PORT_ADDR_L_BYTE_WRITE   ADDR16(PORTA) = MT1_Port.byte.L
// #define MT1_PORT_ADDR_H_BYTE_WRITE   ADDR16(PORTB) = MT1_Port.byte.H
// #define MT2_PORT_ADDR_L_BYTE_WRITE   ADDR16(PORTC) = MT2_Port.byte.L
// #define MT2_PORT_ADDR_H_BYTE_WRITE   ADDR16(PORTD) = MT2_Port.byte.H
// #define MT3_PORT_ADDR_L_BYTE_WRITE   ADDR16(PORTE) = MT3_Port.byte.L
// #define MT3_PORT_ADDR_H_BYTE_WRITE   ADDR16(PORTF) = MT3_Port.byte.H
// #define MT4_PORT_ADDR_L_BYTE_WRITE   ADDR16(PORTG) = MT4_Port.byte.L
// #define MT4_PORT_ADDR_H_BYTE_WRITE   ADDR16(PORTH) = MT4_Port.byte.H

/* CPLD -> MOTOR1 Out Control */
#define MT1_PORT_ADDR_WRITE		ADDR16(PORTA) = MT1_Port.all

#define MT1_PORT_START_PIN        MT1_Port.bit.OUT1   // RSVON
// #define MT1_PORT_STOP_PIN         MT1_Port.bit.OUT2
#define MT1_PORT_RESET_PIN        MT1_Port.bit.OUT3
#define MT1_PORT_BREAK_PIN        MT1_Port.bit.OUT4   // RBRK
#define MT1_PORT_DIR_CW_PIN       MT1_Port.bit.OUT2   // RIN2
#define MT1_PORT_DIR_CCW_PIN      MT1_Port.bit.OUT6
#define MT1_PORT_OUT_ALL_PIN      MT1_Port.all

/* CPLD -> MOTOR2 Out Control */
#define MT2_PORT_ADDR_WRITE		ADDR16(PORTB) = MT2_Port.all

#define LIFT_OUT_ADDR_WRITE 	MT2_PORT_ADDR_WRITE

#define LIFT_OUT_START_PIN 		MT2_Port.bit.OUT1
#define LIFT_OUT_DIR_PIN 		MT2_Port.bit.OUT2
#define LIFT_OUT_BRK_PIN 		MT2_Port.bit.OUT4

#define LIFT_OUT_PIN_ALL        MT2_Port.all

// #define LIFT_POW_ON 			ADDR16(PORTB) = MT2_Port.all = 0x01
// #define LIFT_POW_OFF 			ADDR16(PORTB) = MT2_Port.all = 0x00

/* CPLD -> MOTOR3 Out Control */
#define MT3_PORT_ADDR_WRITE		ADDR16(PORTC) = MT3_Port.all

#define MT3_PORT_START_PIN        MT3_Port.bit.OUT1   // LSVON
// #define MT3_PORT_STOP_PIN         MT3_Port.bit.OUT2
#define MT3_PORT_RESET_PIN        MT3_Port.bit.OUT3
#define MT3_PORT_BREAK_PIN        MT3_Port.bit.OUT4   // LBRK
#define MT3_PORT_DIR_CW_PIN       MT3_Port.bit.OUT2   // LIN2
#define MT3_PORT_DIR_CCW_PIN      MT3_Port.bit.OUT6
#define MT3_PORT_OUT_ALL_PIN      MT3_Port.all

/*  CPLD -> MOTOR4 Out Control */
#define MT4_PORT_ADDR_WRITE		ADDR16(PORTD) = MT4_Port.all

/*  CPLD -> SOUND Out Control */
#define SOUND_OUT_ADDR_WRITE    ADDR16(PORTE) = Sound_Out_Port.all

#define SOUND_OUT_PIN1		Sound_Out_Port.bit.SOUND1
#define SOUND_OUT_PIN2		Sound_Out_Port.bit.SOUND2
#define SOUND_OUT_PIN3		Sound_Out_Port.bit.SOUND3
#define SOUND_OUT_PIN4		Sound_Out_Port.bit.SOUND4
#define SOUND_OUT_PIN5		Sound_Out_Port.bit.SOUND5
#define SOUND_OUT_PIN6		Sound_Out_Port.bit.SOUND6
#define SOUND_OUT_PIN_ALL	Sound_Out_Port.all


/* CPLD -> TIM Sensor Out Control */
// #define TIM_OUT_ADDR_Write  ADDR16(PORTF) = TIM_Out_Port.all

// #define TIM1_OUT_PIN1			TIM_Out_Port.bit.IO1_1
// #define TIM1_OUT_PIN2			TIM_Out_Port.bit.IO1_2
// #define TIM1_OUT_PIN3			TIM_Out_Port.bit.IO1_3
// #define TIM1_OUT_PIN4			TIM_Out_Port.bit.IO1_4

// #define TIM2_OUT_PIN1			TIM_Out_Port.bit.IO2_1
// #define TIM2_OUT_PIN2			TIM_Out_Port.bit.IO2_2
// #define TIM2_OUT_PIN3			TIM_Out_Port.bit.IO2_3
// #define TIM2_OUT_PIN4			TIM_Out_Port.bit.IO2_4

// #define TIM1_OUT_PIN_ALL		TIM_Out_Port.bit4.T1
// #define TIM2_OUT_PIN_ALL		TIM_Out_Port.bit4.T2
// #define TIM_OUT_PIN_ALL			TIM_Out_Port.all

// #define LIFT_POW_ON_PIN           MT2_Port.bit.OUT1   // PON
// #define LIFT_POW_ON               ADDR16(PORTB) = MT1_Port.byte.H = 0x01;
// #define LIFT_POW_OFF              ADDR16(PORTB) = MT1_Port.byte.H = 0x00;

/* CPLD -> Lift Up/Down Out Control */
// #define LIFT_OUT_ADDR_Write     ADDR16(PORTG) = Lift_Out_Port.all

// #define LIFT_OUT_UP_PIN         Lift_Out_Port.bit.PIN_UP
// #define LIFT_OUT_DOWN_PIN       Lift_Out_Port.bit.PIN_DOWN
// #define LIFT_OUT_PIN_ALL        Lift_Out_Port.all

/* CPLD -> PIO Out Control */
#define PIO_OUT_ADDR_L_Write	ADDR16(PORTH) = PIO_Out_Port.byte.L
#define PIO_OUT_ADDR_H_Write	ADDR16(PORTI) = PIO_Out_Port.byte.H

#define PIO_OUT_PIN1			PIO_Out_Port.bit.OUT1
#define PIO_OUT_PIN2			PIO_Out_Port.bit.OUT2
#define PIO_OUT_PIN3			PIO_Out_Port.bit.OUT3
#define PIO_OUT_PIN4			PIO_Out_Port.bit.OUT4
#define PIO_OUT_PIN5			PIO_Out_Port.bit.OUT5
#define PIO_OUT_PIN6			PIO_Out_Port.bit.OUT6
#define PIO_OUT_PIN7			PIO_Out_Port.bit.OUT7
#define PIO_OUT_PIN8			PIO_Out_Port.bit.OUT8
#define PIO_OUT_PIN_BYTE_L 		PIO_Out_Port.byte.L

#define PIO_OUT_MODE			PIO_Out_Port.bit.MODE
#define PIO_OUT_SEL				PIO_Out_Port.bit.SEL
#define PIO_OUT_PIN_BYTE_H 		PIO_Out_Port.byte.H

#define PIO_OUT_PIN_ALL			PIO_Out_Port.all

/* CPLD -> Spare Port Control */
typedef union{
	IO_BIT16_STRUCT bit;
	unsigned char all;
}SPARE_OUT_FRAME;

extern SPARE_OUT_FRAME SPARE_Out_Port;

typedef struct{
	uint16_t KeyStart:1;
	uint16_t KeyStop:1;
	uint16_t KeyReset:1;
	uint16_t KeyAUTO:1;
	uint16_t KeyMANU:1;
	uint16_t KeyFLEMG:1;
	uint16_t KeyFREMG:1;
	uint16_t KeyBRK_REL:1;

	uint16_t KeyBUMP:1;
	uint16_t KeyRLEMG:1;
	uint16_t KeyRREMG:1;
	uint16_t KeyLiftUP_SW:1;
	uint16_t KeyLiftDN_SW:1;
	uint16_t KeyLiftUP_IN:1;
	uint16_t KeyLiftDN_IN:1;
	uint16_t RES16:1;
}KEY_IN_STRUCT;

typedef struct{
	uint16_t L:8;
	uint16_t H:8;
}KEY_IN_BYTE_STRUCT;

typedef union{
	KEY_IN_STRUCT bit;
	KEY_IN_BYTE_STRUCT byte;
	uint16_t all;
}KEY_IN_FRAME;

extern uint8_t Key_Event, Key_Event_Flag;
typedef struct{
	uint8_t EMG_FL:1;
	uint8_t EMG_FR:1;
	uint8_t EMG_RL:1;
	uint8_t EMG_RR:1;
	uint8_t BUMP1:1;
	uint8_t BUMP2:1;
	uint8_t BUMP3:1;
	uint8_t BUMP4:1;
}EMG_IO_STRUCT;

typedef struct{
	uint8_t EMG:4;
	uint8_t BUMP:4;
}EMG_IO_BIT4_STRUCT;

typedef union{
	uint8_t all;
	EMG_IO_STRUCT bit;
	EMG_IO_BIT4_STRUCT bit4;
}EMG_IO_FRMAE;

typedef struct{
	uint8_t MT_ON:1;
	uint8_t MC_ON:1;
	uint8_t LF_ON:1;
	uint8_t LED_RED:1;
	uint8_t LED_GRN:1;
	uint8_t LED_BLU:1;
	uint8_t F_CART:1;
	uint8_t R_CART:1;
}ETC_IO_STRUCT;

typedef union{
	uint8_t all;
	ETC_IO_STRUCT bit;
}ETC_IO_FRAME;

typedef struct{
	uint16_t MAN_FWD:1;
	uint16_t MAN_BWD:1;
	uint16_t MAN_LEFT:1;
	uint16_t MAN_RIGHT:1;
	uint16_t START:1;
	uint16_t STOP:1;
	uint16_t SW1:1;
	uint16_t SW2:1;
	uint16_t SW3:1;
	uint16_t SW4:1;
	uint16_t RES11:1;
	uint16_t RES12:1;
	uint16_t RES13:1;
	uint16_t RES14:1;
	uint16_t RES15:1;
	uint16_t RES16:1;
}REMOTE_IO_STRUCT;

typedef struct{
	REMOTE_IO_STRUCT bit;
	uint16_t all;
}REMOTE_IO_FRAME;

// typedef struct{
// 	uint16_t Error_Code:8;
// 	uint16_t Error_Flag:4;
// 	uint16_t Stop_Status:4;
// }STOP_DEBUG_DATA_BIT;

// typedef union{
// 	STOP_DEBUG_DATA_BIT bit;
// 	uint16_t all;
// }STOP_DEBUG_DATA;

typedef struct{
	uint16_t Seq1:4;
	uint16_t Seq2:4;
	uint16_t Seq3:4;
	uint16_t Seq4:4;
}SEQ_NUM_STRUCT;

typedef union{
	uint16_t all;
	SEQ_NUM_STRUCT bit;
}SEQ_NUM_DATA;

typedef struct{
	uint16_t E_Course_Status:1;
	uint16_t E_Pause_Flag:1;
	uint16_t E_Pause_Num:2;
	uint16_t E_F_Tim_Sensing:1;
	uint16_t E_R_Tim_Sensing:1;
	uint16_t E_On_Tag_Flag:1;
	uint16_t E_Direct_Stop:1;
	uint16_t E_Last_Drive:8;
}EVENT_FLAG_STRUCT;

typedef union{
	uint16_t all;
	EVENT_FLAG_STRUCT bit;
}EVENT_FLAG_DATA;


extern unsigned char Direct_Stop_Flag;
extern REMOTE_IO_FRAME Remote_In;
#define MOTOR_STATUS_STOP       0
#define MOTOR_STATUS_RUN        1

#define KEY_IN_GPIO_Port        GPIOI

extern unsigned int Charge_Test_Count;

static const unsigned int Modbus_CRC16[] = {
	0x0000, 0xc0c1, 0xc181, 0x0140, 0xc301, 0x03c0, 0x0280, 0xc241,
	0xc601, 0x06c0, 0x0780, 0xc741, 0x0500, 0xc5c1, 0xc481, 0x0440,
	0xcc01, 0x0cc0, 0x0d80, 0xcd41, 0x0f00, 0xcfc1, 0xce81, 0x0e40,
	0x0a00, 0xcac1, 0xcb81, 0x0b40, 0xc901, 0x09c0, 0x0880, 0xc841,
	0xd801, 0x18c0, 0x1980, 0xd941, 0x1b00, 0xdbc1, 0xda81, 0x1a40,
	0x1e00, 0xdec1, 0xdf81, 0x1f40, 0xdd01, 0x1dc0, 0x1c80, 0xdc41,
	0x1400, 0xd4c1, 0xd581, 0x1540, 0xd701, 0x17c0, 0x1680, 0xd641,
	0xd201, 0x12c0, 0x1380, 0xd341, 0x1100, 0xd1c1, 0xd081, 0x1040,
	0xf001, 0x30c0, 0x3180, 0xf141, 0x3300, 0xf3c1, 0xf281, 0x3240,
	0x3600, 0xf6c1, 0xf781, 0x3740, 0xf501, 0x35c0, 0x3480, 0xf441,
	0x3c00, 0xfcc1, 0xfd81, 0x3d40, 0xff01, 0x3fc0, 0x3e80, 0xfe41,
	0xfa01, 0x3ac0, 0x3b80, 0xfb41, 0x3900, 0xf9c1, 0xf881, 0x3840,
	0x2800, 0xe8c1, 0xe981, 0x2940, 0xeb01, 0x2bc0, 0x2a80, 0xea41,
	0xee01, 0x2ec0, 0x2f80, 0xef41, 0x2d00, 0xedc1, 0xec81, 0x2c40,
	0xe401, 0x24c0, 0x2580, 0xe541, 0x2700, 0xe7c1, 0xe681, 0x2640,
	0x2200, 0xe2c1, 0xe381, 0x2340, 0xe101, 0x21c0, 0x2080, 0xe041,
	0xa001, 0x60c0, 0x6180, 0xa141, 0x6300, 0xa3c1, 0xa281, 0x6240,
	0x6600, 0xa6c1, 0xa781, 0x6740, 0xa501, 0x65c0, 0x6480, 0xa441,
	0x6c00, 0xacc1, 0xad81, 0x6d40, 0xaf01, 0x6fc0, 0x6e80, 0xae41,
	0xaa01, 0x6ac0, 0x6b80, 0xab41, 0x6900, 0xa9c1, 0xa881, 0x6840,
	0x7800, 0xb8c1, 0xb981, 0x7940, 0xbb01, 0x7bc0, 0x7a80, 0xba41,
	0xbe01, 0x7ec0, 0x7f80, 0xbf41, 0x7d00, 0xbdc1, 0xbc81, 0x7c40,
	0xb401, 0x74c0, 0x7580, 0xb541, 0x7700, 0xb7c1, 0xb681, 0x7640,
	0x7200, 0xb2c1, 0xb381, 0x7340, 0xb101, 0x71c0, 0x7080, 0xb041,
	0x5000, 0x90c1, 0x9181, 0x5140, 0x9301, 0x53c0, 0x5280, 0x9241,
	0x9601, 0x56c0, 0x5780, 0x9741, 0x5500, 0x95c1, 0x9481, 0x5440,
	0x9c01, 0x5cc0, 0x5d80, 0x9d41, 0x5f00, 0x9fc1, 0x9e81, 0x5e40,
	0x5a00, 0x9ac1, 0x9b81, 0x5b40, 0x9901, 0x59c0, 0x5880, 0x9841,
	0x8801, 0x48c0, 0x4980, 0x8941, 0x4b00, 0x8bc1, 0x8a81, 0x4a40,
	0x4e00, 0x8ec1, 0x8f81, 0x4f40, 0x8d01, 0x4dc0, 0x4c80, 0x8c41,
	0x4400, 0x84c1, 0x8581, 0x4540, 0x8701, 0x47c0, 0x4680, 0x8641,
	0x8201, 0x42c0, 0x4380, 0x8341, 0x4100, 0x81c1, 0x8081, 0x4040
};

#define Sound1        0x01
#define Sound2        0x02
#define Sound3        0x04
#define Sound4        0x08
#define Sound5        0x10
#define Sound6        0x20

#define MAX_INT         32767
#define MAX_LONG        2147483647
#define MAX_I_TERM      (MAX_LONG / 2)

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
