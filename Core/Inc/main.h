/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/********************************常量定义**************************************/
#define g   9.8

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/********************************参数声明**************************************
*参数用途:	MPU6050关节角度等信息
*修改日期:	20230526
*******************************************************************************/
struct TranBuf{
	uint8_t rxDataAcc[11]; //加速度原始数据存放
	uint8_t rxDataAngAcc[11]; //角加速度原始数据存放
	uint8_t rxDataAng[11]; //角度原始数据存放
	uint8_t rxData[33];

	uint16_t Accx;
	uint16_t Accy;
	uint16_t Accz; //加速度缓存
	uint16_t AngAccx;
	uint16_t AngAccy;
	uint16_t AngAccz; //角加速度缓存
	uint16_t Angx;
	uint16_t Angy;
	uint16_t Angz; //角度缓存
}; //数据缓存

struct StateBuf{
	uint8_t counter; //计数
	uint8_t sum; //合
	uint8_t Flag; //标志
}; //状态统计

struct DataFit{
	double FitBufKnee[100];
//	double FitBufAnkle[100];
	uint8_t FitFlagKnee;
	uint8_t FitFlagAnkle;
	double PKneeS1[6];
	double PAnkleS1[6];
	double PKneeS2[6];
	double PAnkleS2[6];
	int sizenum;
	int State;
	double FitKnee_0[50];
	double FitKnee_1[50];
	double FitKnee_2[50];
	double FitKnee_3[50];
	int FitKnee_0_num;
	int FitKnee_1_num;
	int FitKnee_2_num;
	int FitKnee_3_num;
	double FitKneeT_0[50];
	double FitKneeT_1[50];
	double FitKneeT_2[50];
	double FitKneeT_3[50];
}; //拟合数据缓存与统计
extern struct DataFit Normal;

struct Data{
	float Accx;
	float Accy;
	float Accz; //加速度
	float AngAccx;
	float AngAccy;
	float AngAccz; //角加速度
	float Angx;
	float Angy;
	float Angz; //角度

	float AccxZero;
	float AccyZero;
	float AcczZero; //加速度初始位置
	float AngAccxZero;
	float AngAccyZero;
	float AngAcczZero; //角加速度初始位置
	float AngxZero;
	float AngyZero;
	float AngzZero; //角度初始位置

	struct TranBuf Buf; //状态
	struct StateBuf State; //缓存
	struct DataFit Fit;

	float AngxCal; //关节角度x方向计算值
	float AngxFilter; //关节角度x方向滤波值
}; //关节各个数据

struct joint{
	struct Data WAIST;
	struct Data Hip;
	struct Data Knee;
	struct Data Ankle;
}; //各个关节

extern struct joint Right; //右关节数据组
extern struct joint Left; //左关节数据组
/********************************参数声明**************************************
*参数用途:	CAN消息
*修改日期:	20230526
*******************************************************************************/
struct DataCan
{
	uint16_t CanID;
	volatile float Pos;
	float Vel;
	float Kp;
	float Kd;
	float Tor;
	float PosOut;
	float VelOut;
	float TorOut;
};

extern struct DataCan MIT_A;
extern struct DataCan MIT_B;

typedef struct{
	uint8_t id;
	uint8_t data[8];
	CAN_RxHeaderTypeDef rx_header;
	CAN_FilterTypeDef filter;
}CANRxMessage ;

typedef struct{
	uint8_t id;
	uint8_t data[8];
	CAN_TxHeaderTypeDef tx_header;
}CANTxMessage ;

extern CANTxMessage can_tx;
extern CANRxMessage can_rx;

/********************************参数声明**************************************
*参数用途:	共同体数据缓存、足底压力数据
*修改日期:	20230526
*******************************************************************************/
union UnionBuf
{
	uint8_t HexBuf[4];
	float FloatBuf;
};

struct DataUnionBuf
{
	uint8_t HexBufSumAng[14];
	uint8_t HexBufSumFoot[34];
	uint8_t HexBufDis[3];
	uint8_t HexBufSum[48];
	int AngCounter;
	int FootCounter;
	int DisCounter;
	int Flag;
	union UnionBuf DataUnionBuf;
	float Data[16];
	float DataZero[16];
	int Point[8];
};

//extern struct DataUnionBuf DataLeftBufAng;
extern struct DataUnionBuf DataLeftBuf;
extern struct DataUnionBuf DataRightBufFoot;

extern int timCounter;
/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/********************************函数声明**************************************/
void huart1_printf(char * fmt,...);
void Huart_IT_Init();
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void MPU6050ModDataBuf(struct Data *AllData, uint8_t cRx);
void FootDataBuf(struct DataUnionBuf *AllData, uint8_t cRx);
void AngDataBuf(struct DataUnionBuf *AllData, uint8_t cRx);
double *createArray(int size);
void DataDiv(struct DataFit *Fit);
void polyfit(int n,double x[],double y[],int poly_n,double p[]);
void gauss_solve(int n,double A[],double x[],double b[]);
void Calculate(int poly_n, int n, double p[], double x[], int Flag);
double Horner_Algorithm(int poly_n, double p[], double x);
double Slop(int poly_n, double p[], double x);
void ModbusRead();
uint16_t crc16_modbus(uint8_t *data, uint16_t length);
void low_pass_filter_init(void);
float low_pass_filter(float value);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define PC_MCU_Pin GPIO_PIN_0
#define PC_MCU_GPIO_Port GPIOB
#define S1_2_Pin GPIO_PIN_1
#define S1_2_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_2
#define LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
