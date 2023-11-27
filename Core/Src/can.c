/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
/* Includes ------------------------------------------------------------------*/
#include "can.h"

/* USER CODE BEGIN 0 */
#include "math.h"
#include "main.h"
#include <stdio.h>
#include "stm32f4xx_it.h"
/* USER CODE END 0 */

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

/* CAN1 init function */
void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 9;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

}
/* CAN2 init function */
void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 9;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_3TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */

  /* USER CODE END CAN2_Init 2 */

}

static uint32_t HAL_RCC_CAN1_CLK_ENABLED=0;

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspInit 0 */

  /* USER CODE END CAN2_MspInit 0 */
    /* CAN2 clock enable */
    __HAL_RCC_CAN2_CLK_ENABLE();
    HAL_RCC_CAN1_CLK_ENABLED++;
    if(HAL_RCC_CAN1_CLK_ENABLED==1){
      __HAL_RCC_CAN1_CLK_ENABLE();
    }

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF9_CAN2;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* CAN2 interrupt Init */
    HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspInit 1 */

  /* USER CODE END CAN2_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN1 GPIO Configuration
    PA11     ------> CAN1_RX
    PA12     ------> CAN1_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
  else if(canHandle->Instance==CAN2)
  {
  /* USER CODE BEGIN CAN2_MspDeInit 0 */

  /* USER CODE END CAN2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN2_CLK_DISABLE();
    HAL_RCC_CAN1_CLK_ENABLED--;
    if(HAL_RCC_CAN1_CLK_ENABLED==0){
      __HAL_RCC_CAN1_CLK_DISABLE();
    }

    /**CAN2 GPIO Configuration
    PB12     ------> CAN2_RX
    PB13     ------> CAN2_TX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_12|GPIO_PIN_13);

    /* CAN2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(CAN2_RX0_IRQn);
  /* USER CODE BEGIN CAN2_MspDeInit 1 */

  /* USER CODE END CAN2_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

/********************************实现函数**************************************
*函数原型:	HAL_StatusTypeDef CAN_SetFilters(CAN_HandleTypeDef hcan)
*功　　能:	设置CAN过滤器，此处全部接收
*修改日期:	20230526
 * 参数		| 介绍
 * ---------+--------------------------------------
 * hcan		| hcan1或hcan2
*******************************************************************************/
HAL_StatusTypeDef CAN1_SetFilters()
{
  CAN_FilterTypeDef canFilter;
  canFilter.FilterBank = 0;
  canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilter.FilterScale = CAN_FILTERSCALE_32BIT;

  canFilter.FilterIdHigh = 0x0000;
  canFilter.FilterIdLow = 0x0000;
  canFilter.FilterMaskIdHigh = 0x0000;
  canFilter.FilterMaskIdLow = 0x0000;

  canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
  canFilter.FilterActivation = ENABLE;
  canFilter.SlaveStartFilterBank = 14;
  HAL_StatusTypeDef result = HAL_CAN_ConfigFilter(&hcan1, &canFilter);
  return result;
}
HAL_StatusTypeDef CAN2_SetFilters()
{
  CAN_FilterTypeDef canFilter;
  canFilter.FilterBank = 14;
  canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
  canFilter.FilterScale = CAN_FILTERSCALE_32BIT;

  canFilter.FilterIdHigh = 0x0000;
  canFilter.FilterIdLow = 0x0000;
  canFilter.FilterMaskIdHigh = 0x0000;
  canFilter.FilterMaskIdLow = 0x0000;

  canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
  canFilter.FilterActivation = ENABLE;
  canFilter.SlaveStartFilterBank = 14;
  HAL_StatusTypeDef result = HAL_CAN_ConfigFilter(&hcan2, &canFilter);
  return result;
}

/********************************实现函数**************************************
*函数原型:	float_to_uint(float x, float x_min, float x_max, int bits)
*功　　能:	控制数据转换为MIT格式
*修改日期:	20230526
 * 参数		| 介绍
 * ---------+--------------------------------------
 * x		| 被转换值
 * x_min	| 最小值
 * x_max	| 最大值
 * bits		| 位数
*******************************************************************************/
int float_to_uint(float x, float x_min, float x_max, int bits)
{
    /// Converts a float to an unsigned int, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}


/********************************实现函数**************************************
*函数原型:	uint_to_float(int x_int, float x_min, float x_max, int bits)
*功　　能:	返回数据转换为float型
*修改日期:	20230526
 * 参数		| 介绍
 * ---------+--------------------------------------
 * x_int	| 被转换值
 * x_min	| 最小值
 * x_max	| 最大值
 * bits		| 位数
*******************************************************************************/
float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /// converts unsigned int to float, given range and number of bits ///
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}


/********************************实现函数**************************************
*函数原型:	EnterMotorMode(uint32_t std_id)
*功　　能:	进入电机模式
*修改日期:	20230526
 * 参数		| 介绍
 * ---------+--------------------------------------
 * std_id	| canID
*******************************************************************************/
void EnterMotorMode(uint32_t std_id, CAN_HandleTypeDef hcan)
{
	uint8_t TxData[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFC};
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = std_id;
	TxHeader.TransmitGlobalTime = DISABLE;
	TxHeader.DLC = 8;

  if(HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
    /* Transmission request Error */
    Error_Handler();
    printf("[info]Error_EnterMotorMode\r\n");
  }
}


/********************************实现函数**************************************
*函数原型:	ExitMotorMode(uint32_t std_id)
*功　　能:	退出电机模式
*修改日期:	20230526
*修改日期:
 * 参数		| 介绍
 * ---------+--------------------------------------
 * std_id	| canID
*******************************************************************************/
void ExitMotorMode(uint32_t std_id, CAN_HandleTypeDef hcan)
{
	uint8_t TxData[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFD};
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t TxMailbox;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId=std_id;
	TxHeader.TransmitGlobalTime = DISABLE;
	TxHeader.DLC = 8;

  if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
    /* Transmission request Error */
    Error_Handler();
    printf("[info]Error_ExitMotorMode\r\n");
  }
}


/********************************实现函数**************************************
*函数原型:	CanSend(uint32_t std_id, float p, float v, float kpf, float kdf, float tff)
*功　　能:	发送MIT格式的CAN数据
*修改日期:	20230526
 * 参数		| 介绍
 * ---------+--------------------------------------
 * std_id	| canID
 * p		| 位置
 * v		| 速度
 * kpf		| 位置比例系数
 * kdf		| 位置微分系数
 * tff		| 扭矩
*******************************************************************************/
void CanSend(uint32_t std_id, float p, float v, float kpf, float kdf, float tff, CAN_HandleTypeDef hcan)
{
  CAN_TxHeaderTypeDef TxHeader;
  uint8_t TxData[8];
  uint32_t TxMailbox;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.StdId = std_id;
  TxHeader.TransmitGlobalTime = DISABLE;
  TxHeader.DLC = 8;

  /// limit data to be within bounds///
  float p_des = fminf(fmaxf(P_MIN, p), P_MAX);
  float v_des = fminf(fmaxf(V_MIN, v), V_MAX);
  float kp = fminf(fmaxf(KP_MIN, kpf), KP_MAX);
  float kd = fminf(fmaxf(KD_MIN, kdf), KD_MAX);
  float t_ff = fminf(fmaxf(T_MIN, tff), T_MAX);
  /// convert floats to unsigned ints///
  uint16_t p_int = float_to_uint(p_des, P_MIN, P_MAX, 16);
  uint16_t v_int = float_to_uint(v_des, V_MIN, V_MAX, 12);
  uint16_t kp_int = float_to_uint(kp, KP_MIN, KP_MAX, 12);
  uint16_t kd_int = float_to_uint(kd, KD_MIN, KD_MAX, 12);
  uint16_t t_int = float_to_uint(t_ff, T_MIN, T_MAX, 12);
  /// pack ints into the can buffer///
  TxData[0] = p_int>>8;
  TxData[1] = p_int&0xFF;
  TxData[2] = v_int>>4;
  TxData[3] = ((v_int&0xF)<<4)|(kp_int>>8);
  TxData[4] = kp_int&0xFF;
  TxData[5] = kd_int>>4;
  TxData[6] = ((kd_int&0xF)<<4)|(t_int>>8);
  TxData[7] = t_int&0xff;

  if (HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox) != HAL_OK)
  {
    /* Transmission request Error */
	//HAL_CAN_AddTxMessage(&hcan1, &TxHeader, TxData, &TxMailbox);
      Error_Handler();
	  printf("[info]Error_CanSend\r\n");
  }
}


/********************************实现函数**************************************
*函数原型:	CanRead()
*功　　能:	读取CAN返回数据
*修改日期:	20230526
*******************************************************************************/
void CanRead(CAN_HandleTypeDef hcan)
{
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[6];

  /*RxHeader.RTR = CAN_RTR_DATA;
  RxHeader.IDE = CAN_ID_STD;
  RxHeader.StdId=std_id;
  RxHeader.DLC = 8;*/

  if(HAL_CAN_GetRxMessage(&hcan, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
	  DMA_usart2_printf("canreaderror\r\n");
  }

  uint16_t id = RxData[0];
  uint16_t p_int = (RxData[1]<<8)|RxData[2];
  uint16_t v_int = (RxData[3]<<4)|(RxData[4]>>4);
  uint16_t i_int = ((RxData[4]&0xF)<<8)|RxData[5];
  /// convert uints to floats ///
  float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
  float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
  float t = uint_to_float(i_int, -T_MAX, T_MAX, 12);

//  DMA_usart2_printf("%.2f\r\n",p);

  if(id == MIT_A.CanID || id == 0x01 || id == 01)
  {
	  MIT_A.PosOut = p;
	  MIT_A.VelOut = v;
	  MIT_A.TorOut = t;
  }
  else if(id == MIT_B.CanID || id == 0x02)
  {
	  if(p>-20){
		  MIT_B.PosOut = p;
		  MIT_B.VelOut = v;
		  MIT_B.TorOut = t;
	  }

  }
}

void CanRead2(CAN_HandleTypeDef hcan)
{
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[6];

  /*RxHeader.RTR = CAN_RTR_DATA;
  RxHeader.IDE = CAN_ID_STD;
  RxHeader.StdId=std_id;
  RxHeader.DLC = 8;*/

  if(HAL_CAN_GetRxMessage(&hcan2, CAN_RX_FIFO0, &RxHeader, RxData) != HAL_OK)
  {
	  DMA_usart2_printf("canreaderror\r\n");
  }

  uint16_t id = RxData[0];
  uint16_t p_int = (RxData[1]<<8)|RxData[2];
  uint16_t v_int = (RxData[3]<<4)|(RxData[4]>>4);
  uint16_t i_int = ((RxData[4]&0xF)<<8)|RxData[5];
  /// convert uints to floats ///
  float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
  float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
  float t = uint_to_float(i_int, -T_MAX, T_MAX, 12);

  if(id == MIT_A.CanID || id == 0x01)
  {
	  MIT_A.PosOut = p;
	  MIT_A.VelOut = v;
	  MIT_A.TorOut = t;
  }
  else if(id == MIT_B.CanID || id == 0x02)
  {
	  MIT_B.PosOut = p;
	  MIT_B.VelOut = v;
	  MIT_B.TorOut = t;
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	  CAN_RxHeaderTypeDef RxHeader;
	  uint8_t RxData[6];

	if(hcan == &hcan1)
	{
//		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&hcan_rx1,RxData);
		  if(HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&RxHeader,RxData) != HAL_OK)
		  {
			  DMA_usart2_printf("canreaderror\r\n");
		  }

//		  uint16_t id = RxData[0];
		  uint16_t p_int = (RxData[1]<<8)|RxData[2];
		  uint16_t v_int = (RxData[3]<<4)|(RxData[4]>>4);
		  uint16_t i_int = ((RxData[4]&0xF)<<8)|RxData[5];
		  /// convert uints to floats ///
		  float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
		  float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
		  float t = uint_to_float(i_int, -T_MAX, T_MAX, 12);

		  MIT_A.PosOut = p;
		  MIT_A.VelOut = v;
		  MIT_A.TorOut = t;
	}
	else if(hcan == &hcan2)
	{
//		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&hcan_rx1,RxData);
		  if(HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO0,&RxHeader,RxData) != HAL_OK)
		  {
			  DMA_usart2_printf("canreaderror\r\n");
		  }

//		  uint16_t id = RxData[0];
		  uint16_t p_int = (RxData[1]<<8)|RxData[2];
		  uint16_t v_int = (RxData[3]<<4)|(RxData[4]>>4);
		  uint16_t i_int = ((RxData[4]&0xF)<<8)|RxData[5];
		  /// convert uints to floats ///
		  float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
		  float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
		  float t = uint_to_float(i_int, -T_MAX, T_MAX, 12);

		  MIT_B.PosOut = p;
		  MIT_B.VelOut = v;
		  MIT_B.TorOut = t;
	}
}

//void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
//{
//	  CAN_RxHeaderTypeDef RxHeader;
//	  uint8_t RxData[6];
//
//	if(hcan == &hcan2)
//	{
////		HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0,&hcan_rx1,RxData);
//		  if(HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO1,&RxHeader,RxData) != HAL_OK)
//		  {
//			  DMA_usart2_printf("canreaderror\r\n");
//		  }
//
////		  uint16_t id = RxData[0];
//		  uint16_t p_int = (RxData[1]<<8)|RxData[2];
//		  uint16_t v_int = (RxData[3]<<4)|(RxData[4]>>4);
//		  uint16_t i_int = ((RxData[4]&0xF)<<8)|RxData[5];
//		  /// convert uints to floats ///
//		  float p = uint_to_float(p_int, P_MIN, P_MAX, 16);
//		  float v = uint_to_float(v_int, V_MIN, V_MAX, 12);
//		  float t = uint_to_float(i_int, -T_MAX, T_MAX, 12);
//
//		  MIT_B.PosOut = p;
//		  MIT_B.VelOut = v;
//		  MIT_B.TorOut = t;
//	}
//}
/* USER CODE END 1 */
