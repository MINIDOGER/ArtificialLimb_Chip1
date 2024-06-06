/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    tim.c
  * @brief   This file provides code for the configuration
  *          of the TIM instances.
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
#include "tim.h"

/* USER CODE BEGIN 0 */
#include "can.h"
#include "stm32f4xx_it.h"

int Tim2Timing = 13;
int Tim2Now = 0;
int SendChange = 0;
int SendSize = 0;
int SendCurrent = 0;

/* USER CODE END 0 */

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim7;

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 89;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}
/* TIM7 init function */
void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 89;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 65535;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspInit 0 */

  /* USER CODE END TIM2_MspInit 0 */
    /* TIM2 clock enable */
    __HAL_RCC_TIM2_CLK_ENABLE();

    /* TIM2 interrupt Init */
    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspInit 1 */

  /* USER CODE END TIM2_MspInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM7)
  {
  /* USER CODE BEGIN TIM7_MspInit 0 */

  /* USER CODE END TIM7_MspInit 0 */
    /* TIM7 clock enable */
    __HAL_RCC_TIM7_CLK_ENABLE();
  /* USER CODE BEGIN TIM7_MspInit 1 */

  /* USER CODE END TIM7_MspInit 1 */
  }
}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

  if(tim_baseHandle->Instance==TIM2)
  {
  /* USER CODE BEGIN TIM2_MspDeInit 0 */

  /* USER CODE END TIM2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM2_CLK_DISABLE();

    /* TIM2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(TIM2_IRQn);
  /* USER CODE BEGIN TIM2_MspDeInit 1 */

  /* USER CODE END TIM2_MspDeInit 1 */
  }
  else if(tim_baseHandle->Instance==TIM7)
  {
  /* USER CODE BEGIN TIM7_MspDeInit 0 */

  /* USER CODE END TIM7_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_TIM7_CLK_DISABLE();
  /* USER CODE BEGIN TIM7_MspDeInit 1 */

  /* USER CODE END TIM7_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/********************************实现函数**************************************
*函数原型:	HAL_Delay_us()
*功　　能:	微妙延迟
*修改日期:	20230712
*******************************************************************************/
void HAL_Delay_us(uint16_t us){
	__HAL_TIM_SetCounter(&htim7, 0);
	__HAL_TIM_ENABLE(&htim7);	//HAL_TIM_Base_Start(&htim7)
	while (__HAL_TIM_GetCounter(&htim7) < us){

	}
	__HAL_TIM_DISABLE(&htim7);	//HAL_TIM_Base_Stop(&htim7)
}

/********************************实现函数**************************************
*函数原型:	HAL_TIM_PeriodElapsedCallback()
*功　　能:	定时器回调函数，03支撑，12摆动
*修改日期:	2023？
*******************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim == &htim2){
		if(Tim2Now >= Tim2Timing){
			//既定轨迹测试
			if(timCounter < 152){
				timCounter++;
			}
			else{
				timCounter = 0;
				//固定轨迹越障
//				if(disf1 == 0 && disf < 4){
//					disf++;
//				}
//				else if(disf1 == 1 && disf < 3){
//					disf++;
//				}
			}

			//定时发送数据
//			if(Normal.Flag_Fit > 0){
//				if(SendChange == 1){
//
//					if(MIT_A.Pos < 10 && MIT_A.Pos > -80){
//						MIT_A.Pos = MIT_A.Pos/180.0f*PI;
//						MIT_A.Pos = low_pass_filter(MIT_A.Pos, 0);
//						MIT_A.Tor = 0;
//						CanSend(MIT_A.CanID,MIT_A.Pos,MIT_A.Vel,MIT_A.Kp,MIT_A.Kd,MIT_A.Tor,hcan1);
//						CanRead(hcan1);
//						DMA_usart2_printf("%.3f,%.3f,%.3f,%.3f\r\n",
//								MIT_A.Pos, MIT_A.PosOut, MIT_A.Pos-MIT_A.PosOut,MIT_A.Tor);
//					}
//
//					switch(Normal.Flag_Send){
//					case 0:
//						MIT_A.Pos = Normal.FitKnee_0[SendCurrent];
//						MIT_A.Tor = Normal.FitKneeT_0[SendCurrent];
//						ModbusRead();
////						DMA_usart2_printf("%.2f,0,%d,%d\r\n",Normal.FitKnee_0[SendCurrent],SendCurrent,SendSize);
//						break;
//					case 1:
//						MIT_A.Pos = Normal.FitKnee_1[SendCurrent];
//						MIT_A.Tor = Normal.FitKneeT_1[SendCurrent];
////						DMA_usart2_printf("%.2f,1,%d,%d\r\n",Normal.FitKnee_1[SendCurrent],SendCurrent,SendSize);
//						break;
//					case 2:
//						MIT_A.Pos = Normal.FitKnee_2[SendCurrent];
//						MIT_A.Tor = Normal.FitKneeT_2[SendCurrent];
////						DMA_usart2_printf("%.2f,2,%d,%d\r\n",Normal.FitKnee_2[SendCurrent],SendCurrent,SendSize);
//						break;
//					case 3:
//						MIT_A.Pos = Normal.FitKnee_3[SendCurrent];
//						MIT_A.Tor = Normal.FitKneeT_3[SendCurrent];
//						ModbusRead();
////						DMA_usart2_printf("%.2f,3,%d,%d\r\n",Normal.FitKnee_3[SendCurrent],SendCurrent,SendSize);
//						break;
//					default:
//						MIT_A.Pos = Normal.FitKnee_1[SendCurrent];
//						MIT_A.Tor = Normal.FitKneeT_1[SendCurrent];
//						break;
//					}
//
//					SendCurrent++;
//					if(SendCurrent >= SendSize){
//						SendChange = 0;
//						SendCurrent = 0;
//						Normal.Flag_Send += 1;
//						if(Normal.Flag_Send >= 4){
//							Normal.Flag_Send = 0;
//						}
//						Normal.Flag_Fit -= 1;
//					}
//				}
//				else if(SendChange == 0){
//					switch(Normal.Flag_Send){
//					case 0:
//						SendSize = Normal.FitKnee_0_num - 4;
//						MIT_A.Pos = Normal.FitKnee_0[SendCurrent];
//						MIT_A.Tor = Normal.FitKneeT_0[SendCurrent];
////						DMA_usart2_printf("%.2f,0,%d,%d\r\n",Normal.FitKnee_0[SendCurrent],SendCurrent,SendSize);
//						break;
//					case 1:
//						SendSize = Normal.FitKnee_1_num - 4;
//						MIT_A.Pos = Normal.FitKnee_1[SendCurrent];
//						MIT_A.Tor = Normal.FitKneeT_1[SendCurrent];
////						DMA_usart2_printf("%.2f,1,%d,%d\r\n",Normal.FitKnee_1[SendCurrent],SendCurrent,SendSize);
//						break;
//					case 2:
//						SendSize = Normal.FitKnee_2_num - 4;
//						MIT_A.Pos = Normal.FitKnee_2[SendCurrent];
//						MIT_A.Tor = Normal.FitKneeT_2[SendCurrent];
////						DMA_usart2_printf("%.2f,2,%d,%d\r\n",Normal.FitKnee_2[SendCurrent],SendCurrent,SendSize);
//						break;
//					case 3:
//						SendSize = Normal.FitKnee_3_num - 4;
//						MIT_A.Pos = Normal.FitKnee_3[SendCurrent];
//						MIT_A.Tor = Normal.FitKneeT_3[SendCurrent];
////						DMA_usart2_printf("%.2f,3,%d,%d\r\n",Normal.FitKnee_3[SendCurrent],SendCurrent,SendSize);
//						break;
//					default:
//						break;
//					}
//
//					SendChange = 1;
//					SendCurrent++;
//				}
//			}

			Tim2Now = 0;
		}
		else{
			Tim2Now++;
		}
	}
}

/********************************实现函数**************************************
*函数原型:	void TIM2_Zero()
*功　　能:	计时器参数归零
*修改日期:	20230114
*******************************************************************************/
void TIM2_Zero(){
	Tim2Timing = 10;
	Tim2Now = 0;
	SendChange = 0;
	SendSize = 0;
	SendCurrent = 0;
}
/* USER CODE END 1 */
