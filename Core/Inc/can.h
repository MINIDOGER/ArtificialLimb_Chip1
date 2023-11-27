/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.h
  * @brief   This file contains all the function prototypes for
  *          the can.c file
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
#ifndef __CAN_H__
#define __CAN_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/********************************常量定义**************************************/
#define P_MIN   -25.5f
#define P_MAX   25.5f
#define V_MIN   -45.0f
#define V_MAX   45.0f
#define KP_MIN  0.0f
#define KP_MAX  500.0f
#define KD_MIN  0.0f
#define KD_MAX  5.0f
#define T_MIN   -18.0f
#define T_MAX   18.0f

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

extern CAN_HandleTypeDef hcan2;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void MX_CAN1_Init(void);
void MX_CAN2_Init(void);

/* USER CODE BEGIN Prototypes */

/********************************函数声明**************************************/
HAL_StatusTypeDef CAN1_SetFilters();
HAL_StatusTypeDef CAN2_SetFilters();
int float_to_uint(float x, float x_min, float x_max, int bits);
float uint_to_float(int x_int, float x_min, float x_max, int bits);
void EnterMotorMode(uint32_t std_id, CAN_HandleTypeDef hcan);
void ExitMotorMode(uint32_t std_id, CAN_HandleTypeDef hcan);
void CanSend(uint32_t std_id, float p, float v, float kpf, float kdf, float tff, CAN_HandleTypeDef hcan);
void CanRead(CAN_HandleTypeDef hcan);
void CanRead2(CAN_HandleTypeDef hcan);
/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __CAN_H__ */

