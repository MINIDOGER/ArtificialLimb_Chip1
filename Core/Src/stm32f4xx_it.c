/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "stdio.h"
#include "string.h"
#include <stdarg.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
uint8_t USART2_TX_BUF[MAX_TX_LEN];   // printf的发送缓冲
volatile uint8_t USART2_TX_FLAG = 0; // USART发送标志，启动发送时置1，加volatile防编译器优化

uint8_t u1rxbuf1[MAX_RX_LEN];         // 数据接收缓冲1
uint8_t u1rxbuf2[MAX_RX_LEN];         // 数据接收缓冲2
uint8_t WhichBufIsReady1 = 0;         // 双缓存指示器。
// 0:u6rxbuf1 被DMA占用接收,  u6rxbuf2 可以读取.
// 1:u6rxbuf2 被DMA占用接收,  u6rxbuf1 可以读取.

uint8_t *p_IsOK1 = u1rxbuf2;        // 指针——指向可以读取的那个缓冲
uint8_t *p_IsToReceive1 = u1rxbuf1; // 指针——指向被占用的那个缓冲

uint8_t u3rxbuf1[MAX_RX_LEN];         // 数据接收缓冲1
uint8_t u3rxbuf2[MAX_RX_LEN];         // 数据接收缓冲2
uint8_t WhichBufIsReady3 = 0;         // 双缓存指示器。
// 0:u6rxbuf1 被DMA占用接收,  u6rxbuf2 可以读取.
// 1:u6rxbuf2 被DMA占用接收,  u6rxbuf1 可以读取.

uint8_t *p_IsOK3 = u3rxbuf2;        // 指针——指向可以读取的那个缓冲
uint8_t *p_IsToReceive3 = u3rxbuf1; // 指针——指向被占用的那个缓冲

uint8_t u4rxbuf1[MAX_RX_LEN];         // 数据接收缓冲1
uint8_t u4rxbuf2[MAX_RX_LEN];         // 数据接收缓冲2
uint8_t WhichBufIsReady4 = 0;         // 双缓存指示器。
// 0:u6rxbuf1 被DMA占用接收,  u6rxbuf2 可以读取.
// 1:u6rxbuf2 被DMA占用接收,  u6rxbuf1 可以读取.

uint8_t *p_IsOK4 = u4rxbuf2;        // 指针——指向可以读取的那个缓冲
uint8_t *p_IsToReceive4 = u4rxbuf1; // 指针——指向被占用的那个缓冲

uint8_t u5rxbuf1[MAX_RX_LEN];         // 数据接收缓冲1
uint8_t u5rxbuf2[MAX_RX_LEN];         // 数据接收缓冲2
uint8_t WhichBufIsReady5 = 0;         // 双缓存指示器。
// 0:u6rxbuf1 被DMA占用接收,  u6rxbuf2 可以读取.
// 1:u6rxbuf2 被DMA占用接收,  u6rxbuf1 可以读取.

uint8_t *p_IsOK5 = u5rxbuf2;        // 指针——指向可以读取的那个缓冲
uint8_t *p_IsToReceive5 = u5rxbuf1; // 指针——指向被占用的那个缓冲

uint8_t u6rxbuf1[MAX_RX_LEN];         // 数据接收缓冲1
uint8_t u6rxbuf2[MAX_RX_LEN];         // 数据接收缓冲2
uint8_t WhichBufIsReady6 = 0;         // 双缓存指示器。
// 0:u6rxbuf1 被DMA占用接收,  u6rxbuf2 可以读取.
// 1:u6rxbuf2 被DMA占用接收,  u6rxbuf1 可以读取.

uint8_t *p_IsOK6 = u6rxbuf2;        // 指针——指向可以读取的那个缓冲
uint8_t *p_IsToReceive6 = u6rxbuf1; // 指针——指向被占用的那个缓冲
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
extern TIM_HandleTypeDef htim2;
extern DMA_HandleTypeDef hdma_uart4_rx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart6;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart5_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream1 global interrupt.
  */
void DMA1_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream1_IRQn 0 */

  /* USER CODE END DMA1_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart3_rx);
  /* USER CODE BEGIN DMA1_Stream1_IRQn 1 */

  /* USER CODE END DMA1_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream2 global interrupt.
  */
void DMA1_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream2_IRQn 0 */

  /* USER CODE END DMA1_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_rx);
  /* USER CODE BEGIN DMA1_Stream2_IRQn 1 */

  /* USER CODE END DMA1_Stream2_IRQn 1 */
}

/**
  * @brief This function handles DMA1 stream6 global interrupt.
  */
void DMA1_Stream6_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream6_IRQn 0 */
  if (__HAL_DMA_GET_FLAG(&hdma_usart2_tx, DMA_FLAG_TCIF2_6) != RESET) //数据发送完成中断
  {
	// __HAL_DMA_CLEAR_FLAG(&hdma_usart2_tx, DMA_FLAG_TCIF2_6);
	// 这一部分其实在 HAL_DMA_IRQHandler(&hdma_usart2_tx) 也完成了。

	__HAL_UART_CLEAR_IDLEFLAG(&huart2); //清除串口空闲中断标志位，发送完成那么串口也是空闲态哦~

	USART2_TX_FLAG = 0; // 重置发送标志位

	huart2.gState = HAL_UART_STATE_READY;
	hdma_usart2_tx.State = HAL_DMA_STATE_READY;
	__HAL_UNLOCK(&hdma_usart2_tx);
	// 这里疑似是HAL库函数的bug，具体可以参考我给的链接
	// huart1,hdma_usart1_tx 的状态要手动复位成READY状态
	// 不然发送函数会一直以为通道忙，就不再发送数据了！
  }
  /* USER CODE END DMA1_Stream6_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart2_tx);
  /* USER CODE BEGIN DMA1_Stream6_IRQn 1 */

  /* USER CODE END DMA1_Stream6_IRQn 1 */
}

/**
  * @brief This function handles CAN1 RX0 interrupt.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM2 global interrupt.
  */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */

  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
  * @brief This function handles USART1 global interrupt.
  */
void USART1_IRQHandler(void)
{
  /* USER CODE BEGIN USART1_IRQn 0 */
  if (RESET != __HAL_UART_GET_FLAG(&huart1, UART_FLAG_IDLE))
  {
	// __HAL_UART_CLEAR_IDLEFLAG(&huart6);
	// 这一部分其实在 HAL_UART_IRQHandler(&huart6) 也完成了。

	HAL_UART_DMAStop(&huart1); // 把DMA接收停掉，防止速度过快导致中断重入，数据被覆写。

//	uint32_t data_length = MAX_RX_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);// 数据总长度=极限接收长度-DMA剩余的接收长度

	if (WhichBufIsReady1)	//WhichBufIsReady=1
	{
	  p_IsOK1 = u1rxbuf2;        // u6rxbuf2 可以读取，就绪指针指向它。
	  p_IsToReceive1 = u1rxbuf1; // u6rxbuf1 作为下一次DMA存储的缓冲，占用指针指向它。
	  WhichBufIsReady1 = 0;		//切换一下指示器状态
	}
	else				//WhichBufIsReady=0
	{
	  p_IsOK1 = u1rxbuf1;        // u6rxbuf1 可以读取，就绪指针指向它。
	  p_IsToReceive1 = u1rxbuf2; // u6rxbuf2 作为下一次DMA存储的缓冲，占用指针指向它。
	  WhichBufIsReady1 = 1;		//切换一下指示器状态
	}

	memcpy(&DataLeftBuf.HexBufSum,p_IsOK6,48);
	LeftDataSumBufDMA(&DataLeftBuf);
//	DMA_usart2_printf("%d\r\n",data_length);


	///不管是复制也好，放进去队列也罢，处理你接收到的数据的代码建议从这里结束
	memset((uint8_t *)p_IsToReceive1, 0, MAX_RX_LEN);	// 把接收数据的指针指向的缓冲区清空
  }
  /* USER CODE END USART1_IRQn 0 */
  HAL_UART_IRQHandler(&huart1);
  /* USER CODE BEGIN USART1_IRQn 1 */
  HAL_UART_Receive_DMA(&huart1, p_IsToReceive1, MAX_RX_LEN); //数据处理完毕，重新启动接收
  /* USER CODE END USART1_IRQn 1 */
}

/**
  * @brief This function handles USART2 global interrupt.
  */
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */

  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */

  /* USER CODE END USART2_IRQn 1 */
}

/**
  * @brief This function handles USART3 global interrupt.
  */
void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */
  if (RESET != __HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE))
  {
	// __HAL_UART_CLEAR_IDLEFLAG(&huart6);
	// 这一部分其实在 HAL_UART_IRQHandler(&huart6) 也完成了。

	HAL_UART_DMAStop(&huart3); // 把DMA接收停掉，防止速度过快导致中断重入，数据被覆写。

//	uint32_t data_length = MAX_RX_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);// 数据总长度=极限接收长度-DMA剩余的接收长度

	if (WhichBufIsReady3)	//WhichBufIsReady=1
	{
	  p_IsOK3 = u3rxbuf2;        // u6rxbuf2 可以读取，就绪指针指向它。
	  p_IsToReceive3 = u3rxbuf1; // u6rxbuf1 作为下一次DMA存储的缓冲，占用指针指向它。
	  WhichBufIsReady3 = 0;		//切换一下指示器状态
	}
	else				//WhichBufIsReady=0
	{
	  p_IsOK3 = u3rxbuf1;        // u6rxbuf1 可以读取，就绪指针指向它。
	  p_IsToReceive3 = u3rxbuf2; // u6rxbuf2 作为下一次DMA存储的缓冲，占用指针指向它。
	  WhichBufIsReady3 = 1;		//切换一下指示器状态
	}

	memcpy(&Right.Hip.Buf.rxData,p_IsOK3,33);
	MPU6050ModDataBufDMA(&Right.Hip);
	Right.Hip.AngxCal = Right.Hip.Angx;


	///不管是复制也好，放进去队列也罢，处理你接收到的数据的代码建议从这里结束
	memset((uint8_t *)p_IsToReceive3, 0, MAX_RX_LEN);	// 把接收数据的指针指向的缓冲区清空
  }
  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
  HAL_UART_Receive_DMA(&huart3, p_IsToReceive3, MAX_RX_LEN); //数据处理完毕，重新启动接收
  /* USER CODE END USART3_IRQn 1 */
}

/**
  * @brief This function handles UART4 global interrupt.
  */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */
  if (RESET != __HAL_UART_GET_FLAG(&huart4, UART_FLAG_IDLE))
  {
	// __HAL_UART_CLEAR_IDLEFLAG(&huart6);
	// 这一部分其实在 HAL_UART_IRQHandler(&huart6) 也完成了。

	HAL_UART_DMAStop(&huart4); // 把DMA接收停掉，防止速度过快导致中断重入，数据被覆写。

//	uint32_t data_length = MAX_RX_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);// 数据总长度=极限接收长度-DMA剩余的接收长度

	if (WhichBufIsReady4)	//WhichBufIsReady=1
	{
	  p_IsOK4 = u4rxbuf2;        // u6rxbuf2 可以读取，就绪指针指向它。
	  p_IsToReceive4 = u4rxbuf1; // u6rxbuf1 作为下一次DMA存储的缓冲，占用指针指向它。
	  WhichBufIsReady4 = 0;		//切换一下指示器状态
	}
	else				//WhichBufIsReady=0
	{
	  p_IsOK4 = u4rxbuf1;        // u6rxbuf1 可以读取，就绪指针指向它。
	  p_IsToReceive4 = u4rxbuf2; // u6rxbuf2 作为下一次DMA存储的缓冲，占用指针指向它。
	  WhichBufIsReady4 = 1;		//切换一下指示器状态
	}

	memcpy(&Right.Knee.Buf.rxData,p_IsOK4,33);
	MPU6050ModDataBufDMA(&Right.Knee);
	Right.Knee.AngxCal = Right.Knee.Angx - Right.Hip.Angx;

	//拟合测试
//	if(Normal.State != 0){
//		switch(Normal.FitStart){
//		case 1:
//			DataDiv(&Normal);
//			break;
//		case 2:
//			DataDiv_2(&Normal);
//			break;
//		default:
//			break;
//		}
//	}



	///不管是复制也好，放进去队列也罢，处理你接收到的数据的代码建议从这里结束
	memset((uint8_t *)p_IsToReceive4, 0, MAX_RX_LEN);	// 把接收数据的指针指向的缓冲区清空
  }
  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */
  HAL_UART_Receive_DMA(&huart4, p_IsToReceive4, MAX_RX_LEN); //数据处理完毕，重新启动接收
  /* USER CODE END UART4_IRQn 1 */
}

/**
  * @brief This function handles UART5 global interrupt.
  */
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */
  if (RESET != __HAL_UART_GET_FLAG(&huart5, UART_FLAG_IDLE))
  {
	// __HAL_UART_CLEAR_IDLEFLAG(&huart6);
	// 这一部分其实在 HAL_UART_IRQHandler(&huart6) 也完成了。

	HAL_UART_DMAStop(&huart5); // 把DMA接收停掉，防止速度过快导致中断重入，数据被覆写。

//	uint32_t data_length = MAX_RX_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);// 数据总长度=极限接收长度-DMA剩余的接收长度

	if (WhichBufIsReady5)	//WhichBufIsReady=1
	{
	  p_IsOK5 = u5rxbuf2;        // u6rxbuf2 可以读取，就绪指针指向它。
	  p_IsToReceive5 = u5rxbuf1; // u6rxbuf1 作为下一次DMA存储的缓冲，占用指针指向它。
	  WhichBufIsReady5 = 0;		//切换一下指示器状态
	}
	else				//WhichBufIsReady=0
	{
	  p_IsOK5 = u5rxbuf1;        // u6rxbuf1 可以读取，就绪指针指向它。
	  p_IsToReceive5 = u5rxbuf2; // u6rxbuf2 作为下一次DMA存储的缓冲，占用指针指向它。
	  WhichBufIsReady5 = 1;		//切换一下指示器状态
	}

	memcpy(&Right.Ankle.Buf.rxData,p_IsOK5,33);
	MPU6050ModDataBufDMA(&Right.Ankle);
	Right.Ankle.AngxCal = - Right.Ankle.Angx - Right.Knee.Angx;


	///不管是复制也好，放进去队列也罢，处理你接收到的数据的代码建议从这里结束
	memset((uint8_t *)p_IsToReceive5, 0, MAX_RX_LEN);	// 把接收数据的指针指向的缓冲区清空
  }
  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */
  HAL_UART_Receive_DMA(&huart5, p_IsToReceive5, MAX_RX_LEN); //数据处理完毕，重新启动接收
  /* USER CODE END UART5_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream1 global interrupt.
  */
void DMA2_Stream1_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */

  /* USER CODE END DMA2_Stream1_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart6_rx);
  /* USER CODE BEGIN DMA2_Stream1_IRQn 1 */

  /* USER CODE END DMA2_Stream1_IRQn 1 */
}

/**
  * @brief This function handles DMA2 stream2 global interrupt.
  */
void DMA2_Stream2_IRQHandler(void)
{
  /* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

  /* USER CODE END DMA2_Stream2_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_usart1_rx);
  /* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

  /* USER CODE END DMA2_Stream2_IRQn 1 */
}

/**
  * @brief This function handles CAN2 RX0 interrupt.
  */
void CAN2_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN2_RX0_IRQn 0 */

  /* USER CODE END CAN2_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan2);
  /* USER CODE BEGIN CAN2_RX0_IRQn 1 */

  /* USER CODE END CAN2_RX0_IRQn 1 */
}

/**
  * @brief This function handles USART6 global interrupt.
  */
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */
  if (RESET != __HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE))
  {
	// __HAL_UART_CLEAR_IDLEFLAG(&huart6);
	// 这一部分其实在 HAL_UART_IRQHandler(&huart6) 也完成了。

	HAL_UART_DMAStop(&huart6); // 把DMA接收停掉，防止速度过快导致中断重入，数据被覆写。

//	uint32_t data_length = MAX_RX_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);// 数据总长度=极限接收长度-DMA剩余的接收长度

	if (WhichBufIsReady6)	//WhichBufIsReady=1
	{
	  p_IsOK6 = u6rxbuf2;        // u6rxbuf2 可以读取，就绪指针指向它。
	  p_IsToReceive6 = u6rxbuf1; // u6rxbuf1 作为下一次DMA存储的缓冲，占用指针指向它。
	  WhichBufIsReady6 = 0;		//切换一下指示器状态
	}
	else				//WhichBufIsReady=0
	{
	  p_IsOK6 = u6rxbuf1;        // u6rxbuf1 可以读取，就绪指针指向它。
	  p_IsToReceive6 = u6rxbuf2; // u6rxbuf2 作为下一次DMA存储的缓冲，占用指针指向它。
	  WhichBufIsReady6 = 1;		//切换一下指示器状态
	}

	memcpy(&DataRightBufFoot.HexBufSumFoot,p_IsOK6,34);
	FootDataBufDMA(&DataRightBufFoot);


	///不管是复制也好，放进去队列也罢，处理你接收到的数据的代码建议从这里结束
	memset((uint8_t *)p_IsToReceive6, 0, MAX_RX_LEN);	// 把接收数据的指针指向的缓冲区清空
  }
  /* USER CODE END USART6_IRQn 0 */
  HAL_UART_IRQHandler(&huart6);
  /* USER CODE BEGIN USART6_IRQn 1 */
  HAL_UART_Receive_DMA(&huart6, p_IsToReceive6, MAX_RX_LEN); //数据处理完毕，重新启动接收
  /* USER CODE END USART6_IRQn 1 */
}

/* USER CODE BEGIN 1 */
void DMA_USART2_Tx_Data(uint8_t *buffer, uint16_t size)
{
  USART2_TX_Wait();                             // 等待上一次发送完成（USART2_TX_FLAG为1即还在发送数据）
  USART2_TX_FLAG = 1;                           // USART2发送标志（启动发送）
  HAL_UART_Transmit_DMA(&huart2, buffer, size); // 发送指定长度的数据
}

void USART2_TX_Wait(void)
{
  uint16_t delay = 20000;
  while (USART2_TX_FLAG)
  {
    delay--;
    if (delay == 0)
      return;
  }
}

void DMA_usart2_printf(char *format, ...)
{
  //VA_LIST 是在C语言中解决变参问题的一组宏，
  //所在头文件：#include <stdarg.h>，用于获取不确定个数的参数。
  va_list arg_ptr;//实例化可变长参数列表

  USART2_TX_Wait(); //等待上一次发送完成（USART1_TX_FLAG为1即还在发送数据）

  va_start(arg_ptr, format);//初始化可变参数列表，设置format为可变长列表的起始点（第一个元素）

  // MAX_TX_LEN+1可接受的最大字符数(非字节数，UNICODE一个字符两个字节), 防止产生数组越界
  vsnprintf((char *)USART2_TX_BUF, MAX_TX_LEN + 1, format, arg_ptr);
  //从USART1_TX_BUF的首地址开始拼合，拼合format内容；MAX_TX_LEN+1限制长度，防止产生数组越界

  va_end(arg_ptr); //注意必须关闭

  DMA_USART2_Tx_Data(USART2_TX_BUF, strlen((const char *)USART2_TX_BUF));
  // 记得把buf里面的东西用HAL发出去
}

void MPU6050ModDataBufDMA(struct Data *AllData)
{
	uint8_t sumA,sumB,sumC = 0;
	for(int i=0;i<10;i++)
	{
		sumA = sumA + AllData->Buf.rxData[i];
		sumB = sumB + AllData->Buf.rxData[i+11];
		sumC = sumC + AllData->Buf.rxData[i+22];
	}
	if(sumA == AllData->Buf.rxData[10] && AllData->Buf.rxData[1] == 0x51)
	{
		AllData->Buf.Accx = (AllData->Buf.rxData[3]<<8)|AllData->Buf.rxData[2];
		AllData->Accx = (float) AllData->Buf.Accx/32768*16*g;
		AllData->Buf.Accy = (AllData->Buf.rxData[5]<<8)|AllData->Buf.rxData[4];
		AllData->Accy = (float) AllData->Buf.Accy/32768*16*g;
		AllData->Buf.Accz = (AllData->Buf.rxData[7]<<8)|AllData->Buf.rxData[6];
		AllData->Accz = (float) AllData->Buf.Accz/32768*16*g;
		if(AllData->Accx > 156.8)
		{
		  AllData->Accx = AllData->Accx - 313.6;
		}
	}
	if(sumB == AllData->Buf.rxData[21] && AllData->Buf.rxData[12] == 0x52)
	{
		AllData->Buf.AngAccx = (AllData->Buf.rxData[14]<<8)|AllData->Buf.rxData[13];
		AllData->AngAccx = (float) AllData->Buf.AngAccx/32768*2000;
		AllData->Buf.AngAccy = (AllData->Buf.rxData[16]<<8)|AllData->Buf.rxData[15];
		AllData->AngAccy = (float) AllData->Buf.AngAccy/32768*2000;
		AllData->Buf.AngAccz = (AllData->Buf.rxData[18]<<8)|AllData->Buf.rxData[17];
		AllData->AngAccz = (float) AllData->Buf.AngAccz/32768*2000;

		if(AllData->AngAccx > 2000)
		{
		  AllData->AngAccx = AllData->AngAccx - 4000;
		}
	}
	if(sumC == AllData->Buf.rxData[32] && AllData->Buf.rxData[23] == 0x53)
	{
		AllData->Buf.Angx = (AllData->Buf.rxData[25]<<8)|AllData->Buf.rxData[24];
		AllData->Angx = (float) AllData->Buf.Angx/32768*180;
		AllData->Buf.Angy = (AllData->Buf.rxData[27]<<8)|AllData->Buf.rxData[26];
		AllData->Angy = (float) AllData->Buf.Angy/32768*180;
		AllData->Buf.Angz = (AllData->Buf.rxData[29]<<8)|AllData->Buf.rxData[28];
		AllData->Angz = (float) AllData->Buf.Angz/32768*180;

		AllData->Angx = AllData->Angx - AllData->AngxZero;
		if(AllData->Angx > 180)
		{
		  AllData->Angx = AllData->Angx - 360;
		}
	}

}

void FootDataBufDMA(struct DataUnionBuf *AllData)
{
	if(AllData->HexBufSumFoot[0] == 0x64 && AllData->HexBufSumFoot[33] == 0x65)
	{
		for(int i=0;i<8;i++)
		{
			for(int j=0;j<4;j++)
			{
				AllData->DataUnionBuf.HexBuf[j] = AllData->HexBufSumFoot[4*i+j+1];
			}
			if(150 < AllData->DataUnionBuf.FloatBuf && AllData->DataUnionBuf.FloatBuf < 3000)
			{
				AllData->Data[AllData->Point[i]] = AllData->DataUnionBuf.FloatBuf - AllData->DataZero[AllData->Point[i]];
			}
		}
	}
}

void LeftDataSumBufDMA(struct DataUnionBuf *AllData)
{
	if(AllData->HexBufSum[0] == 0x62 && AllData->HexBufSum[13] == 0x63)
	{
		for(int i=0;i<3;i++)
		{
			for(int j=0;j<4;j++)
			{
				AllData->DataUnionBuf.HexBuf[j] = AllData->HexBufSum[4*i+j+1];
			}
			switch(i)
			{
			case 0:
				Left.Hip.AngxCal = AllData->DataUnionBuf.FloatBuf;
				break;
			case 1:
				Left.Knee.AngxCal = AllData->DataUnionBuf.FloatBuf;
				break;
			case 2:
				Left.Ankle.AngxCal = AllData->DataUnionBuf.FloatBuf;
				break;
			default:
				break;
			}
		}
	}
	if(AllData->HexBufSum[14] == 0x64 && AllData->HexBufSum[47] == 0x65)
	{
		for(int i=0;i<8;i++)
		{
			for(int j=0;j<4;j++)
			{
				AllData->DataUnionBuf.HexBuf[j] = AllData->HexBufSumFoot[4*i+j+15];
			}
			if(150<AllData->DataUnionBuf.FloatBuf && AllData->DataUnionBuf.FloatBuf<4096)
			{
				AllData->Data[AllData->Point[i]] = AllData->DataUnionBuf.FloatBuf;
			}

		}
	}

}
/* USER CODE END 1 */
