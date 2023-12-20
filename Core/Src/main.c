/* USER CODE BEGIN Header */

/********************************修改说明**************************************
*修改日期:20230526	编写基本(详解见初代码)功能，增加MPU6050封装函数
*修改日期:20230527	补充部分注释，编写基本框架(详解见初代码)
*修改日期:20230530	修复框架BUG
*修改日期:20230621	编写接收副C数据与发送副C指令
*修改日期:20230709	增加CAN1进退电机模式测试指令
*修改日期:20230712	增加MPU6050初始值偏移,增加MIT标志，增加延迟指令
*修改日期:20230721	增鞋垫压力数据接收
*修改日期:20230726	原串口中断和数据输出修改DMA版本，以减小芯片负担，原程序代码也进行了保留
*修改日期:20230730	增加鞋垫数据范围限制
*修改日期:20230731	激光距离Modbus读取距离指令50 03 00 34 00 01 c8 45
*修改日期:20230807	增加鞋垫数据归零指令
*修改日期:20230810	增加基本角度数据拟合函数，未测试待调整
*修改日期:20230818	基本角度数据拟合函数测试无法使用，无输出（待解决-已解决20231202）
*修改日期:20231015	增加距离传感器读取函数与逻辑的判断发送（未测试-已测20231016）
*修改日期:20231016	距离传感器测试，修改部分错误代码，存在问题：暂时解码错误（待解决）
*修改日期:20231017	假肢can通信测试，输出数据
*修改日期:20231129	增加低通滤波器函数，未测试
*修改日期:20231201	修改拟合函数，未测试； 修改数据拟合分割时的判断逻辑
*修改日期:20231202	滤波函数效果暂时还是不理想，拟合函数测试卡死，在S=2测试模式下现已发现可能原因，下一步尝试修复
*修改日期:20231202	修改拟合函数，解决nan和卡死问题，下一步需要验证拟合的正确性。卡死：动态分配内存存在问题；nan：输入数据 个数为0
*修改日期:20231204	修正动态内存分配错误
*修改日期:20231213	成功拟合，数据分割不是很合理，尝试解决，未测试
*******************************************************************************/

/*
//                            _ooOoo_
//                           o8888888o
//                           88" . "88
//                           (| -_- |)
//                            O\ = /O
//                        ____/`---'\____
//                      .   ' \\| |// `.
//                       / \\||| : |||// \
//                     / _||||| -:- |||||- \
//                       | | \\\ - /// | |
//                     | \_| ''\---/'' | |
//                      \ .-\__ `-` ___/-. /
//                   ___`. .' /--.--\ `. . __
//                ."" '< `.___\_<|>_/___.' >'"".
//               | | : `- \`.;`\ _ /`;.`/ - ` : | |
//                 \ \ `-. \_ __\ /__ _/ .-` / /
//         ======`-.____`-.___\_____/___.-`____.-'======
//                            `=---='
//
//         .............................................
//                  佛祖保佑   芯片不烧   永无BUG
//          佛曰:
//                  写字楼里写字间，写字间里程序员；
//                  程序人员写程序，又拿程序换酒钱。
//                  酒醒只在网上坐，酒醉还来网下眠；
//                  酒醉酒醒日复日，网上网下年复年。
//                  但愿老死电脑间，不愿鞠躬老板前；
//                  奔驰宝马贵者趣，公交自行程序员。
//                  别人笑我忒疯癫，我笑自己命太贱；
//                  不见满街漂亮妹，哪个归得程序员？
*/

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include "Kinco_can.h"
#include "user_config.h"
#include "math.h"
#include "stm32f4xx_it.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/********************************忽略特定警告**************************************/
#pragma GCC diagnostic ignored "-Wunknown-pragmas"
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/********************************参数声明**************************************
*参数用途:	MPU6050关节角度等信息
*修改日期:	20230620
*******************************************************************************/
struct joint Right; //右关节数据组
struct joint Left; //左关节数据组
//struct DataUnionBuf DataLeftBuf; //左关节数据组共同体缓存,包含在足底压力缓存中
struct DataUnionBuf DataRightBufAng;

/********************************参数声明**************************************
*参数用途:	CAN消息
*修改日期:	20230526
*******************************************************************************/
struct DataCan MIT_A = {
		.CanID = 0x01,
};
struct DataCan MIT_B = {
		.CanID = 0x02,
};

CANTxMessage can_tx;
CANRxMessage can_rx;
Kinco_servo Exoskeleton1;
/********************************参数声明**************************************
*参数用途:	足底压力数据 Ps:16点索引为0
*修改日期:	20230721
*******************************************************************************/
struct DataUnionBuf DataRightBufFoot = {
		.Point = {3,4,7,8,9,12,13,14},
};
struct DataUnionBuf DataLeftBuf = {
		.Point = {3,4,7,8,9,12,13,14},
};

/********************************参数声明**************************************
*参数用途:	中断数据缓存
*修改日期:	20230527
*******************************************************************************/
struct cRxBuf{
	uint8_t cRx_1;
	uint8_t cRx_2;
	uint8_t cRx_3;
	uint8_t cRx_4;
	uint8_t cRx_5;
	uint8_t cRx_6;
	uint8_t rxBuf_2[256];
	uint32_t rxBufCursor_2;
};
struct cRxBuf cRx;

/********************************参数声明**************************************
*参数用途:	标志位
*修改日期:	20230712
*******************************************************************************/
int Mode = 0;
int ModeFlag = 0;
int PrintfFlag = 0;
int MITFlag = 0;
int DelayFlag = 0;
int Delay = 9;
int DelayUs = 50;


/********************************参数声明**************************************
*参数用途:	计时
*修改日期:	20230918
*******************************************************************************/
int timCounter = 0;

/********************************参数声明**************************************
*参数用途:	滤波器参数
*修改日期:	20231129
*******************************************************************************/
float fc = 5.0f;     		//截止频率
float Ts = 0.01f;    		//采样周期
static float pi = 3.14159f; //π
float alpha = 0;    		//滤波系数

/********************************参数声明**************************************
*参数用途:	用于测试的数据
*修改日期:	20230531
*******************************************************************************/
//double xg_2_2[] = {0.15,0.16,0.168,0.176,0.184,0.192,0.2,0.208,0.216,0.224,0.232,0.24,0.248,0.256,0.264,0.272,0.28,0.288,0.296,0.304,0.312,0.32,0.328,0.336,0.344,0.352,0.36,0.368,0.376,0.384,0.392,0.4};
//double yg_2_2[] = {26.09,25.877,25.18,24.41,23.42,22.27,21.12,20,18.9,17.78,16.64,15.52,14.43,13.44,12.55,11.82,11.24,10.89,10.76,10.78,10.8,10.84,10.97,11.06,11.02,10.82,10.52,10.18,9.84,9.55,9.35,9.29};

//uint8_t MPUData[] = {0x55,0x51,0x14,0x00,0x05,0x03,0x84,0x07,0x78,0xF9,0xBE,0x55,0x52,0x01,0x00,0xFF,0xFF,0x00,0x00,0x78,0xF9,0x17,0x55,0x53,0x88,0x0F,0x90,0xFF,0x2C,0x00,0x78,0xF9,0x6B};

/********************************参数声明**************************************
*参数用途:	参考数据
*修改日期:	20231017
*******************************************************************************/
float KneeReference[] = {
		-0.202,-0.202,-0.202,-0.202,-0.203,-0.204,-0.205,-0.206,-0.208,-0.210,
		-0.211,-0.212,-0.213,-0.214,-0.215,-0.216,-0.217,-0.219,-0.220,-0.221,
		-0.224,-0.226,-0.229,-0.233,-0.237,-0.242,-0.247,-0.253,-0.259,-0.266,
		-0.274,-0.282,-0.292,-0.302,-0.313,-0.325,-0.339,-0.355,-0.372,-0.391,
		-0.410,-0.432,-0.456,-0.481,-0.508,-0.538,-0.568,-0.600,-0.633,-0.666,
		-0.701,-0.734,-0.766,-0.798,-0.828,-0.855,-0.878,-0.899,-0.915,-0.929,
		-0.940,-0.946,-0.950,-0.950,-0.948,-0.944,-0.937,-0.928,-0.915,-0.900,
		-0.883,-0.865,-0.845,-0.821,-0.797,-0.771,-0.744,-0.716,-0.689,-0.662,
		-0.635,-0.609,-0.585,-0.562,-0.542,-0.522,-0.505,-0.490,-0.476,-0.465,
		-0.456,-0.448,-0.442,-0.436,-0.431,-0.428,-0.428,-0.428,-0.430,-0.435,
		-0.441,-0.447,-0.454,-0.460,-0.464,-0.467,-0.469,-0.470,-0.469,-0.467,
		-0.464,-0.460,-0.455,-0.449,-0.443,-0.437,-0.430,-0.422,-0.415,-0.407,
		-0.399,-0.391,-0.383,-0.375,-0.366,-0.359,-0.351,-0.343,-0.336,-0.328,
		-0.321,-0.313,-0.306,-0.299,-0.291,-0.284,-0.277,-0.270,-0.263,-0.256,
		-0.249,-0.242,-0.236,-0.230,-0.225,-0.220,-0.215,-0.211,-0.207,-0.203,
		-0.200,-0.198,-0.196};
float AnkleReference[] = {
		0.275, 0.279, 0.283, 0.288, 0.293, 0.298, 0.303, 0.309, 0.314,0.319,
		0.325, 0.330, 0.335, 0.339, 0.344, 0.348, 0.351, 0.354, 0.357,0.360,
		0.362, 0.364, 0.364, 0.365, 0.366, 0.365, 0.364, 0.362, 0.357,0.353,
		0.348, 0.341, 0.333, 0.323, 0.313, 0.302, 0.291, 0.279, 0.266,0.251,
		0.233, 0.215, 0.195, 0.173, 0.150, 0.125, 0.100, 0.074, 0.047,0.022,
		0.001,-0.018,-0.031,-0.041,-0.045,-0.044,-0.039,-0.029,-0.016,0.000,
		0.018, 0.037, 0.055, 0.072, 0.090, 0.106, 0.120, 0.134, 0.145,0.156,
		0.165, 0.173, 0.178, 0.183, 0.186, 0.188, 0.189, 0.188, 0.185,0.181,
		0.175, 0.167, 0.159, 0.149, 0.139, 0.130, 0.120, 0.112, 0.105,0.099,
		0.097, 0.095, 0.095, 0.094, 0.097, 0.100, 0.103, 0.106, 0.109,0.112,
		0.115, 0.118, 0.121, 0.123, 0.126, 0.129, 0.132, 0.135, 0.138,0.141,
		0.144, 0.147, 0.150, 0.153, 0.156, 0.159, 0.162, 0.165, 0.168,0.171,
		0.174, 0.176, 0.179, 0.182, 0.185, 0.188, 0.191, 0.194, 0.197,0.200,
		0.203, 0.206, 0.209, 0.212, 0.215, 0.219, 0.222, 0.224, 0.226,0.229,
		0.231, 0.233, 0.236, 0.238, 0.241, 0.243, 0.246, 0.249, 0.251,0.254,
		0.258, 0.261, 0.264};
float KneeSpeedReference[] = {
		 0.059 , 0.006 ,-0.042 ,-0.082 ,-0.112 ,-0.133 ,-0.143 ,-0.145 ,-0.141 ,-0.132,
		-0.121 ,-0.110 ,-0.101 ,-0.097 ,-0.098 ,-0.105 ,-0.116 ,-0.143 ,-0.172 ,-0.212,
		-0.259 ,-0.301 ,-0.355 ,-0.410 ,-0.470 ,-0.527 ,-0.577 ,-0.636 ,-0.696 ,-0.763,
		-0.844 ,-0.926 ,-1.027 ,-1.136 ,-1.253 ,-1.389 ,-1.526 ,-1.680 ,-1.843 ,-2.013,
		-2.191 ,-2.374 ,-2.557 ,-2.727 ,-2.896 ,-3.058 ,-3.192 ,-3.304 ,-3.368 ,-3.384,
		-3.357 ,-3.271 ,-3.127 ,-2.917 ,-2.669 ,-2.375 ,-2.056 ,-1.724 ,-1.370 ,-1.023,
		-0.693 ,-0.368 ,-0.073 , 0.208 , 0.474 , 0.720 , 0.970 , 1.208 , 1.448 , 1.672,
		 1.892 , 2.098 , 2.285 , 2.458 , 2.589 , 2.682 , 2.738 , 2.745 , 2.723 , 2.674,
		 2.579 , 2.435 , 2.267 , 2.089 , 1.893 , 1.699 , 1.503 , 1.317 , 1.152 , 0.999,
		 0.850 , 0.704 , 0.549 , 0.391 , 0.222 , 0.049 ,-0.117 ,-0.277 ,-0.406 ,-0.515,
		-0.578 ,-0.597 ,-0.566 ,-0.486 ,-0.372 ,-0.231 ,-0.080 , 0.068 , 0.204 , 0.325,
		 0.422 , 0.493 , 0.557 , 0.613 , 0.661 , 0.701 , 0.735 , 0.760 ,  0.78 , 0.793,
		 0.806 , 0.808 , 0.810 , 0.807 , 0.799 , 0.790 , 0.776 , 0.762 , 0.751 , 0.74,
		 0.734 , 0.732 , 0.732 , 0.733 , 0.730 , 0.724 , 0.716 , 0.704 , 0.690 , 0.671,
		 0.648 , 0.620 , 0.589 , 0.553 , 0.514 , 0.473 , 0.429 , 0.385 , 0.341 , 0.298,
		 0.259 , 0.223 , 0.194};
float AnkleSpeedReference[] = {
		 0.431 , 0.441 , 0.462 , 0.488 , 0.512 , 0.531 , 0.542 , 0.545 , 0.539 , 0.526,
		 0.507 , 0.483 , 0.455 , 0.424 , 0.392 , 0.360 , 0.327 , 0.294 , 0.254 , 0.214,
		 0.170 , 0.127 , 0.080 , 0.013 ,-0.060 ,-0.154 ,-0.258 ,-0.365 ,-0.475 ,-0.581,
		-0.678 ,-0.778 ,-0.873 ,-0.969 ,-1.067 ,-1.159 ,-1.258 ,-1.370 ,-1.508 ,-1.659,
		-1.830 ,-2.020 ,-2.197 ,-2.380 ,-2.519 ,-2.603 ,-2.619 ,-2.545 ,-2.392 ,-2.141,
		-1.807 ,-1.399 ,-0.929 ,-0.435 , 0.070 , 0.540 , 0.965 , 1.309 , 1.563 , 1.737,
		 1.830 , 1.852 , 1.825 , 1.740 , 1.615 , 1.475 , 1.317 , 1.177 , 1.025 , 0.883,
		 0.758 , 0.621 , 0.488 , 0.343 , 0.190 , 0.041 ,-0.119 ,-0.280 ,-0.451 ,-0.604,
		-0.749 ,-0.863 ,-0.933 ,-0.975 ,-0.963 ,-0.906 ,-0.805 ,-0.676 ,-0.519 ,-0.362,
		-0.212 ,-0.065 , 0.060 , 0.171 , 0.269 , 0.347 , 0.412 , 0.483 , 0.530 , 0.579,
		 0.626 , 0.653 , 0.663 , 0.649 , 0.601 , 0.535 , 0.448 , 0.345 , 0.241 , 0.145,
		 0.063 , 0.010 ,-0.012 ,-0.011 , 0.019 , 0.064 , 0.116 , 0.170 , 0.230 , 0.272,
		 0.301 , 0.321 , 0.319 , 0.311 , 0.297 , 0.277 , 0.257 , 0.242 , 0.228 , 0.219,
		 0.216 , 0.215 , 0.213 , 0.215 , 0.217 , 0.219 , 0.223 , 0.228 , 0.233 , 0.237,
		 0.240 , 0.244 , 0.248 , 0.252 , 0.257 , 0.264 , 0.272 , 0.283 , 0.296 , 0.312,
		 0.330 , 0.349 , 0.369};
float KneeKPReference[] = {
		198, 200, 199, 198, 197, 196, 196, 196, 196, 196,
		197, 197, 197, 197, 197, 197, 197, 196, 195, 194,
		193, 191, 190, 188, 187, 185, 184, 182, 180, 178,
		176, 174, 171, 168, 164, 160, 156, 152, 147, 142,
		137, 132, 127, 122, 117, 113, 109, 106, 104, 103,
		104, 107, 111, 117, 124, 132, 141, 151, 161, 171,
		180, 189, 198, 194, 186, 179, 172, 165, 159, 152,
		146, 140, 135, 130, 126, 123, 122, 122, 122, 124,
		126, 130, 135, 140, 146, 151, 157, 162, 167, 171,
		176, 180, 184, 189, 194, 199, 197, 192, 188, 185,
		183, 183, 184, 186, 189, 193, 198, 198, 194, 191,
		188, 186, 184, 182, 181, 180, 179, 178, 178, 177,
		177, 177, 177, 177, 177, 177, 178, 178, 179, 179,
		179, 179, 179, 179, 179, 179, 180, 180, 180, 181,
		181, 182, 183, 184, 185, 186, 188, 189, 190, 191,
		193, 194, 194};
float AnkleKPReference[] = {
		275, 275, 274, 272, 271, 270, 269, 269, 269, 270,
		271, 272, 274, 276, 278, 279, 281, 283, 286, 288,
		290, 293, 295, 299, 297, 291, 285, 279, 273, 267,
		261, 256, 250, 245, 239, 234, 228, 222, 214, 205,
		195, 185, 174, 164, 156, 151, 150, 155, 163, 178,
		197, 220, 247, 275, 296, 269, 245, 225, 211, 201,
		195, 194, 196, 201, 208, 216, 225, 233, 241, 250,
		257, 265, 272, 280, 289, 298, 293, 284, 274, 265,
		257, 251, 247, 244, 245, 248, 254, 261, 270, 279,
		288, 296, 297, 290, 285, 280, 276, 272, 270, 267,
		264, 263, 262, 263, 266, 269, 274, 280, 286, 292,
		296, 299, 299, 299, 299, 296, 293, 290, 287, 284,
		283, 282, 282, 282, 283, 284, 285, 286, 287, 287,
		288, 288, 288, 288, 288, 288, 287, 287, 287, 286,
		286, 286, 286, 286, 285, 285, 284, 284, 283, 282,
		281, 280, 279};
float KneeFr[] = {
		 0.359, 0.035,-0.263,-0.512,-0.702,-0.828,-0.894,-0.908,-0.880,
		-0.825,-0.757,-0.688,-0.634,-0.605,-0.610,-0.658,-0.725,-0.892,
		-1.073,-1.319,-1.610,-1.873,-2.210,-2.549,-2.920,-3.276,-3.586,
		-3.952,-4.322,-4.739,-5.244,-5.751,-6.376,-7.053,-7.779,-8.621,
		-9.472,-10.43,-11.438,-12.494,-13.601,-14.735,-15.866,-16.924,-17.973,
		-18.979,-19.81,-20.503,-20.90,-21.001,-20.830,-20.300,-19.404,-18.103,
		-16.561,-14.742,-12.759,-10.699,-8.507,-6.353,-4.304,-2.290,-0.460,
		 1.285, 2.933, 4.461, 6.010, 7.490, 8.976,10.370,11.734,13.012,
		14.169,15.246,16.059,16.631,16.980,17.023,16.889,16.583,15.992,
		15.099,14.059,12.954,11.741,10.536, 9.322, 8.165, 7.142, 6.192,
		 5.267, 4.360, 3.404, 2.421, 1.374, 0.300,-0.733,-1.721,-2.525,
		-3.198,-3.590,-3.707,-3.519,-3.018,-2.314,-1.437,-0.500, 0.418,
		 1.258, 2.008, 2.613, 3.053, 3.448, 3.800, 4.096, 4.346, 4.556,
		 4.711, 4.832, 4.917, 4.993, 5.009, 5.021, 5.004, 4.953, 4.894,
		 4.809, 4.721, 4.652, 4.585, 4.551, 4.534, 4.535, 4.539, 4.521,
		 4.487, 4.434, 4.365, 4.273, 4.156, 4.013, 3.843, 3.647, 3.427,
		 3.186, 2.927, 2.657, 2.382, 2.109, 1.845, 1.599, 1.379, 1.195,
};
float AnkleFr[] = {
		 0.467, 0.477, 0.500, 0.527, 0.552, 0.571, 0.583, 0.586, 0.580, 0.567,
		 0.546, 0.521, 0.492, 0.460, 0.427, 0.392, 0.358, 0.323, 0.281, 0.240,
		 0.194, 0.148, 0.099, 0.030,-0.047,-0.145,-0.254,-0.366,-0.481,-0.592,
		-0.694,-0.798,-0.897,-0.998,-1.100,-1.197,-1.300,-1.418,-1.562,-1.720,
		-1.899,-2.097,-2.283,-2.474,-2.620,-2.708,-2.725,-2.647,-2.487,-2.224,
		-1.875,-1.448,-0.956,-0.440, 0.090, 0.581, 1.026, 1.385, 1.652, 1.833,
		 1.931, 1.954, 1.926, 1.837, 1.705, 1.560, 1.394, 1.247, 1.088, 0.940,
		 0.809, 0.666, 0.527, 0.375, 0.215, 0.059,-0.108,-0.277,-0.456,-0.617,
		-0.768,-0.887,-0.960,-1.004,-0.992,-0.932,-0.826,-0.691,-0.527,-0.363,
		-0.206,-0.052, 0.079, 0.195, 0.297, 0.379, 0.448, 0.522, 0.571, 0.622,
		 0.671, 0.699, 0.709, 0.695, 0.645, 0.576, 0.485, 0.377, 0.268, 0.168,
		 0.082, 0.027, 0.004, 0.005, 0.036, 0.083, 0.137, 0.194, 0.257, 0.300,
		 0.331, 0.352, 0.349, 0.342, 0.327, 0.306, 0.285, 0.269, 0.254, 0.245,
		 0.242, 0.240, 0.239, 0.241, 0.243, 0.245, 0.250, 0.255, 0.260, 0.264,
		 0.268, 0.271, 0.275, 0.280, 0.285, 0.292, 0.301, 0.312, 0.326, 0.342,
		 0.361, 0.381, 0.402
};

float KneeSin[] = {
//		y=-0.6+0.4*np.sin(x)
//		-0.60, -0.58, -0.57, -0.55, -0.53, -0.52, -0.50, -0.49, -0.47, -0.45,
//		-0.44, -0.42, -0.41, -0.40, -0.38, -0.37, -0.35, -0.34, -0.33, -0.32,
//		-0.31, -0.29, -0.28, -0.27, -0.27, -0.26, -0.25, -0.24, -0.23, -0.23,
//		-0.22, -0.22, -0.21, -0.21, -0.21, -0.20, -0.20, -0.20, -0.20, -0.20,
//		-0.20, -0.20, -0.21, -0.21, -0.21, -0.22, -0.22, -0.23, -0.23, -0.24,
//		-0.25, -0.26, -0.27, -0.27, -0.28, -0.29, -0.31, -0.32, -0.33, -0.34,
//		-0.35, -0.37, -0.38, -0.40, -0.41, -0.42, -0.44, -0.45, -0.47, -0.49,
//		-0.50, -0.52, -0.53, -0.55, -0.57, -0.58, -0.60, -0.62, -0.63, -0.65,
//		-0.67, -0.68, -0.70, -0.71, -0.73, -0.75, -0.76, -0.78, -0.79, -0.80,
//		-0.82, -0.83, -0.85, -0.86, -0.87, -0.88, -0.89, -0.91, -0.92, -0.93,
//		-0.93, -0.94, -0.95, -0.96, -0.97, -0.97, -0.98, -0.98, -0.99, -0.99,
//		-0.99, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -1.00, -0.99, -0.99,
//		-0.99, -0.98, -0.98, -0.97, -0.97, -0.96, -0.95, -0.94, -0.93, -0.93,
//		-0.92, -0.91, -0.89, -0.88, -0.87, -0.86, -0.85, -0.83, -0.82, -0.80,
//		-0.79, -0.78, -0.76, -0.75, -0.73, -0.71, -0.70, -0.68, -0.67, -0.65,
//		-0.63, -0.62, -0.60

//		y=-0.6+0.4*np.sin(2*x)
//		-0.60,-0.57,-0.53,-0.50,-0.47,-0.44,-0.41,-0.38,-0.35,-0.33,-0.31,-0.28,
//		-0.27,-0.25,-0.23,-0.22,-0.21,-0.21,-0.20,-0.20,-0.20,-0.21,-0.21,-0.22,
//		-0.23,-0.25,-0.27,-0.28,-0.31,-0.33,-0.35,-0.38,-0.41,-0.44,-0.47,-0.50,
//		-0.53,-0.57,-0.60,-0.63,-0.67,-0.70,-0.73,-0.76,-0.79,-0.82,-0.85,-0.87,
//		-0.89,-0.92,-0.93,-0.95,-0.97,-0.98,-0.99,-0.99,-1.00,-1.00,-1.00,-0.99,
//		-0.99,-0.98,-0.97,-0.95,-0.93,-0.92,-0.89,-0.87,-0.85,-0.82,-0.79,-0.76,
//		-0.73,-0.70,-0.67,-0.63,-0.60,-0.57,-0.53,-0.50,-0.47,-0.44,-0.41,-0.38,
//		-0.35,-0.33,-0.31,-0.28,-0.27,-0.25,-0.23,-0.22,-0.21,-0.21,-0.20,-0.20,
//		-0.20,-0.21,-0.21,-0.22,-0.23,-0.25,-0.27,-0.28,-0.31,-0.33,-0.35,-0.38,
//		-0.41,-0.44,-0.47,-0.50,-0.53,-0.57,-0.60,-0.63,-0.67,-0.70,-0.73,-0.76,
//		-0.79,-0.82,-0.85,-0.87,-0.89,-0.92,-0.93,-0.95,-0.97,-0.98,-0.99,-0.99,
//		-1.00,-1.00,-1.00,-0.99,-0.99,-0.98,-0.97,-0.95,-0.93,-0.92,-0.89,-0.87,
//		-0.85,-0.82,-0.79,-0.76,-0.73,-0.70,-0.67,-0.63,-0.60

//		y=-0.6+0.4*np.sin(2*x) y=-0.6+0.2*np.sin(2*x)
//		-0.60,-0.57,-0.53,-0.50,-0.47,-0.44,-0.41,-0.38,-0.35,-0.33,
//		-0.31,-0.28,-0.27,-0.25,-0.23,-0.22,-0.21,-0.21,-0.20,-0.20,
//		-0.20,-0.21,-0.21,-0.22,-0.23,-0.25,-0.27,-0.28,-0.31,-0.33,
//		-0.35,-0.38,-0.41,-0.44,-0.47,-0.50,-0.53,-0.57,-0.60,-0.63,
//		-0.67,-0.70,-0.73,-0.76,-0.79,-0.82,-0.85,-0.87,-0.89,-0.92,
//		-0.93,-0.95,-0.97,-0.98,-0.99,-0.99,-1.00,-1.00,-1.00,-0.99,
//		-0.99,-0.98,-0.97,-0.95,-0.93,-0.92,-0.89,-0.87,-0.85,-0.82,
//		-0.79,-0.76,-0.73,-0.70,-0.67,-0.63,-0.60,-0.58,-0.57,-0.55,
//		-0.54,-0.52,-0.50,-0.49,-0.48,-0.46,-0.45,-0.44,-0.43,-0.42,
//		-0.42,-0.41,-0.41,-0.40,-0.40,-0.40,-0.40,-0.40,-0.41,-0.41,
//		-0.42,-0.42,-0.43,-0.44,-0.45,-0.46,-0.48,-0.49,-0.50,-0.52,
//		-0.54,-0.55,-0.57,-0.58,-0.60,-0.62,-0.63,-0.65,-0.66,-0.68,
//		-0.70,-0.71,-0.72,-0.74,-0.75,-0.76,-0.77,-0.78,-0.78,-0.79,
//		-0.79,-0.80,-0.80,-0.80,-0.80,-0.80,-0.79,-0.79,-0.78,-0.78,
//		-0.77,-0.76,-0.75,-0.74,-0.72,-0.71,-0.70,-0.68,-0.66,-0.65,
//		-0.63,-0.62,-0.60

//		阶跃测试
		-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,
		-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,
		-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,
		-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,
		-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,
		-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,
		-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,
		-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,-0.10,
		-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,
		-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,
		-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,
		-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,
		-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,
		-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,
		-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,-0.40,
		-0.10,-0.10,-0.10
};

int PosTar = -1;
int StopFlag = 0;

/********************************参数声明**************************************
*参数用途:	拟合数据参数
*修改日期:	20230531
*******************************************************************************/
struct DataFit Normal = {
		.ploy_n = 5,
		.Fit_Mode = 4,
		.sizenum = 0,
		.State = 0,
		.FitStart = 0,
		.Flag_Div = 0,
		.Flag_Fit = 0,
		.Flag_Send = 0,
};
//int sizenum;
//int dimension = 5;

/********************************参数声明**************************************
*参数用途:	参考数据
*修改日期:	20230809
*******************************************************************************/
float Ank0[] = {};
float Kne0[] = {};

/********************************参数声明**************************************
*参数用途:	测试参数，用以测试拟合函数的可行性
*修改日期:	20231201
*******************************************************************************/
float xtest[100];
double *tempx;
float S1[6];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_UART4_Init();
  MX_UART5_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
	setvbuf(stdout, NULL, _IONBF, 0); //输出缓冲
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	if(HAL_UART_Receive_IT(&huart1, &cRx.cRx_1, 1) != HAL_OK)
	{
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);
		Error_Handler();
//		printf("[info]Error_huart1_IT\r\n");
	}

	if(HAL_UART_Receive_IT(&huart2, &cRx.cRx_2, 1) != HAL_OK)
	{
		__HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
		Error_Handler();
//		printf("[info]Error_huart2_IT\r\n");
	}

//	if(HAL_UART_Receive_IT(&huart3, &cRx.cRx_3, 1) != HAL_OK)
//	{
//		__HAL_UART_ENABLE_IT(&huart3, UART_IT_ERR);
//		Error_Handler();
////		printf("[info]Error_huart3_IT\r\n");
//	}

//	if(HAL_UART_Receive_IT(&huart4, &cRx.cRx_4, 1) != HAL_OK)
//	{
//		__HAL_UART_ENABLE_IT(&huart4, UART_IT_ERR);
//		Error_Handler();
////		printf("[info]Error_huart4_IT\r\n");
//	}

//	if(HAL_UART_Receive_IT(&huart5, &cRx.cRx_5, 1) != HAL_OK)
//	{
//		__HAL_UART_ENABLE_IT(&huart5, UART_IT_ERR);
//		Error_Handler();
////		printf("[info]Error_huart5_IT\r\n");
//	}

//	if(HAL_UART_Receive_IT(&huart6, &cRx.cRx_6, 1) != HAL_OK)
//	{
//		__HAL_UART_ENABLE_IT(&huart6, UART_IT_ERR);
//		Error_Handler();
////		printf("[info]Error_huart6_IT\r\n");
//	}

//	// 开启串口1空闲中断
//	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
////	// 开启DMA发送通道的发送完成中断，才能实现封装发送函数里面的等待功能
////	__HAL_DMA_ENABLE_IT(&hdma_usart2_tx, DMA_IT_TC);
//	// 清除空闲标志位，防止中断误入
//	__HAL_UART_CLEAR_IDLEFLAG(&huart1);
//	// 立即就要打开DMA接收
//	// 不然DMA没有提前准备，第一次接收的数据是读取不出来的
//	HAL_UART_Receive_DMA(&huart1, p_IsToReceive1, MAX_RX_LEN);

	// 开启串口3空闲中断
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
//	// 开启DMA发送通道的发送完成中断，才能实现封装发送函数里面的等待功能
//	__HAL_DMA_ENABLE_IT(&hdma_usart2_tx, DMA_IT_TC);
	// 清除空闲标志位，防止中断误入
	__HAL_UART_CLEAR_IDLEFLAG(&huart3);
	// 立即就要打开DMA接收
	// 不然DMA没有提前准备，第一次接收的数据是读取不出来的
	HAL_UART_Receive_DMA(&huart3, p_IsToReceive3, MAX_RX_LEN);

	// 开启串口4空闲中断
	__HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
//	__HAL_DMA_ENABLE_IT(&hdma_usart2_tx, DMA_IT_TC);
	__HAL_UART_CLEAR_IDLEFLAG(&huart4);
	HAL_UART_Receive_DMA(&huart4, p_IsToReceive4, MAX_RX_LEN);

	// 开启串口5空闲中断
	__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
//	__HAL_DMA_ENABLE_IT(&hdma_usart2_tx, DMA_IT_TC);
	__HAL_UART_CLEAR_IDLEFLAG(&huart5);
	HAL_UART_Receive_DMA(&huart5, p_IsToReceive5, MAX_RX_LEN);

	// 开启串口6空闲中断
	__HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
//	__HAL_DMA_ENABLE_IT(&hdma_usart2_tx, DMA_IT_TC);
	__HAL_UART_CLEAR_IDLEFLAG(&huart6);
	HAL_UART_Receive_DMA(&huart6, p_IsToReceive6, MAX_RX_LEN);


	CAN1_SetFilters();
	CAN2_SetFilters();

    can_tx.tx_header.StdId = 0x00;
    can_tx.tx_header.ExtId = 0x0000;
    can_tx.tx_header.IDE = CAN_ID_STD;    // 标准CANID
    can_tx.tx_header.RTR = CAN_RTR_DATA;
    can_tx.tx_header.DLC = 0x07;    // 发送数据的长度 字节数
    can_tx.tx_header.TransmitGlobalTime = DISABLE;

	HAL_CAN_Start(&hcan1);
	HAL_Delay_us(50);
	HAL_CAN_Start(&hcan2);


//	HAL_TIM_Base_Start_IT(&htim2);

//	HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

	low_pass_filter_init();

	for(int i=1; i<101; i++){
		xtest[i-1] = (double)i/100;
	}

	while (1)
	{
		if(Mode == 1)
		{
			if(ModeFlag == 1)
			{
				switch(MITFlag)
				{
				case 1: //膝踝双关节指令发送
					MIT_A.Pos = KneeReference[timCounter] + 0.09;
//					MIT_A.Vel = KneeSpeedReference[timCounter];
					MIT_A.Kp = KneeKPReference[timCounter];
					CanSend(MIT_A.CanID,MIT_A.Pos,MIT_A.Vel,MIT_A.Kp,MIT_A.Kd,MIT_A.Tor,hcan1);
					CanRead(hcan1);
					HAL_Delay_us(50);
					MIT_B.Pos = AnkleReference[timCounter];
//					MIT_B.Vel = AnkleSpeedReference[timCounter];
					MIT_B.Kp = AnkleKPReference[timCounter];
					CanSend(MIT_B.CanID,MIT_B.Pos,MIT_B.Vel,MIT_B.Kp,MIT_B.Kd,MIT_B.Tor,hcan2);
					CanRead(hcan2);
					break;
				case 2: //膝关节指令发送 0x01
//					MIT_A.Pos = KneeSin[timCounter]; //sin位置跟踪取值

//					if(MIT_A.PosOut <= PosTar && StopFlag == 0){
//						MIT_A.Vel = -MIT_A.Vel;
//						StopFlag = 1;
//					}
//					if(StopFlag == 1){
//						if(MIT_A.PosOut >= 0){
//							Mode = 0;
//							StopFlag = 0;
//						}
//					}

					MIT_A.Pos = KneeReference[timCounter] + 0.09;
					MIT_A.Tor = KneeFr[timCounter];
					MIT_A.Kp = KneeKPReference[timCounter];

					CanSend(MIT_A.CanID,MIT_A.Pos,MIT_A.Vel,MIT_A.Kp,MIT_A.Kd,MIT_A.Tor,hcan1);
					CanRead(hcan1);
					break;
				case 3: //踝关节指令发送 0x02
//					if(MIT_B.PosOut <= -0.8 && StopFlag == 0){
//						MIT_B.Vel = -MIT_B.Vel;
//						StopFlag = 1;
//					}
//					if(StopFlag == 1){
//						if(MIT_B.PosOut >= 0){
//							Mode = 0;
//							StopFlag = 0;
//						}
//					}

					MIT_B.Pos = AnkleReference[timCounter];
					MIT_B.Tor = AnkleFr[timCounter];
//					MIT_B.Kp = AnkleKPReference[timCounter]-50;

					CanSend(MIT_B.CanID,MIT_B.Pos,MIT_B.Vel,MIT_B.Kp,MIT_B.Kd,MIT_B.Tor,hcan2);
					CanRead(hcan2);
				default:
					break;
				}

				switch(PrintfFlag)
				{
				case 0: //电机关节数据输出
//					DMA_usart2_printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
//							MIT_A.PosOut,MIT_A.VelOut,MIT_A.TorOut,
//							MIT_B.PosOut,MIT_B.VelOut,MIT_B.TorOut);

					DMA_usart2_printf("%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
							MIT_A.Pos, MIT_A.PosOut, MIT_A.Pos-MIT_A.PosOut, MIT_B.Pos, MIT_B.PosOut, MIT_B.Pos-MIT_B.PosOut);

//					DMA_usart2_printf("%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
//							MIT_A.Pos, MIT_A.PosOut, MIT_A.Vel, MIT_A.VelOut, MIT_A.TorOut);

//					DMA_usart2_printf("%.3f,%.3f,%.3f,%.3f,%.3f\r\n",
//							MIT_B.Pos, MIT_B.PosOut, MIT_B.Vel, MIT_B.VelOut, MIT_B.TorOut);
					break;
				case 1: //右侧关节角度、角加速度、加速度输出

					DMA_usart2_printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
							Right.Hip.AngxCal,Right.Knee.AngxCal,Right.Ankle.AngxCal,
							Right.Hip.AngAccx,Right.Knee.AngAccx,Right.Ankle.AngAccx,
							Right.Hip.Accx,Right.Knee.Accx,Right.Ankle.Accx);
					break;
				case 2: //左右关节角度输出
//					printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",Right.Hip.Angx,Right.Knee.Angx,Right.Ankle.Angx,Right.Knee.Angx-Right.Hip.Angx,Right.Ankle.Angx-Right.Knee.Angx,
//							Left.Hip.Angx,Left.Knee.Angx,Left.Ankle.Angx,Left.Knee.Angx-Left.Hip.Angx,Left.Ankle.Angx-Left.Knee.Angx);
					DMA_usart2_printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
							Right.Hip.AngxCal,Right.Knee.AngxCal,Right.Ankle.AngxCal,
							Left.Hip.AngxCal,Left.Knee.AngxCal,Left.Ankle.AngxCal);
					break;
				case 3:
//					//左侧关节角度、左侧足底压力输出
//					DMA_usart2_printf("%.2f,%.2f,%.2f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f\r\n",
//							Left.Hip.AngxCal,Left.Knee.AngxCal,Left.Ankle.AngxCal,
//							DataLeftBuf.Data[3],DataLeftBuf.Data[4],DataLeftBuf.Data[7],DataLeftBuf.Data[8],
//							DataLeftBuf.Data[9],DataLeftBuf.Data[12],DataLeftBuf.Data[13],DataLeftBuf.Data[14]);

					//右侧关节角度、左侧足底压力输出
					DMA_usart2_printf("%.2f,%.2f,%.2f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f\r\n",
							Right.Hip.AngxCal,Right.Knee.AngxCal,Right.Ankle.AngxCal,
							DataRightBufFoot.Data[3],DataRightBufFoot.Data[4],DataRightBufFoot.Data[7],DataRightBufFoot.Data[8],
							DataRightBufFoot.Data[9],DataRightBufFoot.Data[12],DataRightBufFoot.Data[13],DataRightBufFoot.Data[14]);
					break;
				case 4: //左右足底压力输出
					DMA_usart2_printf("%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f,%.0f\r\n",
							DataRightBufFoot.Data[3],DataRightBufFoot.Data[4],DataRightBufFoot.Data[7],DataRightBufFoot.Data[8],
							DataRightBufFoot.Data[9],DataRightBufFoot.Data[12],DataRightBufFoot.Data[13],DataRightBufFoot.Data[14],
							DataLeftBuf.Data[3],DataLeftBuf.Data[4],DataLeftBuf.Data[7],DataLeftBuf.Data[8],
							DataLeftBuf.Data[9],DataLeftBuf.Data[12],DataLeftBuf.Data[13],DataLeftBuf.Data[14]);
					break;
				case 5: //拟合数据输出
//					DataDiv(&Normal);
//					DMA_usart2_printf("%f,%f,%f,%f,%f,%f\r\n",
//							Normal.PKneeS1[0],Normal.PKneeS1[1],Normal.PKneeS1[2],Normal.PKneeS1[3],Normal.PKneeS1[4],Normal.PKneeS1[5]);
				default:
					break;
				}
			}
			else if(ModeFlag == 0)
			{
				switch(MITFlag)
				{
				case 1: //膝踝双关节指令发送
					EnterMotorMode(MIT_A.CanID,hcan1);
					HAL_Delay_us(50);
					EnterMotorMode(MIT_B.CanID,hcan2);
					break;
				case 2: //膝关节指令发送 0x01
					EnterMotorMode(MIT_A.CanID,hcan1);
					break;
				case 3: //踝关节指令发送 0x02
					EnterMotorMode(MIT_B.CanID,hcan2);
				default:
					break;
				}
				HAL_TIM_Base_Start_IT(&htim2);
				Normal.State = 1;
				ModeFlag = 1;
			}
		}

		else if(Mode == 2)
		{
			//低通滤波
//			Right.Hip.AngxFilter = low_pass_filter(Right.Hip.AngxCal);
//			Right.Knee.AngxFilter = low_pass_filter(Right.Knee.AngxCal);
//			Right.Ankle.AngxFilter = low_pass_filter(Right.Ankle.AngxCal);
//			DMA_usart2_printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
//					Right.Hip.AngxCal,Right.Knee.AngxCal,Right.Ankle.AngxCal,
//					Right.Hip.AngxFilter,Right.Knee.AngxFilter,Right.Ankle.AngxFilter);


//			tempx = (double *)calloc(100 , sizeof(double));
//			tempx = createArray(100);
//
//			for(int i=0; i<100; i++){
//				tempx[i] = KneeReference[i];
//			}
//			polyfit(100, xtest, tempx, 5, Normal.PKneeS1);
//			DMA_usart2_printf("%f\r\n",Normal.PKneeS1[0]);
//			free(tempx);

		}
		else if(Mode == 0)
		{
			if(ModeFlag == 1)
			{
				switch(MITFlag)
				{
				case 1:
					ExitMotorMode(MIT_A.CanID,hcan1);
					HAL_Delay_us(50);
					ExitMotorMode(MIT_B.CanID,hcan2);
					break;
				case 2:
					ExitMotorMode(MIT_A.CanID,hcan1);
					break;
				case 3:
					ExitMotorMode(MIT_B.CanID,hcan2);
				default:
					break;
				}
				HAL_TIM_Base_Stop_IT(&htim2);
				timCounter = 0;
				ModeFlag = 0;
			}
			Normal.State = 0;
			while(1)
			{
//				DMA_usart2_printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
//						Right.Hip.AngxCal,Right.Knee.AngxCal,Right.Ankle.AngxCal,
//						Right.Hip.AngAccx,Right.Knee.AngAccx,Right.Ankle.AngAccx,
//						Right.Hip.Accx,Right.Knee.Accx,Right.Ankle.Accx);
//				HAL_Delay(Delay);
				if(Mode != 0)break;
			}
		}
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		switch(DelayFlag)
		{
		case 0:
			HAL_Delay(Delay);
			break;
		case 1:
			HAL_Delay_us(DelayUs);
			break;
		default:
			break;
		}
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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

  /** Initializes the CPU, AHB and APB buses clocks
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
#pragma region

/********************************实现函数**************************************
*函数原型:	printf()
*功　　能:	输出重定向，串口2输出
*修改日期:	20230526
*******************************************************************************/
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif
PUTCHAR_PROTOTYPE
{
	HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	return ch;
}


/********************************实现函数**************************************
*函数原型:	huart1_printf()
*功　　能:	输出重定向，串口1,2输出
*修改日期:	20230526
*******************************************************************************/
void huart1_printf(char * fmt,...)
{
    char buffer[100];
    uint16_t i=0;
    va_list arg_ptr;
    va_start(arg_ptr,fmt);
    vsnprintf(buffer,100,fmt,arg_ptr);
    while(i<99&&buffer[i])
    {
        HAL_UART_Transmit(&huart1,(uint8_t *)&buffer[i],1,0xFFFF);
        i++;
    }
    va_end(arg_ptr);
}

void huart2_printf(char * fmt,...)
{
    char buffer[100];
    uint16_t i=0;
    va_list arg_ptr;
    va_start(arg_ptr,fmt);
    vsnprintf(buffer,100,fmt,arg_ptr);
    while(i<99&&buffer[i])
    {
        HAL_UART_Transmit(&huart2,(uint8_t *)&buffer[i],1,0xFFFF);
        i++;
    }
    va_end(arg_ptr);
}

/********************************实现函数**************************************
*函数原型:	Huart_IT_Init()
*功　　能:	串口中断初始化(暂不能用)
*修改日期:	20230527
*******************************************************************************/
void Huart_IT_Init()
{
	if(HAL_UART_Receive_IT(&huart1, &cRx.cRx_1, 1) != HAL_OK)
	{
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);
		Error_Handler();
//		printf("[info]Error_huart1_IT\r\n");
	}
	if(HAL_UART_Receive_IT(&huart2, &cRx.cRx_2, 1) != HAL_OK)
	{
		__HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
		Error_Handler();
//		printf("[info]Error_huart2_IT\r\n");
	}
	if(HAL_UART_Receive_IT(&huart3, &cRx.cRx_3, 1) != HAL_OK)
	{
		__HAL_UART_ENABLE_IT(&huart3, UART_IT_ERR);
		Error_Handler();
//		printf("[info]Error_huart3_IT\r\n");
	}
	if(HAL_UART_Receive_IT(&huart4, &cRx.cRx_4, 1) != HAL_OK)
	{
		__HAL_UART_ENABLE_IT(&huart4, UART_IT_ERR);
		Error_Handler();
//		printf("[info]Error_huart4_IT\r\n");
	}
	if(HAL_UART_Receive_IT(&huart5, &cRx.cRx_5, 1) != HAL_OK)
	{
		__HAL_UART_ENABLE_IT(&huart5, UART_IT_ERR);
		Error_Handler();
//		printf("[info]Error_huart5_IT\r\n");
	}
	if(HAL_UART_Receive_IT(&huart6, &cRx.cRx_6, 1) != HAL_OK)
	{
		__HAL_UART_ENABLE_IT(&huart6, UART_IT_ERR);
		Error_Handler();
//		printf("[info]Error_huart6_IT\r\n");
	}
}

/********************************实现函数**************************************
*函数原型:	HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
*功　　能:	串口中断
*修改日期:	20230530
 * 参数		| 介绍
 * ---------+--------------------------------------
 * huart	| 产生中断的串口
*******************************************************************************/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	/********************************串口1**************************************/
	if(huart->Instance == USART1)
	{
//		if(DataLeftBufAng.AngCounter < 1)
//		{
//			if(cRx.cRx_1 != 0x62)
//			{
//				DataLeftBufAng.AngCounter = 0;
//			}
//			else
//			{
//				DataLeftBufAng.HexBufSumAng[DataLeftBufAng.AngCounter] = cRx.cRx_1;
//				DataLeftBufAng.AngCounter++;
//			}
//		}
//		else if(DataLeftBufAng.AngCounter < 13)
//		{
//			DataLeftBufAng.HexBufSumAng[DataLeftBufAng.AngCounter] = cRx.cRx_1;
//			DataLeftBufAng.AngCounter++;
//		}
//		else if(DataLeftBufAng.AngCounter == 13)
//		{
//			if(cRx.cRx_1 == 0x63)
//			{
//				DataLeftBufAng.HexBufSumAng[DataLeftBufAng.AngCounter] = cRx.cRx_1;
//				for(int i=0;i<3;i++)
//				{
//					for(int j=0;j<4;j++)
//					{
//						DataLeftBufAng.DataUnionBuf.HexBuf[j] = DataLeftBufAng.HexBufSumAng[4*i+j+1];
//					}
//					switch(i)
//					{
//					case 0:
//						Left.Hip.AngxCal = DataLeftBufAng.DataUnionBuf.FloatBuf;
//						break;
//					case 1:
//						Left.Knee.AngxCal = DataLeftBufAng.DataUnionBuf.FloatBuf;
//						break;
//					case 2:
//						Left.Ankle.AngxCal = DataLeftBufAng.DataUnionBuf.FloatBuf;
//						break;
//					default:
//						break;
//					}
//				}
//			}
//			DataLeftBufAng.AngCounter = 0;
//		}


		switch(cRx.cRx_1)
		{
		case 0x62:
			DataLeftBuf.Flag = 0;
//			DMA_usart2_printf("0\r\n");
			break;
		case 0x64:
			DataLeftBuf.Flag = 1;
//			DMA_usart2_printf("1\r\n");
			break;
		case 0x66:
			DataLeftBuf.Flag = 2;
			DMA_usart2_printf("2\r\n");
			break;
		default:
			break;
		}

		switch(DataLeftBuf.Flag)
		{
		case 0:
			AngDataBuf(&DataLeftBuf, cRx.cRx_1);
			break;
		case 1:
			FootDataBuf(&DataLeftBuf, cRx.cRx_1);
			break;
		case 2:
			switch(DataLeftBuf.DisCounter){
			case 0:
				if(cRx.cRx_1 != 0x66)
				{
					DataLeftBuf.DisCounter = 0;
				}
				else{
					DataLeftBuf.HexBufDis[DataLeftBuf.DisCounter] = cRx.cRx_1;
					DataLeftBuf.DisCounter++;
				}
				break;
			case 1:
				DataLeftBuf.HexBufDis[DataLeftBuf.DisCounter] = cRx.cRx_1;
				DataLeftBuf.DisCounter++;
				break;
			case 2:
				if(cRx.cRx_1 == 0x67)
				{
					DataLeftBuf.HexBufDis[DataLeftBuf.DisCounter] = cRx.cRx_1;
					if(DataLeftBuf.HexBufDis[1] == 0x01){
						DMA_usart2_printf("Dis1");
					}
					else if(DataLeftBuf.HexBufDis[1] == 0x00){
						DMA_usart2_printf("Dis0");
					}
				}
				DataLeftBuf.DisCounter = 0;
				break;
			default:
				break;
			}
			break;
		default:
			break;
		}

		if(HAL_UART_Receive_IT(&huart1, &cRx.cRx_1, 1) != HAL_OK)
		{
			__HAL_UART_ENABLE_IT(&huart1, UART_IT_ERR);
			Error_Handler();
//			printf("[info]Error_huart1_IT\r\n");
		}
	}

	/********************************串口2**************************************/
	else if(huart->Instance == USART2)
	{
		if(cRx.cRx_2 == '\n')
		{
			float operand;
			char operator1, operator2;
			cRx.rxBuf_2[cRx.rxBufCursor_2] = '\0';
			if(sscanf((const char*)cRx.rxBuf_2, "%c%c%f", &operator1, &operator2, &operand) == 3)
			{
				if(operator1 == 'S' && operator2 == '=')
				{
					Mode = (int)operand;
					//printf("[Info] Mode=%d\r\n",Mode);
					huart1_printf("S=%d\n",Mode);
				}
				else if(Mode != 1)
				{
					switch(operator1)
					{
					case 'c':
						switch(operator2)
						{
						case 'A':
							MIT_A.CanID = (uint16_t)operand;
							break;
						case 'B':
							MIT_B.CanID = (uint16_t)operand;
							break;
						case 'd':
							if(operand == 0){
								printf("[Info]\n"
										"FM%d\n"
										"FP%d\n"
										"FD%d\n"
										"Fm%d\n"
										"Fu%d\n",
										MITFlag,PrintfFlag,DelayFlag,Delay,DelayUs);
							}
						default:
							printf("[Info]ErrorFun\r\n");
							break;
						}
						printf("[InfoCanID] %d,%d\r\n",MIT_A.CanID,MIT_B.CanID);
						break;
					case 'p':
						switch(operator2)
						{
						case 'A':
							MIT_A.Pos = operand;
							break;
						case 'B':
							MIT_B.Pos = operand;
							break;
						default:
							printf("[Info]ErrorFun\r\n");
							break;
						}
						printf("[InfoPos] %.2f,%.2f\r\n",MIT_A.Pos,MIT_B.Pos);
						break;
					case 'v':
						switch(operator2)
						{
						case 'A':
							MIT_A.Vel = operand;
							break;
						case 'B':
							MIT_B.Vel = operand;
							break;
						default:
							printf("[Info]ErrorFun\r\n");
							break;
						}
						printf("[InfoVel] %.2f,%.2f\r\n",MIT_A.Vel,MIT_B.Vel);
						break;
					case 'P':
						switch(operator2)
						{
						case 'A':
							MIT_A.Kp = operand;
							break;
						case 'B':
							MIT_B.Kp = operand;
							break;
						default:
							printf("[Info]ErrorFun\r\n");
							break;
						}
						printf("[InfoKp] %.2f,%.2f\r\n",MIT_A.Kp,MIT_B.Kp);
						break;
					case 'V':
						switch(operator2)
						{
						case 'A':
							MIT_A.Kd = operand;
							break;
						case 'B':
							MIT_B.Kd = operand;
							break;
						default:
							printf("[Info]ErrorFun\r\n");
							break;
						}
						printf("[InfoKd] %.2f,%.2f\r\n",MIT_A.Kd,MIT_B.Kd);
						break;
					case 't':
						switch(operator2)
						{
						case 'A':
							MIT_A.Tor = operand;
							break;
						case 'B':
							MIT_B.Tor = operand;
							break;
						default:
							printf("[Info]ErrorFun\r\n");
							break;
						}
						printf("[InfoTor] %.2f,%.2f\r\n",MIT_A.Tor,MIT_B.Tor);
						break;
					case 'C':
						switch(operator2)
						{
						case 'A':
							if(operand == 1){
								EnterMotorMode(MIT_A.CanID,hcan1);
							}
							else{
								ExitMotorMode(MIT_A.CanID,hcan1);
							}
							break;
						case 'B':
							if(operand == 1){
								EnterMotorMode(MIT_B.CanID,hcan2);
							}
							else{
								ExitMotorMode(MIT_B.CanID,hcan2);
							}
							break;
						default:
							printf("[Info]ErrorFun\r\n");
							break;
						}
						printf("[InfoCan]\r\n");
						break;
					case 'M':
						switch(operator2)
						{
						case 'Z':
							if(operand == 1){
								Right.Hip.AngxZero = Right.Hip.Angx;
								Right.Knee.AngxZero = Right.Knee.Angx;
								Right.Ankle.AngxZero = Right.Ankle.Angx;
								Left.Hip.AngxZero = Left.Hip.Angx;
								Left.Knee.AngxZero = Left.Knee.Angx;
								Left.Ankle.AngxZero = Left.Ankle.Angx;
								Left.WAIST.AccxZero = Left.WAIST.Angx;
							}
							else if(operand == 0){
								Right.Hip.AngxZero = 0;
								Right.Knee.AngxZero = 0;
								Right.Ankle.AngxZero = 0;
								Left.Hip.AngxZero = 0;
								Left.Knee.AngxZero = 0;
								Left.Ankle.AngxZero = 0;
								Left.WAIST.AccxZero = 0;
							}
							break;
						case 'B':
							break;
						default:
							printf("[Info]ErrorFun\r\n");
							break;
						}
						huart1_printf("M%c%d\n",operator2,(int)operand);
						printf("[InfoMPU6050]\r\n");
						break;
					case 'F':
						switch(operator2)
						{
						case 'P':
							PrintfFlag = (int)operand;
							printf("[InfoFlag]Print%d\r\n",(int)operand);
							break;
						case 'M':
							MITFlag = (int)operand;
							printf("[InfoFlag]Mode%d\r\n",(int)operand);
							break;
						case 'D':
							DelayFlag = (int)operand;
							printf("[InfoFlag]delay%d\r\n",(int)operand);
						case 'm':
							Delay = (int)operand;
							printf("[InfoFlag]ms%d\r\n",(int)operand);
							break;
						case 'u':
							DelayUs = (int)operand;
							printf("[InfoFlag]us%d\r\n",(int)operand);
						case 'Z':
							if(operand == 1){
								for(int i=0;i<8;i++)
								{
									DataRightBufFoot.DataZero[DataRightBufFoot.Point[i]] = DataRightBufFoot.Data[DataRightBufFoot.Point[i]];
									DataLeftBuf.DataZero[DataRightBufFoot.Point[i]] = DataLeftBuf.Data[DataRightBufFoot.Point[i]];
								}
								printf("[InfoFlag]Timing\r\n");
							}
							else if(operand == 0){
								for(int i=0;i<8;i++)
								{
									DataRightBufFoot.DataZero[DataRightBufFoot.Point[i]] = 0;
									DataLeftBuf.DataZero[DataRightBufFoot.Point[i]] = 0;
								}
								printf("[InfoFlag]Zero\r\n");
							}
//							huart1_printf("F%c%d\n",operator2,(int)operand);
							break;
						case 'f':
							fc = operand;
							low_pass_filter_init();
							printf("[InfoFlag]fs=%.1f\r\n",operand);
							break;
						case 'T':
							Ts = operand;
							low_pass_filter_init();
							printf("[InfoFlag]Ts=%.2f\r\n",operand);
							break;
						case 'F':
							Normal.FitStart = (int)operand;
							Normal.Fit_Mode = (int)operand + 3;
							printf("[InfoFlag]FS=%d\r\n",(int)operand);
							break;
						default:
							printf("[Info]ErrorFun\r\n");
							break;
						}
//						printf("[InfoFlag]\r\n");
						break;
					default:
						break;
					}
				}
			}
			cRx.rxBufCursor_2 = 0;
		}
		else
		{
			if(cRx.rxBufCursor_2 < 255)
			{
				cRx.rxBuf_2[cRx.rxBufCursor_2++] = cRx.cRx_2;
			}
		}

		if(HAL_UART_Receive_IT(&huart2, &cRx.cRx_2, 1) != HAL_OK)
		{
			__HAL_UART_ENABLE_IT(&huart2, UART_IT_ERR);
			Error_Handler();
//			printf("[info]Error_huart2_IT\r\n");
		}
	}

	/********************************串口3**************************************/
	else if(huart->Instance == USART3)
	{
		MPU6050ModDataBuf(&Right.Hip,cRx.cRx_3);
		Right.Hip.AngxCal = Right.Hip.Angx;
		if(HAL_UART_Receive_IT(&huart3, &cRx.cRx_3, 1) != HAL_OK)
		{
			__HAL_UART_ENABLE_IT(&huart3, UART_IT_ERR);
			Error_Handler();
//			printf("[info]Error_huart3_IT\r\n");
		}
	}

	/********************************串口4**************************************/
	else if(huart->Instance == UART4)
	{
		MPU6050ModDataBuf(&Right.Knee,cRx.cRx_4);
		Right.Knee.AngxCal = Right.Knee.Angx - Right.Hip.Angx;
		if(HAL_UART_Receive_IT(&huart4, &cRx.cRx_4, 1) != HAL_OK)
		{
			__HAL_UART_ENABLE_IT(&huart4, UART_IT_ERR);
			Error_Handler();
//			printf("[info]Error_huart4_IT\r\n");
		}
	}

	/********************************串口5**************************************/
	else if(huart->Instance == UART5)
	{
		MPU6050ModDataBuf(&Right.Ankle,cRx.cRx_5);
		Right.Ankle.AngxCal = Right.Ankle.Angx - Right.Knee.Angx;
		if(HAL_UART_Receive_IT(&huart5, &cRx.cRx_5, 1) != HAL_OK)
		{
			__HAL_UART_ENABLE_IT(&huart5, UART_IT_ERR);
			Error_Handler();
//			printf("[info]Error_huart5_IT\r\n");
		}
	}

	/********************************串口6**************************************/
	else if(huart->Instance == USART6)
	{
		FootDataBuf(&DataRightBufFoot, cRx.cRx_6);
		if(HAL_UART_Receive_IT(&huart6, &cRx.cRx_6, 1) != HAL_OK)
		{
			__HAL_UART_ENABLE_IT(&huart6, UART_IT_ERR);
			Error_Handler();
//			printf("[info]Error_huart6_IT\r\n");
		}
	}
}


/********************************实现函数**************************************
*函数原型:	void MPU6050ModDataBuf(struct Data AllData, uint8_t cRx)
*功　　能:	MPU6050角度等数据读取函数，用于串口中断
*修改日期:	20230526
 * 参数		| 介绍
 * ---------+--------------------------------------
 * AllData	| 关节结构体，储存各关节数据
 * cRx		| 中断时串口读取的数据
*******************************************************************************/
void MPU6050ModDataBuf(struct Data *AllData, uint8_t cRx)
{
	  if(AllData->State.counter < 1)
	  {
		  if(cRx != 0x55)
		  {
			  AllData->State.counter = 0;
			  AllData->State.sum = 0;
		  }
		  else
		  {
			  ++AllData->State.counter;
			  AllData->State.sum += cRx;
			  AllData->Buf.rxDataAcc[AllData->State.counter] = cRx;
			  AllData->Buf.rxDataAngAcc[AllData->State.counter] = cRx;
			  AllData->Buf.rxDataAng[AllData->State.counter] = cRx;
		  }
	  }
	  else if(AllData->State.counter < 2)
	  {
		  AllData->Buf.rxDataAcc[AllData->State.counter] = cRx;
		  AllData->Buf.rxDataAngAcc[AllData->State.counter] = cRx;
		  AllData->Buf.rxDataAng[AllData->State.counter] = cRx;
		  switch(cRx)
		  {
		  case 0x51:
			  //采集加速度
			  AllData->State.Flag = 1;
			  ++AllData->State.counter;
			  AllData->State.sum += cRx;

			  //不采集加速度
//			  AllData->State.Flag = 0;
//			  AllData->State.counter = 0;
//			  AllData->State.sum = 0;
			  break;
		  case 0x52:
			  //采集角速度
			  AllData->State.Flag = 2;
			  ++AllData->State.counter;
			  AllData->State.sum += cRx;

			  //不采集角速度
//			  AllData->State.Flag = 0;
//			  AllData->State.counter = 0;
//			  AllData->State.sum = 0;
			  break;
		  case 0x53:
			  AllData->State.Flag = 3;
			  ++AllData->State.counter;
			  AllData->State.sum += cRx;
			  break;
		  default:
			  AllData->State.Flag = 0;
			  AllData->State.counter = 0;
			  AllData->State.sum = 0;
			  break;
		  }
	  }
	  else if(AllData->State.counter < 11)
	  {
		  switch (AllData->State.Flag)
		  {
		  case 1:
			  AllData->Buf.rxDataAcc[ AllData->State.counter] = cRx;
			  ++AllData->State.counter;
			  AllData->State.sum += cRx;
			  break;
		  case 2:
			  AllData->Buf.rxDataAngAcc[ AllData->State.counter] = cRx;
			  ++AllData->State.counter;
			  AllData->State.sum += cRx;
			  break;
		  case 3:
			  AllData->Buf.rxDataAng[ AllData->State.counter] = cRx;
			  ++AllData->State.counter;
			  AllData->State.sum += cRx;
			  break;
		  default:
			  AllData->State.Flag = 0;
			  AllData->State.counter = 0;
			  AllData->State.sum = 0;
			  break;
		  }
	  }
	  else if(AllData->State.counter == 11)
	  {
		  if(AllData->State.sum != cRx)
		  {
			  AllData->State.counter = 0;

		  }
		  else
		  {
			  AllData->Buf.rxDataAcc[AllData->State.counter] = cRx;
			  AllData->Buf.rxDataAngAcc[AllData->State.counter] = cRx;
			  AllData->Buf.rxDataAng[AllData->State.counter] = cRx;
		  }
		  AllData->State.counter = 0;
		  AllData->State.sum = 0;

		  switch(AllData->State.Flag)
		  {
		  case 1:
			  AllData->Buf.Accx = (AllData->Buf.rxDataAcc[3]<<8)|AllData->Buf.rxDataAcc[2];
			  AllData->Accx = (float) AllData->Buf.Accx/32768*16*g;
			  AllData->Buf.Accy = (AllData->Buf.rxDataAcc[5]<<8)|AllData->Buf.rxDataAcc[4];
			  AllData->Accy = (float) AllData->Buf.Accy/32768*16*g;
			  AllData->Buf.Accz = (AllData->Buf.rxDataAcc[7]<<8)|AllData->Buf.rxDataAcc[6];
			  AllData->Accz = (float) AllData->Buf.Accz/32768*16*g;

			  if(AllData->Accx > 156.8)
			  {
				  AllData->Accx = AllData->Accx - 313.6;
			  }
			  break;
		  case 2:
			  AllData->Buf.AngAccx = (AllData->Buf.rxDataAngAcc[3]<<8)|AllData->Buf.rxDataAngAcc[2];
			  AllData->AngAccx = (float) AllData->Buf.AngAccx/32768*2000;
			  AllData->Buf.AngAccy = (AllData->Buf.rxDataAngAcc[5]<<8)|AllData->Buf.rxDataAngAcc[4];
			  AllData->AngAccy = (float) AllData->Buf.AngAccy/32768*2000;
			  AllData->Buf.AngAccz = (AllData->Buf.rxDataAngAcc[7]<<8)|AllData->Buf.rxDataAngAcc[6];
			  AllData->AngAccz = (float) AllData->Buf.AngAccz/32768*2000;

			  if(AllData->AngAccx > 2000)
			  {
				  AllData->AngAccx = AllData->AngAccx - 4000;
			  }
			  break;
		  case 3:
			  AllData->Buf.Angx = (AllData->Buf.rxDataAng[3]<<8)|AllData->Buf.rxDataAng[2];
			  AllData->Angx = (float) AllData->Buf.Angx/32768*180;
			  AllData->Buf.Angy = (AllData->Buf.rxDataAng[5]<<8)|AllData->Buf.rxDataAng[4];
			  AllData->Angy = (float) AllData->Buf.Angy/32768*180;
			  AllData->Buf.Angz = (AllData->Buf.rxDataAng[7]<<8)|AllData->Buf.rxDataAng[6];
			  AllData->Angz = (float) AllData->Buf.Angz/32768*180;

			  AllData->Angx = AllData->Angx - AllData->AngxZero;
			  if(AllData->Angx > 180)
			  {
				  AllData->Angx = AllData->Angx - 360;
			  }
			  break;
		  default:
			  break;
		  }

		  AllData->State.Flag = 0;
	  }
}

/********************************实现函数**************************************
*函数原型:	void FootDataBuf()
*功　　能:	足底压力共同体数据
*修改日期:	20230721
 * 参数		| 介绍
 * ---------+--------------------------------------
 * AllData	| 共同体数据
 * cRx		| 中断时串口读取的数据
*******************************************************************************/
void FootDataBuf(struct DataUnionBuf *AllData, uint8_t cRx)
{
	if(AllData->FootCounter < 1)
	{
		if(cRx != 0x64)
		{
			AllData->FootCounter = 0;
		}
		else
		{
			AllData->HexBufSumFoot[AllData->FootCounter] = cRx;
			AllData->FootCounter++;
		}
	}
	else if(AllData->FootCounter < 33)
	{
		AllData->HexBufSumFoot[AllData->FootCounter] = cRx;
		AllData->FootCounter++;
	}
	else if(AllData->FootCounter == 33)
	{
		if(cRx == 0x65)
		{
			AllData->HexBufSumFoot[AllData->FootCounter] = cRx;
			for(int i=0;i<8;i++)
			{
				for(int j=0;j<4;j++)
				{
					AllData->DataUnionBuf.HexBuf[j] = AllData->HexBufSumFoot[4*i+j+1];
				}
				if(AllData->DataUnionBuf.FloatBuf>0 && AllData->DataUnionBuf.FloatBuf<3000)
				{
					AllData->Data[AllData->Point[i]] = AllData->DataUnionBuf.FloatBuf-AllData->DataZero[AllData->Point[i]];
				}

			}
		}
		AllData->FootCounter = 0;
	}
}
/********************************实现函数**************************************
*函数原型:	void AngDataBuf()
*功　　能:	左侧角度共同体数据
*修改日期:	20230721
 * 参数		| 介绍
 * ---------+--------------------------------------
 * AllData	| 共同体数据
 * cRx		| 中断时串口读取的数据
*******************************************************************************/
void AngDataBuf(struct DataUnionBuf *AllData, uint8_t cRx)
{
	if(AllData->AngCounter < 1)
	{
		if(cRx != 0x62)
		{
			AllData->AngCounter = 0;
		}
		else
		{
			AllData->HexBufSumAng[AllData->AngCounter] = cRx;
			AllData->AngCounter++;
		}
	}
	else if(AllData->AngCounter < 13)
	{
		AllData->HexBufSumAng[AllData->AngCounter] = cRx;
		AllData->AngCounter++;
	}
	else if(AllData->AngCounter == 13)
	{
		if(cRx == 0x63)
		{
			AllData->HexBufSumAng[AllData->AngCounter] = cRx;
			for(int i=0;i<3;i++)
			{
				for(int j=0;j<4;j++)
				{
					AllData->DataUnionBuf.HexBuf[j] = AllData->HexBufSumAng[4*i+j+1];
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
		AllData->AngCounter = 0;
	}
}

/********************************实现函数**************************************
*函数原型:	float *createArray(int size)
*功　　能:	动态分配内存
*功　　能:	记得在使用完后 free(array); 否则会造成内存泄露
*修改日期:	20230809
 * 参数		| 介绍
 * ---------+--------------------------------------
 * size		| 生成数组大小
*******************************************************************************/
double *createArray(int size)
{
	double *array = (double *) malloc(size * sizeof(double));  // 动态分配内存空间

    if (array == NULL) {
        printf("Memory allocation failed.\n");
//        exit(1);  // 内存分配失败，退出程序
    }

    // 初始化数组
    for (int i = 0; i < size; i++) {
        array[i] = 0;
    }

    return array;
}

/********************************实现函数**************************************
*函数原型:	void DataDiv
*功　　能:	左侧数据分割便于拟合
*修改日期:	20230810
 * 参数		| 介绍
 * ---------+--------------------------------------
 * Fit		| 数据储存位置
*******************************************************************************/
//void DataDiv(struct DataFit *Fit)
//{
//	switch(Fit->State)
//	{
//	case 0:
//		if(DataLeftBuf.Data[4]>0 && DataLeftBuf.Data[4]<1000)
//		{
//			Fit->FitBufKnee[Fit->sizenum] = Left.Knee.AngxCal;
//			Fit->FitBufAnkle[Fit->sizenum] = Left.Ankle.AngxCal;
//			Fit->sizenum = Fit->sizenum + 1;
//		}
//		else
//		{
//			double *arrayKnee = createArray(Fit->sizenum);
//			double *arrayAnkle = createArray(Fit->sizenum);
//			double *arrayY = createArray(Fit->sizenum);
//
//			memcpy(&arrayKnee,Fit->FitBufKnee,Fit->sizenum);
//			memcpy(&arrayAnkle,Fit->FitBufAnkle,Fit->sizenum);
//
//			int sizenum = sizeof(&arrayKnee)/ sizeof(arrayKnee[0]);
//
//			polyfit(sizenum, arrayKnee, arrayY, dimension, Fit->PKneeS1);
//			polyfit(sizenum, arrayAnkle, arrayY, dimension, Fit->PAnkleS1);
//
//			Fit->State = 1;
//			Fit->sizenum = 0;
//
//			free(arrayKnee);
//			free(arrayAnkle);
//			free(arrayY);
//		}
//		break;
//	case 1:
//		if(DataLeftBuf.Data[4]>1000)
//		{
//			Fit->FitBufKnee[Fit->sizenum] = Left.Knee.AngxCal;
//			Fit->FitBufAnkle[Fit->sizenum] = Left.Ankle.AngxCal;
//			Fit->sizenum = Fit->sizenum + 1;
//		}
//		else
//		{
//			double *arrayKnee = createArray(Fit->sizenum);
//			double *arrayAnkle = createArray(Fit->sizenum);
//			double *arrayY = createArray(Fit->sizenum);
//
//			memcpy(&arrayKnee,Fit->FitBufKnee,Fit->sizenum);
//			memcpy(&arrayAnkle,Fit->FitBufAnkle,Fit->sizenum);
//
//			int sizenum = sizeof(&arrayKnee)/ sizeof(arrayKnee[0]);
//
//			polyfit(sizenum, arrayKnee, arrayY, dimension, Fit->PKneeS2);
//			polyfit(sizenum, arrayAnkle, arrayY, dimension, Fit->PAnkleS2);
//
//			Fit->State = 0;
//			Fit->sizenum = 0;
//
//			free(arrayKnee);
//			free(arrayAnkle);
//			free(arrayY);
//		}
//		break;
//	default:
//		break;
//	}
//}

/********************************实现函数**************************************
*函数原型:	void DataDiv
*功　　能:	数据分割便于拟合
*修改日期:	20231201
 * 参数		| 介绍
 * ---------+--------------------------------------
 * Fit		| 数据储存位置
*******************************************************************************/
void DataDiv(struct DataFit *Fit){
	if(Fit->FitFlagKnee == 0){
		Fit->FitBufKnee[Fit->sizenum] = Right.Knee.AngxCal;
		if(Fit->sizenum > 3){
			if(	((Fit->FitBufKnee[Fit->sizenum]-Fit->FitBufKnee[Fit->sizenum-1])*
				(Fit->FitBufKnee[Fit->sizenum-1]-Fit->FitBufKnee[Fit->sizenum-2])<0
				&& Fit->sizenum > 15)
				|| Fit->sizenum > 65){
				Fit->FitFlagKnee = 1;
			}
		}
		Fit->sizenum++;
	}
	else if(Fit->FitFlagKnee == 1){
		//内存分配方式1
		double *arrayKnee = createArray(Fit->sizenum);
		double *arrayX = createArray(Fit->sizenum);
		double Init_x = 0;

		//内存分配方式2
//		double *arrayKnee = (double *)calloc(Fit->sizenum + 1 , sizeof(double));
//		double *arrayX = (double *)calloc(Fit->sizenum + 1 , sizeof(double));

		for(int i=0;i<Fit->sizenum;i++){
			Init_x += 0.01;
			arrayX[i] = Init_x;
			arrayKnee[i] = Fit->FitBufKnee[i];// /180*3.14
			DMA_usart2_printf("%f\r\n",Fit->FitBufKnee[i]);
		}
		DMA_usart2_printf("%d\r\n",Fit->sizenum);

//		int sizenum = sizeof(&arrayKnee)/ sizeof(arrayKnee[0]);
//		DMA_usart2_printf("%d\r\n",sizenum);

		polyfit(Fit->sizenum, arrayX, arrayKnee, Normal.ploy_n, Fit->PKneeS1);
		Normal.Flag_Div += 1;
		Normal.Flag_Send += 1;
		if(Normal.Flag_Div >= Normal.Fit_Mode){
			Normal.Flag_Div = 1;
		}
		Calculate(Normal.ploy_n, Fit->sizenum, Fit->PKneeS1, arrayX, Normal.Flag_Div);
//		DMA_usart2_printf("%f,%f,%f,%f,%f,%f\r\n",
//				Normal.PKneeS1[0],Normal.PKneeS1[1],Normal.PKneeS1[2],Normal.PKneeS1[3],Normal.PKneeS1[4],Normal.PKneeS1[5]);

		Fit->FitFlagKnee = 0;
		Fit->sizenum = 0;

		switch(Normal.Flag_Div){
		case 1:
			Fit->FitBufKnee[Fit->sizenum] = Normal.FitKnee_0[Normal.FitKnee_0_num-1];
			break;
		case 2:
			Fit->FitBufKnee[Fit->sizenum] = Normal.FitKnee_1[Normal.FitKnee_1_num-1];
			break;
		case 3:
			Fit->FitBufKnee[Fit->sizenum] = Normal.FitKnee_2[Normal.FitKnee_2_num-1];
			break;
		case 4:
			Fit->FitBufKnee[Fit->sizenum] = Normal.FitKnee_3[Normal.FitKnee_3_num-1];
			break;
		default:
			break;
		}
		Fit->sizenum++;

		free(arrayKnee);
		free(arrayX);
	}
}

/********************************实现函数**************************************
*函数原型:	void DataDiv_2
*功　　能:	数据分割便于拟合
*修改日期:	20231201
 * 参数		| 介绍
 * ---------+--------------------------------------
 * Fit		| 数据储存位置
*******************************************************************************/
void DataDiv_2(struct DataFit *Fit){
	if(Fit->FitFlagKnee == 0){
		Fit->FitBufKnee[Fit->sizenum] = Right.Knee.AngxCal;
		Fit->FitBufFoot_12[Fit->sizenum] = DataRightBufFoot.Data[12];
		Fit->sizenum++;

		if(DataRightBufFoot.Data[12] > 1500){
			if(Fit->sizenum > 2 && Fit->FitBufFoot_12[Fit->sizenum] - Fit->FitBufFoot_12[Fit->sizenum - 1] <= 0){
				if(Fit->Flag_Div == 4){
					Fit->Flag_Div = 1;
					Fit->FitFlagKnee = 1;
				}
				else if(Fit->Flag_Div == 0){
					Fit->Flag_Div = 1;
					Fit->sizenum = 0;
				}
			}
		}

		else if(Right.Knee.AngxCal < -40){
			if(Fit->sizenum > 2 && Fit->FitBufKnee[Fit->sizenum] - Fit->FitBufKnee[Fit->sizenum - 1] >= 0){
				if(Fit->Flag_Div == 1){
					Fit->Flag_Div = 2;
					Fit->FitFlagKnee = 1;
				}
				else if(Fit->Flag_Div != 1){

				}
			}
		}

		else if(DataRightBufFoot.Data[3] > 400 && DataRightBufFoot.Data[4] > 400){
			if(Fit->Flag_Div == 2){
				Fit->Flag_Div = 3;
				Fit->FitFlagKnee = 1;
			}
			if(Fit->Flag_Div != 2){

			}
		}

		else if(DataRightBufFoot.Data[12] > 600){
			if(Fit->Flag_Div == 3){
				Fit->Flag_Div = 4;
				Fit->FitFlagKnee = 1;
			}
			if(Fit->Flag_Div != 3){

			}
		}


	}
	else if(Fit->FitFlagKnee == 1){
		//内存分配方式1
		double *arrayKnee = createArray(Fit->sizenum);
		double *arrayX = createArray(Fit->sizenum);
		double Init_x = 0;

		//内存分配方式2
//		double *arrayKnee = (double *)calloc(Fit->sizenum + 1 , sizeof(double));
//		double *arrayX = (double *)calloc(Fit->sizenum + 1 , sizeof(double));

		for(int i=0;i<Fit->sizenum;i++){
			Init_x += 0.01;
			arrayX[i] = Init_x;
			arrayKnee[i] = Fit->FitBufKnee[i];// /180*3.14
			DMA_usart2_printf("%f\r\n",Fit->FitBufKnee[i]);
		}
		DMA_usart2_printf("%d\r\n",Fit->sizenum);

//		int sizenum = sizeof(&arrayKnee)/ sizeof(arrayKnee[0]);
//		DMA_usart2_printf("%d\r\n",sizenum);

		polyfit(Fit->sizenum, arrayX, arrayKnee, Normal.ploy_n, Fit->PKneeS1);
		Normal.Flag_Send += 1;
//		if(Normal.Flag_Div >= Normal.Fit_Mode){
//			Normal.Flag_Div = 1;
//		}

		Calculate(Normal.ploy_n, Fit->sizenum, Fit->PKneeS1, arrayX, Normal.Flag_Div);
//		DMA_usart2_printf("%f,%f,%f,%f,%f,%f\r\n",
//				Normal.PKneeS1[0],Normal.PKneeS1[1],Normal.PKneeS1[2],Normal.PKneeS1[3],Normal.PKneeS1[4],Normal.PKneeS1[5]);

		Fit->FitFlagKnee = 0;
		Fit->sizenum = 0;

		switch(Normal.Flag_Div){
		case 1:
			Fit->FitBufKnee[Fit->sizenum] = Normal.FitKnee_0[Normal.FitKnee_0_num-1];
			break;
		case 2:
			Fit->FitBufKnee[Fit->sizenum] = Normal.FitKnee_1[Normal.FitKnee_1_num-1];
			break;
		case 3:
			Fit->FitBufKnee[Fit->sizenum] = Normal.FitKnee_2[Normal.FitKnee_2_num-1];
			break;
		case 4:
			Fit->FitBufKnee[Fit->sizenum] = Normal.FitKnee_3[Normal.FitKnee_3_num-1];
			break;
		default:
			break;
		}
		Fit->sizenum++;

		free(arrayKnee);
		free(arrayX);
	}
}


/********************************实现函数**************************************
*函数原型:	void polyfit(n,x,y,poly_n,a)
*功　　能:	拟合y=a0+a1*x+a2*x^2+……+apoly_n*x^poly_n
*修改日期:	20230526
 * 参数		| 介绍
 * ---------+--------------------------------------
 * n		| 数据个数
 * x		| 自变量
 * y		| 因变量
 * poly_n	| 多项式项数
 * p		| 系数
*******************************************************************************/
void polyfit(int n,double x[],double y[],int poly_n,double p[])
{
	int i,j;
	double *tempx,*tempy,*sumxx,*sumxy,*ata;

	tempx = (double *)calloc(n , sizeof(double));
	sumxx = (double *)calloc((poly_n*2+1) , sizeof(double));
	tempy = (double *)calloc(n , sizeof(double));
	sumxy = (double *)calloc((poly_n+1) , sizeof(double));
	ata = (double *)calloc( (poly_n+1)*(poly_n+1) , sizeof(double) );
	for (i=0;i<n;i++)
	{
		tempx[i]=1;
		tempy[i]=y[i];
	}
	for (i=0;i<2*poly_n+1;i++)
	{
		for (sumxx[i]=0,j=0;j<n;j++)
		{
			sumxx[i]+=tempx[j];
			tempx[j]*=x[j];
		}
	}

	for (i=0;i<poly_n+1;i++)
	{
		for (sumxy[i]=0,j=0;j<n;j++)
		{
			sumxy[i]+=tempy[j];
			tempy[j]*=x[j];
		}
	}

	for (i=0;i<poly_n+1;i++)
	{
		for (j=0;j<poly_n+1;j++)
		{
			ata[i*(poly_n+1)+j]=sumxx[i+j];
		}
	}
	gauss_solve(poly_n+1,ata,p,sumxy);

	free(tempx);
	free(sumxx);
	free(tempy);
	free(sumxy);
	free(ata);
}

/********************************实现函数**************************************
*函数原型:	void gauss_solve(int n,double A[],double x[],double b[])
*功　　能:	高斯消元法计算得到 n 次多项式的系数
*修改日期:	20230526
 * 参数		| 介绍
 * ---------+--------------------------------------
 * n		| 系数个数
 * A		| 线性矩阵
 * x		| 拟合结果
 * b		| 线性方程组的y值
*******************************************************************************/
void gauss_solve(int n,double A[],double x[],double b[])
{
	int i,j,k,r;
	double max;
	for (k=0;k<n-1;k++)
	{
		max=fabs(A[k*n+k]);					// find maxmum
		r=k;
		for (i=k+1;i<n-1;i++)
		{
			if (max<fabs(A[i*n+i]))
			{
				max=fabs(A[i*n+i]);
				r=i;
			}
		}
		if (r!=k)
		{
			for (i=0;i<n;i++)		//change array:A[k]&A[r]
			{
				max=A[k*n+i];
				A[k*n+i]=A[r*n+i];
				A[r*n+i]=max;
			}

			max=b[k];                    //change array:b[k]&b[r]
			b[k]=b[r];
			b[r]=max;
		}

		for (i=k+1;i<n;i++)
		{
			for (j=k+1;j<n;j++)
				A[i*n+j]-=A[i*n+k]*A[k*n+j]/A[k*n+k];
			b[i]-=A[i*n+k]*b[k]/A[k*n+k];
		}
	}

	for (i=n-1;i>=0;x[i]/=A[i*n+i],i--)
	{
		for (j=i+1,x[i]=b[i];j<n;j++)
			x[i]-=A[i*n+j]*x[j];
	}
}

/********************************实现函数**************************************
*函数原型:	void Calculate(int poly_n, int n, double p[], double x[], int Flag)
*功　　能:	计算拟合函数值
*修改日期:	20231207
 * 参数		| 介绍
 * ---------+--------------------------------------
 * 无		| 无
*******************************************************************************/
void Calculate(int poly_n, int n, double p[], double x[], int Flag){
	double *q = createArray(poly_n);
	Slop(poly_n, p, q);
	switch (Flag){
	case 1:
		Normal.FitKnee_0_num = n;
		for(int i=0; i<n; i++){
			Normal.FitKnee_0[i] = Horner_Algorithm(poly_n,p,x[i]);
			Normal.FitKneeT_0[i] = Horner_Algorithm(poly_n-1,q,x[i]);
			DMA_usart2_printf("%f\r\n",Normal.FitKnee_0[i]);
		}
		break;
	case 2:
		Normal.FitKnee_1_num = n;
		for(int i=0; i<n; i++){
			Normal.FitKnee_1[i] = Horner_Algorithm(poly_n,p,x[i]);
			Normal.FitKneeT_1[i] = Horner_Algorithm(poly_n-1,q,x[i]);
			DMA_usart2_printf("%f\r\n",Normal.FitKnee_1[i]);
		}
		break;
	case 3:
		Normal.FitKnee_2_num = n;
		for(int i=0; i<n; i++){
			Normal.FitKnee_2[i] = Horner_Algorithm(poly_n,p,x[i]);
			Normal.FitKneeT_2[i] = Horner_Algorithm(poly_n-1,q,x[i]);
			DMA_usart2_printf("%f\r\n",Normal.FitKnee_2[i]);
		}
		break;
	case 4:
		Normal.FitKnee_3_num = n;
		for(int i=0; i<n; i++){
			Normal.FitKnee_3[i] = Horner_Algorithm(poly_n,p,x[i]);
			Normal.FitKneeT_3[i] = Horner_Algorithm(poly_n-1,q,x[i]);
			DMA_usart2_printf("%f\r\n",Normal.FitKnee_3[i]);
		}
		break;
	default:
		break;
	}
	DMA_usart2_printf("%d\r\n",Flag);
	free(q);
}

/********************************实现函数**************************************
*函数原型:	double Horner_Algorithm(int poly_n, double p[], double x)
*功　　能:	秦九韶算法
*修改日期:	20231207
 * 参数		| 介绍
 * ---------+--------------------------------------
 * 无		| 无
*******************************************************************************/
double Horner_Algorithm(int poly_n, double p[], double x){
	double y = 0.0;	//存放多项式的值
	for (int j = poly_n; j>=0; j--)
	{
		y = (x * y) + p[j];	//计算多项式的值。
	}
	return y;
}

/********************************实现函数**************************************
*函数原型:	void Slop(int poly_n, double p[], double q[])
*功　　能:	计算斜率多项式系数
*修改日期:	20231215
 * 参数		| 介绍
 * ---------+--------------------------------------
 * 无		| 无
*******************************************************************************/
void Slop(int poly_n, double p[], double q[]){
	for(int i = 1; i<poly_n+1; i++){
		q[i-1] = i*p[i];
	}
}

/********************************实现函数**************************************
*函数原型:	void ModbusRead()
*功　　能:	读取距离传感器数值
*修改日期:	20231010
 * 参数		| 介绍
 * ---------+--------------------------------------
 * 无		| 无
*******************************************************************************/
void ModbusRead(){
	uint8_t ModbusData[] = {0x50,0x03,0x00,0x34,0x00,0x01,0xc8,0x45};
	for(int i = 0; i < 7; i++){
		huart1_printf("%x",ModbusData[i]);
	}
}

/********************************实现函数**************************************
*函数原型:	void ModbusRead()
*功　　能:	计算ModbusCRC校验值
*修改日期:	20231012
 * 参数		| 介绍
 * ---------+--------------------------------------
 * 无		| 无
*******************************************************************************/
uint16_t crc16_modbus(uint8_t *data, uint16_t length) {
    uint16_t crc = 0xFFFF;
    uint16_t i, j;
    for (i = 0; i < length; i++) {
        crc ^= data[i];
        for (j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc >>= 1;
                crc ^= 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

/********************************实现函数**************************************
*函数原型:	low_pass_filter_init()
*功　　能:	滤波器初始化
*修改日期:	20231129
 * 参数		| 介绍
 * ---------+--------------------------------------
 * 无		| 无
*******************************************************************************/
void low_pass_filter_init(void){
    float b = 2.0 * pi * fc * Ts;
    alpha = b / (b + 1);
}

/********************************实现函数**************************************
*函数原型:	low_pass_filter()
*功　　能:	滤波器
*修改日期:	20231129
 * 参数		| 介绍
 * ---------+--------------------------------------
* value		| 值
*******************************************************************************/
float low_pass_filter(float value){
    static float out_last = 0; //上一次滤波值
    float out;

  /***************** 如果第一次进入，则给 out_last 赋值 ******************/
    static char fisrt_flag = 1;
    if (fisrt_flag == 1){
        fisrt_flag = 0;
        out_last = value;
    }

  /*************************** 一阶滤波 *********************************/
    out = out_last + alpha * (value - out_last);
    out_last = out;

    return out;
}

#pragma endregion

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
