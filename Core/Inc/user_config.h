/*
 * user_config.h
 *
 *  Created on: Nov 24, 2023
 *      Author: MINI1
 */

#ifndef INC_USER_CONFIG_H_
#define INC_USER_CONFIG_H_

//23.05.30 WYC 定义每一个步科驱动器的设备站号
#define Wheel1_Walking_id	1		//23.05.30 1号舵轮行走轮的设备站号
#define Wheel1_Steering_id	2

#define Wheel2_Walking_id	3		//23.05.30 2号舵轮行走轮的设备站号
#define Wheel2_Steering_id	4

#define Wheel3_Walking_id	5		//23.05.30 3号舵轮行走轮的设备站号
#define Wheel3_Steering_id	6

//23.05.30 WYC 定义每一轮的旋转方向
#define Wheel1_Walking_sign		1		//23.05.30 1号舵轮行走轮的旋转方向
#define Wheel1_Steering_sign	1

#define Wheel2_Walking_sign		1
#define Wheel2_Steering_sign	1

#define Wheel3_Walking_sign		1
#define Wheel3_Steering_sign	1

//
#define Steer_vel			100.0	//转向轮电机端的梯形速度		【rpm】

#define Walk_D			0.09	//23.05.30 WYC 行走轮的半径 【m】

#define Steer_GR		81		//23.05.30 WYC 转向轮的减速比
#define Walk_GR			50		//23.05.30 WYC 行走轮的减速比

#define MAX_VEL			0.7		//23.05.30 WYC 行走轮最大输出转速 【m/s】
#define MAX_ROTA		0.8		//23.05.31 wyc 小车自转的转速 【rad/s】

#define DVEL			0.01	//23.06.01 WYC 在立即速度模式下 进行速度规划采用的加速度 [m/ss]
#define CAL_DT			0.05	//23.06.01 WYC 小车进行计算的周期	[s]

//23.05.30 WYC 定义pi
#define _pi		3.1415926536
#define _2pi	6.2831853072

#define rad2dec	57.295779513

//23.05.30 WYC 走规定轨迹时，需要的几个时间变量
#define Task1_time		3		//23.05.30 WYC 任务1 执行的时间	[s]		前进
#define Task2_time		6		//23.05.30 WYC 任务2 执行的时间	[s]		后退
#define Task3_time		2.5		//23.05.30 WYC 任务3 执行的时间	[s]		停止下来进行调转向
#define Task4_time		3		//23.05.30 WYC 任务4 执行的时间	[s]		横移前进
#define Task5_time		6		//23.05.30 WYC 任务5 执行的时间	[s]		横移后退
#define Task6_time		2.5		//23.05.30 WYC 任务6 执行的时间	[s]		停止下来调整转向
#define Task7_time		3		//23.05.30 WYC 任务7 执行的时间	[s]		逆时针旋转
#define Task8_time		6		//23.05.30 WYC 任务8 执行的时间	[s]		瞬时针旋转
#define Task9_time		2.5		//23.05.30 WYC 任务9 执行的时间	[s]	//	回零

//23.06.01 WYC 用于测试时，走方形轨迹用




extern float 		display_vel;		//23.05.30 WYC 测试使用的转速 行走轮输出的转速  	【m/s】
extern float		display_pos;		//23.05.30 WYC 测试使用的位置 转向轮输出的角度	【rad】

extern float    display_cnt;	//23.05.30
extern float    display_cal_period;			//23.05.30 WYC [s]
extern float    display_period;	//23.05.30 WYC 设置的周期为 20【s】

//23.05.31 WYC 计算出摇杆值映射到速度的系数
extern float JOY_K_walk;		//行走轮的转速	[m/s]
extern float JOY_K_steer;		//转向轮的 【rad/s】


#endif /* INC_USER_CONFIG_H_ */
