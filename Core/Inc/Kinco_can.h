/*
 * Kinco_can.h
 *
 *  Created on: Nov 24, 2023
 *      Author: MINI1
 */

#ifndef INC_KINCO_CAN_H_
#define INC_KINCO_CAN_H_

#include "main.h"
/* 	23.05.29 WYC 参考刘国才老师提供的程序进行改变的临时使用的库
 * 				 在上位机处 CANID即为步科驱动器中的PDO站号
 * 				 CAN0-CAN7中包含的数据，即为pdo1映射的CANopen地址所代表的数据
 * */

//23.05.29 WYC 此处的结构体，需要和步科驱动器中的设置相对应

#define K_VEL	4474		//23.05.30 wyc 此处C语言发送到驱动器处时，所要乘的系数   SEND_VEL = VEL_DES * K_VEL
#define K_POS   46		//23.05.30 wyc 此处为位置控制所需要乘的系数

//23.05.30 WYC 控制字所对应的功能
typedef enum{

    DIS_MOTOR 	= 0X06,		//电机断电
    EN_MOTOR 	= 0X0F,		//电机上电
    FAST_STOP	= 0X0B,		//快速停止
	ABS_POS		= 0X2F,		//绝对定位方式
	R_POS		= 0X4F,		//相对定位方式
	IM_POS		= 0X103F,	//立即绝对位置
	OAR_POS		= 0X1F,		//原点定位
	CLR_ERR		= 0X86,		//清除内部故障

}Control_Word;

//23.05.30 WYC 工作模式
typedef enum{

	POS_MODE = 1,					//位置模式
	VEL_MODE = 3,					//速度模式
	TOR_MODE = 4,					//力矩模式
	IM_POS_MODE = -3,				//立即速度模式
	PULSE_MODE = -4,				//脉冲模式
	FIND_ZERO_MODE = 6,				//找原点模式
	INTERPOLATION_MOTION_MODE = 7,	//运动插补模式

}Work_Mode;

typedef struct{

	uint8_t 	device_id;			//23.05.30 WYC 设备站号
	uint16_t	GR;				//减速比		//23.06.01 WYC 后期应该挪到别处
	int8_t		sign;			//方向    对于转向的轮子则用于转向的方向；；对于行走的轮子用于行走的转向

	//23.05.29 WYC 发送数据给步科的 所需要的数据
	Control_Word 	control_word;		//23.05.29 WYC 控制字
	Work_Mode  		target_mode;			//23.05.29 WYC 目标模式

	float target_vel_des;		//23.05.29 WYC 在 速度模式 下的目标转动速度 [m/s] 行走轮最终输出的线速度  //若是为位置模式，则用于设置梯形速度
	float tartget_pos_des;		//23.05.29 WYC 在 位置模式 下的目标转动位置 [rad] 转向轮经过减速器后输出的角度

	float vel_des_out;			//23.06.01 WYC 对速度进行规划后 发送到驱动器的目标速度值	[m/s]

	float D_vel;				//23.06.01 WYC 速度模式下的加速度		[m/(ss)]

	float DT;					//23.06.01 WYC 驱动器的控制周期		[s]

}Kinco_servo;

//外骨骼关节
extern Kinco_servo Exoskeleton1;

void Init_Vel_Mode(Kinco_servo * Exo,CAN_HandleTypeDef hcan);	//23.05.30 WYC 初始化为速度模式
void Init_Pos_Mode(Kinco_servo * Exo,CAN_HandleTypeDef hcan);	//23.05.30 WYC 初始化为位置模式

void Send_Vel_Des(Kinco_servo * Exo,CAN_HandleTypeDef hcan);		//发送目标速度		[m/s]
void Send_Pos_Des(Kinco_servo * Exo,CAN_HandleTypeDef hcan);		//发送目标位置		[rad]

void Dis_device(Kinco_servo * Exo,CAN_HandleTypeDef hcan);

void Vel_process(Kinco_servo * Exo,CAN_HandleTypeDef hcan);		//23.06.01 WYC 对速度进行规划

#endif /* INC_KINCO_CAN_H_ */
