/*
 * Kinco_can.c
 *
 *  Created on: Nov 24, 2023
 *      Author: MINI1
 */

#include "Kinco_can.h"
#include "main.h"
#include "can.h"

#include "user_config.h"


//23.05.30 WYC 初始化为速度模式
void Init_Vel_Mode(Kinco_servo * Exo,CAN_HandleTypeDef hcan){
//	uint32_t 	TxMailbox;
//
//	can_tx.data[0] = EN_MOTOR;
//	can_tx.data[1] = 0;
//	can_tx.data[2] = VEL_MODE;
//	can_tx.data[3] = 0;
//	can_tx.data[4] = 0;
//	can_tx.data[5] = 0;
//	can_tx.data[6] = 0;
//
//	can_tx.tx_header.StdId = 0x200 | Exo->device_id;
//	can_tx.tx_header.DLC = 0x07;    // 发送数据的长度 字节数
//
//	HAL_CAN_AddTxMessage(&hcan, &can_tx.tx_header, can_tx.data, &TxMailbox);	// Send response

}

//23.05.30 WYC 初始化为位置模式
void Init_Pos_Mode(Kinco_servo * Exo,CAN_HandleTypeDef hcan){

//	can_tx.data[0] = EN_MOTOR;
//	can_tx.data[1] = 0;
//	can_tx.data[2] = VEL_MODE;
//	can_tx.data[3] = 0;
//	can_tx.data[4] = 0;
//	can_tx.data[5] = 0;
//	can_tx.data[6] = 0;
//
//	can_tx.tx_header.StdId = 0x200 | Exo->device_id;
//	can_tx.tx_header.DLC = 0x07;    // 发送数据的长度 字节数
//
//	HAL_CAN_AddTxMessage(&hcan, &can_tx.tx_header, can_tx.data, &TxMailbox);	// Send response

}

//23.05.30 WYC 发送目标速度给 驱动器
void Send_Vel_Des(Kinco_servo * Exo,CAN_HandleTypeDef hcan){
	uint32_t 	TxMailbox;

	float test_vel;
//	test_vel = -100.0*K_VEL;

	test_vel = Exo->sign * Exo->target_vel_des * 60 /(_2pi * Walk_D) * K_VEL * Exo->GR;

	can_tx.data[0] = EN_MOTOR;
	can_tx.data[1] = 0;
	can_tx.data[2] = VEL_MODE;
	can_tx.data[3] = ((int)test_vel) & 0xff;
	can_tx.data[4] = (((int)test_vel) >> 8) & 0xff;
	can_tx.data[5] = (((int)test_vel) >> 16) & 0xff;
	can_tx.data[6] = (((int)test_vel) >> 24) & 0xff;

	can_tx.tx_header.StdId = 0x200 | Exo->device_id;
	can_tx.tx_header.DLC = 0x07;    // 发送数据的长度 字节数

	HAL_CAN_AddTxMessage(&hcan, &can_tx.tx_header, can_tx.data, &TxMailbox);	// Send response

}

//23.05.30 WYC 发送目标弧度给驱动器
void Send_Pos_Des(Kinco_servo * Exo,CAN_HandleTypeDef hcan){
	uint32_t 	TxMailbox;
	float test_pos;
//	test_pos = -0.0* K_POS*81.0;

	test_pos = Exo->sign * Exo->tartget_pos_des * rad2dec * K_POS * Exo->GR;

	can_tx.data[0] = ABS_POS;
	can_tx.data[1] = 0;
	can_tx.data[2] = POS_MODE;
	can_tx.data[3] = ((int)test_pos) & 0xff;
	can_tx.data[4] = (((int)test_pos) >> 8) & 0xff;
	can_tx.data[5] = (((int)test_pos) >> 16) & 0xff;
	can_tx.data[6] = (((int)test_pos) >> 24) & 0xff;

	can_tx.tx_header.StdId = 0x200 | Exo->device_id;
	can_tx.tx_header.DLC = 0x07;    // 发送数据的长度 字节数

	HAL_CAN_AddTxMessage(&hcan, &can_tx.tx_header, can_tx.data, &TxMailbox);	// Send response
	HAL_Delay(4);

	float test_vel;		//23.05.30 WYC 梯形速度值必须为正的
	test_vel = Exo->target_vel_des * K_VEL;

	//23.05.30 WYC 发送梯形速度 [能够发送控制字，但不能发送梯形速度] 需为正值

	can_tx.data[0] = ABS_POS + 0x10;
	can_tx.data[1] = 0;

	can_tx.data[2] = ((int)test_vel) & 0xff;
	can_tx.data[3] = (((int)test_vel) >> 8) & 0xff;
	can_tx.data[4] = (((int)test_vel) >> 16) & 0xff;
	can_tx.data[5] = (((int)test_vel) >> 24) & 0xff;

	can_tx.tx_header.StdId = 0x300 | Exo->device_id;
	can_tx.tx_header.DLC = 0x06;    // 发送数据的长度 字节数

	HAL_CAN_AddTxMessage(&hcan, &can_tx.tx_header, can_tx.data, &TxMailbox);	// Send response
}

void Dis_device(Kinco_servo * Exo,CAN_HandleTypeDef hcan){

	uint32_t 	TxMailbox;

	can_tx.data[0] = DIS_MOTOR;
	can_tx.data[1] = 0;
	can_tx.data[2] = POS_MODE;
	can_tx.data[3] = 0.0;
	can_tx.data[4] = 0.0;
	can_tx.data[5] = 0.0;
	can_tx.data[6] = 0.0;

	can_tx.tx_header.StdId = 0x200 | Exo->device_id;
	can_tx.tx_header.DLC = 0x07;    // 发送数据的长度 字节数

	HAL_CAN_AddTxMessage(&hcan, &can_tx.tx_header, can_tx.data, &TxMailbox);	// Send response

	HAL_Delay(4);
}

//23.06.01 WYC 对速度进行规划
void Vel_process(Kinco_servo * Exo,CAN_HandleTypeDef hcan){

	//目标速度与上一次发送给驱动器的值作差  根据差值进行速度的加减或者直接为目标速度
	if((Exo->target_vel_des - Exo->vel_des_out) >= Exo->D_vel){	//需要进行加速
		Exo->vel_des_out += Exo->DT * Exo->D_vel;
	}
	else if(((Exo->target_vel_des - Exo->vel_des_out) < Exo->D_vel) || ((Exo->target_vel_des - Exo->vel_des_out) > -Exo->D_vel)){
		Exo->vel_des_out = Exo->target_vel_des;
	}
	else if((Exo->target_vel_des - Exo->vel_des_out) <= -Exo->D_vel){
		Exo->vel_des_out -= Exo->DT * Exo->D_vel;
	}
	else{

	}
}
