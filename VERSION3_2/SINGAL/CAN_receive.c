/*
 * @Author: AFShk
 * @Date: 2022-03-13 21:03:59
 * @LastEditors: AFShk
 * @LastEditTime: 2022-03-31 19:33:53
 * @FilePath: \VERSION3_2\SINGAL\CAN_receive.c
 * @Description: 
 * 
 * Copyright (c) 2022 by 用户/公司名, All Rights Reserved. 
 */
#include "CAN_receive.h"
#include "can.h"
static CAN_TxHeaderTypeDef  portal_frame_tx_message;
static uint8_t              portal_frame_can_send_data[8];
static CAN_TxHeaderTypeDef  chassis_tx_message;
static uint8_t              chassis_can_send_data[8];

void CAN_cmd_portal_frame(int16_t left, int16_t right,int16_t drawer)//发送龙门架数据
{
    uint32_t send_mail_box;
    portal_frame_tx_message.StdId = 0x1FF;
    portal_frame_tx_message.IDE = CAN_ID_STD;
    portal_frame_tx_message.RTR = CAN_RTR_DATA;
    portal_frame_tx_message.DLC = 0x08;
    portal_frame_can_send_data[0] = (left>> 8);
    portal_frame_can_send_data[1] = left;
    portal_frame_can_send_data[2] = (right >> 8);
    portal_frame_can_send_data[3] = right;
	  portal_frame_can_send_data[4] = (drawer >> 8);
    portal_frame_can_send_data[5] = drawer;
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &portal_frame_tx_message,portal_frame_can_send_data, &send_mail_box);
}

void CAN_cmd_chassis(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)//发送底盘电机数据
{
    uint32_t send_mail_box;
    chassis_tx_message.StdId = CAN_CHASSIS_ALL_ID;
    chassis_tx_message.IDE = CAN_ID_STD;
    chassis_tx_message.RTR = CAN_RTR_DATA;
    chassis_tx_message.DLC = 0x08;
    chassis_can_send_data[0] = motor1 >> 8;
    chassis_can_send_data[1] = motor1;
    chassis_can_send_data[2] = motor2 >> 8;
    chassis_can_send_data[3] = motor2;
    chassis_can_send_data[4] = motor3 >> 8;
    chassis_can_send_data[5] = motor3;
    chassis_can_send_data[6] = motor4 >> 8;
    chassis_can_send_data[7] = motor4;
    HAL_CAN_AddTxMessage(&CHASSIS_CAN, &chassis_tx_message, chassis_can_send_data, &send_mail_box);
}

