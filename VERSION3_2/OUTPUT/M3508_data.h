/*
 * @Author: AFShk
 * @Date: 2022-03-13 21:03:28
 * @LastEditors: AFShk
 * @LastEditTime: 2022-03-28 14:57:20
 * @FilePath: \VERSION3_2\OUTPUT\M3508_data.h
 * @Description: 
 * 
 * Copyright (c) 2022 by 用户/公司名, All Rights Reserved. 
 */
#ifndef __M3508_DATA_H__
#define __M3508_DATA_H__
#include "movement_struct.h"
#include "main.h"
extern void motor_12_code(uint8_t i);
extern void motor_03_code(uint8_t i);
extern void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);

#endif

