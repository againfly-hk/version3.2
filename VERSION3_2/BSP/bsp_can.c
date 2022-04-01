/*
 * @Author: AFShk
 * @Date: 2022-03-13 21:03:09
 * @LastEditors: AFShk
 * @LastEditTime: 2022-03-31 14:09:43
 * @FilePath: \VERSION3_2d:\FILE\RC\RC2022\version3.2\VERSION3_2\BSP\bsp_can.c
 * @Description: 
 * 
 * Copyright (c) 2022 by 用户/公司名, All Rights Reserved. 
 */
#include "bsp_can.h"
#include "main.h"

extern CAN_HandleTypeDef hcan1;

void can1_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
}


