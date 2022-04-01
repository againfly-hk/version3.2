/*
 * @Author: AFShk
 * @Date: 2022-03-31 14:06:35
 * @LastEditors: AFShk
 * @LastEditTime: 2022-03-31 20:09:47
 * @FilePath: \VERSION3_2\SINGAL\openmv_communicate.c
 * @Description: 
 * 
 * Copyright (c) 2022 by 用户/公司名, All Rights Reserved. 
 */
#include "openmv_communicate.h"
#include "main.h"
#include "can.h"
extern CAN_HandleTypeDef hcan2;
static CAN_TxHeaderTypeDef opemmv_tx_message;
Openmv_Struct Openmv_TX,Openmv_RX;

void Openmv_Data_Transmit(Openmv_Struct* openmv)
{
	uint32_t send_mail_box;
	uint8_t data_tx[8];
	opemmv_tx_message.StdId = 0x101;
	opemmv_tx_message.IDE = CAN_ID_STD;
	opemmv_tx_message.RTR = CAN_RTR_DATA;
	opemmv_tx_message.DLC = 0x08;
	data_tx[0]=Openmv_TX.mode;
	data_tx[1]=Openmv_TX.data1;
	data_tx[2]=Openmv_TX.data2;
	data_tx[3]=Openmv_TX.data2>>8;
	data_tx[4]=Openmv_TX.data3;
	data_tx[5]=Openmv_TX.data3>>8;
	data_tx[6]=Openmv_TX.data4;
	data_tx[7]=Openmv_TX.data5;
	HAL_CAN_AddTxMessage(&hcan2,&opemmv_tx_message,data_tx,&send_mail_box);
}

void can2_filter_init(void)
{
		CAN_FilterTypeDef can_filter_st;
		can_filter_st.FilterActivation = ENABLE;
		can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
		can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
		can_filter_st.FilterIdHigh = 0x0000;
		can_filter_st.FilterIdLow = 0x0000;
		can_filter_st.FilterMaskIdHigh = 0x0000;
		can_filter_st.FilterMaskIdLow = 0x0000;
		can_filter_st.FilterBank = 14;
		can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
		can_filter_st.SlaveStartFilterBank = 14;
		HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
}

