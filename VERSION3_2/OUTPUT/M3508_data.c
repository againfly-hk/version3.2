/*
 * @Author: AFShk
 * @Date: 2022-03-13 21:03:28
 * @LastEditors: AFShk
 * @LastEditTime: 2022-03-28 15:00:42
 * @FilePath: \VERSION3_2\OUTPUT\M3508_data.c
 * @Description: 
 * 
 * Copyright (c) 2022 by 用户/公司名, All Rights Reserved. 
 */
#include "M3508_data.h"
#include "CAN_receive.h"
#include "main.h"
#include "pid.h"
#include "frame.h"
#include "Callback.h"
#include "math.h"
#include "can.h"
#include "move_func_task.h"
#include "openmv_communicate.h"
extern Openmv_Struct Openmv_TX,Openmv_RX;
//eztern
extern CAN_HandleTypeDef hcan1;
extern pid_type_def frame_pid[2];
extern int frame_high;
//declare
motor_measure_t motor_chassis[7];
code motor_3508[4];

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{   
	CAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];
	if(hcan==&hcan1){
		int8_t i;
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
    switch(rx_header.StdId)
    {
			{
        case CAN_3508_M1_ID:
        case CAN_3508_M2_ID:
        case CAN_3508_M3_ID:
        case CAN_3508_M4_ID:
        case CAN_PORTAL_FRAME_LEFT_ID:
        case CAN_PORTAL_FRAME_RIGHT_ID:
				i=rx_header.StdId-0x201;
				get_motor_measure(&motor_chassis[i],rx_data);//0,1,2,3 motor 4,5 frame
				break;
       }
       default:
       {
				 break;
       }
    }//reveive data to calculata
		if(i==4||i==5){
			if(frame_high==0)	frame_reset();
			else	frame_code_control(i);
		}
		else if((i==1)||(i==2))	motor_12_code(i);
		else if((i==0)||(i==3)) motor_03_code(i);		
	}
	else if(hcan==&hcan2){
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		if(rx_header.StdId==0x102){
			Openmv_RX.mode=rx_data[0];
			Openmv_RX.data1=rx_data[1];
			Openmv_RX.data2=rx_data[2]|rx_data[3]<<8;
			Openmv_RX.data3=rx_data[4]|rx_data[5]<<8;
			Openmv_RX.data4=rx_data[6];
			Openmv_RX.data5=rx_data[7];
		}
	}
}

void motor_12_code(uint8_t i)//this is coder motor 1,2
{
	if((motor_chassis[i].last_ecd>6000)&&(motor_chassis[i].ecd<3000))
		motor_3508[i].delta=(8192-motor_chassis[i].last_ecd)+(motor_chassis[i].ecd);
	else if((motor_chassis[i].ecd>6000)&&(motor_chassis[i].last_ecd<1000))
		motor_3508[i].delta=-((8192-motor_chassis[i].ecd)+motor_chassis[i].last_ecd);
	else
		motor_3508[i].delta=motor_chassis[i].ecd-motor_chassis[i].last_ecd;
	motor_3508[i].last_value=motor_3508[i].change;
	motor_3508[i].change+=motor_3508[i].delta;
//	if((motor_3508[i].last_value<0x7fffffff)&&(motor_3508[i].last_value>0x70000000)&&(motor_3508[i].change<0xffffff))	motor_3508[i].cnt++;
//	if((motor_3508[i].last_value<-0x7fffffff)&&(motor_3508[i].last_value>-0x70000000)&&(motor_3508[i].change>-0xffffff))	motor_3508[i].cnt--;
}

void motor_03_code(uint8_t i)//this is coder motor 0,3;
{
	if((motor_chassis[i].last_ecd>6000)&&(motor_chassis[i].ecd<3000))
		motor_3508[i].delta=(8192-motor_chassis[i].last_ecd)+(motor_chassis[i].ecd);
	else if((motor_chassis[i].ecd>6000)&&(motor_chassis[i].last_ecd<1000))
		motor_3508[i].delta=-((8192-motor_chassis[i].ecd)+motor_chassis[i].last_ecd);
	else
		motor_3508[i].delta=motor_chassis[i].ecd-motor_chassis[i].last_ecd;
	motor_3508[i].last_value=motor_3508[i].change;
	motor_3508[i].change-=motor_3508[i].delta;
}




