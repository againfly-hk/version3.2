/*
 * @Author: AFShk
 * @Date: 2022-03-06 00:51:37
 * @LastEditors: AFShk
 * @LastEditTime: 2022-03-14 21:58:26
 * @FilePath: \VERSION3_2\OSTASK\Move_Order.c
 * @Description: 
 * 
 * Copyright (c) 2022 by 用户/公司名, All Rights Reserved. 
 */

#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "line_detect.h"
#include "tim.h"
#include "can.h"
#include "AHRS_MiddleWare.h"
#include "AHRS.h"

#include "door_control.h"
#include "ist8310.h"
#include "bmi088_driver.h"
#include "frame.h"
#include "pid.h"
#include "callback.h"
#include "struct_typedef.h"
#include "Buzzer.h"

//testing private
uint8_t move_angel_flag=0;

extern uint8_t accel_flag;
extern uint8_t temp_flag;
extern uint8_t gyro_flag;
extern float quat[];
extern float mag[];
extern float gyro_erro[3];
extern float accel_errodata[3];
extern bmi088_real_data_t	imu_real_data;
extern CAR car;
extern uint8_t accel_fliter_flag;


void OrderTask(void const * argument){
	Buzzer_PlayMusic(Music_Dayu);//播放音乐
	while(temp_flag==0){
		osDelay(1);
	}
	osDelay(10);
	for(uint16_t i=0;i<100;i++){
		for(uint8_t j=0;j<3;j++){
			gyro_erro[i]+=imu_real_data.gyro[i];
			accel_errodata[i]+=imu_real_data.accel[i];
		}
		osDelay(10);
	}
	for(uint8_t i=0;i<3;i++){
		gyro_erro[i]=gyro_erro[i]/100.0f;
		accel_errodata[i]=accel_errodata[i]/100.0f;
	}
	gyro_flag=1;
	osDelay(10);
	gyro_flag=2;
	osDelay(10);
	while(accel_fliter_flag!=2){
		osDelay(5);
	}
	osDelay(50);
	AHRS_init(quat, car.raccel, mag);//初始化quat，姿态解算完成
	osDelay(25);
	car.begin_yaw=get_yaw(quat)*180.0f/3.1415926f;
	move_angel_flag=1;
    for(;;){
		//现在有的数据：红外数据、超声波数据、电机编码器、车的姿态
		//超声波数据暂时不使用
		//龙门架高度作为一个单独的程序控制
		//编码器用来控制运动轨迹，使用和速度一样的解算方式
		
        osDelay(10);
    }
}
