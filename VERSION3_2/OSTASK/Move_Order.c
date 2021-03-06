/*
 * @Author: AFShk
 * @Date: 2022-03-06 00:51:37
 * @LastEditors: AFShk
 * @LastEditTime: 2022-03-31 20:03:14
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
#include "bmi088.h"
#include "frame.h"
#include "pid.h"
#include "callback.h"
#include "struct_typedef.h"
#include "Buzzer.h"
#include "condition.h"
#include "move_func_task.h"
#include "math.h"
#include "stdlib.h"
#include "arm_math.h"
//testing private
#ifdef move_yaw_using
uint8_t move_angle_flag=0;
#endif
uint8_t order_flag=0;
order car_order;//移动的命令
extern uint8_t temp_flag;
extern uint8_t gyro_flag;
extern float quat[];
extern float mag[];
extern float gyro_erro[3];
extern float accel_errodata[3];
extern bmi088_real_data_t	imu_real_data;
extern CAR car;
extern uint8_t accel_fliter_flag;
extern int frame_high;
extern int frame_change[2];
extern uint8_t no_motor_flag;
extern uint8_t sensor_flag;

pid_type_def move_pid;
volatile uint16_t order_cnt=0;
volatile uint8_t order_complete=0;
volatile order chassis_order[100]={
//这里是控制龙门架高度的调试部分,大小为5
111		,0		,100000 	,0		,\
333		,0		,500		,0		,\
111		,0		,300000  	,0 		,\
333		,0		,500		,0		,\
111		,0		,200000		,0		,\
//

};
volatile uint16_t order_max=5;
/*
typedef struct{
	int16_t vector_x;
	int16_t vector_y;
	int16_t expect_speed;
	int16_t expext_distance;
}order;
//if vector_x =111,龙门架抬升，speed——flag代表高度
//if vector_x=222,车旋转，speed为旋转角度
//if vector_x=255,听从openmv的命令
//if vector_x=333,delay时间
*/

void OrderTask(void const * argument){
	Buzzer_PlayMusic(Music_Dayu);//播放音乐
	while(temp_flag==0&&sensor_flag==0)	{osDelay(1);}
	TIM5->CCR3=0;
	TIM5->CCR2=500;
	for(uint16_t i=0;i<2000;i++){
		for(uint8_t j=0;j<3;j++){
			gyro_erro[j]+=imu_real_data.gyro[j];
			accel_errodata[j]+=imu_real_data.accel[j];
		}
		osDelay(3);
	}
	for(uint8_t i=0;i<3;i++){
		gyro_erro[i]=gyro_erro[i]/2000.0f;
		accel_errodata[i]=accel_errodata[i]/2000.0f;
	}
	osDelay(5);
	gyro_flag=1;
	osDelay(1);//初始化四元数
	gyro_flag=2;
	osDelay(50);//开始解算四元数
	
	car.begin_yaw=get_yaw(quat)*180.0f/3.1415926f;//以当前yaw作为初始角度，上电时一定要记住放好
	car.begin_ryaw=car.begin_yaw;				//其实后面还可以加一个自动变正的代码
	
	float pid[3]={0.0f ,0.0f ,0.0f};
	PID_init(&move_pid ,PID_POSITION ,pid ,200 ,50);//这个用来解算速度，速度向量还是决定方向，暂时没有方向修正的功能
	#ifdef move_yaw_using
		move_angle_flag=1;
	#endif
	#ifndef sbus_using
		HAL_TIM_Base_Start_IT(&htim7);
  #endif
	order_flag=1;
	osDelay(2);
  for(;;){
		//现在有的数据：红外数据、超声波数据、电机编码器、车的姿态
		//超声波数据暂时不使用
		//龙门架高度作为一个单独的程序控制
		//编码器用来控制运动轨迹，使用和速度一样的解算方式
		//需要有移动距离的pid，转动角度的pid，
		//这里面执行相关的order
		if(order_cnt<order_max){
			if((chassis_order[order_cnt].vector_x!=111)\
				&&(chassis_order[order_cnt].vector_x!=222)\
				&&(chassis_order[order_cnt].vector_x!=255)){//位移模式
				move_pid.max_out=chassis_order[order_cnt].expect_speed;
				while(order_complete==0){

				}
			}
			else if(chassis_order[order_cnt].vector_x==111){//if vector_x =111,龙门架抬升，speed为高度
				frame_high=chassis_order[order_cnt].expect_speed;
				while(order_complete==0){
					if(abs(frame_high-frame_change[0])<10000)
						order_complete=1;
					osDelay(10);
				}
			}//这里来控制龙门架高度；
			else if(chassis_order[order_cnt].vector_x==222){//if vector_x=222,车旋转，speed为旋转角度
				car.begin_yaw=angle_calc(car.begin_yaw,chassis_order[order_cnt].expect_speed);
				while(order_complete==0){
					if(fabs(car.yaw-car.begin_yaw)<1.0f)
						order_complete=1;
					osDelay(10);
				}
			}//这里用来控制车的yaw的角度
			else if(chassis_order[order_cnt].vector_x==255){//if vector_x=255,听从openmv的命令
				while(order_complete==0){
					//
				}
			}
			order_complete=0;
			order_cnt++;
			//osDelay(10);
		}
		else{
			car.vx=0;
			car.vy=0;
			car.w2=0;
			no_motor_flag=1;
			osDelay(10);
		}
    }
}
//现在使用定时器来进行任务的解算，计算能力完全够
//这里面进行任务的转换；


