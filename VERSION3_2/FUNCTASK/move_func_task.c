/*
 * @Author: AFShk
 * @Date: 2022-03-03 22:49:55
 * @LastEditors: AFShk
 * @LastEditTime: 2022-03-11 19:53:40
 * @FilePath: \RC2022\Function_Task\move_func_task.c
 * @Description: 
 * 
 * Copyright (c) 2022 by 用户/公司名, All Rights Reserved. 
 */
 
 //这里是控制的解耦电机的程序
#include "main.h"
#include "pid.h"
#include "CAN_receive.h"
#include "move_func_task.h"

extern motor_measure_t motor_chassis[7];
extern uint8_t move_angel_flag;
extern code motor_3508[4];
extern uint8_t no_motor_flag;
pid_type_def motor_move_speed_pid[4];
CAR car;
float angleK=300;
float angle_limit=30;
float angle_delta=0.0f;

void coder_mover_calc(uint8_t num)
{
//	car.carx=motor_3508[1].cnt
}
void move_pid_init()
{
	float speed_pid[3]={3.8, 1, 10};
	PID_init(&motor_move_speed_pid[0],PID_POSITION,speed_pid,6000,2000);
	PID_init(&motor_move_speed_pid[1],PID_POSITION,speed_pid,6000,2000);
	PID_init(&motor_move_speed_pid[2],PID_POSITION,speed_pid,6000,2000);
	PID_init(&motor_move_speed_pid[3],PID_POSITION,speed_pid,6000,2000);	
}

void move_pid_calc(void)
{
//	if(move_angel_flag==1){
//		angle_delta=(car.begin_yaw-car.yaw)*angleK;
//		if(angle_delta>angle_limit)	  angle_delta=angle_limit;
//			if(angle_delta<-angle_limit)	angle_delta=-angle_limit;
//	}
		car.v1=(-1*(1*car.vx+1*car.vy+28*car.w+angle_delta)*23.7946f);//cm/s->rpm
		car.v2=(-1*(-1*car.vx+1*car.vy+28*car.w+angle_delta)*23.7946f);//cm
		car.v3=(-1*(-1*car.vx-1*car.vy+28*car.w+angle_delta)*23.7946f);
		car.v4=(-1*(1*car.vx+-1*car.vy+28*car.w+angle_delta)*23.7946f);
		PID_calc(&motor_move_speed_pid[0],motor_chassis[0].speed_rpm,car.v1);
		PID_calc(&motor_move_speed_pid[1],motor_chassis[1].speed_rpm,car.v2);
		PID_calc(&motor_move_speed_pid[2],motor_chassis[2].speed_rpm,car.v3);
		PID_calc(&motor_move_speed_pid[3],motor_chassis[3].speed_rpm,car.v4);
		if(no_motor_flag==0)
			CAN_cmd_chassis(motor_move_speed_pid[0].out,motor_move_speed_pid[1].out,motor_move_speed_pid[2].out,motor_move_speed_pid[3].out);	
		else if(no_motor_flag==1)
			CAN_cmd_chassis(0,0,0,0);
}

//void codemove_init()
//{
//	#ifdef speed_control_using
//  float codemove_pid[3]={0.002,0.000,0};
//	PID_init(&motor_move_displace_pid[0],PID_POSITION,codemove_pid,20,5);
//	PID_init(&motor_move_displace_pid[1],PID_POSITION,codemove_pid,20,5);
//	PID_init(&motor_move_displace_pid[2],PID_POSITION,codemove_pid,20,5);
//	PID_init(&motor_move_displace_pid[3],PID_POSITION,codemove_pid,20,5);
//	#endif
//
//	float speed_pid[3]={3.8,1,10};
//	PID_init(&motor_move_speed_pid[0],PID_POSITION,speed_pid,6000,2000);
//	PID_init(&motor_move_speed_pid[1],PID_POSITION,speed_pid,6000,2000);
//	PID_init(&motor_move_speed_pid[2],PID_POSITION,speed_pid,6000,2000);
//	PID_init(&motor_move_speed_pid[3],PID_POSITION,speed_pid,6000,2000);
//
//	#ifdef yaw_control
//	float yaw_pid[3]={0.02,,0,0};
//	#endif
//}
