/*
 * @Author: AFShk
 * @Date: 2022-03-03 22:49:55
 * @LastEditors: AFShk
 * @LastEditTime: 2022-03-23 16:27:18
 * @FilePath: \version3.2\VERSION3_2\FUNCTASK\move_func_task.c
 * @Description: 
 * 
 * Copyright (c) 2022 by 用户/公司名, All Rights Reserved. 
 */
 
 //这里是控制的解耦电机的程序
#include "main.h"
#include "pid.h"
#include "CAN_receive.h"
#include "move_func_task.h"
#include "arm_math.h"

extern motor_measure_t motor_chassis[7];
extern uint8_t move_angel_flag;
extern code motor_3508[4];
extern uint8_t no_motor_flag;
CAR car;
float angleK=300;
float angle_limit=30;
float angle_delta=0.0f;
pid_type_def yaw_angle_pid;
pid_type_def motor_move_speed_pid[4];
float yaw_debuff=0;

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
  angle_pid_init();
}

void move_pid_calc(void)//
{
    angle_pid_calc(car.begin_yaw ,car.yaw);
		car.v1=(-1*(1*car.vx+1*car.vy-car.w)*23.7946f);//cm/s->rpm
		car.v2=(-1*(-1*car.vx+1*car.vy-car.w)*23.7946f);//cm
		car.v3=(-1*(-1*car.vx-1*car.vy-car.w)*23.7946f);
		car.v4=(-1*(1*car.vx+-1*car.vy-car.w)*23.7946f);
		PID_calc(&motor_move_speed_pid[0],motor_chassis[0].speed_rpm,car.v1);
		PID_calc(&motor_move_speed_pid[1],motor_chassis[1].speed_rpm,car.v2);		
	  PID_calc(&motor_move_speed_pid[2],motor_chassis[2].speed_rpm,car.v3);
		PID_calc(&motor_move_speed_pid[3],motor_chassis[3].speed_rpm,car.v4);
		if(no_motor_flag==0)
			CAN_cmd_chassis(motor_move_speed_pid[0].out,motor_move_speed_pid[1].out,motor_move_speed_pid[2].out,motor_move_speed_pid[3].out);	
		else if(no_motor_flag==1)
			CAN_cmd_chassis(0,0,0,0);
}
float angel_calc(float begin ,float add)
{
	//一圈等于360度，分别是正180°和-180°度
	if(begin+add>=180.0f)
		return begin+add-360.0f;
	else if(begin+add<=-180.0f)
		return begin+add+360.0f;
  else
		return begin+add;
}

void angle_pid_init()//初始化yaw轴的pid
{
	float pid[3]={100 ,0 ,0};
	PID_init(&yaw_angle_pid ,PID_POSITION ,pid ,100 ,50);
}

void angle_pid_calc(float order ,float now)//yaw的pid解析
{
	float delta;
	if(order > 90&&now < -90)
		delta = -(360+now-order);
	else if(order < -90&&now > -90)
		delta = (360-now+order);
	else
		delta = order-now;
	//解算yaw
	car.w=PID_calc(&yaw_angle_pid, delta ,0);
	if(fabs(car.w)<0.08f)
		car.w=0;
	if(fabs(order-now)<15.0f&&fabs(order-now)>4.0f)
		car.w+=motor_chassis[0].speed_rpm*yaw_debuff;
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
