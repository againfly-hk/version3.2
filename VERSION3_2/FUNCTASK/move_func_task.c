/*
 * @Author: AFShk
 * @Date: 2022-03-03 22:49:55
 * @LastEditors: AFShk
 * @LastEditTime: 2022-03-29 12:56:47
 * @FilePath: \VERSION3_2\FUNCTASK\move_func_task.c
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
#include "condition.h"
#include "math.h"

//extern
extern motor_measure_t motor_chassis[7];//4+2
extern uint8_t move_angle_flag;
extern code motor_3508[4];
extern uint8_t no_motor_flag;

uint8_t angle_ok;
volatile CAR car;
float angleK=300;
float angle_limit=30;
float angle_delta=0.0f;
float yaw_debuff=0.0005f;
pid_type_def yaw_angle_pid;
pid_type_def motor_move_speed_pid[4];

int16_t int16_abs(int16_t num);

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
	#ifdef move_yaw_using
    	angle_pid_init();
	#endif
}

void move_pid_calc(void)//
{
	#ifdef move_yaw_using
    	angle_pid_calc(car.begin_yaw ,car.yaw);
	#endif
	float carw=28*car.w1+car.w2;
	car.v1=(-1*(1*car.vx+1*car.vy-carw)*23.7946f);//cm/s->rpm
	car.v2=(-1*(-1*car.vx+1*car.vy-carw)*23.7946f);//cm
	car.v3=(-1*(-1*car.vx-1*car.vy-carw)*23.7946f);
	car.v4=(-1*(1*car.vx+-1*car.vy-carw)*23.7946f);
	PID_calc(&motor_move_speed_pid[0],motor_chassis[0].speed_rpm,car.v1);
	PID_calc(&motor_move_speed_pid[1],motor_chassis[1].speed_rpm,car.v2);
	PID_calc(&motor_move_speed_pid[2],motor_chassis[2].speed_rpm,car.v3);
	PID_calc(&motor_move_speed_pid[3],motor_chassis[3].speed_rpm,car.v4);

	if(no_motor_flag==0)
		CAN_cmd_chassis(motor_move_speed_pid[0].out,motor_move_speed_pid[1].out,motor_move_speed_pid[2].out,motor_move_speed_pid[3].out);	
	else if(no_motor_flag==1)
		CAN_cmd_chassis(0,0,0,0);
}
float angle_calc(float begin ,float add)//角度和添加的角度
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
	float pid[3]={5 ,0 ,0};
	PID_init(&yaw_angle_pid ,PID_POSITION ,pid ,200 ,100);
}

void angle_pid_calc(float order ,float now)//yaw的pid解析
{
	#ifdef sbus_using
		if(car.w1!=0){
			car.w2=0;
			car.begin_yaw=car.yaw;
			reurn;
		}
	#endif
	float delta;
	if(car.w1==0)
	{

		if(order > 90&&now < -90)
			delta = -(360+now-order);
		else if(order < -90&&now > -90)
			delta = (360-now+order);
		else
			delta = order-now;
		//解算yaw
		yaw_angle_pid.Iout=0;
		car.w2=PID_calc(&yaw_angle_pid, delta ,0);
	}
	if(fabs(car.w2)<0.05f)//滤去小幅度
		car.w2=0;
	if(fabs(delta)<2.0f)
		angle_ok=1;
	else
		angle_ok=0;
	if(fabs(order-now)<25.0f&&fabs(order-now)>8.0f)
	{
		if(car.w2>0)
			car.w2-=int16_abs(motor_chassis[0].speed_rpm)*yaw_debuff;
		else if(car.w2<0)
			car.w2+=int16_abs(motor_chassis[0].speed_rpm)*yaw_debuff;
	}
}

int16_t int16_abs(int16_t num)
{
	if(num<0)
		return -num;
	else
		return num;
}
//void coder_postion_calc(void)
//{
//	motor_coder_record[0]+=motor_3508[0].change;
//	motor_coder_record[1]+=motor_3508[1].change;
//	motor_coder_record[2]+=motor_3508[2].change;
//	motor_coder_record[3]+=motor_3508[3].change;
//}

/*
机赛车的底盘：
1     0
2     3
用速度矩阵和姿态矩阵解算解算位移
AFSpace_explorer
*/

