/*
 * @Author: your name
 * @Date: 2022-03-03 09:56:33
 * @LastEditTime: 2022-03-06 12:39:36
 * @LastEditors: Please set LastEditors
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: \RC2022\Os_Task\line_detect.c
 */
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "line_detect.h"
#include "usart.h"
#include "sbus.h"
#include "can.h"
#include "frame.h"

#include "Data_Processing.h"
#include "struct_typedef.h"
#include "CAN_receive.h"
#include "move_func_task.h"
#include "Buzzer.h"
#include "door_control.h"

#include "ist8310.h"
#include "bmi088_driver.h"

#define sbus_using

//这里里面主要是用遥控器控制的代码
extern uint8_t rec_sbus_data[];
extern uint8_t sbus_erroflag;
extern uint16_t sbus_channel[16];//sbus data
extern CAR car;
extern int frame_high;

uint8_t no_motor_flag=1;
uint8_t rx_line_buff[5];
uint8_t rx_echo_buff[2];
int rx_echo;
float accel_fliter1[3]={0.0f ,0.0f ,0.0f};
float accel_fliter2[3]={0.0f ,0.0f ,0.0f};
float accel_fliter3[3]={0.0f ,0.0f ,0.0f};

void sbus_order(void);//函数的声明
void line_detect_init(void);//初始化配置函数
	

void line_detect_task(void const * argument)
{
	line_detect_init();
	while(1){
		sbus_order();
		move_pid_calc();
		osDelay(3);
	}
}

void line_detect_init(void)//初始化配置函数
{
	while(ist8310_init())	{osDelay(2);}
	while(BMI088_init())	{osDelay(2);}
	door_left();
	right_door_on();
	left_door_on();
	front_door_down();
	back_door_down();
	door_reset();
	osDelay(5);
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
	__HAL_UART_ENABLE_IT(&huart6,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1,rx_line_buff,5);
	HAL_UART_Receive_DMA(&huart3,rec_sbus_data,25);
	HAL_UART_Receive_DMA(&huart6,rx_echo_buff,2);
	frame_pid_init();
	move_pid_init();
	HAL_CAN_Start(&hcan1);
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	#ifndef sbus_using
		no_motor_flag=0;
	#endif
}
	
void sbus_order(void)//遥控器数据处理函数
{
	#ifdef sbus_using
	if(sbus_channel[7]>100&&sbus_channel[7]<600)
		no_motor_flag=0;
	else
		no_motor_flag=1;//安全模式
	
	if(no_motor_flag==0&&sbus_channel[4]<1500&&sbus_channel[5]>100){//速度的解算
		if(sbus_channel[5]<1500)	frame_high=(int)match(400,1700,310000,0,sbus_channel[2]);
		if(sbus_channel[1]>=1100)
			car.vx=(sbus_channel[1]-1024)/5.0f;
		else if(sbus_channel[1]<=950)
			car.vx=-(1024-sbus_channel[1])/5.0f;
		else car.vx=0;

		if(sbus_channel[0]>=1100)
			car.vy=-(sbus_channel[0]-1024)/10.0f;
		else if(sbus_channel[0]<=950)
			car.vy=(1024-sbus_channel[0])/10.0f;
		else car.vy=0;

		if(sbus_channel[3]>=1100)
			car.w=(sbus_channel[3]-1024)/100.0f;
		else if(sbus_channel[3]<=950)
			car.w=-(1024-sbus_channel[3])/100.0f;
		else car.w=0;
	}
	else{
		car.vx=0;
		car.vy=0;
		car.w=0;
		frame_high=0;
	}
	#endif
}
