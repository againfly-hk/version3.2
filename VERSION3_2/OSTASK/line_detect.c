/*
 * @Author: your name
 * @Date: 2022-03-03 09:56:33
 * @LastEditTime: 2022-04-01 12:11:28
 * @LastEditors: AFShk
 * @Description: 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 * @FilePath: \VERSION3_2\OSTASK\line_detect.c
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
#include "condition.h"
#include "openmv_communicate.h"
#include "ist8310.h"
#include "bmi088_driver.h"
#include "bmi088.h"
#include "callback.h"
#include "pid.h"
//这里里面主要是用遥控器控制的代码
#ifdef sbus_using
extern uint8_t rec_sbus_data[];
extern uint8_t sbus_erroflag;
extern uint16_t sbus_channel[16];//sbus data
#endif
#ifdef echo_using
uint8_t rx_echo_buff[2];
int16_t rx_echo;
#endif
extern uint8_t order_flag;
extern volatile CAR car;
extern int frame_high;
extern uint8_t gyro_flag;
extern Openmv_Struct Openmv_TX,Openmv_RX;
uint8_t no_motor_flag=1;
uint8_t rx_line_buff[5];
uint8_t sensor_flag=0;
static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};//pid data
extern pid_type_def	imu_temp_pid;//temp pid

void sbus_order(void);//函数的声明
void line_detect_init(void);//初始化配置函数

void line_detect_task(void const * argument)
{
	line_detect_init();//初始化各个传感器的位置及数据等
	while(1){
		#ifdef sbus_using
			sbus_order();
			move_pid_calc();
		  osDelay(2);
		#endif // 使用遥控器控制

		#ifndef sbus_using//自动控制模式
		Openmv_Data_Transmit(&Openmv_TX);
		osDelay(50);
//		car.vx=20;
//		car.vy=0;
//		osDelay(5000);
//		car.vx=0;
//		car.vy=20;
//		osDelay(5000);
//		car.vx=-20;
//		car.vy=0;
//		osDelay(5000);
//		car.vx=0;
//		car.vy=-20;
//		osDelay(5000);
		#endif
	}
}

void line_detect_init(void)//初始化配置函数
{
	HAL_TIM_Base_Start(&htim10);//imu_temp
  HAL_TIM_PWM_Start(&htim10,TIM_CHANNEL_1);
  PID_init(&imu_temp_pid,PID_POSITION,imu_temp_PID,TEMPERATURE_PID_MAX_OUT,TEMPERATURE_PID_MAX_IOUT);//imu heat pid
	TIM5->CCR1=0;
	TIM5->CCR3=500;//传感器初始化	
	while(BMI088_init()) osDelay(1);	
	while(ist8310_init())	osDelay(1);
	sensor_flag=1;
	#ifdef sbus_using
	__HAL_UART_ENABLE_IT(&huart3,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart3,rec_sbus_data,25);
	#endif
	#ifdef echo_using
	__HAL_UART_ENABLE_IT(&huart6,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart6,rx_echo_buff,2);//接受超声波数据
	#endif
	__HAL_UART_ENABLE_IT(&huart1,UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1,rx_line_buff,5);//接收红外数据
	while(order_flag==0){
		osDelay(1);
	}
	frame_pid_init();
	move_pid_init();
	osDelay(1);
	HAL_CAN_Start(&hcan1);//can1 to control chassis
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
	HAL_CAN_Start(&hcan2);//can2 to communicate with OpenMV
	HAL_CAN_ActivateNotification(&hcan2,CAN_IT_RX_FIFO0_MSG_PENDING);
	osDelay(1);
	TIM5->CCR2=1000;
}

#ifdef sbus_using
void sbus_order(void)//遥控器数据处理函数
{
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
			car.w1=-(sbus_channel[3]-1024)/100.0f;
		else if(sbus_channel[3]<=950)
			car.w1=(1024-sbus_channel[3])/100.0f;
		else car.w1=0;
	}
	else{
		car.vx=0;
		car.vy=0;
		car.w1=0;
		frame_high=0;
	}
}
#endif
