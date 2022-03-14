/*
 * @Author: AFShk
 * @Date: 2022-03-03 10:39:41
 * @LastEditors: AFShk
 * @LastEditTime: 2022-03-13 16:52:47
 * @FilePath: \RC2022\Signal\Callback.c
 * @Description:
 *
 * Copyright (c) 2022 by 用户/公司名, All Rights Reserved. 
 */

#include "usart.h"
#include "sbus.h"
#include "Callback.h"
#include "tim.h"

#include "CAN_receive.h"
#include "bmi088_driver.h"
#include "bmi088.h"
#include "ist8310.h"

#include "pid.h"
#include "AHRS_MiddleWare.h"
#include "AHRS.h"
//exti callback
extern CAR car;

bmi088_real_data_t	imu_real_data;
pid_type_def	imu_temp_pid;

uint8_t accel_flag=0;
uint8_t temp_flag=0;
uint8_t gyro_flag=0;
uint8_t failure_warning=0;

float gyro_erro[3]={0,0,0};
float accel_errodata[3]={0,0,0};
float mag[3]={0,0,0};

float quat[4];
uint16_t gyrocnt=0;
uint16_t accelcnt=0;

extern float accel_fliter1[3];
extern float accel_fliter2[3];
extern float accel_fliter3[3];
//float accel_kf[3]={1.929454f, -0.931783f, 0.002329f};
float accel_kf[3]={1.929454039488895f, -0.93177349823448126f, 0.002329458745586203f};
//float accel_kf[3]={0.5,0.3,0.2};
//float accel_kf[3]={1.6f,-0.595f,0.0045f};
uint8_t accel_fliter_flag=1;

int motor_coder_record[4];

//FLITER

//usart rxcallback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart3)
		sbus_calc();
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin==INT1_ACCEL_Pin){
		BMI088_read(imu_real_data.gyro,imu_real_data.accel,&imu_real_data.temp);
		if(gyro_flag==2)
		{
			if(accel_fliter_flag==0){
				accel_fliter1[0]=0.0f;
				accel_fliter1[1]=0.0f;
				accel_fliter1[2]=0.0f;

				accel_fliter2[0]=0.0f;
				accel_fliter2[1]=0.0f;
				accel_fliter2[2]=0.0f;

				accel_fliter3[0]=0.0f;
				accel_fliter3[1]=0.0f;
				accel_fliter3[2]=0.0f;
				accel_fliter1[0]=imu_real_data.accel[0];
				accel_fliter1[1]=imu_real_data.accel[0];
				accel_fliter1[2]=imu_real_data.accel[0];

				accel_fliter2[0]=imu_real_data.accel[1];
				accel_fliter2[1]=imu_real_data.accel[1];
				accel_fliter2[2]=imu_real_data.accel[1];

				accel_fliter3[0]=imu_real_data.accel[2];
				accel_fliter3[1]=imu_real_data.accel[2];
				accel_fliter3[2]=imu_real_data.accel[2];

				accel_fliter_flag=1;
				return;
			}
			if(accel_fliter_flag==1){
				accel_fliter_flag=2;
			}
			if(accel_fliter_flag==2)
			{
				accel_fliter1[0]=accel_fliter1[1];
				accel_fliter1[1]=accel_fliter1[2];
				accel_fliter1[2]=accel_fliter1[0]*accel_kf[0]+accel_fliter1[1]*accel_kf[1]+imu_real_data.accel[0]*accel_kf[2];

				accel_fliter2[0]=accel_fliter2[1];
				accel_fliter2[1]=accel_fliter2[2];
				accel_fliter2[2]=accel_fliter2[0]*accel_kf[0]+accel_fliter2[1]*accel_kf[1]+imu_real_data.accel[1]*accel_kf[2];

				accel_fliter3[0]=accel_fliter3[1];
				accel_fliter3[1]=accel_fliter3[2];
				accel_fliter3[2]=accel_fliter3[0]*accel_kf[0]+accel_fliter3[1]*accel_kf[1]+imu_real_data.accel[2]*accel_kf[2];
							
				accel_fliter1[2]=imu_real_data.accel[0];
				accel_fliter2[2]=imu_real_data.accel[1];
				accel_fliter3[2]=imu_real_data.accel[2];
				
				car.raccel[0]=accel_fliter1[2];
				car.raccel[1]=accel_fliter2[2];
				car.raccel[2]=accel_fliter3[2];
			}						
		}

	}
	if(GPIO_Pin==INT1_GYRO_Pin){
		//温度控制/////////////////////////////////////////////////////////////////////////
		//BMI088_read(imu_real_data.gyro,imu_real_data.accel,&imu_real_data.temp);
		float temp=0;
		BMI088_read_gyro(imu_real_data.gyro,&temp);
		if(temp>25&&temp<70)	imu_real_data.temp=temp;
		if(!temp_flag){
			if(imu_real_data.temp>45.0f){
				temp_flag=1;
				imu_temp_pid.Iout=MPU6500_TEMP_PWM_MAX/2;
				__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,imu_temp_pid.Iout);
				return;
			}
			__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,MPU6500_TEMP_PWM_MAX);
			return;
		}
		else if(temp_flag!=0){
			PID_calc(&imu_temp_pid,imu_real_data.temp,45.0f);
			if (imu_temp_pid.out < 0.0f)	imu_temp_pid.out = 0.0f;
			__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,imu_temp_pid.out);
		}
		//////////////////////////////////////////////////////////////
		if(temp_flag!=0&&gyro_flag==2&&accel_fliter_flag==2){
			for(uint8_t i=0;i<3;i++){
				car.rgyro[i]=imu_real_data.gyro[i]-gyro_erro[i];
//				gyro_erro[i]+=0.00005f*imu_real_data.gyro[i];
			}
			if(accel_fliter_flag==2){
				AHRS_update(quat,0.001f,car.rgyro,car.raccel,mag);
				car.yaw=get_yaw(quat)*180.0f/3.1415926f;
			}
		}
  }
	if(GPIO_Pin==IST8310_EXIT_Pin){
		ist8310_read_mag(mag);
	}
	if(GPIO_Pin==Failture_Pin){
		failure_warning=10;//????????10??????????????????????????
	}
}

/*
void accel_fliter(){

}

void gyro_fliter(){

}
*/
