/*
 * @Author: AFShk
 * @Date: 2022-03-03 10:39:41
 * @LastEditors: AFShk
 * @LastEditTime: 2022-03-28 10:50:03
 * @FilePath: \VERSION3_2\SINGAL\Callback.c
 * @Description:
 *
 * Copyright (c) 2022 by 用户/公司名, All Rights Reserved.
 */

//之前出现的bug居然是因为数组越界，现在应该可以实现之前设计的所有功能呀
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
#include "condition.h"

//exti callback
extern CAR car;
bmi088_real_data_t	imu_real_data;
pid_type_def	imu_temp_pid;
uint8_t temp_flag=0;
uint8_t gyro_flag=0;
uint8_t failure_warning=0;
uint8_t accel_fliter_flag=1;
float gyro_erro[3]={0,0,0};
float accel_errodata[3]={0,0,0};
float mag[3]={0,0,0};
float quat[4];
float accel_fliter1[3];//accel fliter
float accel_fliter2[3];
float accel_fliter3[3];
float accel_kf[3]={1.929454039488895f, -0.93177349823448126f, 0.002329458745586203f};//accel fliter k
int motor_coder_record[4];
//float test11[3],test12[3];
//float test2[3],temp1,temp2;
//FLITER


//usart rxcallback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	#ifdef sbus_using
		if(huart == &huart3)
			sbus_calc();//解析遥控器数据
    #endif
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin==INT1_ACCEL_Pin){
		float temp;
		BMI088_read(imu_real_data.gyro,imu_real_data.accel,&temp);
//		BMI088_read(test11,test12,&temp);
//		if(temp>0&&temp<70)
//			imu_real_data.temp=temp;
//		temp1=temp;
		if(gyro_flag==2)
		{
			car.raccel[0]=imu_real_data.accel[0];
			car.raccel[1]=imu_real_data.accel[1];
			car.raccel[2]=imu_real_data.accel[2];
//			if(accel_fliter_flag==1)
//			{
//				accel_fliter1[0]=accel_fliter1[1];
//				accel_fliter1[1]=accel_fliter1[2];
//				accel_fliter1[2]=accel_fliter1[0]*accel_kf[0]+accel_fliter1[1]*accel_kf[1]+imu_real_data.accel[0]*accel_kf[2];
//				accel_fliter2[0]=accel_fliter2[1];
//				accel_fliter2[1]=accel_fliter2[2];
//				accel_fliter2[2]=accel_fliter2[0]*accel_kf[0]+accel_fliter2[1]*accel_kf[1]+imu_real_data.accel[1]*accel_kf[2];
//				accel_fliter3[0]=accel_fliter3[1];
//				accel_fliter3[1]=accel_fliter3[2];
//				accel_fliter3[2]=accel_fliter3[0]*accel_kf[0]+accel_fliter3[1]*accel_kf[1]+imu_real_data.accel[2]*accel_kf[2];
//				accel_fliter1[2]=imu_real_data.accel[0];
//				accel_fliter2[2]=imu_real_data.accel[1];
//				accel_fliter3[2]=imu_real_data.accel[2];//低情商：丑陋的代码；高情商：给后续留出优化空间
//				car.raccel[0]=accel_fliter1[2];//-accel_errodata[0];
//				car.raccel[1]=accel_fliter2[2];//-accel_errodata[1];
//				car.raccel[2]=accel_fliter3[2];
//			}
//			else if(accel_fliter_flag==0){//初始化数据，取第一次数据
//				accel_fliter1[0]=imu_real_data.accel[0];
//				accel_fliter1[1]=accel_fliter1[0];
//				accel_fliter1[2]=accel_fliter1[0];
//				accel_fliter2[0]=imu_real_data.accel[1];
//				accel_fliter2[1]=accel_fliter2[0];
//				accel_fliter2[2]=accel_fliter2[0];
//				accel_fliter3[0]=imu_real_data.accel[2];
//				accel_fliter3[1]=accel_fliter3[0];
//				accel_fliter3[2]=accel_fliter3[0];//其实这里可以用数据结构直接优化，后面再写
//				accel_fliter_flag=1;
//				return;
//			}
		}
	}
	if(GPIO_Pin==INT1_GYRO_Pin){
		//温度控制/////////////////////////////////////////////////////////////////////////
		//BMI088_read(imu_real_data.gyro,imu_real_data.accel,&imu_real_data.temp);
		float temp;
		BMI088_read_gyro(imu_real_data.gyro,&temp);
//		BMI088_read_gyro(test2,&temp);
		if(temp>25&&temp<70)
			imu_real_data.temp=temp;
//		temp2=temp;
		if(!temp_flag){
			if(imu_real_data.temp>43.0f){
				temp_flag=1;
				imu_temp_pid.Iout=MPU6500_TEMP_PWM_MAX/2;
				__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,imu_temp_pid.Iout);
				return;
			}
			__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,MPU6500_TEMP_PWM_MAX);
			return;
		}//温度未到43度时最大功率加热
		else if(temp_flag!=0){
			PID_calc(&imu_temp_pid,imu_real_data.temp,45.0f);
			if (imu_temp_pid.out < 0.0f)	imu_temp_pid.out = 0.0f;
			__HAL_TIM_SetCompare(&htim10,TIM_CHANNEL_1,imu_temp_pid.out);
		}//PID控制温度

		if(temp_flag!=0&&gyro_flag==2&&accel_fliter_flag==1){
			for(uint8_t i=0;i<3;i++){
				car.rgyro[i]=imu_real_data.gyro[i]-gyro_erro[i];
//				gyro_erro[i]=(gyro_erro[i]+0.000005f*imu_real_data.gyro[i])/1.000005f;
			}//陀螺仪数据消除误差
			AHRS_update(quat,0.001f,car.rgyro,car.raccel,mag);//更新四元数
			car.yaw=get_yaw(quat)*180.0f/3.1415926f;//更新角度
		}
  }
	if(GPIO_Pin==IST8310_EXIT_Pin){
		ist8310_read_mag(mag);//读取磁力计角度
	}
	if(GPIO_Pin==Failture_Pin){
		failure_warning=10;//安全模式
	}
}

/*
void accel_fliter(){

}

void gyro_fliter(){

}
*/
