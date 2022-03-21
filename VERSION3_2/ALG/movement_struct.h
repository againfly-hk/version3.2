#ifndef __MOVEMENT_STRUCT_H__
#define __MOVEMENT_STRUCT_H__

#include "struct_typedef.h"

typedef struct
{
    uint16_t ecd;
    int16_t speed_rpm;
    int16_t given_current;
    uint8_t temperate;
    int16_t last_ecd;
} motor_measure_t;//

typedef struct{
	int change;
	int delta;
	int last_value;
	int cnt;
}code;//3508编码器数据记录

typedef struct{
	float raccel[3];//car accel
	float rgyro[3];//car gyro
	float yaw;//car yaw
	float begin_yaw;//car yaw begin
	int carx,cary;
	float v1,v2,v3,v4,vx,vy,w;//与车速度相关的的定义
}CAR;//车体参数记录
#endif
