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
	float yaw,begin_yaw,begin_ryaw;//car yaw//car yaw begin
	float v1,v2,v3,v4;
	int carx,cary;
	float vx,vy,w1,w2;//与车速度相关的的定义
}CAR;//车体参数记录

typedef struct{
	uint8_t vector_x;
	uint8_t vector_y;
	uint8_t expect_speed;
	uint8_t expext_distance;
}order;
//if vector_x =111,龙门架抬升，speed——flag代表高度
//if vector_x=222,车旋转，speed为w
//if vector_x=255,听从openmv的命令
#endif

