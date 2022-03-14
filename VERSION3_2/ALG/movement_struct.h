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
	float raccel[3];
	float rgyro[3];
	float yaw;
	float begin_yaw;
	float v1,v2,v3,v4,vx,vy,w;//开始时磁力计数据	
}CAR;//车体参数记录
#endif
