/*
 * @Author: AFShk
 * @Date: 2022-03-31 14:06:49
 * @LastEditors: AFShk
 * @LastEditTime: 2022-03-31 20:05:33
 * @FilePath: \VERSION3_2\SINGAL\openmv_communicate.h
 * @Description: 
 * 
 * Copyright (c) 2022 by 用户/公司名, All Rights Reserved. 
 */
/*
 * @Author: AFShk
 * @Date: 2022-03-31 14:06:49
 * @LastEditors: AFShk
 * @LastEditTime: 2022-03-31 14:06:49
 * @FilePath: \VERSION3_2\SINGAL\openmv_communicate.h
 * @Description: 
 * 
 * Copyright (c) 2022 by 用户/公司名, All Rights Reserved. 
 */
#ifndef __OPENMV_COMMUNICATE_H__
#define __OPENMV_COMMUNICATE_H__

#include "main.h"

typedef struct{
    uint8_t mode;
    uint8_t data1;
    int16_t data2;
    int16_t data3;
    uint8_t data4;
    uint8_t data5;
    /* data */
}Openmv_Struct;

extern void can2_filter_init(void);
extern void Openmv_Data_Transmit(Openmv_Struct* openmv);
#endif

