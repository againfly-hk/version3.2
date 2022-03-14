/*
 * @Author: AFShk
 * @Date: 2022-03-03 10:07:03
 * @LastEditors: AFShk
 * @LastEditTime: 2022-03-07 20:41:20
 * @FilePath: \RC2022\Sensor\bmi088.c
 * @Description: 
 * 
 * Copyright (c) 2022 by 用户/公司名, All Rights Reserved. 
 */

#include "bmi088.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_delay.h"

extern SPI_HandleTypeDef hspi1;

void BMI088_delay_ms(uint16_t ms)
{
    osDelay(ms);
}

void BMI088_delay_us(uint16_t us)
{
    delay_us(us);
}

void BMI088_ACCEL_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
}

void BMI088_ACCEL_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
}

void BMI088_GYRO_NS_L(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
}

void BMI088_GYRO_NS_H(void)
{
    HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
}

uint8_t BMI088_read_write_byte(uint8_t txdata)
{
    uint8_t rx_data;
    HAL_SPI_TransmitReceive(&hspi1, &txdata, &rx_data, 1, 1000);
    return rx_data;
}
