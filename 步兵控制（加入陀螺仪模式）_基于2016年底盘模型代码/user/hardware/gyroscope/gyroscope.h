#ifndef _gyroscope_H
#define _gyroscope_H

#include "stm32f4xx.h"

#define MPU6050_NVIC 1
#define MPU6050_RX_BUF_NUM 30
#define GYRO_FRAME_LENGTH  26

#pragma pack(1) //强制1字节对齐

typedef union
{
    struct
    {
		u8 header;
		float INS_Angle[3];
		float INS_gyro[3];
		u8 tail;
    } data;
    uint8_t buf[MPU6050_RX_BUF_NUM];

} MPU6050_Pack_t;

void gyroscope_init(void);
const float *get_MPU6050_Angle_point(void);
const float *get_MPU6050_Gyro_Point(void);

#endif

