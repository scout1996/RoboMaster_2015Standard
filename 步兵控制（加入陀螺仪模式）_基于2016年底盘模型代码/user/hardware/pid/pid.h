#ifndef PID_H
#define PID_H

#include "main.h"

enum PID_MODE
{
    PID_POSITION = 0,
    PID_DELTA
};

enum DATA_MODE
{
    DATA_GYRO = 0,
    DATA_NORMAL,
};


typedef struct
{
    uint8_t  mode;
	  uint8_t  data_mode;
    //PID 三参数
    float Kp;
    float Ki;
    float Kd;

    float max_out;  //最大输出
    float max_iout; //最大积分输出

    float set;
    float fdb;

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次

} PidTypeDef;


extern void PID_Init(PidTypeDef *pid, uint8_t mode, uint8_t data_mode, const float PID[3], float max_out, float max_iout); //PID初始化
extern float PID_Calc(PidTypeDef *pid, float ref, float set);  //PID计算
extern void PID_clear(PidTypeDef *pid);   //PID清除

#endif
