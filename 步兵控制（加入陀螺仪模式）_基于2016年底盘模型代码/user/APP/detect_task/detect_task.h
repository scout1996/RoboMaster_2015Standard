#ifndef DETECT_Task_H
#define DETECT_Task_H

#include "main.h"

#define DETECT_TASK_INIT_TIME    57
#define DETECT_CONTROL_TIME      10

//红灯闪，灭函数，切换闪灭
#define DETECT_LED_R_ON()       led_red_on()
#define DETECT_LED_R_OFF()      led_red_off()
#define DETECT_LED_R_TOGGLE()   led_red_toggle()

//流水灯闪灭函数
#define DETECT_FLOW_LED_ON(i)   flow_led_on(i)
#define DETECT_FLOW_LED_OFF(i)  flow_led_off(i)

//错误码以及对应设备顺序
enum errorList
{
    DBUSTOE = 0,
    YawGimbalMotorTOE,
    PitchGimbalMotorTOE,
    TriggerMotorTOE,
    ChassisMotor1TOE,   //4
    ChassisMotor2TOE,
    ChassisMotor3TOE,
    ChassisMotor4TOE,   //7

    errorListLength,
};

typedef __packed struct
{
    uint32_t newTime;                 //最新记录时间
    uint32_t lastTime;                //上次记录时间
    uint32_t Losttime;                //掉线时间
    uint32_t worktime;                //上线时间
    uint16_t setOfflineTime : 12;     //设置离线时间阈值
    uint16_t setOnlineTime  : 12;     //设置上线时间阈值
    uint8_t enable : 1;               //设备监测使能按钮
    uint8_t Priority : 4;             //设备优先级
    uint8_t isLost : 1;               //设备掉线状态
    float frequency;                   //设备更新频率

} error_t;

void   detect_task(void *pvParameters);      //离线任务函数
uint8_t toe_is_error(uint8_t err);           //返回对应的设备是否在线
const  error_t *getErrorListPoint(void);    //返回设备列表地址
void   DetectHook(uint8_t toe);             //离线钩子函数
void DetectInit(uint32_t time);   //初始化错误列表
void DetectDisplay(void);         //显示错误信息

#endif
