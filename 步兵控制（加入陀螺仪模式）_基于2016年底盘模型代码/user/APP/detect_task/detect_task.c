/**
  ****************************(C) COPYRIGHT 2019 IronSprit***********************
  * @file       detect_task.c/h
  * @brief      设备离线判断任务，通过freeRTOS滴答时间作为系统时间，设备获取数据后
  *             调用DetectHook记录对应设备的时间，在该任务会通过判断记录时间与系统
  *             时间之差来判断掉线，同时将最高的优先级的任务通过LED的方式改变，包括
  *             八个流水灯显示SBUB遥控器，三个云台上的电机，4个底盘电机，另外也通过
  *             红灯闪烁次数来显示错误码。
  * @history
  *  Version       Date            Author          status
  *  V1.0.0      2019-6-16          DJI             完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 IronSprit************************
*/

#include "Detect_Task.h"
#include "led.h"
#include "buzzer.h"
#include "CAN_Receive.h"
#include "Remote_Control.h"
#include "FreeRTOS.h"
#include "task.h"

//定义监控设备结构体
static error_t    errorList[errorListLength + 1];

//掉线判断任务
void detect_task(void *pvParameters)
{
    static  uint32_t  systemTime;
    systemTime = xTaskGetTickCount();//用于获取系统当前运行的时钟节拍数

    //初始化
    DetectInit(systemTime);

    //空闲一段时间
    vTaskDelay(DETECT_TASK_INIT_TIME);

    while (1)
    {
        systemTime = xTaskGetTickCount();

        for (int i = 0; i < errorListLength; i++)
        {
            //未使能，跳过设备监测
            if (errorList[i].enable == 0)
            {
                continue;
            }

            //刚上线排除
            if (systemTime - errorList[i].worktime < errorList[i].setOnlineTime)
            {
                //刚刚上线，可能存在数据不稳定
                errorList[i].isLost = 0;
            }
            //超时掉线
            else if (systemTime - errorList[i].newTime > errorList[i].setOfflineTime)
            {
                if (errorList[i].isLost == 0)
                {
                    //记录错误以及掉线时间
                    errorList[i].isLost = 1;
                    errorList[i].Losttime = systemTime;
                }
            }
            //正常工作
            else
            {
                errorList[i].isLost = 0;

                //计算频率
                if (errorList[i].newTime > errorList[i].lastTime)
                {
                    errorList[i].frequency = configTICK_RATE_HZ / (float)(errorList[i].newTime - errorList[i].lastTime);
                }
            }
        }
        DetectDisplay();
        vTaskDelay(DETECT_CONTROL_TIME);
    }
}

//设备接收数据钩子函数
void DetectHook(uint8_t toe)
{
    errorList[toe].lastTime = errorList[toe].newTime;
    errorList[toe].newTime  = xTaskGetTickCount();

    //更新丢失情况
    if (errorList[toe].isLost)
    {
        errorList[toe].isLost   = 0;
        errorList[toe].worktime = errorList[toe].newTime;
    }
}

//返回设备列表地址
const error_t *getErrorListPoint(void)
{
    return errorList;
}

//返回对应的设备是否丢失
uint8_t toe_is_error(uint8_t err)
{
    return (errorList[err].isLost == 1);
}

void DetectDisplay(void)
{
    uint8_t i = 0;
    uint8_t error_flag =0;
    static uint16_t detect_counter =0;

    //8个流水显示各监测设备状态
    for (i = 0; i < errorListLength; i++)
    {
        if (errorList[i].isLost && (errorList[i].enable == 1) )
        {
            DETECT_FLOW_LED_OFF(i);
            error_flag =1;
        }
        else
        {
            DETECT_FLOW_LED_ON(i);
        }
    }

    //拥有错误则状态红灯闪烁
    detect_counter++;
    if(detect_counter == 30 )
    {
        if(error_flag == 1 )
        {
            DETECT_LED_R_TOGGLE();
        }
        else
        {
            DETECT_LED_R_OFF();
        }

        detect_counter =0;
    }

    //蜂鸣器提示音提示有错
    if(error_flag == 1)
    {
        buzzer_on(10,15000);
    }
    else
    {
        buzzer_off();
    }
}

void DetectInit(uint32_t time)
{
    //离线时间阈值 ，上线时间阈值，优先级，使能状态
    uint16_t setItem[errorListLength][4] =
    {
        {30, 40,  15, 1},   //DBUS
        {2,   3,  14, 1},   //yaw
        {2,   3,  13, 1},   //pitch
        {10, 10,  12, 0},   //trigger
        {10, 10,  11, 1},   //motor1
        {10, 10,  10, 1},   //motor2
        {10, 10,   9, 1},   //motor3
        {10, 10,   8, 1},   //motor4
    };

    for (uint8_t i = 0; i < errorListLength; i++)
    {
        errorList[i].setOfflineTime = setItem[i][0];
        errorList[i].setOnlineTime =  setItem[i][1];
        errorList[i].Priority = setItem[i][2];
        errorList[i].enable = setItem[i][3];

        errorList[i].isLost = 1;
        errorList[i].frequency = 0.0f;
        errorList[i].newTime =  time;
        errorList[i].lastTime = time;
        errorList[i].Losttime = time;
        errorList[i].worktime = time;
    }

}
