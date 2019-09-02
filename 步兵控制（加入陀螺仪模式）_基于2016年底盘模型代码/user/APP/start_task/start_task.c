/**
  ****************************(C) COPYRIGHT 2019 IronSprit***********************
  * @file       start_task.c/h

  * @brief      启动任务，将多任务开启，分配资源，给定任务优先级               

  * @note       临界区能保护一段代码不被其他任务或中断打断
									 
  * @history
  *  Version       Date            Author          status
  *  V1.0.0       2019-6-14        Young            完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 IronSprit************************
*/

#include "start_task.h"
#include "Detect_Task.h"
#include "User_Task.h"
#include "chassis_task.h"
#include "Gimbal_Task.h"

static TaskHandle_t StartTask_Handler; //任务句柄
static TaskHandle_t ChassisTask_Handler;
static TaskHandle_t DetectTask_Handler;
static TaskHandle_t UserTask_Handler;
static TaskHandle_t GimbalTask_Handler;

/*--------创建我自己的任务-------*/
void Create_MyTask(void)
{
    //以动态方法创建开始任务
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数(一般是不用的)
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄(比较重要，任务创建成功以后会返回此任务的任务句柄)
}

/*--------开始任务的实现-------*/
void start_task(void *pvParameters)//void* 无论参数是什么类型的，都能传入
{
    taskENTER_CRITICAL();    //进入临界区

    xTaskCreate((TaskFunction_t)gimbal_task,
                (const char *)"GimbalTask",
                (uint16_t)Gimbal_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Gimbal_TASK_PRIO,
                (TaskHandle_t *)&GimbalTask_Handler);	
								
	  xTaskCreate((TaskFunction_t)chassis_task,
                (const char *)"ChassisTask",
                (uint16_t)Chassis_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Chassis_TASK_PRIO,
                (TaskHandle_t *)&ChassisTask_Handler);							

    xTaskCreate((TaskFunction_t)detect_task,
                (const char *)"DetectTask",
                (uint16_t)Detect_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)Detect_TASK_PRIO,
                (TaskHandle_t *)&DetectTask_Handler);								

    xTaskCreate((TaskFunction_t)user_task,
                (const char *)"UserTask",
                (uint16_t)User_STK_SIZE,
                (void *)NULL,
                (UBaseType_t)User_TASK_PRIO,
                (TaskHandle_t *)&UserTask_Handler);								

    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}

