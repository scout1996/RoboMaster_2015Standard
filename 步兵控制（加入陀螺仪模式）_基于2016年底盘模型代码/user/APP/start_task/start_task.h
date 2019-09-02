#ifndef _start_task_H
#define _start_task_H

#include "FreeRTOS.h"
#include "task.h"

/*--------创建开始任务-------*/
#define START_TASK_PRIO 1            //任务优先级
#define START_STK_SIZE  512          //任务堆栈大小

/*--------创建用户任务-------*/
#define User_TASK_PRIO  2
#define User_STK_SIZE   120

/*--------创建自检任务-------*/
#define Detect_TASK_PRIO  3
#define Detect_STK_SIZE   512

/*--------创建底盘任务-------*/
#define Chassis_TASK_PRIO   4
#define Chassis_STK_SIZE    512

/*--------创建云台任务-------*/
#define Gimbal_TASK_PRIO    5
#define Gimbal_STK_SIZE     512

void Create_MyTask(void);
void start_task(void *pvParameters);

#endif

