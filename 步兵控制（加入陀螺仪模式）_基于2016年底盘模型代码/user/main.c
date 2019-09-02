/**
  ****************************(C) COPYRIGHT 2019 IronSprit***********************
  * @project    基于2015年的官方步兵机械平台上搭建RM开发板A型编写的程序，实现步兵
	              底盘与云台联动的功能

  * @brief                     

  * @note       问题：陀螺仪yaw数值一直飘              
	              
								解决办法：陀螺仪板子上的程序要比底盘板子上的程序先跑，陀螺仪读值才能稳定。
								         在陀螺仪初始化程序前加2s的延时，等待陀螺仪板子上的程序初始化成功
												 ，读值稳定后，再去读陀螺仪的值，这样就陀螺仪就能稳定了
								
  * @history
  *  Version       Date            Author          status
  *  V2.0.0      2019-7-14         Young            完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 IronSprit************************
*/

#include "stm32f4xx.h"
#include "buzzer.h"
#include "can.h"
#include "delay.h"
#include "led.h"
#include "sys.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_control.h"
#include "start_task.h"
#include "gyroscope.h"

void Program_Init(void)
{
	  //设置系统中断优先级分组4
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
	
	  //初始化延时函数（滴答时钟）
   delay_init(configTICK_RATE_HZ);

    //初始化LED	
    LED_init();	
   
	  //蜂鸣器初始化
    buzzer_init(30000-1, 90-1);
	  
    //陀螺仪初始化
    delay_ms(2000);	
	  gyroscope_init();
	
	  //CAN接口初始化
    CAN1_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
	  CAN2_mode_init(CAN_SJW_1tq, CAN_BS2_2tq, CAN_BS1_6tq, 5, CAN_Mode_Normal);
	
	  //遥控器初始化
    remote_control_init();
}

int main(void)
{
    Program_Init();
	  delay_ms(100);
	  Create_MyTask();
	  vTaskStartScheduler();          //开启任务调度
    while(1);
}
