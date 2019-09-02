/**
  ****************************(C) COPYRIGHT 2019 IronSprit**************************
  * @file       user_task.c/h
  * @brief      普通心跳程序，如果设备无错误，绿灯1Hz闪烁
  * @note       
  * @history
  *  Version       Date            Author          status
  *  V2.0.0       2019-6-16        Young            完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 IronSprit**************************
  */

#include "User_Task.h"
#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Detect_Task.h"
#include "led.h"

void user_task(void *pvParameters)
{
    while (1)
    {
			 if(toe_is_error(DBUSTOE)== 0 && toe_is_error(ChassisMotor1TOE)==0 && toe_is_error(ChassisMotor2TOE)==0 && toe_is_error(ChassisMotor3TOE)==0 && toe_is_error(ChassisMotor4TOE)==0)
			 {
			     led_green_toggle();
           vTaskDelay(1000);		
			 }
       else
			 {
			     led_green_off();
			 }				 
    }
}
