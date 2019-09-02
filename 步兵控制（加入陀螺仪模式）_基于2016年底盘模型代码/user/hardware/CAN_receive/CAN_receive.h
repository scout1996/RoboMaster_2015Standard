#ifndef _CANTASK_H
#define _CANTASK_H

#include "main.h"
#include "stm32f4xx.h"

#define CHASSIS_CAN   CAN1                                                                   
#define GIMBAL_CAN    CAN2

/* 定义发送和返回报文的ID号 */
typedef enum
{
	  //发送报文ID
    CAN_CHASSIS_LOW_ID  = 0x200, 
	  CAN_CHASSIS_HIGH_ID = 0x1FF,
	  CAN_GIMBAL_ALL_ID   = 0x1FF,
	
	  //接收报文ID
    CAN_EC60_M1_ID = 0x206,  //此电机使用云台电机的电调
    CAN_EC60_M2_ID = 0x202,
    CAN_EC60_M3_ID = 0x203,
    CAN_EC60_M4_ID = 0x204,
	
	  CAN_YAW_MOTOR_ID     = 0x205,
    CAN_PITCH_MOTOR_ID   = 0x206,
	
} can_msg_id_e;


//电机数据结构体
typedef struct
{
    uint16_t  ecd;            //机械角度
    int16_t  actual_current;  //实际电流
    int16_t  given_current;   //给定电流
    uint8_t  hall_value;      //霍尔开关值
    int16_t  last_ecd;
	
} motor_data_t;

//发送底盘控制命令
void CAN_CMD_CHASSIS(uint32_t SendID, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
//发送云台控制命令
void CAN_CMD_GIMBAL(int16_t yaw, int16_t pitch,int16_t rev1, int16_t rev2);

//返回底盘电机变量地址，通过指针方式获取原始数据,i的范围是0-3，对应0x201-0x204,
const motor_data_t *get_Chassis_Motor_Data_Point(uint8_t i);
//返回yaw电机变量地址，通过指针方式获取原始数据
const motor_data_t *get_Yaw_Gimbal_Motor_Data_Point(void);
//返回pitch电机变量地址，通过指针方式获取原始数据
const motor_data_t *get_Pitch_Gimbal_Motor_Data_Point(void);

//CAN接收处理函数		
static void CAN_hook(CanRxMsg *rx_message);

#endif
