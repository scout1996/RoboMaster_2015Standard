/**
  ****************************(C) COPYRIGHT 2019 IronSprit***********************
  * @file       can_receive.c/h

  * @brief      完成can设备数据收发任务，该文件是通过can中断完成接收，并实时检测设备
                是否离线	

  * @note       该文件不是freeRTOS任务
									 
  * @history
  *  Version       Date            Author          status
  *  V2.0.0       2019-6-30        Young            完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 IronSprit************************
*/

#include "CAN_Receive.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Detect_Task.h"

//EC60电机数据读取
#define get_EC60motor_data(ptr, rx_message)                                                    \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->actual_current = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);\
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->hall_value = (rx_message)->Data[6];                                             \
    }

//6623电机数据读取
#define get_6623motor_data(ptr, rx_message)                                                    \
    {                                                                                          \
        (ptr)->last_ecd = (ptr)->ecd;                                                          \
        (ptr)->ecd = (uint16_t)((rx_message)->Data[0] << 8 | (rx_message)->Data[1]);           \
        (ptr)->actual_current = (uint16_t)((rx_message)->Data[2] << 8 | (rx_message)->Data[3]);\
        (ptr)->given_current = (uint16_t)((rx_message)->Data[4] << 8 | (rx_message)->Data[5]); \
        (ptr)->hall_value = (rx_message)->Data[6];                                             \
    }
		
//声明电机变量
static motor_data_t   motor_yaw, motor_pitch ,motor_chassis[4]; 		

//发送云台控制命令，其中rev为保留字节
void CAN_CMD_GIMBAL(int16_t yaw, int16_t pitch,int16_t rev1, int16_t rev2)
{
	  CanTxMsg  GIMBAL_TxMessage;
    GIMBAL_TxMessage.StdId = CAN_GIMBAL_ALL_ID;
    GIMBAL_TxMessage.IDE = CAN_ID_STD;
    GIMBAL_TxMessage.RTR = CAN_RTR_DATA;
    GIMBAL_TxMessage.DLC = 0x08;
    GIMBAL_TxMessage.Data[0] = (yaw >> 8);
    GIMBAL_TxMessage.Data[1] = yaw;
    GIMBAL_TxMessage.Data[2] = (pitch >> 8);
    GIMBAL_TxMessage.Data[3] = pitch;
    GIMBAL_TxMessage.Data[4] = (rev1 >> 8);
    GIMBAL_TxMessage.Data[5] = rev1;
    GIMBAL_TxMessage.Data[6] = (rev2 >> 8);
    GIMBAL_TxMessage.Data[7] = rev2;
	
	  CAN_Transmit(GIMBAL_CAN, &GIMBAL_TxMessage);
}

//发送底盘控制命令
//因为底盘电机M1的电调ID为0x206，发送ID为0x1FF，故增加发送ID的形参以供区别
void CAN_CMD_CHASSIS(uint32_t SendID, int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    CanTxMsg  CHASSIS_TxMessage;
    CHASSIS_TxMessage.StdId = SendID;
    CHASSIS_TxMessage.IDE = CAN_ID_STD;   //标识符选择位：标准ID
    CHASSIS_TxMessage.RTR = CAN_RTR_DATA; //远程请求位：数据帧
    CHASSIS_TxMessage.DLC = 0x08;         //数据长度
    CHASSIS_TxMessage.Data[0] = motor1 >> 8;
    CHASSIS_TxMessage.Data[1] = motor1;
    CHASSIS_TxMessage.Data[2] = motor2 >> 8;
    CHASSIS_TxMessage.Data[3] = motor2;
    CHASSIS_TxMessage.Data[4] = motor3 >> 8;
    CHASSIS_TxMessage.Data[5] = motor3;
    CHASSIS_TxMessage.Data[6] = motor4 >> 8;
    CHASSIS_TxMessage.Data[7] = motor4;

    CAN_Transmit(CHASSIS_CAN, &CHASSIS_TxMessage);
}

//返回yaw电机变量地址，通过指针方式获取原始数据
const motor_data_t *get_Yaw_Gimbal_Motor_Data_Point(void)
{
    return &motor_yaw;
}
//返回pitch电机变量地址，通过指针方式获取原始数据
const motor_data_t *get_Pitch_Gimbal_Motor_Data_Point(void)
{
    return &motor_pitch;
}
//返回底盘电机变量地址，通过指针方式获取原始数据
const motor_data_t*  get_Chassis_Motor_Data_Point(uint8_t i)
{
    return &motor_chassis[(i & 0x03)];
}

//统一处理can中断函数，并且记录发送数据的时间，作为离线判断依据
//对于第一个电机的ID为0x206做特别处理
static void CAN1_hook(CanRxMsg *rx_message)
{
	static uint8_t i = 0;
	
  switch (rx_message->StdId)
  {
			case CAN_EC60_M1_ID:
			{
			    //处理电机数据宏函数
					get_EC60motor_data(&motor_chassis[0], rx_message);
					//记录时间,检查是否掉线
					DetectHook(ChassisMotor1TOE);
					break;
			}
			case CAN_EC60_M2_ID:
			case CAN_EC60_M3_ID:
			case CAN_EC60_M4_ID:
			{			
					//处理电机ID号
					i = rx_message->StdId - 0x201;
					//处理电机数据宏函数
					get_EC60motor_data(&motor_chassis[i], rx_message);
					//记录时间,检查是否掉线
					DetectHook(ChassisMotor1TOE + i);
					break;
			}

			default:  break;
  }
}

//因为底盘1号电机的ID与云台pitch电机冲突，所以需要分开处理can1和can2的数据
static void CAN2_hook(CanRxMsg *rx_message)
{
    switch (rx_message->StdId)
		{
				case CAN_YAW_MOTOR_ID:
				{
						//处理电机数据宏函数
						get_6623motor_data(&motor_yaw, rx_message);
						DetectHook(YawGimbalMotorTOE);
						break;
				}
				case CAN_PITCH_MOTOR_ID:
				{
						//处理电机数据宏函数
						get_6623motor_data(&motor_pitch, rx_message);
						DetectHook(PitchGimbalMotorTOE);
						break;
				}

				default:  break;
		}
}

//can1中断
void CAN1_RX0_IRQHandler(void)
{
    static CanRxMsg rx1_message;

    if (CAN_GetITStatus(CAN1, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
        CAN_Receive(CAN1, CAN_FIFO0, &rx1_message);
        CAN1_hook(&rx1_message);
    }
}


//can2中断
void CAN2_RX0_IRQHandler(void)
{
    static CanRxMsg rx2_message;
	
    if (CAN_GetITStatus(CAN2, CAN_IT_FMP0) != RESET)
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
        CAN_Receive(CAN2, CAN_FIFO0, &rx2_message);
        CAN2_hook(&rx2_message);
    }
}

