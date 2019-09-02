#ifndef _GIMBAL_TASK_H
#define _GIMBAL_TASK_H

#include "main.h"
#include "CAN_Receive.h"
#include "chassis_task.h"
#include "pid.h"
#include "remote_control.h"
#include "gyroscope.h"

#define USED_GYRO

//任务初始化延时时间
#define  GIMBAL_TASK_INIT_TIME    201
//云台控制周期
#define  GIMBAL_CONTROL_TIME       2

//YAW ,PITCH控制通道以及状态开关通道
#define  YAW_CHANNEL               0
#define  PITCH_CHANNEL             1
#define  MODEL_CHANNEL             0

//遥控器死区设定
#define  GIMBAL_RC_DEADLINE        10

//YAW ,PITCH角度增量（度）与遥控器输入比例
#define  YAW_RC_SEN             -0.0007f
#define  PITCH_RC_SEN           -0.0007f

//YAW,PITCH角度增量（度）和鼠标输入的比例
#define  YAW_MOUSE_SEN          0.0045f
#define  PITCH_MOUSE_SEN        0.00585f

//电机码盘值最大以及中值
#define  Half_ecd_range           4096
#define  ecd_range                8191

//陀螺仪角度值最大值
#define  Half_rad_range            180
#define  rad_range                 360

//定义PITCH,YAW两轴角度极限(度)
#define  MAX_YAW_RANGLE_ANGLE      80.0f
#define  MIN_YAW_RANGLE_ANGLE     -90.0f

#define  MAX_PITCH_RANGLE_ANGLE    15.0f
#define  MIN_PITCH_RANGLE_ANGLE   -15.0f

//电机编码值(0~8191)转化成角度值(-180~180)     360/8192
#define  Motor_Ecd_to_Rad       0.0439453125f

//定义PITCH,YAW电机零点编码值
#define  PITCH_OFFSET_ECD      7010
#define  YAW_OFFSET_ECD        3720


//云台初始化回中值，允许的误差,并且在误差范围内停止一段时间以及最大时间6s后解除初始化状态，
#define GIMBAL_INIT_ANGLE_ERROR      0.1f
#define GIMBAL_INIT_STOP_TIME_MS      100
#define GIMBAL_INIT_TIME_MS          1500

//云台初始化回中值的速度以及控制到的角度
#define GIMBAL_INIT_PITCH_SPEED  0.004f
#define GIMBAL_INIT_YAW_SPEED    0.008f

//云台初始化绝对角度位置
#define   INIT_YAW_SET       0.0f
#define   INIT_PITCH_SET     0.0f

//一阶滤波参数设定
//按照本文件中的滤波函数写法，宏定义值越小，灵敏度越小，但平稳性提高，只能在可接受的灵敏度范围内取得尽可能好的平稳度
#define GIMBAL_ACCEL_RELATIVE_SPEED_NUM     0.1666666667f


/*-------------------云台电机陀螺仪绝对角PID---------------------*/

//pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_ABSOLUTE_SPEED_PID_KP            40.0f
#define PITCH_ABSOLUTE_SPEED_PID_KI             0.8f
#define PITCH_ABSOLUTE_SPEED_PID_KD             0.0f
#define PITCH_ABSOLUTE_SPEED_PID_MAX_OUT    13000.0f
#define PITCH_ABSOLUTE_SPEED_PID_MAX_IOUT    1000.0f

//yaw 速度环 PID参数以及 PID最大输出，积分输出
#define YAW_ABSOLUTE_SPEED_PID_KP              35.0f
#define YAW_ABSOLUTE_SPEED_PID_KI               0.8f
#define YAW_ABSOLUTE_SPEED_PID_KD               0.0f
#define YAW_ABSOLUTE_SPEED_PID_MAX_OUT      13000.0f
#define YAW_ABSOLUTE_SPEED_PID_MAX_IOUT      1000.0f

//pitch 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define PITCH_ABSOLUTE_POSITION_PID_KP           12.0f
#define PITCH_ABSOLUTE_POSITION_PID_KI            0.0f
#define PITCH_ABSOLUTE_POSITION_PID_KD           50.0f
#define PITCH_ABSOLUTE_POSITION_PID_MAX_OUT     400.0f
#define PITCH_ABSOLUTE_POSITION_PID_MAX_IOUT      0.0f

//yaw 角度环 角度由陀螺仪解算 PID参数以及 PID最大输出，积分输出
#define YAW_ABSOLUTE_POSITION_PID_KP             5.0f
#define YAW_ABSOLUTE_POSITION_PID_KI              0.0f
#define YAW_ABSOLUTE_POSITION_PID_KD            20.0f
#define YAW_ABSOLUTE_POSITION_PID_MAX_OUT       600.0f
#define YAW_ABSOLUTE_POSITION_PID_MAX_IOUT        0.0f



/*-------------------云台电机机械角相对角PID---------------------*/

//pitch 速度环 PID参数以及 PID最大输出，积分输出
#define PITCH_RELATIVE_SPEED_PID_KP          800.0f
#define PITCH_RELATIVE_SPEED_PID_KI           10.0f
#define PITCH_RELATIVE_SPEED_PID_KD           0.0f
#define PITCH_RELATIVE_SPEED_PID_MAX_OUT  10000.0f
#define PITCH_RELATIVE_SPEED_PID_MAX_IOUT  200.0f

//yaw 速度环 PID参数以及 PID最大输出，积分输出
#define YAW_RELATIVE_SPEED_PID_KP            500.0f
#define YAW_RELATIVE_SPEED_PID_KI             30.0f
#define YAW_RELATIVE_SPEED_PID_KD            0.0f
#define YAW_RELATIVE_SPEED_PID_MAX_OUT    10000.0f
#define YAW_RELATIVE_SPEED_PID_MAX_IOUT    3000.0f

//pitch 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define PITCH_RELATIVE_POSITION_PID_KP         1.0f
#define PITCH_RELATIVE_POSITION_PID_KI          0.0f
#define PITCH_RELATIVE_POSITION_PID_KD          2.0f
#define PITCH_RELATIVE_POSITION_PID_MAX_OUT   400.0f
#define PITCH_RELATIVE_POSITION_PID_MAX_IOUT    0.0f

//yaw 角度环 角度由编码器 PID参数以及 PID最大输出，积分输出
#define YAW_RELATIVE_POSITION_PID_KP           0.15f
#define YAW_RELATIVE_POSITION_PID_KI           0.0f
#define YAW_RELATIVE_POSITION_PID_KD           2.5f
#define YAW_RELATIVE_POSITION_PID_MAX_OUT     400.0f
#define YAW_RELATIVE_POSITION_PID_MAX_IOUT      0.0f


typedef enum
{
    GIMBAL_INIT = 0,   //初始化模式
    GIMBAL_RELAX,      //失能模式
    GIMBAL_STOP,       //停止模式
    GIMBAL_GYRO,       //陀螺仪控制模式
    GIMBAL_KEY_TO_ALIGN,//一键回中模式
    GIMBAL_ENCONDE,    //机械角控制模式

} gimbal_motor_mode_e;


typedef struct
{
    uint8_t  data_mode;
	
	  float kp;
    float ki;
    float kd;

    float set;
    float fdb;
    float err[2];

    float max_out;
    float max_iout;

    float Pout;
    float Iout;
    float Dout;

    float out;
} Gimbal_PID_t;


typedef  struct
{
    const motor_data_t   *gimbal_motor_data;    //电机基础数据

    Gimbal_PID_t    gimbal_relative_position_pid;    //机械角位置PID
    Gimbal_PID_t    gimbal_absolute_position_pid;    //陀螺仪位置PID
    Gimbal_PID_t    gimbal_relative_speed_pid;       //机械角速度PID
    Gimbal_PID_t    gimbal_absolute_speed_pid;       //陀螺仪速度PID

    float relative_angle;       //当前机械相对角  
    float relative_angle_set;   //当前机械相对角设定

    float absolute_angle;       //当前陀螺仪绝对角（-180 ~ 180 度）
    float absolute_angle_set;   //当前陀螺仪绝对角设定

    float relative_speed;        //当前机械转速（度/s）
    float relative_speed_set;    //当前机械转速设定（度/s）

    float absolute_speed;        //当前陀螺仪转速（度/s）
    float absolute_speed_set;    //当前陀螺仪转速设定（度/s）

    float max_relative_angle;    //机械角最大限制角度
    float min_relative_angle;    //机械角最小限制角度

    int16_t  give_current;

} Gimbal_Motor_t;


typedef  struct
{
    gimbal_motor_mode_e    gimbal_mode;        //云台当前模式
    gimbal_motor_mode_e    last_gimbal_mode;   //云台上次模式

    const RC_ctrl_t       *gimbal_rc_ctrl;          //云台遥控器指针

    const float            *gimbal_INS_angle_point;  //陀螺仪角度数据指针
    const float            *gimbal_INS_gyro_point;   //陀螺仪角速度数据指针

    Gimbal_Motor_t      gimbal_yaw_motor;       //YAW轴电机数据包
    Gimbal_Motor_t      gimbal_pitch_motor;     //pitch轴电机数据包

} Gimbal_Control_t;


void gimbal_task(void *pvParameters);
float  Get_YAWmotor_relative_angle(void);
static float RAD_Format(float rad);
#endif

