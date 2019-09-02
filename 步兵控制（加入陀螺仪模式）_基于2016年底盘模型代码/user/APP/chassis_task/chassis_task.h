#ifndef CHASSISTASK_H
#define CHASSISTASK_H

#include "arm_math.h"
#include "CAN_Receive.h"
#include "pid.h"
#include "Remote_Control.h"
#include "user_lib.h"
#include "Gimbal_Task.h"

//任务开始空闲时间间隔
#define CHASSIS_TASK_INIT_TIME    357
//底盘任务控制间隔  ms
#define CHASSIS_CONTROL_TIME_MS    2
//底盘任务控制间隔   s
#define CHASSIS_CONTROL_TIME_S    0.002


//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL           2
//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL           3
//旋转的遥控器通道号码
#define CHASSIS_Z_CHANNEL           0
//遥控器模式通道开关
#define CHASSIS_MODE_SWITCH         0


//前后左右键盘按键定义
#define CHASSIS_FRONT_KEY     KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY      KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY      KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY     KEY_PRESSED_OFFSET_D
//定义旋转按键
#define CHASSIS_TURNLEFT_KEY      KEY_PRESSED_OFFSET_Q
#define CHASSIS_TURNRIGHT_KEY     KEY_PRESSED_OFFSET_E

//定义回正按键
#define CHASSIS_STRAIGHT_KEY       KEY_PRESSED_OFFSET_F
//定义功能控制按键
#define  CHASSIS_SPEEDDOWN_KEY     KEY_PRESSED_OFFSET_CTRL
//定义加速按键
#define  CHASSIS_SPEEDUP_KEY       KEY_PRESSED_OFFSET_SHIFT


//底盘电机最大速度
#define MAX_WHEEL_SPEED               3.5f
//底盘运动过程最大前进速度(m/s)
#define NORMAL_MAX_CHASSIS_SPEED_X    2.5f
//底盘运动过程最大平移速度(m/s)
#define NORMAL_MAX_CHASSIS_SPEED_Y    2.3f
//底盘运动过程最大旋转速度(度/s)
#define NORMAL_MAX_CHASSIS_SPEED_Z  180.0f


//一阶滤波参数设定
//按照本工程中的滤波函数写法，宏定义值越小，灵敏度越大，但平稳性降低，只能在可接受的灵敏度范围内取得尽可能好的平稳度
#define CHASSIS_ACCEL_X_NUM        0.1666666667f
#define CHASSIS_ACCEL_Y_NUM        0.3333333333f
#define CHASSIS_ACCEL_Z_NUM        0.1666666667f

//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN    0.0009f
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN    0.0009f
//遥控器旋转摇杆（max 660）转化成车体旋转速度（m/s）的比例
#define CHASSIS_VZ_RC_SEN    0.0016f

//EC60转速（r/min）转化成底盘速度(m/s)的比例
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN    0.007984881329f

//遥控器死区限制
#define CHASSIS_RC_DEADLINE        10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX   0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY   0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ   0.25f

//设置云台安装比例分权(旋转速度,设置前后左右轮不同设定速度的比例分权) （0为在云台安在车中心，不需要补偿）
#define CHASSIS_WZ_SET_SCALE       0.1f

//设置车身中心轮距
#define MOTOR_DISTANCE_TO_CENTER   0.145f

//设定缓冲危险值
#define   chassis_power_danger      20.0f
//定义底盘限制功率
#define   chassis_standard_power    80.0f

//底盘电机速度PID
#define EC60_MOTOR_SPEED_PID_KP          10000.0f
#define EC60_MOTOR_SPEED_PID_KI          120.0f
#define EC60_MOTOR_SPEED_PID_KD          3000.0f
#define EC60_MOTOR_SPEED_PID_MAX_OUT     15000.0f
#define EC60_MOTOR_SPEED_PID_MAX_IOUT    4000.0f

//底盘跟随位置PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP          0.035f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI           0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD           2.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT    200.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT     0.0f


//底盘状态机设定
typedef enum
{
    CHASSIS_RELAX,
    CHASSIS_STOP,
    CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,
    CHASSIS_VECTOR_NO_FOLLOW_YAW,

} chassis_mode_e;


//单电机信息数据包
typedef struct
{
    const motor_data_t   *chassis_motor_data;
    float speed;
    float speed_set;
    int16_t give_current;

} Chassis_Motor_t;


typedef struct
{
    uint8_t  data_mode;
	
	  float Kp;
    float Ki;
    float Kd;

    float set;
    float fdb;
    float error[2];

    float max_out;
    float max_iout;

    float Pout;
    float Iout;
    float Dout;

    float out;
} Chassis_PID_t;


typedef  struct
{
    const RC_ctrl_t         *chassis_RC;          //底盘使用的遥控器指针
    //const Referee_System_t  *chassis_referee;     //底盘裁判系统数据

    chassis_mode_e         chassis_mode;          //底盘控制状态机
    chassis_mode_e         last_chassis_mode;     //底盘上次控制状态机

    Chassis_Motor_t        motor_chassis[4];      //底盘单电机数据
    Chassis_PID_t          motor_speed_pid[4];    //底盘电机速度pid
    Chassis_PID_t          chassis_follow_pid;    //底盘跟随角度pid

    first_order_filter_type_t     chassis_cmd_slow_set_vx;    //底盘前后一阶滤波
    first_order_filter_type_t     chassis_cmd_slow_set_vy;    //底盘左右一阶滤波
    first_order_filter_type_t     chassis_cmd_slow_set_vz;    //底盘旋转一阶滤波

    float vx;                         //底盘速度 前进方向 前为正，      单位 m/s
    float vy;                         //底盘速度 左右方向 左为正       单位 m/s
    float wz;                         //底盘角速度，逆时针为正           单位 度/s

    float vx_set;                     //底盘设定速度 前进方向 前为正，  单位  m/s
    float vy_set;                     //底盘设定速度 左右方向 左为正，  单位  m/s
    float wz_set;                     //底盘角速度，  逆时针为正         单位  度/s

    float last_vx_set;                //底盘设定速度 前进方向 前为正，  单位  m/s
    float last_vy_set;                //底盘设定速度 左右方向 左为正，  单位  m/s
    float last_wz_set;                //底盘角速度，  逆时针为正         单位  度/s


    float  error_aobut_gimbalYAW;        //底盘相对云台误差

} chassis_move_t;



void chassis_task(void *pvParameters);
uint8_t get_chassis_mode(void);
void chassis_rc_to_control_vector(float *vx_set, float *vy_set, chassis_move_t *chassis_move_rc_to_vector);
float CHASSIS_PID_Calc(Chassis_PID_t *pid, float fdb, float set);

#endif
