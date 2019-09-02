/**
  ****************************(C) COPYRIGHT 2019 IronSprit***********************
  * @file       gimbal_task.c/h
	
  * @brief      完成云台控制任务.
	
  * @note       遇到一个很奇怪的问题，在云台任务里面调用pid.c里面的pid_init会进入stm32
	              硬件错误中断死循环，而云台任务的调用方法与底盘一样，故不知是何原因。
								
								解决办法：在云台任务里重新写一遍PID相关程序，调用GIMBAL_PID_Init不会
								          出错
  * @history

  *  Version       Date            Author          status
  *  V2.0.0      2019-7-9          Young            完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 IronSprit************************
*/

#include "Gimbal_Task.h"
#include "gimbal_behaviour.h"
#include "arm_math.h"
#include "user_lib.h"
#include "Detect_Task.h"
#include "FreeRTOS.h"
#include "task.h"


#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }
		
		
//云台控制所有相关数据
static Gimbal_Control_t     gimbal_move;

//云台初始化
static void GIMBAL_Init(Gimbal_Control_t  *gimbal_init);
//云台状态设置
static void GIMBAL_Set_Mode(Gimbal_Control_t *gimbal_set_mode);
//云台数据更新
static void GIMBAL_Feedback_Update(Gimbal_Control_t *gimbal_feedback_update);
//云台状态切换保存数据
static void GIMBAL_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change);
//设置云台控制量
static void GIMBAL_Set_Contorl(Gimbal_Control_t *gimbal_set_control);
//云台控制pid计算
static void GIMBAL_Control_loop(Gimbal_Control_t *gimbal_control_loop);
//云台PID总清除
static void  gimbal_total_pid_clear(Gimbal_Control_t *gimbal_clear);
//云台电机机械角变化时所对应的相对角度计算
static float motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);
//电机机械角数据转变为角速度
static void MechanicalAngle_ConvertTo_AngularVelocity(Gimbal_Motor_t  *gimbal_motor);
//对控制的目标值进限制以防超最大相对角度
static void GIMBAL_relative_angle_limit(Gimbal_Motor_t *gimbal_motor, float add);
static void GIMBAL_Yaw_absolute_angle_limit(Gimbal_Motor_t *gimbal_motor, float add);
static void GIMBAL_Pitch_absolute_angle_limit(Gimbal_Motor_t *gimbal_motor, float add);
//初始化云台PID
static void GIMBAL_PID_Init(Gimbal_PID_t *pid, uint8_t data_mode, float maxout, float max_iout, float kp, float ki, float kd);
//云台pid数据计算
static float GIMBAL_PID_Calc(Gimbal_PID_t *pid, float fdb, float set);
//云台pid数据清理
static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear);




//开始云台主任务
void gimbal_task(void *pvParameters)
{
    //初始化延时
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //云台初始化
    GIMBAL_Init( &gimbal_move);

    //判断电机是否都上线
    while ( toe_is_error(YawGimbalMotorTOE) || toe_is_error(PitchGimbalMotorTOE) )
    {
        vTaskDelay(GIMBAL_CONTROL_TIME);
    }

    while (1)
    {
        //设置云台控制模式
        GIMBAL_Set_Mode(&gimbal_move);
        //控制模式切换 控制数据过渡
        GIMBAL_Mode_Change_Control_Transit(&gimbal_move);
        //云台数据反馈
        GIMBAL_Feedback_Update(&gimbal_move);
        //设置云台控制量
        GIMBAL_Set_Contorl(&gimbal_move);
        //云台控制PID计算
        GIMBAL_Control_loop(&gimbal_move);

        if ( gimbal_move.gimbal_mode == GIMBAL_RELAX )
        {
            CAN_CMD_GIMBAL(0,0,NULL,NULL);
        }
        else
        {
            CAN_CMD_GIMBAL(gimbal_move.gimbal_yaw_motor.give_current, gimbal_move.gimbal_pitch_motor.give_current, NULL ,NULL);
        }

        vTaskDelay(GIMBAL_CONTROL_TIME);
    }
}



/*-------------------------------------返回YAW轴电机机械值相对角---------------------------------*/


//返回电机数据包指针
float  Get_YAWmotor_relative_angle(void)
{
    return  gimbal_move.gimbal_yaw_motor.relative_angle ;
}



/**
  * @brief          云台初始化
  * @author         Young
  * @param[in]      数据类型为Gimbal_Control_t的变量
  * @retval         无
  * @waring         yaw、pitch电机初始化时机械角度给定为0，不直接给相应电机的机械角度，避免出现yaw电机先往右靠，再回中的现象
  */
static void GIMBAL_Init(Gimbal_Control_t *gimbal_init)
{
    //电机数据指针获取
    gimbal_init->gimbal_yaw_motor.gimbal_motor_data   = get_Yaw_Gimbal_Motor_Data_Point();
    gimbal_init->gimbal_pitch_motor.gimbal_motor_data = get_Pitch_Gimbal_Motor_Data_Point();

    //陀螺仪数据指针获取
    gimbal_init->gimbal_INS_angle_point = get_MPU6050_Angle_point();
    gimbal_init->gimbal_INS_gyro_point  = get_MPU6050_Gyro_Point();

    //遥控器数据指针获取
    gimbal_init->gimbal_rc_ctrl = get_remote_control_point();

    //初始化云台模式
    gimbal_init->gimbal_mode = gimbal_init->last_gimbal_mode = GIMBAL_INIT;

    //初始化yaw电机pid
    GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_absolute_position_pid, DATA_GYRO,  YAW_ABSOLUTE_POSITION_PID_MAX_OUT, YAW_ABSOLUTE_POSITION_PID_MAX_IOUT, YAW_ABSOLUTE_POSITION_PID_KP, YAW_ABSOLUTE_POSITION_PID_KI, YAW_ABSOLUTE_POSITION_PID_KD);
    GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_relative_position_pid, DATA_GYRO,  YAW_RELATIVE_POSITION_PID_MAX_OUT, YAW_RELATIVE_POSITION_PID_MAX_IOUT, YAW_RELATIVE_POSITION_PID_KP, YAW_RELATIVE_POSITION_PID_KI, YAW_RELATIVE_POSITION_PID_KD);
    GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_absolute_speed_pid,    DATA_NORMAL,YAW_ABSOLUTE_SPEED_PID_MAX_OUT,    YAW_ABSOLUTE_SPEED_PID_MAX_IOUT,    YAW_ABSOLUTE_SPEED_PID_KP,    YAW_ABSOLUTE_SPEED_PID_KI,    YAW_ABSOLUTE_SPEED_PID_KD);
    GIMBAL_PID_Init(&gimbal_init->gimbal_yaw_motor.gimbal_relative_speed_pid,    DATA_NORMAL,YAW_RELATIVE_SPEED_PID_MAX_OUT,    YAW_RELATIVE_SPEED_PID_MAX_IOUT,    YAW_RELATIVE_SPEED_PID_KP,    YAW_RELATIVE_SPEED_PID_KI,    YAW_RELATIVE_SPEED_PID_KD);

    //初始化pitch电机pid
    GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_absolute_position_pid, DATA_GYRO,   PITCH_ABSOLUTE_POSITION_PID_MAX_OUT, PITCH_ABSOLUTE_POSITION_PID_MAX_IOUT, PITCH_ABSOLUTE_POSITION_PID_KP, PITCH_ABSOLUTE_POSITION_PID_KI, PITCH_ABSOLUTE_POSITION_PID_KD);
    GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_relative_position_pid, DATA_GYRO,   PITCH_RELATIVE_POSITION_PID_MAX_OUT, PITCH_RELATIVE_POSITION_PID_MAX_IOUT, PITCH_RELATIVE_POSITION_PID_KP, PITCH_RELATIVE_POSITION_PID_KI, PITCH_RELATIVE_POSITION_PID_KD);
    GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_absolute_speed_pid,    DATA_NORMAL, PITCH_ABSOLUTE_SPEED_PID_MAX_OUT,    PITCH_ABSOLUTE_SPEED_PID_MAX_IOUT,    PITCH_ABSOLUTE_SPEED_PID_KP,    PITCH_ABSOLUTE_SPEED_PID_KI,    PITCH_ABSOLUTE_SPEED_PID_KD);
    GIMBAL_PID_Init(&gimbal_init->gimbal_pitch_motor.gimbal_relative_speed_pid,    DATA_NORMAL, PITCH_RELATIVE_SPEED_PID_MAX_OUT,    PITCH_RELATIVE_SPEED_PID_MAX_IOUT,    PITCH_RELATIVE_SPEED_PID_KP,    PITCH_RELATIVE_SPEED_PID_KI,    PITCH_RELATIVE_SPEED_PID_KD);

    //定义YAW PITCH电机极限偏角
    gimbal_init->gimbal_yaw_motor.max_relative_angle = MAX_YAW_RANGLE_ANGLE;
    gimbal_init->gimbal_yaw_motor.min_relative_angle = MIN_YAW_RANGLE_ANGLE;

    gimbal_init->gimbal_pitch_motor.max_relative_angle = MAX_PITCH_RANGLE_ANGLE;
    gimbal_init->gimbal_pitch_motor.min_relative_angle = MIN_PITCH_RANGLE_ANGLE;

    //清除所有PID,数据准备
    gimbal_total_pid_clear(gimbal_init);

    GIMBAL_Feedback_Update(gimbal_init);

    gimbal_init->gimbal_yaw_motor.absolute_angle_set = gimbal_init->gimbal_yaw_motor.absolute_angle;
    gimbal_init->gimbal_yaw_motor.relative_angle_set = 0;
    gimbal_init->gimbal_yaw_motor.absolute_speed_set = gimbal_init->gimbal_yaw_motor.absolute_speed;
    gimbal_init->gimbal_yaw_motor.relative_speed_set = gimbal_init->gimbal_yaw_motor.relative_speed;

    gimbal_init->gimbal_pitch_motor.absolute_angle_set = gimbal_init->gimbal_pitch_motor.absolute_angle;
    gimbal_init->gimbal_pitch_motor.relative_angle_set = 0;
    gimbal_init->gimbal_pitch_motor.absolute_speed_set = gimbal_init->gimbal_pitch_motor.absolute_speed;
    gimbal_init->gimbal_pitch_motor.relative_speed_set = gimbal_init->gimbal_pitch_motor.relative_speed;
}


/*----------------------------云台电机PID函数---------------------------------------*/


//初始化云台PID
static void GIMBAL_PID_Init(Gimbal_PID_t *pid, uint8_t data_mode, float maxout, float max_iout, float kp, float ki, float kd)
{
    if (pid == NULL)
    {
        return;
    }
		pid->data_mode = data_mode;
		
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->err[0] = pid->err[1] = pid->fdb = 0.0f;
    pid->Iout = pid->Dout = pid->Pout = pid->out = 0.0f;
		
    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

//云台pid数据计算
static float GIMBAL_PID_Calc(Gimbal_PID_t *pid, float fdb, float set)
{
	  if (pid == NULL)
    {
        return 0.0f;
    }
		
		pid->err[1] = pid->err[0];
		
    pid->fdb = fdb;
    pid->set = set;

    pid->err[0] = set - fdb;
		
    if(pid->data_mode == DATA_GYRO)
		{
		  if( pid->err[0] >  180.0f)  pid->err[0] -= 360.0f; 
		  if( pid->err[0] < -180.0f)  pid->err[0] += 360.0f; 
		}	
		else if(pid->data_mode == DATA_NORMAL)
		{
       //不做处理
		}
		
    pid->Pout = pid->kp * pid->err[0];
    pid->Iout += pid->ki * pid->err[0];
    pid->Dout = pid->kd * (pid->err[0]-pid->err[1]);
    LimitMax(pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
    return pid->out;
}

//云台pid数据清理
static void Gimbal_PID_clear(Gimbal_PID_t *gimbal_pid_clear)
{
    if (gimbal_pid_clear == NULL)
    {
        return;
    }
    gimbal_pid_clear->err[0] = gimbal_pid_clear->err[1] = gimbal_pid_clear->set = gimbal_pid_clear->fdb = 0.0f;
    gimbal_pid_clear->out = gimbal_pid_clear->Pout = gimbal_pid_clear->Iout = gimbal_pid_clear->Dout = 0.0f;
}

//云台PID控制数据总清除
void  gimbal_total_pid_clear(Gimbal_Control_t *gimbal_clear)
{
    Gimbal_PID_clear(&gimbal_clear->gimbal_yaw_motor.gimbal_absolute_position_pid);
    Gimbal_PID_clear(&gimbal_clear->gimbal_yaw_motor.gimbal_relative_position_pid);
    Gimbal_PID_clear(&gimbal_clear->gimbal_yaw_motor.gimbal_absolute_speed_pid);
    Gimbal_PID_clear(&gimbal_clear->gimbal_yaw_motor.gimbal_relative_speed_pid);

    Gimbal_PID_clear(&gimbal_clear->gimbal_pitch_motor.gimbal_absolute_position_pid);
    Gimbal_PID_clear(&gimbal_clear->gimbal_pitch_motor.gimbal_relative_position_pid);
    Gimbal_PID_clear(&gimbal_clear->gimbal_pitch_motor.gimbal_absolute_speed_pid);
    Gimbal_PID_clear(&gimbal_clear->gimbal_pitch_motor.gimbal_relative_speed_pid);
}


/*----------------------------云台电机PID函数---------------------------------------*/


//云台模式设定
static void GIMBAL_Set_Mode(Gimbal_Control_t  *gimbal_set_mode)
{
    if (gimbal_set_mode == NULL)
    {
        return;
    }
    gimbal_behaviour_mode_set(gimbal_set_mode);
}



//云台数据更新
static void GIMBAL_Feedback_Update(Gimbal_Control_t  *gimbal_feedback_update)
{
    if (gimbal_feedback_update == NULL)
    {
        return;
    }

    //云台数据更新
    gimbal_feedback_update->gimbal_pitch_motor.absolute_angle = *(gimbal_feedback_update->gimbal_INS_angle_point + 1);
    gimbal_feedback_update->gimbal_pitch_motor.relative_angle = motor_ecd_to_angle_change(gimbal_feedback_update->gimbal_pitch_motor.gimbal_motor_data->ecd, PITCH_OFFSET_ECD );
    gimbal_feedback_update->gimbal_pitch_motor.absolute_speed = *(gimbal_feedback_update->gimbal_INS_gyro_point + 1);
    MechanicalAngle_ConvertTo_AngularVelocity(&gimbal_feedback_update->gimbal_pitch_motor);

    gimbal_feedback_update->gimbal_yaw_motor.absolute_angle = *(gimbal_feedback_update->gimbal_INS_angle_point);
    gimbal_feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(gimbal_feedback_update->gimbal_yaw_motor.gimbal_motor_data->ecd, YAW_OFFSET_ECD );
    gimbal_feedback_update->gimbal_yaw_motor.absolute_speed = *(gimbal_feedback_update->gimbal_INS_gyro_point);
    MechanicalAngle_ConvertTo_AngularVelocity(&gimbal_feedback_update->gimbal_yaw_motor);
}



/**
  * @brief          云台电机机械编码值变化时所对应的相对角度计算
  * @author         Young
  * @param[in]      电机机械编码值
  * @param[in]      电机机械编码值的设定偏移值
  * @retval         相对机械编码值变换为角度值
  * @waring         云台电机的角度值是根据设定偏移值算出的，偏移值是初始化云台时的目标位置
  */
static float motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    int32_t relative_ecd = ecd - offset_ecd;

    if (relative_ecd > Half_ecd_range)
    {
        relative_ecd -= ecd_range;
    }
    else if (relative_ecd < -Half_ecd_range)
    {
        relative_ecd += ecd_range;
    }

    return relative_ecd * Motor_Ecd_to_Rad;
}

/**
  * @brief          电机机械角数据转变为角速度
  * @author         Young
  * @param[in]      数据类型为Gimbal_Motor_t的变量
  * @retval         返回空
  * @waring         用一阶滤波平滑通过机械角度换算的速度值
  */
static void MechanicalAngle_ConvertTo_AngularVelocity(Gimbal_Motor_t  *gimbal_motor)
{
	  int16_t  error_Angle = 0;
	  
    error_Angle = gimbal_motor->gimbal_motor_data->ecd - gimbal_motor->gimbal_motor_data->last_ecd;
		if(error_Angle <= -Half_ecd_range)  //判断是否过了EC60电机机械角度的零点。因为电机每运行到一个状态对应一个机械角度，该机械角度值固定。
		{   //例：零点左右对应值分别为8000和100。由于定时周期短，在周期内，不可能运动半圈。一圈的值为8192.
				error_Angle += ecd_range;   //若当前值为100，上一次值为8000，error_Angle为-7900，负的离谱，补偿8191，变为291，即为转过的正确的机械角度值，过了零点，转过1圈，记录转过的圈数加1
		}
		else if(error_Angle > Half_ecd_range)
		{
				error_Angle -= ecd_range;
		}
		gimbal_motor->relative_speed =  GIMBAL_ACCEL_RELATIVE_SPEED_NUM * error_Angle + (1-GIMBAL_ACCEL_RELATIVE_SPEED_NUM)*gimbal_motor->relative_speed; 
}




//云台状态切换保存，用于状态切换过渡
static void GIMBAL_Mode_Change_Control_Transit(Gimbal_Control_t *gimbal_mode_change)
{
    if( gimbal_mode_change == NULL )
    {
        return;
    }
    if( gimbal_mode_change->last_gimbal_mode == gimbal_mode_change->gimbal_mode )
    {
        return;
    }
    //模式切换数据保存
    gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle ;
    gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle ;
    gimbal_mode_change->gimbal_pitch_motor.absolute_speed_set = gimbal_mode_change->gimbal_pitch_motor.absolute_speed ;
    gimbal_mode_change->gimbal_pitch_motor.relative_speed_set = gimbal_mode_change->gimbal_pitch_motor.relative_speed ;

    gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle ;
    gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle ;
    gimbal_mode_change->gimbal_yaw_motor.absolute_speed_set = gimbal_mode_change->gimbal_yaw_motor.absolute_speed ;
    gimbal_mode_change->gimbal_yaw_motor.relative_speed_set = gimbal_mode_change->gimbal_yaw_motor.relative_speed ;

    //PID控制数据清零
    gimbal_total_pid_clear(gimbal_mode_change);

    //记录上次模式
    gimbal_mode_change->last_gimbal_mode = gimbal_mode_change->gimbal_mode;
}




//云台控制量设置
static void GIMBAL_Set_Contorl(Gimbal_Control_t  *gimbal_set_control)
{
    if (gimbal_set_control == NULL)
    {
        return;
    }

    float add_yaw_angle   = 0.0f;
    float add_pitch_angle = 0.0f;

    //角度增量设定
    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, gimbal_set_control);

    //角度增量赋值及限幅
    if( gimbal_set_control->gimbal_mode == GIMBAL_STOP  || gimbal_set_control->gimbal_mode == GIMBAL_ENCONDE ||
            gimbal_set_control->gimbal_mode == GIMBAL_INIT  )
    {
        GIMBAL_relative_angle_limit( &gimbal_set_control->gimbal_yaw_motor, add_yaw_angle);
        GIMBAL_relative_angle_limit( &gimbal_set_control->gimbal_pitch_motor, add_pitch_angle);
    }
    else if( gimbal_set_control->gimbal_mode == GIMBAL_GYRO)
    {
        GIMBAL_Yaw_absolute_angle_limit( &gimbal_set_control->gimbal_yaw_motor, add_yaw_angle);
        GIMBAL_Pitch_absolute_angle_limit( &gimbal_set_control->gimbal_pitch_motor, -add_pitch_angle);//因为陀螺仪pitch与pitch轴电机机械角变化方向不一样,所以要改变变量方向
    }
}



//陀螺仪yaw角度控制量给定及限制
#ifdef USED_GYRO
static void GIMBAL_Yaw_absolute_angle_limit(Gimbal_Motor_t *gimbal_motor, float add)
{
    static float bias_angle;

    if (gimbal_motor == NULL)
    {
        return;
    }
    //计算当前控制误差角度
    bias_angle = RAD_Format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);

    //云台相对角度+ 误差角度 + 新增角度 如果大于 最大机械角度
    if (gimbal_motor->relative_angle + bias_angle + add >  (gimbal_motor->max_relative_angle))
    {
        //如果是往最大机械角度控制方向
        if (add > 0.0f)
        {
            add = (gimbal_motor->max_relative_angle) - gimbal_motor->relative_angle - bias_angle;
        }
    }
    else if (gimbal_motor->relative_angle + bias_angle + add < (gimbal_motor->min_relative_angle) )
    {
        if (add < 0.0f)
        {
            add = (gimbal_motor->min_relative_angle)- gimbal_motor->relative_angle - bias_angle;
        }
    }

    gimbal_motor->absolute_angle_set = RAD_Format( (gimbal_motor->absolute_angle_set) + add );
}
#endif


//陀螺仪pitch角度控制量给定及限制，因为陀螺仪pitch与pitch轴电机机械角变化方向不一样,所以要改变变量方向
#ifdef USED_GYRO
static void GIMBAL_Pitch_absolute_angle_limit(Gimbal_Motor_t *gimbal_motor, float add)
{
    static float bias_angle;

    if (gimbal_motor == NULL)
    {
        return;
    }
    //计算当前控制误差角度
    bias_angle = RAD_Format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);

    //云台相对角度+ 误差角度 + 新增角度 如果大于 最大机械角度
    if (-gimbal_motor->relative_angle + bias_angle + add >  (gimbal_motor->max_relative_angle))
    {
        //如果是往最大机械角度控制方向
        if (add > 0.0f)
        {
            add = (gimbal_motor->max_relative_angle) + gimbal_motor->relative_angle - bias_angle;
        }
    }
    else if (-gimbal_motor->relative_angle + bias_angle + add < (gimbal_motor->min_relative_angle) )
    {
        if (add < 0.0f)
        {
            add = (gimbal_motor->min_relative_angle)+ gimbal_motor->relative_angle - bias_angle;
        }
    }
    gimbal_motor->absolute_angle_set = RAD_Format( (gimbal_motor->absolute_angle_set) + add );
}
#endif


//陀螺仪角度规整
#ifdef USED_GYRO
static float RAD_Format(float rad)
{
    if ( rad > Half_rad_range)
    {
        rad -= rad_range;
    }
    else if (rad < (-Half_rad_range))
    {
        rad += rad_range;
    }
    return 	rad;
}
#endif


//机械角角度控制量给定及限制
static void GIMBAL_relative_angle_limit(Gimbal_Motor_t *gimbal_motor, float add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->relative_angle_set += add;

    //是否超过最大 最小值
    if (gimbal_motor->relative_angle_set > (gimbal_motor->max_relative_angle) )
    {
        gimbal_motor->relative_angle_set = gimbal_motor->max_relative_angle;
    }
    else if (gimbal_motor->relative_angle_set < (gimbal_motor->min_relative_angle) )
    {
        gimbal_motor->relative_angle_set = gimbal_motor->min_relative_angle;
    }
}



/**
  * @brief          云台控制状态使用不同控制pid
  * @author         Young
  * @param[in]      数据类型为Gimbal_Control_t变量
  * @retval         返回空
  * @waring         位置外环，速度内环
                    云台pitch轴向上和向下需要的力不一样，减去500是为了在PID输出为0时，能基本稳住云台pitch,其实就是平分向上和向下力的偏移量
  */
static void GIMBAL_Control_loop( Gimbal_Control_t  *gimbal_control_loop )
{
    if (gimbal_control_loop == NULL)
    {
        return;
    }
		
		//yaw不同模式对于不同的控制函数
    if ( gimbal_control_loop->gimbal_mode == GIMBAL_INIT || gimbal_control_loop->gimbal_mode == GIMBAL_STOP || gimbal_control_loop->gimbal_mode == GIMBAL_ENCONDE )
    {

        /*YAW轴*/
				gimbal_control_loop->gimbal_yaw_motor.relative_speed_set = GIMBAL_PID_Calc( &gimbal_control_loop->gimbal_yaw_motor.gimbal_relative_position_pid,
								gimbal_control_loop->gimbal_yaw_motor.relative_angle,
								gimbal_control_loop->gimbal_yaw_motor.relative_angle_set ) ;

				gimbal_control_loop->gimbal_yaw_motor.give_current = GIMBAL_PID_Calc( &gimbal_control_loop->gimbal_yaw_motor.gimbal_relative_speed_pid,
								gimbal_control_loop->gimbal_yaw_motor.relative_speed,
								gimbal_control_loop->gimbal_yaw_motor.relative_speed_set);

				/*PITCH轴*/
				gimbal_control_loop->gimbal_pitch_motor.relative_speed_set = GIMBAL_PID_Calc( &gimbal_control_loop->gimbal_pitch_motor.gimbal_relative_position_pid,
								gimbal_control_loop->gimbal_pitch_motor.relative_angle,
								gimbal_control_loop->gimbal_pitch_motor.relative_angle_set ) ;

				gimbal_control_loop->gimbal_pitch_motor.give_current = GIMBAL_PID_Calc( &gimbal_control_loop->gimbal_pitch_motor.gimbal_relative_speed_pid,
								gimbal_control_loop->gimbal_pitch_motor.relative_speed,
								gimbal_control_loop->gimbal_pitch_motor.relative_speed_set );  

    }
    else if ( gimbal_control_loop->gimbal_mode == GIMBAL_GYRO)
    {
        /*YAW轴*/
        gimbal_control_loop->gimbal_yaw_motor.absolute_speed_set = GIMBAL_PID_Calc( &gimbal_control_loop->gimbal_yaw_motor.gimbal_absolute_position_pid,
                gimbal_control_loop->gimbal_yaw_motor.absolute_angle,
                gimbal_control_loop->gimbal_yaw_motor.absolute_angle_set ) ;

        gimbal_control_loop->gimbal_yaw_motor.give_current = GIMBAL_PID_Calc( &gimbal_control_loop->gimbal_yaw_motor.gimbal_absolute_speed_pid,
                gimbal_control_loop->gimbal_yaw_motor.absolute_speed,
                gimbal_control_loop->gimbal_yaw_motor.absolute_speed_set) ;
          

  			/*PITCH轴*/
        gimbal_control_loop->gimbal_pitch_motor.absolute_speed_set = GIMBAL_PID_Calc( &gimbal_control_loop->gimbal_pitch_motor.gimbal_absolute_position_pid,
                gimbal_control_loop->gimbal_pitch_motor.absolute_angle,
                gimbal_control_loop->gimbal_pitch_motor.absolute_angle_set ) ;

        gimbal_control_loop->gimbal_pitch_motor.give_current = -GIMBAL_PID_Calc( &gimbal_control_loop->gimbal_pitch_motor.gimbal_absolute_speed_pid,
                gimbal_control_loop->gimbal_pitch_motor.absolute_speed,
                gimbal_control_loop->gimbal_pitch_motor.absolute_speed_set)-500;   //因为陀螺仪pitch与pitch轴电机机械角变化方向不一样,所以要改变变量方向   
    }
}


