/**
  ****************************(C) COPYRIGHT 2019 IronSprit***********************
  * @file       gimbal_behaviour.c/h
  * @brief      完成云台任务行为层。
  * @note       刚上电时先回正pitch、再回正yaw,此时回正速度正常，若遥控器离线，再上线
	              时，云台回正速度就会很慢，超过初始化时间，还没回正就进入停止模式
								
								解决办法：若遥控器离线，再上线时，yaw、pitch同时回正
  * @history

  *  Version       Date            Author          status
  *  V2.0.0      2019-7-11         Young            完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 IronSprit************************
*/

#include "gimbal_behaviour.h"
#include "chassis_task.h"
#include "arm_math.h"
#include "Detect_Task.h"
#include "user_lib.h"

uint8_t  RemoteControl_Offline_Flag = 0;

#define rc_deadline_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }



/*---------------------------------------模式设定行为层------------------------------------*/


void gimbal_behaviour_mode_set(Gimbal_Control_t   *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }

    //初始化模式下
    if (gimbal_mode_set->gimbal_mode == GIMBAL_INIT)
    {
        static uint16_t init_time = 0;
        static uint16_t init_stop_time = 0;
			
        if (init_time < GIMBAL_INIT_TIME_MS)
        {
            init_time++;
        }
        //yaw、pitch电机的relative_angle是根据目的位置的偏差算出来的，当到达目的位置后，relative_angle为0
        if ( (fabsf(gimbal_mode_set->gimbal_yaw_motor.relative_angle - INIT_YAW_SET) < GIMBAL_INIT_ANGLE_ERROR) &&
                (fabsf(gimbal_mode_set->gimbal_pitch_motor.relative_angle - INIT_PITCH_SET) < GIMBAL_INIT_ANGLE_ERROR) )
        {
            //到达初始化位置
            if (init_stop_time < GIMBAL_INIT_STOP_TIME_MS)
            {
                init_stop_time++;
            }
        }
        //超过初始化最大时间6S，或者已经稳定到目标值一段时间200MS，退出初始化状态
        if ( (init_time < GIMBAL_INIT_TIME_MS) && (init_stop_time < GIMBAL_INIT_STOP_TIME_MS) )
        {
            return;
        }
        else
        {
            init_stop_time = 0;
            init_time = 0;
            gimbal_mode_set->gimbal_mode = GIMBAL_STOP;
					  RemoteControl_Offline_Flag = 0;
        }
    }
    //放松模式下
    else if( gimbal_mode_set->gimbal_mode == GIMBAL_RELAX )
    {
        if( toe_is_error(DBUSTOE)== 0 )
        {
            gimbal_mode_set->gimbal_mode = GIMBAL_INIT ;
					  RemoteControl_Offline_Flag = 1;
        }
    }
    //停止模式下
    else if( gimbal_mode_set->gimbal_mode == GIMBAL_STOP )
    {
        if( toe_is_error(DBUSTOE)== 1 )                                            gimbal_mode_set->gimbal_mode = GIMBAL_RELAX;
        if( switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[MODEL_CHANNEL]))  gimbal_mode_set->gimbal_mode = GIMBAL_GYRO;
        if( switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[MODEL_CHANNEL])) 	 gimbal_mode_set->gimbal_mode = GIMBAL_ENCONDE;
    }
		 //陀螺仪模式下
    else if( gimbal_mode_set->gimbal_mode == GIMBAL_GYRO )
    {
        if( toe_is_error(DBUSTOE)== 1 )                                            gimbal_mode_set->gimbal_mode = GIMBAL_RELAX;
        if( switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[MODEL_CHANNEL]))   gimbal_mode_set->gimbal_mode = GIMBAL_STOP;
        if( switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[MODEL_CHANNEL]))    gimbal_mode_set->gimbal_mode = GIMBAL_ENCONDE;
    }
    //机械角模式下
    else if( gimbal_mode_set->gimbal_mode == GIMBAL_ENCONDE )
    {
        if( toe_is_error(DBUSTOE)== 1 )                                                   gimbal_mode_set->gimbal_mode = GIMBAL_RELAX;
        else if( switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[MODEL_CHANNEL]))     gimbal_mode_set->gimbal_mode = GIMBAL_STOP;
        else if( switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[MODEL_CHANNEL]))    gimbal_mode_set->gimbal_mode = GIMBAL_GYRO;
        //else if( get_chassis_mode() == 1 )                                                 gimbal_mode_set->gimbal_mode = GIMBAL_KEY_TO_ALIGN;
    }
    //云台在小车能动的情况下保持回中
    else if( gimbal_mode_set->gimbal_mode == GIMBAL_KEY_TO_ALIGN )
    {
        if( toe_is_error(DBUSTOE)== 1 )                                                    gimbal_mode_set->gimbal_mode = GIMBAL_RELAX;
        else if( switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[MODEL_CHANNEL]))      gimbal_mode_set->gimbal_mode = GIMBAL_STOP;
        else if( switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[MODEL_CHANNEL]))     gimbal_mode_set->gimbal_mode = GIMBAL_GYRO;
        //else if( get_chassis_mode() == 0 )                                                 gimbal_mode_set->gimbal_mode = GIMBAL_ENCONDE;
    }
}





/*-----------------------------------------动作控制行为层-----------------------------------*/

/**
  * @brief          云台行为控制，根据不同行为采用不同控制函数
  * @author         RM
  * @param[in]      设置的yaw角度增加值，单位 rad
  * @param[in]      设置的pitch角度增加值，单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */

void gimbal_behaviour_control_set(float *add_yaw, float *add_pitch, Gimbal_Control_t *gimbal_control_set)
{

    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    static float rc_add_yaw = 0.0f, rc_add_pitch= 0.0f;
    static int16_t yaw_channel = 0, pitch_channel = 0;

    //遥控器的数据处理死区
    rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel, GIMBAL_RC_DEADLINE);
    rc_deadline_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel, GIMBAL_RC_DEADLINE);

    //遥控器+键鼠数据叠加
    rc_add_yaw = yaw_channel * YAW_RC_SEN - gimbal_control_set->gimbal_rc_ctrl->mouse.x * YAW_MOUSE_SEN;
    rc_add_pitch = pitch_channel * PITCH_RC_SEN + gimbal_control_set->gimbal_rc_ctrl->mouse.y * PITCH_MOUSE_SEN;


    if (gimbal_control_set->gimbal_mode == GIMBAL_INIT)
    {
        gimbal_init_control(&rc_add_yaw, &rc_add_pitch, gimbal_control_set);
    }
    else if (gimbal_control_set->gimbal_mode == GIMBAL_RELAX)
    {
        gimbal_relax_control(&rc_add_yaw, &rc_add_pitch, gimbal_control_set);
    }
    else if (gimbal_control_set->gimbal_mode == GIMBAL_STOP )
    {
        gimbal_stop_control(&rc_add_yaw, &rc_add_pitch, gimbal_control_set);
    }
		else if (gimbal_control_set->gimbal_mode == GIMBAL_GYRO )
    {
        gimbal_gyro_control(&rc_add_yaw, &rc_add_pitch, gimbal_control_set);
    }
    else if (gimbal_control_set->gimbal_mode == GIMBAL_KEY_TO_ALIGN )
    {
        gimbal_key_to_align_control(&rc_add_yaw, &rc_add_pitch, gimbal_control_set);
    }
    else if (gimbal_control_set->gimbal_mode == GIMBAL_ENCONDE )
    {
        gimbal_encoder_control(&rc_add_yaw, &rc_add_pitch, gimbal_control_set);
    }

    //将控制增加量赋值
    *add_yaw = rc_add_yaw;
    *add_pitch = rc_add_pitch;
}


/**
  * @brief          当遥控器离线时，云台处于放松模式，此时云台不动
  * @author         Young
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_relax_control(float *yaw, float *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    *yaw   = 0.0f;
    *pitch = 0.0f;
}

/**
  * @brief          云台初始化模式控制
  * @author         Young
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  * @waring         上电时先回正pitch，再回正yaw。如果遥控器离线，再次上线时，pitch、yaw同时回正（此时回正速度才正常）
  */
static void gimbal_init_control(float *yaw, float *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
		
    if(RemoteControl_Offline_Flag)//当遥控器离线时，再次上线，yaw、pitch同时回正，此时云台回正速度正常
		{
		    *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.relative_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
		}
		else
		{
		    //初始化状态控制量计算
				if ( fabsf(INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.relative_angle) > GIMBAL_INIT_ANGLE_ERROR )
				{
						*pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.relative_angle) * GIMBAL_INIT_PITCH_SPEED;
						*yaw = 0.0f;
				}
				else
				{
						*pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.relative_angle) * GIMBAL_INIT_PITCH_SPEED;
						*yaw = (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
				}
		}
}

/**
  * @brief          云台停止模式控制，此时云台不动
  * @author         Young
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_stop_control(float *yaw, float *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    *yaw =    0.0f;
    *pitch =  0.0f;
}

/**
  * @brief          云台陀螺仪控制，电机是陀螺仪角度控制，
  * @author         Young
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
#ifdef USED_GYRO
static void gimbal_gyro_control(float *yaw, float *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    //不需要处理，可加入一键调头代码
}
#endif


/**
  * @brief          云台一键对齐控制
  * @author         Young
  * @param[in]      yaw轴角度控制，为角度的增量 单位 rad
  * @param[in]      pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_key_to_align_control(float *yaw, float *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    
		*pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.relative_angle) * GIMBAL_INIT_PITCH_SPEED;
		*yaw = (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
}


/**
  * @brief          云台编码值控制，电机是相对角度控制，
  * @author         Young
  * @param[in]      yaw轴角度控制，为角度的增量 单位 度
  * @param[in]      pitch轴角度控制，为角度的增量 单位 度
  * @param[in]      云台数据指针
  * @retval         返回空
  */
static void gimbal_encoder_control(float *yaw, float *pitch, Gimbal_Control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    //不需要处理，
}
