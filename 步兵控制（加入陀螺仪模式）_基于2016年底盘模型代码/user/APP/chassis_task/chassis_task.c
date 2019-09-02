/**
  ****************************(C) COPYRIGHT 2019 IronSprit***********************
  * @file       chassis.c/h

  * @brief      完成底盘控制任务。

  * @note       1、枚举可以直接当宏定义使用，我第一次见这种不规则用法
	              2、建立麦轮底盘的逆运动学模型实现全向移动
								3、按照本工程中的滤波函数写法，宏定义值越小，灵敏度越大，但平稳性
								   降低，只能在可接受的灵敏度范围内取得尽可能好的平稳度

  * @history

  *  Version       Date            Author          status
  *  V2.0.0       2019-6-16        Young            完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 IronSprit************************
*/

#include "chassis_task.h"
#include "chassis_behaviour.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Detect_Task.h"
#include "stdlib.h"


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
		
//死区限制
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



//底盘运动数据
static chassis_move_t   chassis_move;

//底盘初始化
static void chassis_init(chassis_move_t *chassis_move_init);
//底盘状态机选择
static void chassis_set_mode(chassis_move_t *chassis_move_mode);
//底盘数据更新
static void chassis_feedback_update(chassis_move_t *chassis_move_update);
//底盘状态改变后处理
void chassis_mode_change_control_transit(chassis_move_t *chassis_move_transit);
//底盘设置根据遥控器控制量
static void chassis_set_contorl(chassis_move_t *chassis_move_control);
//底盘PID计算以及运动分解
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop);

//四轮速度分配
static void chassis_vector_to_mecanum_wheel_speed(const float vx_set, const float vy_set, const float wz_set, float wheel_speed[4]);
//底盘PID相关函数
static void CHASSIS_PID_Init(Chassis_PID_t *pid, uint8_t data_mode, float maxout, float max_iout, float kp, float ki, float kd);
float CHASSIS_PID_Calc(Chassis_PID_t *pid, float fdb, float set);
static void CHASSIS_PID_clear(Chassis_PID_t *chassis_pid_clear);

/*------------------------------------底盘主任务-------------------------------------*/


void chassis_task(void *pvParameters)
{
    //空闲一段时间
    vTaskDelay(CHASSIS_TASK_INIT_TIME);

    //底盘初始化
    chassis_init(&chassis_move);

    //判断底盘电机是否都在线
    while (toe_is_error(ChassisMotor1TOE) || toe_is_error(ChassisMotor2TOE) || toe_is_error(ChassisMotor3TOE) || toe_is_error(ChassisMotor4TOE) || toe_is_error(DBUSTOE))
    {
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);
    }

    while (1)
    {
        //遥控器设置状态
        chassis_set_mode(&chassis_move);
        //遥控器状态切换数据保存
        chassis_mode_change_control_transit(&chassis_move);
        //底盘数据更新
        chassis_feedback_update(&chassis_move);
        //底盘控制量设置
        chassis_set_contorl(&chassis_move);
        //底盘控制PID计算
        chassis_control_loop(&chassis_move);

        CAN_CMD_CHASSIS(CAN_CHASSIS_LOW_ID,0, chassis_move.motor_chassis[1].give_current,chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
        CAN_CMD_CHASSIS(CAN_CHASSIS_HIGH_ID,0,chassis_move.motor_chassis[0].give_current,0,0);

        //系统延时
        vTaskDelay(CHASSIS_CONTROL_TIME_MS);

    }
}



/*----------------------------底盘电机PID函数---------------------------------------*/


//初始化底盘PID
static void CHASSIS_PID_Init(Chassis_PID_t *pid, uint8_t data_mode, float maxout, float max_iout, float kp, float ki, float kd)
{
    if (pid == NULL)
    {
        return;
    }
		pid->data_mode = data_mode;
		
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

    pid->error[0] = pid->error[1] = pid->fdb = 0.0f;
    pid->Iout = pid->Dout = pid->Pout = pid->out = 0.0f;
		
    pid->max_iout = max_iout;
    pid->max_out = maxout;
}

//云台pid数据计算
float CHASSIS_PID_Calc(Chassis_PID_t *pid, float fdb, float set)
{
	  if (pid == NULL)
    {
        return 0.0f;
    }
		
		pid->error[1] = pid->error[0];
		
    pid->fdb = fdb;
    pid->set = set;

    pid->error[0] = set - fdb;
		
    if(pid->data_mode == DATA_GYRO)
		{
		  if( pid->error[0] >  180.0f)  pid->error[0] -= 360.0f; 
		  if( pid->error[0] < -180.0f)  pid->error[0] += 360.0f; 
		}	
		else if(pid->data_mode == DATA_NORMAL)
		{
       //不做处理
		}
		
    pid->Pout = pid->Kp * pid->error[0];
    pid->Iout += pid->Ki * pid->error[0];
    pid->Dout = pid->Kd * (pid->error[0]-pid->error[1]);
    LimitMax(pid->Iout, pid->max_iout);
    pid->out = pid->Pout + pid->Iout + pid->Dout;
    LimitMax(pid->out, pid->max_out);
    return pid->out;
}

//云台pid数据清理
static void CHASSIS_PID_clear(Chassis_PID_t *chassis_pid_clear)
{
    if (chassis_pid_clear == NULL)
    {
        return;
    }
    chassis_pid_clear->error[0] = chassis_pid_clear->error[1] = chassis_pid_clear->set = chassis_pid_clear->fdb = 0.0f;
    chassis_pid_clear->out = chassis_pid_clear->Pout = chassis_pid_clear->Iout = chassis_pid_clear->Dout = 0.0f;
}



/*--------------------------------返回底盘状态机-----------------------------------*/

 uint8_t get_chassis_mode(void)
{
   if( chassis_move.chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW ) 
	 {
	   return 1;
	 } 
	 else
	 {
	   return 0;
	 }
}

/*-----------------------------------底盘初始化------------------------------------------*/


static void chassis_init(chassis_move_t  *chassis_move_init)
{
    if (chassis_move_init == NULL)
    {
        return;
    }

    //一阶滤波参数值
    const static float chassis_x_order_filter[1] = {CHASSIS_ACCEL_X_NUM};
    const static float chassis_y_order_filter[1] = {CHASSIS_ACCEL_Y_NUM};

    //底盘开机状态为停止
    chassis_move_init->chassis_mode = CHASSIS_STOP;

    //获取遥控器指针
    chassis_move_init->chassis_RC = get_remote_control_point();

    //初始化底盘电机PID
    for ( uint8_t i = 0; i < 4; i++)
    {
        chassis_move_init->motor_chassis[i].chassis_motor_data = get_Chassis_Motor_Data_Point(i);
        CHASSIS_PID_Init(&chassis_move_init->motor_speed_pid[i], DATA_NORMAL, EC60_MOTOR_SPEED_PID_MAX_OUT, EC60_MOTOR_SPEED_PID_MAX_IOUT,EC60_MOTOR_SPEED_PID_KP,EC60_MOTOR_SPEED_PID_KI,EC60_MOTOR_SPEED_PID_KD);
    }
    //初始化底盘跟随云台PID
		CHASSIS_PID_Init(&chassis_move_init->chassis_follow_pid, DATA_GYRO, CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT, CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT,CHASSIS_FOLLOW_GIMBAL_PID_KP,CHASSIS_FOLLOW_GIMBAL_PID_KI,CHASSIS_FOLLOW_GIMBAL_PID_KD);
		
    //用一阶滤波代替斜波函数生成
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vx, CHASSIS_CONTROL_TIME_S, chassis_x_order_filter);
    first_order_filter_init(&chassis_move_init->chassis_cmd_slow_set_vy, CHASSIS_CONTROL_TIME_S, chassis_y_order_filter);

    //更新一下数据
    chassis_feedback_update(chassis_move_init);
}



/*---------------------------------底盘模式设定----------------------------------------------*/


/*模式设定*/
static void chassis_set_mode(chassis_move_t *chassis_move_mode)
{
    if (chassis_move_mode == NULL)
    {
        return;
    }

    chassis_behaviour_mode_set(chassis_move_mode);
}



/*--------------------------------模式切换数据处理------------------------------------------*/


void chassis_mode_change_control_transit(chassis_move_t  *chassis_move_transit)
{
    if (chassis_move_transit == NULL)
    {
        return;
    }

    if (chassis_move_transit->last_chassis_mode == chassis_move_transit->chassis_mode)
    {
        return;
    }

    //底盘跟随PID清零
    CHASSIS_PID_clear(&chassis_move_transit->chassis_follow_pid);

    //特定数据清零
    chassis_move_transit->vx_set = 0;
    chassis_move_transit->vy_set = 0;
    chassis_move_transit->wz_set = 0;

    chassis_move_transit->last_chassis_mode = chassis_move_transit->chassis_mode;
}


/*---------------------------------------底盘数据跟新-----------------------------------*/



static void chassis_feedback_update(chassis_move_t *chassis_move_update)
{
    int16_t  error_Angle = 0;

    if (chassis_move_update == NULL)
    {
        return;
    }

    for (uint8_t i = 0; i < 4; i++)
    {
        //更新电机速度
        error_Angle = chassis_move_update->motor_chassis[i].chassis_motor_data->ecd - chassis_move_update->motor_chassis[i].chassis_motor_data->last_ecd;
        if(error_Angle <= -4096)  //判断是否过了EC60电机机械角度的零点。因为电机每运行到一个状态对应一个机械角度，该机械角度值固定。
        {   //例：零点左右对应值分别为8000和100。由于定时周期短，在周期内，不可能运动半圈。一圈的值为8192.
            error_Angle += 8191;   //若当前值为100，上一次值为8000，error_Angle为-7900，负的离谱，补偿8192，变为292，即为转过的正确的机械角度值，过了零点，转过1圈，记录转过的圈数加1
        }
        else if(error_Angle > 4096)
        {
            error_Angle -= 8191;
        }
        chassis_move_update->motor_chassis[i].speed = CHASSIS_MOTOR_RPM_TO_VECTOR_SEN * (error_Angle+chassis_move_update->motor_chassis[i].speed/2);
    }

    //更新底盘前进速度 x， 平移速度y，旋转速度wz，坐标系为右手系
    chassis_move_update->vx = (-chassis_move_update->motor_chassis[0].speed + chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VX;
    chassis_move_update->vy = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed + chassis_move_update->motor_chassis[2].speed + chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_VY;
    chassis_move_update->wz = (-chassis_move_update->motor_chassis[0].speed - chassis_move_update->motor_chassis[1].speed - chassis_move_update->motor_chassis[2].speed - chassis_move_update->motor_chassis[3].speed) * MOTOR_SPEED_TO_CHASSIS_SPEED_WZ / MOTOR_DISTANCE_TO_CENTER;
    
		//底盘相对云台误差角度
		chassis_move_update->error_aobut_gimbalYAW = Get_YAWmotor_relative_angle();
		
    chassis_move_update->last_vx_set = chassis_move_update->vx_set;
    chassis_move_update->last_vy_set = chassis_move_update->vy_set;
    chassis_move_update->last_wz_set = chassis_move_update->wz_set;
}



/*---------------------------------------底盘控制量输入-------------------------------------*/


//设置输入控制量
static void chassis_set_contorl(chassis_move_t  *chassis_move_control)
{
    if (chassis_move_control == NULL)
    {
        return;
    }

    //设置速度
    float vx_set = 0.0f, vy_set = 0.0f, wz_set = 0.0f;

    chassis_behaviour_control_set(&vx_set, &vy_set, &wz_set, chassis_move_control);

    //跟随云台模式
    if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW)
    {
        /*以云台朝向建立右手坐标系1，以底盘朝向建立右手坐标系2，遥控器控制云台转动，故vx_set、vy_set是基于云台坐标系的，先求出vx_set和vy_set
			    的反正切值夹角（有正负）,再加上云台与底盘之间的夹角，之后求出应该往底盘坐标系上分解的vx、vy的速度，即使底盘没转过来，底盘也会往云台方向斜移了*/
			  static float Gimbal_V = 0.0f, delta_angle = 0.0f;
			
				arm_sqrt_f32(vx_set*vx_set+vy_set*vy_set,&Gimbal_V);
			  delta_angle = chassis_move_control->error_aobut_gimbalYAW/57.3f + atan2(vy_set,vx_set);
			
				chassis_move_control->vx_set = arm_cos_f32(delta_angle)*Gimbal_V;
				chassis_move_control->vy_set = arm_sin_f32(delta_angle)*Gimbal_V;
				chassis_move_control->wz_set = wz_set;
    }
    //不跟随云台模式
    else if (chassis_move_control->chassis_mode == CHASSIS_VECTOR_NO_FOLLOW_YAW || chassis_move_control->chassis_mode ==  CHASSIS_STOP || chassis_move_control->chassis_mode ==  CHASSIS_RELAX )
    {
        //计算实际底盘的速度设定
        chassis_move_control->vx_set = vx_set;
        chassis_move_control->vy_set = vy_set;
        chassis_move_control->wz_set = wz_set;
    }

    //速度限幅
    chassis_move_control->vx_set = float_constrain(chassis_move_control->vx_set, - NORMAL_MAX_CHASSIS_SPEED_X, NORMAL_MAX_CHASSIS_SPEED_X );
    chassis_move_control->vy_set = float_constrain(chassis_move_control->vy_set, - NORMAL_MAX_CHASSIS_SPEED_Y, NORMAL_MAX_CHASSIS_SPEED_Y );
    chassis_move_control->wz_set = float_constrain(chassis_move_control->wz_set, - NORMAL_MAX_CHASSIS_SPEED_Z, NORMAL_MAX_CHASSIS_SPEED_Z );
}






//遥控器的数据处理vx速度，vy，vw速度
void chassis_rc_to_control_vector(float *vx_set, float *vy_set, chassis_move_t *chassis_move_rc_to_vector)
{
    if (chassis_move_rc_to_vector == NULL || vx_set == NULL || vy_set == NULL)
    {
        return;
    }

    //遥控器原始通道值
    int16_t vx_channel, vy_channel;
    float vx_set_channel, vy_set_channel;

    //死区限制,因为遥控器可能存在差异 摇杆在中间，其值不为0
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_X_CHANNEL], vx_channel, CHASSIS_RC_DEADLINE);
    rc_deadline_limit(chassis_move_rc_to_vector->chassis_RC->rc.ch[CHASSIS_Y_CHANNEL], vy_channel, CHASSIS_RC_DEADLINE);

    //转化为车体前进速度
    vx_set_channel = vx_channel * (-CHASSIS_VX_RC_SEN);
    vy_set_channel = vy_channel * (-CHASSIS_VY_RC_SEN);

    //键盘控制步兵前后左右移动
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_FRONT_KEY)
    {
        vx_set_channel =   -NORMAL_MAX_CHASSIS_SPEED_X*0.55f;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_BACK_KEY)
    {
        vx_set_channel =   NORMAL_MAX_CHASSIS_SPEED_X*0.55f;
    }
    if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_LEFT_KEY)
    {
        vy_set_channel =   NORMAL_MAX_CHASSIS_SPEED_Y*0.55f;
    }
    else if (chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_RIGHT_KEY)
    {
        vy_set_channel = - NORMAL_MAX_CHASSIS_SPEED_Y*0.55f;
    }

    //SHIFT加速
    if( chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_SPEEDUP_KEY)
    {
        vx_set_channel = vx_set_channel*1.6f;
        vy_set_channel = vy_set_channel*1.6f;
    }
    //ctrl减速
    else if( chassis_move_rc_to_vector->chassis_RC->key.v & CHASSIS_SPEEDDOWN_KEY)
    {
        vx_set_channel = vx_set_channel*0.6f;
        vy_set_channel = vy_set_channel*0.6f;
    }

    //一阶低通滤波代替斜波作为底盘速度输入
		if(chassis_move_rc_to_vector->chassis_RC->key.v) //使用键盘时，考虑一阶滤波
		{
		    first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vx, vx_set_channel);
				first_order_filter_cali(&chassis_move_rc_to_vector->chassis_cmd_slow_set_vy, vy_set_channel);
		}
    else //使用遥控器时，因为遥控器本身输入模拟量，带有一阶滤波特性，不考虑一阶滤波
		{
		    chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = vx_set_channel;
			  chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = vy_set_channel;
		}

    //当使用键盘或鼠标给停止信号，不需要缓慢减速，直接减速到零
    if (vx_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN && vx_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VX_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out = 0.0f;
    }
    if (vy_set_channel < CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN && vy_set_channel > -CHASSIS_RC_DEADLINE * CHASSIS_VY_RC_SEN)
    {
        chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out = 0.0f;
    }   
		
    *vx_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vx.out;
    *vy_set = chassis_move_rc_to_vector->chassis_cmd_slow_set_vy.out;
}



/*--------------------------------------------底盘控制数据计算----------------------------------------------*/



//底盘控制计算
static void chassis_control_loop(chassis_move_t *chassis_move_control_loop)
{
    float max_vector = 0.0f, vector_rate = 0.0f;
    float temp = 0.0f;

    float wheel_speed[4] = {0.0f, 0.0f, 0.0f, 0.0f};

    //麦轮运动速度分解
    chassis_vector_to_mecanum_wheel_speed(chassis_move_control_loop->vx_set,
                                          chassis_move_control_loop->vy_set, chassis_move_control_loop->wz_set, wheel_speed);


    //赋值轮子控制速度，并限制其最大速度
    for (uint8_t i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].speed_set = wheel_speed[i];

        temp = fabs(chassis_move_control_loop->motor_chassis[i].speed_set);
        if (max_vector < temp)   max_vector = temp;
    }

    if ( max_vector > MAX_WHEEL_SPEED )
    {
        vector_rate = MAX_WHEEL_SPEED / max_vector;
        for (uint8_t i = 0; i < 4; i++)
        {
            chassis_move_control_loop->motor_chassis[i].speed_set *= vector_rate;//等比例减少给定速度
        }
    }

    //计算pid
    for (uint8_t i = 0; i < 4; i++)
    {
        CHASSIS_PID_Calc(&chassis_move_control_loop->motor_speed_pid[i], chassis_move_control_loop->motor_chassis[i].speed, chassis_move_control_loop->motor_chassis[i].speed_set);
    }

    //赋值电流值
    for (uint8_t i = 0; i < 4; i++)
    {
        chassis_move_control_loop->motor_chassis[i].give_current = (int16_t)(chassis_move_control_loop->motor_speed_pid[i].out);
    }

}




//四轮速度分解
//此方程是O-长方形麦轮底盘的逆运动学模型，实现全向移动控制
//逆运动学模型（inverse kinematic model）得到的公式则是可以根据底盘的运动状态解算出四个轮子的速度
static void chassis_vector_to_mecanum_wheel_speed(const float vx_set, const float vy_set, const float wz_set, float wheel_speed[4])
{
    //旋转的时候， 由于云台靠前，所以是前面两轮 0,1 旋转的速度变慢， 后面两轮 2,3 旋转的速度变快
    wheel_speed[0] = -vx_set - vy_set + (  CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[1] = vx_set - vy_set  + (  CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[2] = vx_set + vy_set  + (- CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
    wheel_speed[3] = -vx_set + vy_set + (- CHASSIS_WZ_SET_SCALE - 1.0f) * MOTOR_DISTANCE_TO_CENTER * wz_set;
}

