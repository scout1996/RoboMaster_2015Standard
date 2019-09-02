#include "sys.h"
#include "delay.h"
#include "led.h"
#include "usart.h"
#include "mpuiic.h"
#include "stdio.h"
#include "key.h"
#include "pwm.h"
#include "timer.h"
#include "math.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"

void usart_send(u8 str[],u8 len);

#define q30  1073741824.0f
#define GYRO_SEN       0.03051757812499f
#define RAD_TO_ANGLE   57.295779513082320876798154814105f

#pragma pack(push)
#pragma pack(1)

typedef union
{
    struct
    {
		u8 header;
		float INS_Angle[3];
		float INS_gyro[3];
		u8 tail;
    } data;
    uint8_t buf[26];

} MPU6050_Pack_t;

#pragma pack(pop)
MPU6050_Pack_t MPU6050_Pack;
int main(void)
{
	float q0=1.0f,q1=0.0f,q2=0.0f,q3=0.0f;
	unsigned long sensor_timestamp;
	short gyro[3], accel[3], sensors;
	unsigned char more;
	long quat[4]; 
	short last_gyro[3];
		
	MPU6050_Pack.data.header=0x55;
	MPU6050_Pack.data.tail=0xaa;
	
    delay_init();
	LED_Init();
	uart_init(921600);//921600
	while(mpu_dmp_init())
	{
		LED0=!LED0;
        delay_ms(400);
	}
	LED0_ON;
	LED1_ON;
	TIM3_Int_Init(2000-1,72-1);
	while(1)
	{
		dmp_read_fifo(gyro, accel, quat, &sensor_timestamp, &sensors,&more)	; 
		/* Gyro and accel data are written to the FIFO by the DMP in chip frame and hardware units.
		 * This behavior is convenient because it keeps the gyro and accel outputs of dmp_read_fifo and mpu_read_fifo consistent.
		**/
		/*if (sensors & INV_XYZ_GYRO )
		send_packet(PACKET_TYPE_GYRO, gyro);
		if (sensors & INV_XYZ_ACCEL)
		send_packet(PACKET_TYPE_ACCEL, accel); */
		/* Unlike gyro and accel, quaternions are written to the FIFO in the body frame, q30.
		 * The orientation is set by the scalar passed to dmp_set_orientation during initialization. 
		**/
		if(sensors&INV_WXYZ_QUAT) 
		{
			q0 = quat[0] / q30;	//q30格式转换为浮点数
			q1 = quat[1] / q30;
			q2 = quat[2] / q30;
			q3 = quat[3] / q30; 
			//计算得到俯仰角/横滚角/航向角
			MPU6050_Pack.data.INS_Angle[1] = asin(-2 * q1 * q3 + 2 * q0* q2)* RAD_TO_ANGLE;	// pitch
			MPU6050_Pack.data.INS_Angle[2] = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2* q2 + 1)* RAD_TO_ANGLE;	// roll
			MPU6050_Pack.data.INS_Angle[0] = atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * RAD_TO_ANGLE;	//yaw
		}
		if (sensors & INV_XYZ_GYRO )
		{
			MPU6050_Pack.data.INS_gyro[2] = GYRO_SEN * (gyro[0]*0.15f + last_gyro[0]*0.85f);
			MPU6050_Pack.data.INS_gyro[1] = GYRO_SEN * (gyro[1]*0.15f + last_gyro[1]*0.85f);
			MPU6050_Pack.data.INS_gyro[0] = GYRO_SEN * (gyro[2]*0.15f + last_gyro[2]*0.85f);
			last_gyro[0]=gyro[0];
			last_gyro[1]=gyro[1];
			last_gyro[2]=gyro[2];
		}
		
		usart_send(MPU6050_Pack.buf,sizeof(MPU6050_Pack));
//		printf("pitch:%.3f   roll:%.3f   yaw:%.3f \r\n",MPU6050_Pack.data.INS_Angle[1],MPU6050_Pack.data.INS_Angle[2],MPU6050_Pack.data.INS_Angle[0]);
	}

}
//定时器3中断服务程序
void TIM3_IRQHandler(void)   //TIM3中断
{
    if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)  //检查TIM3更新中断发生与否
    {
        TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //清除TIMx更新中断标志
    }
}
void usart_send(u8 str[],u8 len)
{
	u8 i=0;
	for(;i<len;i++)
	{
		while((USART1->SR&0X40)==0);//循环发送,直到发送完毕
		USART1->DR = str[i];
	}
}


