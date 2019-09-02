#ifndef __OLED_H
#define __OLED_H
#include "sys.h"
#include "stdlib.h"

//-----------------OLED OLED_IIC端口定义----------------

//IO方向设置
#define OLED_SDA_IN()  {GPIOA->CRL&=0XFFFFF0FF;GPIOA->CRL|=(u32)8<<8;}
#define OLED_SDA_OUT() {GPIOA->CRL&=0XFFFFF0FF;GPIOA->CRL|=(u32)3<<8;}
//IO操作函数
#define OLED_IIC_SCL    PAout(3) //SCL
#define OLED_IIC_SDA    PAout(2) //SDA	 
#define OLED_READ_SDA   PAin(2)  //输入SDA 

//OLED控制用函数
void OLED_Init(void);
void OLED_SetPos(unsigned char x, unsigned char y);
void OLED_Fill(unsigned char fill_Data);
void OLED_CLS(void);
void OLED_ON(void);
void OLED_OFF(void);
void OLED_ShowStr(unsigned char x, unsigned char y,char ch[], unsigned char TextSize);
void OLED_DrawBMP(unsigned char x0,unsigned char y0,unsigned char x1,unsigned char y1,unsigned char BMP[]);

//OLED_IIC操作函数
void OLED_IIC_Start(void);				//发送OLED_IIC开始信号
void OLED_IIC_Stop(void);	  			//发送OLED_IIC停止信号
void OLED_IIC_Send_Byte(u8 txd);			//OLED_IIC发送一个字节
u8 OLED_IIC_Wait_Ack(void); 				//OLED_IIC等待ACK信号
void I2C_WriteByte(u8 addr,u8 data);
#endif




