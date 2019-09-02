#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"
#include "delay.h"

//产生OLED_IIC起始信号
void OLED_IIC_Start(void)
{
    OLED_SDA_OUT();     //sda线输出
    OLED_IIC_SDA = 1;
    OLED_IIC_SCL = 1;
//	delay_us(1);
    OLED_IIC_SDA = 0; //START:when CLK is high,DATA change form high to low
//	delay_us(1);
    OLED_IIC_SCL = 0; //钳住I2C总线，准备发送或接收数据
}
//产生OLED_IIC停止信号
void OLED_IIC_Stop(void)
{
    OLED_SDA_OUT();//sda线输出
    OLED_IIC_SCL = 0;
    OLED_IIC_SDA = 0;
    OLED_IIC_SCL = 1; //STOP:when CLK is high DATA change form low to high
//	delay_us(1);
    OLED_IIC_SDA = 1; //发送I2C总线结束信号
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
u8 OLED_IIC_Wait_Ack(void)
{
    u8 ucErrTime = 0;
    OLED_SDA_IN();      //SDA设置为输入
    OLED_IIC_SDA = 1;
    OLED_IIC_SCL = 1;
    while(OLED_READ_SDA)
    {
        ucErrTime++;
        if(ucErrTime > 250)
        {
            OLED_IIC_Stop();
            return 1;
        }
    }
    OLED_IIC_SCL = 0; //时钟输出0
    return 0;
}

//OLED_IIC发送一个字节
void OLED_IIC_Send_Byte(u8 txd)
{
    u8 t;
    OLED_SDA_OUT();
    OLED_IIC_SCL = 0; //拉低时钟开始数据传输
    for(t = 0; t < 8; t++)
    {
        OLED_IIC_SDA = (txd & 0x80) >> 7;
        txd <<= 1;
//		delay_us(1);
        OLED_IIC_SCL = 1;
//		delay_us(1);
        OLED_IIC_SCL = 0;
//		delay_us(1);
    }
}

void I2C_WriteByte(u8 addr, u8 data)
{
    OLED_IIC_Start();
    OLED_IIC_Send_Byte(0x78);            //Slave address,SA0=0
    OLED_IIC_Wait_Ack();
    OLED_IIC_Send_Byte(addr);			//write command
    OLED_IIC_Wait_Ack();
    OLED_IIC_Send_Byte(data);
    OLED_IIC_Wait_Ack();
    OLED_IIC_Stop();
}
/**
 * @brief  WriteCmd，向OLED写入命令
 * @param  I2C_Command：命令代码
 * @retval 无
 */
void WriteCmd(unsigned char I2C_Command)//写命令
{
    I2C_WriteByte(0x00, I2C_Command);
}

/**
 * @brief  WriteDat，向OLED写入数据
 * @param  I2C_Data：数据
 * @retval 无
 */
void WriteDat(unsigned char I2C_Data)//写数据
{
    I2C_WriteByte(0x40, I2C_Data);
}

//初始化SSD1306
void OLED_Init(void)
{
    GPIO_InitTypeDef  GPIO_InitStructure;
    RCC_APB2PeriphClockCmd(	RCC_APB2Periph_GPIOA, ENABLE );	//使能GPIOB时钟

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2|GPIO_Pin_3;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP ;   //推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化

    OLED_IIC_SCL = 1;
    OLED_IIC_SDA = 1;


    WriteCmd(0xAE); //关闭显示
    WriteCmd(0xD5); //设置时钟分频因子,震荡频率
    WriteCmd(0x50);   //[3:0],分频因子;[7:4],震荡频率
    WriteCmd(0xA8); //设置驱动路数
    WriteCmd(0X3F); //默认0X3F(1/64)
    WriteCmd(0xD3); //设置显示偏移
    WriteCmd(0X00); //默认为0

    WriteCmd(0x40); //设置显示开始行 [5:0],行数.

    WriteCmd(0x8D); //电荷泵设置
    WriteCmd(0x14); //bit2，开启/关闭
    WriteCmd(0x20); //设置内存地址模式
    WriteCmd(0x02); //[1:0],00，列地址模式;01，行地址模式;10,页地址模式;默认10;
    WriteCmd(0xA1); //段重定义设置,bit0:0,0->0;1,0->127;
    WriteCmd(0xC0); //设置COM扫描方向;bit3:0,普通模式;1,重定义模式 COM[N-1]->COM0;N:驱动路数
    WriteCmd(0xDA); //设置COM硬件引脚配置
    WriteCmd(0x12); //[5:4]配置

    WriteCmd(0x81); //对比度设置
    WriteCmd(0xEF); //1~255;默认0X7F (亮度设置,越大越亮)
    WriteCmd(0xD9); //设置预充电周期
    WriteCmd(0xf1); //[3:0],PHASE 1;[7:4],PHASE 2;
    WriteCmd(0xDB); //设置VCOMH 电压倍率
    WriteCmd(0x30); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

    WriteCmd(0xA4); //全局显示开启;bit0:1,开启;0,关闭;(白屏/黑屏)
    WriteCmd(0xA6); //设置显示方式;bit0:1,反相显示;0,正常显示
    WriteCmd(0xAF); //开启显示

    OLED_Fill(0x00);
}
/**
 * @brief  OLED_SetPos，设置光标
 * @param  x,光标x位置
 *		   y，光标y位置
 * @retval 无
 */
void OLED_SetPos(unsigned char x, unsigned char y) //设置起始点坐标
{
    WriteCmd(0xb0 + y);
    WriteCmd(((x & 0xf0) >> 4) | 0x10);
    WriteCmd((x & 0x0f) | 0x01);
}

/**
 * @brief  OLED_Fill，填充整个屏幕
 * @param  fill_Data:要填充的数据
* @retval 无
 */
void OLED_Fill(unsigned char fill_Data)//全屏填充
{
    unsigned char m, n;
    for(m = 0; m < 8; m++)
    {
        WriteCmd(0xb0 + m);		//page0-page1
        WriteCmd(0x00);		//low column start address
        WriteCmd(0x10);		//high column start address
        for(n = 0; n < 128; n++)
        {
            WriteDat(fill_Data);
        }
    }
}

/**
 * @brief  OLED_CLS，清屏
 * @param  无
* @retval 无
 */
void OLED_CLS(void)//清屏
{
    OLED_Fill(0x00);
}


/**
 * @brief  OLED_ON，将OLED从休眠中唤醒
 * @param  无
* @retval 无
 */
void OLED_ON(void)
{
    WriteCmd(0X8D);  //设置电荷泵
    WriteCmd(0X14);  //开启电荷泵
    WriteCmd(0XAF);  //OLED唤醒
}


/**
 * @brief  OLED_OFF，让OLED休眠 -- 休眠模式下,OLED功耗不到10uA
 * @param  无
* @retval 无
 */
void OLED_OFF(void)
{
    WriteCmd(0X8D);  //设置电荷泵
    WriteCmd(0X10);  //关闭电荷泵
    WriteCmd(0XAE);  //OLED休眠
}

unsigned char reverse8( unsigned char c )
{
    c = ( c & 0x55 ) << 1 | ( c & 0xAA ) >> 1;
    c = ( c & 0x33 ) << 2 | ( c & 0xCC ) >> 2;
    c = ( c & 0x0F ) << 4 | ( c & 0xF0 ) >> 4;
    return c;
}
/**
 * @brief  OLED_ShowStr，显示codetab.h中的ASCII字符,有6*8和8*16可选择
 * @param  x,y : 起始点坐标(x:0~127, y:0~7);
*					ch[] :- 要显示的字符串;
*					TextSize : 字符大小(1:6*8 ; 2:8*16)
* @retval 无
 */
void OLED_ShowStr(unsigned char x, unsigned char y, char ch[], unsigned char TextSize)
{
    unsigned char c = 0, i = 0, j = 0;
    switch(TextSize)
    {
    case 6:
    {
        while(ch[j] != '\0')
        {
            c = ch[j] - 32;
            if(x > 126)
            {
                x = 0;
                y++;
            }
            OLED_SetPos(x, 7-y);
            for(i = 0; i < 6; i++)
                WriteDat(reverse8(F6x8[c][i]));
            x += 6;
            j++;
        }
    }
    break;
    case 8:
    {
        while(ch[j] != '\0')
        {
            c = ch[j] - 32;
            if(x > 120)
            {
                x = 0;
                y++;
            }
            OLED_SetPos(x, 7-y);
            for(i = 0; i < 8; i++)
                WriteDat(reverse8(F8X16[c * 16 + i]));
            OLED_SetPos(x, 6-y);
            for(i = 0; i < 8; i++)
                WriteDat(reverse8(F8X16[c * 16 + i + 8]));
            x += 8;
            j++;
        }
    }
    break;
    }
}

/**
 * @brief  OLED_DrawBMP，显示BMP位图
 * @param  x0,y0 :起始点坐标(x0:0~127, y0:0~7);
*					x1,y1 : 起点对角线(结束点)的坐标(x1:1~128,y1:1~8)
* @retval 无
 */
void OLED_DrawBMP(unsigned char x0, unsigned char y0, unsigned char x1, unsigned char y1, unsigned char BMP[])
{
    unsigned int j = 0;
    unsigned char x, y;

    if(y1 % 8 == 0)
        y = y1 / 8;
    else
        y = y1 / 8 + 1;
    for(y = y0; y < y1; y++)
    {
        OLED_SetPos(x0, y);
        for(x = x0; x < x1; x++)
        {
            WriteDat(BMP[j++]);
        }
    }
}








