/**
  ****************************(C) COPYRIGHT 2019 IronSprit***********************
  * @file       gyroscope.c/h

  * @brief      接收另外一块单片机全速运行发来的陀螺仪数据，利用串口7空闲中断与DMA双
	              缓冲区传输方式节约CPU资源。

  * @note       1、该任务是通过串口空闲中断启动，不是freeRTOS任务
	*             2、用static声明外部变量,会改变其连接方式，使其只在本文件内部有效，
	                 而其他文件不可连接或引用该变量。
	* 					  3、使用static用于函数定义时，对函数的连接方式产生影响，使得函数
								   只在本文件内部有效，对其他文件是不可见的。不用担心与其他文件的
									 同名函数产生干扰，另外也是对函数本身的一种保护机制。
	*							4、注意看串口7空闲中断里面何时读值存储区0和1
	              5、struct默认以最长成员的对齐方式作为自身的对齐方式，当union与struct
								   共用时，就会导致陀螺仪数据一直出错，强制1字节对齐后，问题解决

  * @history
  *  Version       Date            Author          status
  *  V3.0.0       2019-6-30        Young            完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 IronSprit************************
*/

#include "gyroscope.h"
#include "string.h"

MPU6050_Pack_t MPU6050_Pack[2];
static float INS_Angle[3] = {0.0f};      //欧拉角 单位度,yaw,pitch,roll
static float INS_gyro[3]  = {0.0f};      //角速度dps

/**
  * @brief          串口DMA与中断配置
  * @author         Young
  * @param[in]      缓冲区0地址
  * @param[in]      缓冲区1地址
  * @param[in]      DMA传输字节数
  * @retval         返回空
  */
void MPU6050_UartDMA_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t size)
{
    /* -------------- Enable Module Clock Source ----------------------------*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE | RCC_AHB1Periph_DMA1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART7, ENABLE);

    RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART7, ENABLE);
    RCC_APB1PeriphResetCmd(RCC_APB1Periph_UART7, DISABLE);

    GPIO_PinAFConfig(GPIOE, GPIO_PinSource7, GPIO_AF_UART7);
    GPIO_PinAFConfig(GPIOE, GPIO_PinSource8, GPIO_AF_UART7);

    /* -------------- Configure GPIO ---------------------------------------*/
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        USART_InitTypeDef USART_InitStructure;
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_8;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOE, &GPIO_InitStructure);

        USART_DeInit(UART7);

        USART_InitStructure.USART_BaudRate = 921600;  //波特率为921600
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_Even; //偶校验
        USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_Init(UART7, &USART_InitStructure);

        USART_DMACmd(UART7, USART_DMAReq_Rx, ENABLE);

        USART_ClearFlag(UART7, USART_FLAG_IDLE);
        USART_ITConfig(UART7, USART_IT_IDLE, ENABLE);

        USART_Cmd(UART7, ENABLE);
    }

    /* -------------- Configure NVIC ---------------------------------------*/
    {
        NVIC_InitTypeDef NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel = UART7_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = MPU6050_NVIC;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }

    {
        DMA_InitTypeDef DMA_InitStructure;
        DMA_DeInit(DMA1_Stream3);//uart7_rx

        DMA_InitStructure.DMA_Channel = DMA_Channel_5;  //通道选择
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (UART7->DR);  //DMA外设地址
        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rx1_buf;            //DMA 存储器地址
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;               //外设到存储器模式
        DMA_InitStructure.DMA_BufferSize = size;                              //数据传输量
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;      //外设非增量模式
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;               //存储器增量模式
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;        //存储器数据长度:8位
        DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;                       // 使用循环发送模式
        DMA_InitStructure.DMA_Priority = DMA_Priority_High;                   //次高优先级
        DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;           //存储器突发单次传输，一次传输一个字节
        DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;   //外设突发单次传输
        DMA_Init(DMA1_Stream3, &DMA_InitStructure);

        //配置DMA双缓冲区模式
        DMA_DoubleBufferModeConfig(DMA1_Stream3, (uint32_t)rx2_buf, DMA_Memory_0);//内存0 即rx1_buf先被传输
        DMA_DoubleBufferModeCmd(DMA1_Stream3, ENABLE);

        DMA_Cmd(DMA1_Stream3, DISABLE); //Add a disable
        DMA_Cmd(DMA1_Stream3, ENABLE);
    }
}

void gyroscope_init(void)
{
    MPU6050_UartDMA_Init(MPU6050_Pack[0].buf,MPU6050_Pack[1].buf,MPU6050_RX_BUF_NUM);
}

const float *get_MPU6050_Angle_point(void)
{
    return INS_Angle;
}

const float *get_MPU6050_Gyro_Point(void)
{
    return INS_gyro;
}

//串口7空闲中断
void UART7_IRQHandler(void)
{
    if (USART_GetITStatus(UART7, USART_IT_IDLE) != RESET)
    {
        static uint16_t this_time_rx_len = 0;
			  
        if(DMA_GetCurrentMemoryTarget(DMA1_Stream3) == 0) //如果返回值为0，说明DMA正在访问缓冲区0，而串口空闲中断的到来，说明缓冲区0的数据已经收集完成，可以处理了
        {
            this_time_rx_len = UART7->SR;
            this_time_rx_len = UART7->DR; //清除UART_IT_IDLE标志位
					
					  //重新设置DMA
            DMA_Cmd(DMA1_Stream3, DISABLE);
					  this_time_rx_len = MPU6050_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream3);//得到接收的数据个数
            DMA_SetCurrDataCounter(DMA1_Stream3, MPU6050_RX_BUF_NUM);//设置DMA下一次接收的字节数
            DMA1_Stream3->CR |= DMA_SxCR_CT;/* Set Memory 1 as next memory address */       
            DMA_Cmd(DMA1_Stream3, ENABLE);
		        
					  if( this_time_rx_len == GYRO_FRAME_LENGTH)
						{
						    if((MPU6050_Pack[0].data.header==0x55)&&(MPU6050_Pack[0].data.tail==0xAA))
								{
										memcpy(INS_Angle,MPU6050_Pack[0].data.INS_Angle,sizeof(INS_Angle));
										memcpy(INS_gyro,MPU6050_Pack[0].data.INS_gyro,sizeof(INS_gyro));
								}
						}					
        }
        else
        {
					  this_time_rx_len = UART7->SR;
            this_time_rx_len = UART7->DR; //清除UART_IT_IDLE标志位
					
            //重新设置DMA
            DMA_Cmd(DMA1_Stream3, DISABLE);
					  this_time_rx_len = MPU6050_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA1_Stream3);//得到接收的数据个数
            DMA_SetCurrDataCounter(DMA1_Stream3, MPU6050_RX_BUF_NUM);
            DMA1_Stream6->CR &= ~(DMA_SxCR_CT);/* Set Memory 0 as next memory address */
            DMA_Cmd(DMA1_Stream3, ENABLE);
			      
					  if( this_time_rx_len == GYRO_FRAME_LENGTH)
						{
						    if((MPU6050_Pack[1].data.header==0x55)&&(MPU6050_Pack[1].data.tail==0xAA))
								{
										memcpy(INS_Angle,MPU6050_Pack[1].data.INS_Angle,sizeof(INS_Angle));
										memcpy(INS_gyro,MPU6050_Pack[1].data.INS_gyro,sizeof(INS_gyro));
								}
						}				
        }
    }
}

