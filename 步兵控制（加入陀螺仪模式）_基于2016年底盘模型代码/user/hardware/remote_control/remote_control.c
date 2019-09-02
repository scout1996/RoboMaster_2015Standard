/**
  ****************************(C) COPYRIGHT 2019 IronSprit***********************
  * @file       remote_control.c/h

  * @brief      遥控器处理，遥控器是通过类似DBUS的协议传输，利用串口1空闲中断与DMA双
	              缓冲区传输方式节约CPU资源，同时实时检测DBUS是否离线。               

  * @note       1、该任务是通过串口空闲中断启动，不是freeRTOS任务
	*             2、用static声明外部变量,会改变其连接方式，使其只在本文件内部有效，
	                 而其他文件不可连接或引用该变量。
	* 					  3、使用static用于函数定义时，对函数的连接方式产生影响，使得函数
								   只在本文件内部有效，对其他文件是不可见的。不用担心与其他文件的
									 同名函数产生干扰，另外也是对函数本身的一种保护机制。
	*							4、注意看串口1空闲中断里面何时读值存储区0和1
	              5、typedef __packed struct是保持1字节对齐，即结构体的大小由成员变量
								   的真实值决定
									 
  * @history
  *  Version       Date            Author          status
  *  V2.0.0       2019-6-14        Young            完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 IronSprit************************
*/

#include "remote_control.h"
#include "Detect_Task.h"
#include "stm32f4xx.h"

//遥控器控制变量
static RC_ctrl_t  rc_ctrl;
//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
static uint8_t   SBUS_rx_buf[2][SBUS_RX_BUF_NUM];

void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t size)
{
    /* -------------- Enable Module Clock Source ----------------------------*/
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB | RCC_AHB1Periph_DMA2, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, DISABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1); //PB7  usart1 rx
    /* -------------- Configure GPIO ---------------------------------------*/
    {
        GPIO_InitTypeDef GPIO_InitStructure;
        USART_InitTypeDef USART_InitStructure;
        GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
        GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
        GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
        GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
        GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
        GPIO_Init(GPIOB, &GPIO_InitStructure);

        USART_DeInit(USART1);

        USART_InitStructure.USART_BaudRate = 100000;
        USART_InitStructure.USART_WordLength = USART_WordLength_8b;
        USART_InitStructure.USART_StopBits = USART_StopBits_1;
        USART_InitStructure.USART_Parity = USART_Parity_Even;  //偶校验
        USART_InitStructure.USART_Mode = USART_Mode_Rx;
        USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
        USART_Init(USART1, &USART_InitStructure);

        USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

        USART_ClearFlag(USART1, USART_FLAG_IDLE);
        USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);

        USART_Cmd(USART1, ENABLE);
    }

    /* -------------- Configure NVIC ---------------------------------------*/
    {
        NVIC_InitTypeDef NVIC_InitStructure;
        NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
        NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = RC_NVIC;
        NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
        NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
        NVIC_Init(&NVIC_InitStructure);
    }

    /* -------------- Configure DMA -----------------------------------------*/
    {
        DMA_InitTypeDef DMA_InitStructure;
        DMA_DeInit(DMA2_Stream2);

        DMA_InitStructure.DMA_Channel = DMA_Channel_4;   //通道选择
        DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR); //DMA外设地址
        DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)rx1_buf;            //DMA 存储器地址
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;               //外设到存储器模式
        DMA_InitStructure.DMA_BufferSize = size;                              //数据传输量
        DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;      //外设非增量模式
        DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;               //存储器增量模式
        DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;//外设数据长度:8位
        DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;       //存储器数据长度:8位
        DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;// 使用循环发送模式
        DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;//最高优先级
        DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
        DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
        DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;//存储器突发单次传输，一次传输一个字节
        DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;//外设突发单次传输
        DMA_Init(DMA2_Stream2, &DMA_InitStructure);
			
			  //配置DMA双缓冲区模式
        DMA_DoubleBufferModeConfig(DMA2_Stream2, (uint32_t)rx2_buf, DMA_Memory_0);//内存0 即rx1_buf先被传输
        DMA_DoubleBufferModeCmd(DMA2_Stream2, ENABLE);
				
        DMA_Cmd(DMA2_Stream2, DISABLE); //Add a disable
        DMA_Cmd(DMA2_Stream2, ENABLE);
    }
}

//返回遥控器控制变量，通过指针传递方式传递信息
const RC_ctrl_t* get_remote_control_point(void)
{
    return &rc_ctrl;
}

//遥控器接收初始化
void remote_control_init(void)
{
    RC_init(SBUS_rx_buf[0], SBUS_rx_buf[1], SBUS_RX_BUF_NUM);
}

//数据处理函数
//把DMA接收的原始数据转化为遥控器需要使用的数据
static void DBUS_TO_RC(volatile const uint8_t *sbus_buf, RC_ctrl_t *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->rc.ch[0] = (sbus_buf[0] | (sbus_buf[1] << 8)) & 0x07ff;        //!< Channel 0
    rc_ctrl->rc.ch[1] = ((sbus_buf[1] >> 3) | (sbus_buf[2] << 5)) & 0x07ff; //!< Channel 1
    rc_ctrl->rc.ch[2] = ((sbus_buf[2] >> 6) | (sbus_buf[3] << 2) |          //!< Channel 2
                         (sbus_buf[4] << 10)) & 0x07ff;
    rc_ctrl->rc.ch[3] = ((sbus_buf[4] >> 1) | (sbus_buf[5] << 7)) & 0x07ff; //!< Channel 3
    rc_ctrl->rc.s[0] = ((sbus_buf[5] >> 4) & 0x0003);                       //!< Switch left
    rc_ctrl->rc.s[1] = ((sbus_buf[5] >> 4) & 0x000C) >> 2;                  //!< Switch right
    rc_ctrl->mouse.x = sbus_buf[6] | (sbus_buf[7] << 8);                    //!< Mouse X axis
    rc_ctrl->mouse.y = sbus_buf[8] | (sbus_buf[9] << 8);                    //!< Mouse Y axis
    rc_ctrl->mouse.z = sbus_buf[10] | (sbus_buf[11] << 8);                  //!< Mouse Z axis
    rc_ctrl->mouse.press_l = sbus_buf[12];                                  //!< Mouse Left Is Press ?
    rc_ctrl->mouse.press_r = sbus_buf[13];                                  //!< Mouse Right Is Press ?
    rc_ctrl->key.v = sbus_buf[14] | (sbus_buf[15] << 8);                    //!< KeyBoard value
    rc_ctrl->rc.ch[4] = sbus_buf[16] | (sbus_buf[17] << 8);                 //NULL

    rc_ctrl->rc.ch[0] -= RC_CH_VALUE_OFFSET; //减去中间值1024
    rc_ctrl->rc.ch[1] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[2] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[3] -= RC_CH_VALUE_OFFSET;
    rc_ctrl->rc.ch[4] -= RC_CH_VALUE_OFFSET;
}

//串口1空闲中断服务函数
//根据DMA双缓冲模式来分别处理遥控器数据，同时检测DBUS是否掉线
void USART1_IRQHandler(void)
{
    if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        static uint16_t this_time_rx_len = 0;

        if(DMA_GetCurrentMemoryTarget(DMA2_Stream2) == 0) //如果返回值为0，说明DMA正在访问缓冲区0，而串口空闲中断的到来，说明缓冲区0的数据已经收集完成，可以处理了
        {
            this_time_rx_len = USART1->SR;
            this_time_rx_len = USART1->DR; //清除USART_IT_IDLE标志位
					
					  //重新设置DMA
            DMA_Cmd(DMA2_Stream2, DISABLE);
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA2_Stream2);//得到接收的数据个数
            DMA_SetCurrDataCounter(DMA2_Stream2, SBUS_RX_BUF_NUM);//设置DMA下一次接收的字节数
            DMA2_Stream2->CR |= DMA_SxCR_CT;/* Set Memory 1 as next memory address */
            DMA_Cmd(DMA2_Stream2, ENABLE);
            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                DBUS_TO_RC(SBUS_rx_buf[0], &rc_ctrl);
                //记录数据接收时间
                DetectHook(DBUSTOE);
            }
        }
        else
        {
					  this_time_rx_len = USART1->SR;
            this_time_rx_len = USART1->DR; //清除USART_IT_IDLE标志位
					
            //重新设置DMA
            DMA_Cmd(DMA2_Stream2, DISABLE);
            this_time_rx_len = SBUS_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA2_Stream2);
            DMA_SetCurrDataCounter(DMA2_Stream2, SBUS_RX_BUF_NUM);
            DMA2_Stream2->CR &= ~(DMA_SxCR_CT);/* Set Memory 0 as next memory address */
            DMA_Cmd(DMA2_Stream2, ENABLE);
            if(this_time_rx_len == RC_FRAME_LENGTH)
            {
                //处理遥控器数据
                DBUS_TO_RC(SBUS_rx_buf[1], &rc_ctrl);
                //记录数据接收时间
                DetectHook(DBUSTOE);
            }
        }
    }
}

