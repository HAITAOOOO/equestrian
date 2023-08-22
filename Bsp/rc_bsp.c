/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note       该任务是通过串口中断启动，不是freeRTOS任务
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-01-2019     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "rc_bsp.h"
#include "main.h"
#include "stdlib.h"//abs();
#include "string.h"//memset();
#include "posture_ctrl.h"
#include "jump.h"

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;

int allow;

//接收原始数据，为18个字节，给了36个字节长度，防止DMA传输越界
uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

/**
  * @brief          遥控器协议解析
  * @param[in]      sbus_buf: 原生数据指针
  * @param[out]     rc_ctrl: 遥控器数据指针
  * @retval         none
  */
void sbus_to_rc(volatile const uint8_t *sbus_buf, RC *rc_ctrl)
{
    if (sbus_buf == NULL || rc_ctrl == NULL)
    {
        return;
    }

    rc_ctrl->ch1 = (sbus_buf[0] | sbus_buf[1] << 8) & 0x07FF;
    rc_ctrl->ch1 -= 1024;
    rc_ctrl->ch2 = (sbus_buf[1] >> 3 | sbus_buf[2] << 5) & 0x07FF;
    rc_ctrl->ch2 -= 1024;
    rc_ctrl->ch3 = (sbus_buf[2] >> 6 | sbus_buf[3] << 2 | sbus_buf[4] << 10) & 0x07FF;
    rc_ctrl->ch3 -= 1024;
    rc_ctrl->ch4 = (sbus_buf[4] >> 1 | sbus_buf[5] << 7) & 0x07FF;
    rc_ctrl->ch4 -= 1024;
    rc_ctrl->ch5 = (sbus_buf[16] | sbus_buf[17] << 8)  ;
    rc_ctrl->ch5 -= 1024;

    rc_ctrl->sw1 = ((sbus_buf[5] >> 4) & 0x000C) >> 2;
    rc_ctrl->sw2 = (sbus_buf[5] >> 4) & 0x0003;

    if ((abs(rc_ctrl->ch1) > 660) || \
            (abs(rc_ctrl->ch2) > 660) || \
            (abs(rc_ctrl->ch3) > 660) || \
            (abs(rc_ctrl->ch4) > 660)	|| \
            (abs(rc_ctrl->ch5) > 660))
    {
        memset(rc_ctrl, 0, sizeof(RC));
    }
}

/**
  * @brief          遥控器封装
  * @param[in]      none
  * @retval         none
  */
//例：remote_control_init(); main函数里，while前
void remote_control_init(void)
{
    RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
}

/**
  * @brief          遥控器初始化
  * @param[in]      rx1_buf：内存缓冲区1
  * @param[in]			rx2_buf：内存缓冲区2
  * @retval         遥控器数据指针
  */
//例：RC_init(sbus_rx_buf[0], sbus_rx_buf[1], SBUS_RX_BUF_NUM);
void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    //使能DMA串口接收
    SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);

    //enalbe idle interrupt
    //使能空闲中断
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);

    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_rx);
    while(hdma_usart1_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_rx);
    }

    hdma_usart1_rx.Instance->PAR = (uint32_t) & (USART1->DR);
    //内存缓冲区1
    hdma_usart1_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    //内存缓冲区2
    hdma_usart1_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    //数据长度
    hdma_usart1_rx.Instance->NDTR = dma_buf_num;
    //使能双缓冲区
    SET_BIT(hdma_usart1_rx.Instance->CR, DMA_SxCR_DBM);

    //使能DMA
    __HAL_DMA_ENABLE(&hdma_usart1_rx);

}


void control_1(void)
{
    if(rc_ctrl.ch4==0 && rc_ctrl.ch3==0)
    {
        state = STOP;
    }
    else if(rc_ctrl.ch4<-20 && rc_ctrl.ch3>=-330 && rc_ctrl.ch3==0)
    {
        state = WALK_BACKl;
    }
    else if(rc_ctrl.ch4<-330 && rc_ctrl.ch3>=-660 && rc_ctrl.ch3==0)
    {
        state = WALK_BACK;
    }
    else if(rc_ctrl.ch4>20 && rc_ctrl.ch3<=330 && rc_ctrl.ch3==0)
    {
        state = TROTl;
    }
    else if(rc_ctrl.ch4>330 && rc_ctrl.ch3<=660 && rc_ctrl.ch3==0)
    {
        state = TROT;
    }
    else if(rc_ctrl.ch3<-20 && rc_ctrl.ch3>=-330 && rc_ctrl.ch4==0)
    {
        state = ROTAT_LEFTl;
    }
    else if(rc_ctrl.ch3<-330 && rc_ctrl.ch3>=-660 && rc_ctrl.ch4==0)
    {
        state = ROTAT_LEFT;
    }
    else if(rc_ctrl.ch3>20 && rc_ctrl.ch3<=330 &&rc_ctrl.ch4==0)
    {
        state = ROTAT_RIGHTl;
    }
    else if(rc_ctrl.ch3>330 && rc_ctrl.ch3<=660 && rc_ctrl.ch4==0)
    {
        state = ROTAT_RIGHT;
    }
    if(rc_ctrl.ch2>200 && rc_ctrl.ch1==0)
    {
        StartJump2(HAL_GetTick());
        allow = 0;
    }
    else if(rc_ctrl.ch2<-200 && rc_ctrl.ch1==0)
    {
        StartJump1(HAL_GetTick());
        allow = 0;
    }
}

void control_2(void)
{
    if(rc_ctrl.ch4==0 && rc_ctrl.ch3==0 && rc_ctrl.ch2==0 && rc_ctrl.ch1==0)
    {
        state = STOP;
    }
    else if(rc_ctrl.ch4>0)
        state = TROT;
    else if(rc_ctrl.ch4<-20)
    {
        state = WALK;
    }
    else if(rc_ctrl.ch2<-20 && rc_ctrl.ch1==0)
    {
        state = CLIMBING;
    }
    else if(rc_ctrl.ch2>20 && rc_ctrl.ch1==0)
    {
        state = CLIMBINGUP;
    }
    else if(rc_ctrl.ch1>20 && rc_ctrl.ch2==0)
    {
        state = LADDER;
    };
}
