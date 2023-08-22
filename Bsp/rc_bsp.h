/**
  ****************************(C) COPYRIGHT 2016 DJI****************************
  * @file       remote_control.c/h
  * @brief      遥控器处理，遥控器是通过类似SBUS的协议传输，利用DMA传输方式节约CPU
  *             资源，利用串口空闲中断来拉起处理函数，同时提供一些掉线重启DMA，串口
  *             的方式保证热插拔的稳定性。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. 完成
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2016 DJI****************************
  */
#ifndef RC_BSP_H
#define RC_BSP_H

#include "main.h"
//设定长度
#define SBUS_RX_BUF_NUM 36u
//标准长度
#define RC_FRAME_LENGTH 18u

#define RC_CH_VALUE_MIN         ((uint16_t)364)
#define RC_CH_VALUE_OFFSET      ((uint16_t)1024)
#define RC_CH_VALUE_MAX         ((uint16_t)1684)

extern uint8_t sbus_rx_buf[2][SBUS_RX_BUF_NUM];

typedef __packed struct
{
    /* rocker channel information */
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;
    int16_t ch5;

    /* left and right lever information */
    uint8_t sw1;
    uint8_t sw2;
} RC;

#define UP        1
#define MIDDLE    3
#define DOWN      2

extern RC rc_ctrl;
extern int allow;
void remote_control_init(void);
void sbus_to_rc(volatile const uint8_t *sbus_buf, RC *rc_ctrl);
void RC_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);

void control_1(void);
void control_2(void);

#endif
