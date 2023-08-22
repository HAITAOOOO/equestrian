#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "main.h"
#include "pid_bsp.h"
/*
	将pid初始化与pid计算拆开的好处在于减少了pid重复初始化的时间，
	坏处是每一个pid结构数据不能被覆盖，即有一个电机必须初始化一个结构体（串级两个），增大了内存占用
*/
#define PI 3.14159265358979f

extern PID_TypeDef motor_pid_speed[8],motor_pid_pos[8];

void PID_init(void);
void CAN_Send_PID(void);

#endif
