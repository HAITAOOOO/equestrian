#ifndef __JUMP_H_
#define __JUMP_H_
#include "main.h"

extern int16_t pid_spd_out_limit;
extern float jump_angle;
extern float jump_extension; // 最大伸腿长度 [cm]

void StartJump1(float start_time_s);
void ExecuteJump1(void);

void StartJump1_LADDER(float start_time_s);
void ExecuteJump1_LADDER(void);

void StartJump2(float start_time_s);
void ExecuteJump2(void);

#endif
