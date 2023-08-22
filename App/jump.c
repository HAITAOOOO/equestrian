#include "jump.h"
#include <math.h>
#include "rc_bsp.h"
#include "posture_ctrl.h"
#include "motor.h"

float start_time_ = 0.0f;
float jump_extension = 29.0f; // 最大伸腿长度 [cm]

void StartJump1(float start_time_s) {
    start_time_ = start_time_s;
    state = JUMP1;
}

void StartJump1_LADDER(float start_time_s) {
    start_time_ = start_time_s;
    state = LADDER;
}

void ExecuteJump1()
{
    const float prep_time = 0.8f; // 准备时间 [s]		0.8
    const float launch_time = 0.2f ; // 收缩腿前的持续时间 [s]		0.2
		//const float recovery_time = 0.2f;	//收腿时间
    const float fall_time = 0.8f; //收缩腿后持续时间恢复正常行为 [s]		0.8
    const float stance_height = 14.2f; // 跳跃之前腿的高度 [cm]  14.2
    const float fall_extension = 14.2f; // 降落时的期望腿长 [cm]
    float t = HAL_GetTick()/1000.0f - start_time_/1000.0f; // 跳跃开始后的时间


    if (t < prep_time)
    {
        x = -stance_height*sin(18*PI/180);
        y = stance_height*cos(21*PI/180);
        Cartesian_To_Theta(1.0);
        Leg_Gain gains = {5, 0.00, 8, 0.00};
        CommandAllLegs(gains);
    }
    else if (t >= prep_time && t < prep_time + launch_time)
    {
        x = -jump_extension*sin(31*PI/180);
        y = jump_extension*cos(31*PI/180);
        Cartesian_To_Theta(1.0);
        Leg_Gain gains = {18.6, 0.00, 11.6, 0.00};
        CommandAllLegs(gains);
    }
//		else if (t >= prep_time && t < prep_time + launch_time+ recovery_time)
//    {
//        x = -jump_extension*sin(29*PI/180)+(5-(-jump_extension*sin(29*PI/180)))*(t- prep_time - launch_time)/recovery_time;//(t- prep_time - launch_time)/recovery_time
//        y = jump_extension*cos(31*PI/180)+(fall_extension-(jump_extension*cos(31*PI/180)))*(t- prep_time - launch_time)/recovery_time;//(t- prep_time - launch_time)/recovery_time
//        CartesianToTheta(1.0);
//        LegGain gains = {5.0, 0.00, 8.0, 0.00};
//        CommandAllLegs(gains);
//    }
    else if (t >= prep_time + launch_time && t < prep_time + launch_time+ fall_time)
    {
        x = 3.6;
        y = fall_extension;
        Cartesian_To_Theta(1.0);
        Leg_Gain gains = {13, 0.00, 5, 0.00};
        CommandAllLegs(gains);
    }
    else
    {
        state = STANDUP;
    }
}

void StartJump2(float start_time_s) {
    start_time_ = start_time_s;
    state = JUMP2;
}

void ExecuteJump2() 
{
    const float prep_time = 0.8f; // 准备时间 [s]		0.8
    const float launch_time = 0.2f ; // 收缩腿前的持续时间 [s]		0.2
    const float fall_time = 0.8f; //收缩腿后持续时间恢复正常行为 [s]		0.8
    const float stance_height = 14.2f; // 跳跃之前腿的高度 [cm]  14.2
    const float fall_extension = 14.2f; // 降落时的期望腿长 [cm]
    float t = HAL_GetTick()/1000.0f - start_time_/1000.0f; // 跳跃开始后的时间

    if (t < prep_time) 
    {
        x = -stance_height*sin(18.5f*PI/180);		//-4.5
        y = stance_height*cos(21*PI/180);		//13.2
        Cartesian_To_Theta(1.0);
        Leg_Gain gains = {5, 0.00, 8, 0.00};
        CommandAllLegs_pre(gains);
        x = -stance_height*sin(21*PI/180);		//-5.1
        y = stance_height*cos(21*PI/180);
        Cartesian_To_Theta(1.0);
        CommandAllLegs_later(gains);
    }
    else if (t >= prep_time && t < prep_time + launch_time) 
    {
        x = -jump_extension*sin(29*PI/180);			//-14
        y = jump_extension*cos(31*PI/180);			//24.9
        Cartesian_To_Theta(1.0);
        Leg_Gain gains = {63.0, 0.00, 48.0, 0.00};
        CommandAllLegs(gains);
    } 
    else if (t >= prep_time + launch_time && t < prep_time + launch_time + fall_time) 
    {
        x = 5;
        y = fall_extension;
        Cartesian_To_Theta(1.0);
        Leg_Gain gains = {8, 0.00, 5, 0.00};
        CommandAllLegs_pre(gains);
        x = 0;
        y = 15;
        Cartesian_To_Theta(1.0);
        CommandAllLegs_later(gains); 
    }
    else 
    {
        state = STOP;
    }
}
