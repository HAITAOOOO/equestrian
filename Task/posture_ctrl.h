#ifndef __POSTURE_CTRL_H_
#define __POSTURE_CTRL_H_

#include <stdbool.h>

#define YES 1
#define NO 0

#define L1 10 //小腿
#define L2 20 //大腿

#define ReductionAndAngleRatio 432.355555f  //19*8191/360=432.355555 19：3508减速比 8192/360 机械角度与角度值的换算

#define PI 3.14159265358979f

#define Is_Ready  1
#define Not_Ready 0

extern bool Is_Motor_Ready;

extern float x, y, theta1, theta2;//坐标 θ1 θ2:电机转动角度

typedef enum	//姿态结构体
{
    TROT = 				0, 	//小跑
    CLIMBING = 		1, 	//攀爬
    WALK = 				2, 	//行走
    WALK_BACK = 	3, 	//后退
    ROTAT_LEFT = 	4, 	//左转
    ROTAT_RIGHT = 5,	//右转
    SPACEWALK = 	6,	//太空步
    LADDER = 			7, 	// 爬梯
    CLIMBINGUP =	8,	//向上攀爬
    ROLLLEFT = 		9,	//向左翻滚？
    ROLLRIGHT = 	10,	//向右翻滚？
    FORWARD = 		11,	//向前
    BACK = 				12,	//后退
    WALK_BACKl = 	13,	//后退1
    TROTl = 			14,	//小跑1
    ROTAT_LEFTl = 15,	//左转1
    ROTAT_RIGHTl =16,	//右转1
    STOP = 				17,	//停止
    REALSE = 			18,	//relax？ 释放
    JUMP1 = 			19,	//小跳
    JUMP2 = 			20,	//大跳
    STANDUP = 		21,	//站立
} state_TypeDef;

typedef struct  // 腿部PID增益结构体
{

    float kp_pos;		//位置环
    float kd_pos;

    float kp_spd;		//速度环
    float kd_spd;

} Leg_Gain;

typedef struct  						//步态腿部参数结构体
{
    float stance_height ;   // 狗身到地面的距离 (cm)
    float step_length ;     // 一步的距离 (cm)
    float up_amp ;          // 上部振幅y (cm)
    float down_amp ;        // 下部振幅 (cm)
    float swing_percent ;  // 摆动相百分比 (0~1)
    float freq ;            // 一步的频率 (Hz)
} Gait_Param;

typedef struct {  //独立步态参数

    Gait_Param detached_params_0;
    Gait_Param detached_params_1;

    Gait_Param detached_params_2;
    Gait_Param detached_params_3;

} Detached_Param;



extern state_TypeDef state;       //状态
extern float gp;
extern bool climbing_offset_flag;//除开站立外其他姿态补偿一个倾斜角度

void Cartesian_To_Theta(int leg_direction);
void Sin_Trajectory( float t, Gait_Param params, float gaitOffset );
void Set_Position(int LegId);
void Coupled_MoveLeg( float t, Gait_Param params, float gait_offset, float leg_direction, int LegId );
void gait_detached(	Detached_Param d_params,float leg0_offset, float leg1_offset,float leg2_offset, float leg3_offset,float leg0_direction, float leg1_direction,float leg2_direction, float leg3_direction);
void Change_The_Gains_Of_PD(Leg_Gain gains);
void CommandAllLegs(Leg_Gain gains);
void CommandAllLegs_pre(Leg_Gain gains);
void CommandAllLegs_later(Leg_Gain gains);
void Posture_Control(void);
#endif
