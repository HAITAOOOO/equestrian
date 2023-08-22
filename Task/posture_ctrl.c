#include "posture_ctrl.h"
#include <math.h>
#include "motor.h"
#include "cmsis_os.h"
#include "rc_bsp.h"
#include "jump.h"

bool 	Is_Motor_Ready = Not_Ready;
float x, y, theta1, theta2;
state_TypeDef state = REALSE;

//设定每一种步态的增益: {kp_pos, kd_pos, kp_spd, kd_spd}
Leg_Gain leg_gains[] =
{
    {22.0, 0.00, 8.0, 0.00},   // TROT...............................OK
    {22.0, 0.00, 8.0, 0.00},   // CLIMBING
    {22.0, 0.00, 9.0, 0.00},   // WALK...............................暂定
    {22.0, 0.00, 8.0, 0.00},   // WALKBACK...........................OK
    {22.0, 0.00, 8.0, 0.00},   // ROTAT_LEFT.........................OK
    {22.0, 0.00, 8.0, 0.00},   // ROTAT_RIGHT........................暂定
    {12.0, 0.00, 6.6, 0.00},   // SPACEWALK..........................未使用
    {20.0, 0.00, 5.6, 0.00},   // LADDER.............................测试
    {22.0, 0.00, 8.6, 0.00},   // CLIMBINGUP.........................OK
    {12.0, 0.00, 6.6, 0.00},   // ROLLLEFT...........................OK
    {12.0, 0.00, 6.6, 0.00},   // ROLLRIGHT..........................OK
    {12.0, 0.00, 6.6, 0.00},   // FORWAED............................OK
    {12.0, 0.00, 6.6, 0.00},   // BACK...............................OK
    {22.0, 0.00, 6.6, 0.00},   // WALK_BACKl.........................OK
    {22.0, 0.00, 6.6, 0.00},   // TROTl..............................OK
    {22.0, 0.00, 6.6, 0.00},	 // ROTAT_LEFTl
    {22.0, 0.00, 6.6, 0.00},	 // ROTAT_RIGHTl
};
//设定每一种步态的参数
/*
typedef struct  						//步态腿部参数结构体
{
    float stance_height ;   // 狗身到地面的距离 (cm)
    float step_length ;     // 一步的距离 (cm)
    float up_amp ;          // 上部振幅y (cm)
    float down_amp ;        // 下部振幅 (cm)
    float flight_percent ;  // 摆动相百分比 (0-1)
    float freq ;            // 一步的频率 (Hz)
} gait_param;
*/

//第一层对应每一种步态，第二层对应该步态下的腿部参数
Detached_Param detached_params[] = {

    {   {22.0, 19, 6.00, 1.00, 0.45, 3.5},
        {22.0, 19, 6.00, 1.00, 0.45, 3.5},
        {22.0, 19, 6.00, 1.00, 0.45, 3.5},
        {22.0, 19, 6.00, 1.00, 0.45, 3.5}
    },    //TROT 对角小跑.....................................OK

    {   {25.0, 9.0, 3.00, 1.00, 0.25, 2.6},
        {25.0, 9.0, 3.00, 1.00, 0.25, 2.6},
        {16.0, 9.0, 3.00, 1.00, 0.25, 2.6},
        {16.0, 9.0, 3.00, 1.00, 0.25, 2.6}
    },     //LIMBING爬坡.....................................OK

    {   {22, 13.00, 6.0, 0.10, 0.3, 1.6},
        {22, 13.00, 6.0, 0.10, 0.3, 1.6},
        {22, 13.00, 6.0, 0.10, 0.3, 1.6},
        {22, 13.00, 6.0, 0.10, 0.3, 1.6}
    },       // WALK............................................暂定

    {   {22.0, 19, 5.00, 0.00, 0.45, 3.5},
        {22.0, 19, 5.00, 0.00, 0.45, 3.5},
        {22.0, 19, 5.00, 0.00, 0.45, 3.5},
        {22.0, 19, 5.00, 0.00, 0.45, 3.5}
    },      // WALKBACK........................................采用未分离数组

    {   {22.0, 13.60, 5, 2.00, 0.5, 3.6},
        {22.0, 13.60, 5, 2.00, 0.5, 3.6},
        {22.0, 13.60, 5, 2.00, 0.5, 3.6},
        {22.0, 13.60, 5, 2.00, 0.5, 3.6}
    },       // ROTATE_LEFT.....................................OK

    {   {22.0, 13.60, 5, 2.00, 0.5, 3.6},
        {22.0, 13.60, 5, 2.00, 0.5, 3.6},
        {22.0, 13.60, 5, 2.00, 0.5, 3.6},
        {22.0, 13.60, 5, 2.00, 0.5, 3.6}
    },       // ROTATE_RIGHT....................................暂定

    {   {27.0, 0.00, 1.00, 1.00, 0.3, 1.6},
        {23.0, 20.00, 10.00, 0.00, 0.75, 3.6},
        {23.0, 20.00, 10.00, 0.00, 0.75, 3.6},
        {27.0, 0.00, 1.00, 1.00, 0.3, 1.6}
    },    // SPACEWALK.....................................未使用

    {   {28.6, 8.0, 16.6, 1.00, 0.35, 1.5},
        {28.6, 8.0, 16.6, 1.00, 0.35, 1.5},
        {28.6, 8.0, 16.6, 1.00, 0.35, 1.5},
        {28.6, 8.0, 16.6, 1.00, 0.35, 1.5}
    },   // LADDER..........................................测试

    {   {19.0, 15.0, 5.00, 1.00, 0.25, 2.3},
        {19.0, 15.0, 5.00, 1.00, 0.25, 2.3},
        {27.0, 15.0, 5.00, 1.00, 0.25, 2.3},
        {27.0, 15.0, 5.00, 1.00, 0.25, 2.3}
    },     //LIMBINGUP.........................................OK

    {   {21.0, 0.0, 0.00, 0.00, 0.5, 2.6},
        {21.0, 0.0, 0.00, 0.00, 0.5, 2.6},
        {21.0, 0.0, 0.00, 0.00, 0.5, 2.6},
        {21.0, 0.0, 0.00, 0.00, 0.5, 2.6}
    },     //ROLLLEFT..............................................OK

    {   {24.0, 0.0, 0.00, 0.00, 0.5, 2.6},
        {16.0, 0.0, 0.00, 0.00, 0.5, 2.6},
        {24.0, 0.0, 0.00, 0.00, 0.5, 2.6},
        {16.0, 0.0, 0.00, 0.00, 0.5, 2.6}
    },     //ROLLRIGHT..............................................OK

    {   {16.0, 0.0, 0.00, 0.00, 0.5, 2.6},
        {16.0, 0.0, 0.00, 0.00, 0.5, 2.6},
        {24.0, 0.0, 0.00, 0.00, 0.5, 2.6},
        {24.0, 0.0, 0.00, 0.00, 0.5, 2.6}
    },     //FORWARD..............................................OK

    {   {24.0, 0.0, 0.00, 0.00, 0.5, 2.6},
        {24.0, 0.0, 0.00, 0.00, 0.5, 2.6},
        {16.0, 0.0, 0.00, 0.00, 0.5, 2.6},
        {16.0, 0.0, 0.00, 0.00, 0.5, 2.6}
    },     //BACK..............................................OK

    {   {21.0, 5.0, 5.00, 1.00, 0.45, 2.6},
        {21.0, 5.0, 5.00, 1.00, 0.45, 2.6},
        {21.0, 5.0, 5.00, 1.00, 0.45, 2.6},
        {21.0, 5.0, 5.00, 1.00, 0.45, 2.6}
    },     //WALK_BACKl...........................................OK

    {   {21.0, 15.0, 5.00, 1.00, 0.45, 2.6},
        {21.0, 15.0, 5.00, 1.00, 0.45, 2.6},
        {21.0, 15.0, 5.00, 1.00, 0.45, 2.6},
        {21.0, 15.0, 5.00, 1.00, 0.45, 2.6}
    },     //TROTl...........................................OK

    {   {21.0, 5.0, 5.00, 1.00, 0.45, 2.6},
        {21.0, 5.0, 5.00, 1.00, 0.45, 2.6},
        {21.0, 5.0, 5.00, 1.00, 0.45, 2.6},
        {21.0, 5.0, 5.00, 1.00, 0.45, 2.6}
    },

    {   {21.0, 5.0, 5.00, 1.00, 0.45, 2.6},
        {21.0, 5.0, 5.00, 1.00, 0.45, 2.6},
        {21.0, 5.0, 5.00, 1.00, 0.45, 2.6},
        {21.0, 5.0, 5.00, 1.00, 0.45, 2.6}
    },
};



/**
  * @brief          笛卡尔坐标转换到θ(theta)  即将xy转换成theta
	* @param[in]      leg_direction：行进方向 1:向前  -1向后
	* @param[in]			隐性参数x,y(全局)
	* @param[out]			隐性参数theta1,theta2(全局)
  * @retval         none
  */
void Cartesian_To_Theta(int leg_direction)
{
    float L=0;   			//当前足端在坐标系中的长度。
    double N=0,M=0; 	//解算过程中产生的角度
    float A1=0,A2=0;  //Angle1 Angle2 腿上一对电机的角度。

    //此处x，y为全局变量
    L=sqrt(	pow(x,2) + pow(y,2) );
    if(L<10) L=10;
    else if(L>30) L=30;

    N=asin(x/L)*180.0f/PI;
    M=acos((pow(L,2)+pow(L1,2)-pow(L2,2))/(2*L1*L)	)*180.0/PI;
    A1=M-N;
    A2=M+N;

    if(leg_direction==1)
    {
        theta2=(A1-90.0f);
        theta1=(A2-90.0f);
    } else if(leg_direction==-1)
    {
        theta1=(A1-90.0f);
        theta2=(A2-90.0f);
    }
}

/**
  * @brief          根据步态参数生成足端正弦轨迹轨迹
	* @param[in]      t：滴答定时器计数值
	* @param[in]      params: 步态参数
	* @param[in]      gaitOffset: 步态时差补偿，类似相位差
  * @retval         none
  */
float gp;//需为全局变量
void Sin_Trajectory( float t, Gait_Param params, float gaitOffset )
{
    static float p = 0;
    static float prev_t = 0;
    float stanceHeight = params.stance_height;
    float downAMP = params.down_amp;
    float upAMP = params.up_amp;
    float swingPercent = params.swing_percent;
    float stepLength = params.step_length;
    float FREQ = params.freq;

    p += FREQ * (t - prev_t);
    prev_t = t;

    gp = fmod((p+gaitOffset),1.0);//当前的时间对1取余从而得到当前处于一个周期的哪一个阶段，一个周期长度为1
    if (gp <= swingPercent) //当当前时刻位于摆动相
    {
        x = (gp/swingPercent)*stepLength - stepLength/2.0f;
        y = -upAMP*sin(PI*gp/swingPercent) + stanceHeight;
    }
    else									 //当当前时刻位于支撑相
    {
        float percentBack = (gp-swingPercent)/(1.0f-swingPercent);
        x = -percentBack*stepLength + stepLength/2.0f;
        y = downAMP*sin(PI*percentBack) + stanceHeight;
    }
}


/**
	* @brief          向电机发送角度 theta1为内侧脚  theta2为外侧脚  电机顺时针转为负
	* @param[in]      LegId：腿ID
  * @retval         none
  */
bool climbing_offset_flag = NO;//除开站立外其他姿态补偿一个倾斜角度
void Set_Position(int LegId)
{
    if(state == STANDUP)
        climbing_offset_flag = YES;
    else
        climbing_offset_flag = NO;

    if(climbing_offset_flag==YES)
    {
        if(LegId==0)
        {
            motor_pid_pos[1].ref_angle=(-theta1)*ReductionAndAngleRatio;
            motor_pid_pos[0].ref_angle=(-theta2)*ReductionAndAngleRatio;
        }
        else if(LegId==1)
        {
            motor_pid_pos[2].ref_angle=(theta1)*ReductionAndAngleRatio;
            motor_pid_pos[3].ref_angle=(theta2)*ReductionAndAngleRatio;
        }
        else if(LegId==2)
        {
            motor_pid_pos[5].ref_angle=(-theta1)*ReductionAndAngleRatio;
            motor_pid_pos[4].ref_angle=(-theta2)*ReductionAndAngleRatio;
        }
        else if(LegId==3)
        {
            motor_pid_pos[6].ref_angle=(+theta1)*ReductionAndAngleRatio;
            motor_pid_pos[7].ref_angle=(+theta2)*ReductionAndAngleRatio;
        }
        climbing_offset_flag = NO;
    }
    else {
        if(LegId==0)
        {
            motor_pid_pos[1].ref_angle=(-theta1-45)*ReductionAndAngleRatio;
            motor_pid_pos[0].ref_angle=(-theta2+105)*ReductionAndAngleRatio;
        }
        else if(LegId==1)
        {
            motor_pid_pos[2].ref_angle=(theta1+45)*ReductionAndAngleRatio;
            motor_pid_pos[3].ref_angle=(theta2-105)*ReductionAndAngleRatio;
        }

        else if(LegId==2)
        {
            motor_pid_pos[5].ref_angle=(-theta1-45)*ReductionAndAngleRatio;
            motor_pid_pos[4].ref_angle=(-theta2+105)*ReductionAndAngleRatio;
        }
        else if(LegId==3)
        {
            motor_pid_pos[6].ref_angle=(theta1+45)*ReductionAndAngleRatio;
            motor_pid_pos[7].ref_angle=(theta2-105)*ReductionAndAngleRatio;
        }
    }
    Is_Motor_Ready= Is_Ready;		//数据填充完毕
}
/**
	* @brief          根据步态参数生成足端x,y，再由x，y生成电机转动角度，将生成角度赋值到电机发送值
	* @param[in]      t：滴答定时器计数值
	* @param[in]      params: 步态参数
	* @param[in]      gaitOffset: 步态时差补偿，类似相位差
	* @param[in]      leg_direction：行进方向 1:向前  -1向后
	* @param[in]      LegId：腿ID
  * @retval         none
  */
void Coupled_MoveLeg( float t, Gait_Param params, float gait_offset, float leg_direction, int LegId )
{
    Sin_Trajectory( t, params, gait_offset );    //根据步态参数生成足端x,y
    Cartesian_To_Theta( leg_direction );		     //由x，y生成电机转动角度
    Set_Position(LegId);		        					   //将生成角度赋值到电机发送值
}

/**
  * @brief          步态_分离参数控制
  * @param[in]      d_params：包括四个步态参数结构体，在Posture_Control函数开始时被赋值
	* @param[in]			leg(0-4)_offset:步态相位差
	* @param[in]			leg(0-4)_direction:腿方向
  * @retval         none
  */
void gait_detached(	Detached_Param d_params,
                    float leg0_offset, float leg1_offset,float leg2_offset, float leg3_offset,
                    float leg0_direction, float leg1_direction,float leg2_direction, float leg3_direction)
{
    float t = HAL_GetTick()/1000.0;

    Coupled_MoveLeg( t, d_params.detached_params_0, leg0_offset, leg0_direction, 0);
    Coupled_MoveLeg( t, d_params.detached_params_1, leg1_offset, leg1_direction, 1);
    Coupled_MoveLeg( t, d_params.detached_params_2, leg2_offset, leg2_direction, 2);
    Coupled_MoveLeg( t, d_params.detached_params_3, leg3_offset, leg3_direction, 3);
}

/**
  * @brief          改变腿部增益
	* @param[in]      gains:腿部增益 kp_pos kd_pos kp_spd kd_spd
  * @retval         none
  */
void Change_The_Gains_Of_PD(Leg_Gain gains)
{
    for(int i =0; i<8; i++)
    {
        pid_reset(&motor_pid_pos[i],gains.kp_pos,0,gains.kd_pos);
        pid_reset(&motor_pid_speed[i],gains.kp_spd,0,gains.kd_spd);
    }
}

/**
	* @brief          改变kpkd，用于根据用户自己设定的x，y进行的参数传递
  * @param[in]      gains:腿部增益 kp_pos kd_pos kp_spd kd_spd
	* @param[in]      隐性输入：theta1，theta2
  * @retval         none
  */
void CommandAllLegs(Leg_Gain gains)
{
    Change_The_Gains_Of_PD(gains);
    Set_Position(0);
    Set_Position(1);
    Set_Position(2);
    Set_Position(3);
}
/**
	* @brief          同上，为了使jump时前后腿有差别而写
  * @param[in]      gains:腿部增益 kp_pos kd_pos kp_spd kd_spd
	* @param[in]      隐性输入：theta1，theta2
  * @retval         none
  */
void CommandAllLegs_pre(Leg_Gain gains)
{
    Change_The_Gains_Of_PD(gains);
    Set_Position(0);
    Set_Position(1);
}

/**
	* @brief          同上  和CommandAllLegs_pre同一个gains 覆盖问题？？
  * @param[in]      gains:腿部增益 kp_pos kd_pos kp_spd kd_spd
	* @param[in]      隐性输入：theta1，theta2
  * @retval         none
  */
void CommandAllLegs_later(Leg_Gain gains)
{
    Change_The_Gains_Of_PD(gains);
    Set_Position(2);
    Set_Position(3);
}

/**
  * @brief          姿态控制
  * @param[in]      none
	* @param[in]      隐性输入：各种全局变量：state，gait_params，detached_params，leg_gains
  * @retval         none
  */
Detached_Param detached_param;
Leg_Gain leg_gain;			  //全局 局部？？
Leg_Gain gain_standup = {8.5, 0.00, 6.6, 0.00};
Leg_Gain gain_stop = {16.6, 0.00, 6.6, 0.00};
void Posture_Control(void)
{

    detached_param  =  detached_params[state];	 //分离参数
    leg_gain      	=  leg_gains[state];				 //腿部增益

    switch(state)
    {
    case STANDUP :															//STANDUP(站立)，STOP(停止)两种静止状态 相对于其他状态，没有Sin_Trajectory，而是用户自己设定x，y
        x = 11;
        y = 0;
        Cartesian_To_Theta(1.0);								//根据x，y计算theta1，theta2
        CommandAllLegs(gain_standup);										//根据theta1，theta2计算输出角度 含Change_The_Gains_Of_PD(gain);
        break;

    case STOP:
        x=0;
        y = 21;
        Cartesian_To_Theta(1.0);
        CommandAllLegs(gain_stop);
        break;

    case REALSE:// 释放 什么都不做
        vTaskDelay(500);
        break;

    case CLIMBING://爬坡
        Change_The_Gains_Of_PD(leg_gain);																			//根据state改变增益
        gait_detached(detached_param, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0);//根据state改变detached_param(步态参数)，前4：相位差，后4：方向
        break;

    case WALK_BACKl://慢速后退
        Change_The_Gains_Of_PD(leg_gain);
        gait_detached(detached_param, 0.0, 0.5, 0.5, 0.0, -1.0, -1.0, -1.0, -1.0);
        break;

    case WALK://前进
        Change_The_Gains_Of_PD(leg_gain);
        gait_detached(detached_param, 0.5, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0);
        break;

    case WALK_BACK://后退
        Change_The_Gains_Of_PD(leg_gain);
        gait_detached(detached_param,0.0, 0.5, 0.5, 0.0, -1.0, -1.0, -1.0, -1.0);
        break;

    case SPACEWALK://
        Change_The_Gains_Of_PD(leg_gain);
        gait_detached(detached_param, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0);
        break;

    case LADDER://阶梯
        Change_The_Gains_Of_PD(leg_gain);
        gait_detached(detached_param,0.06, 0.5, 0.46, 0.0, 1.0, 1.0, 1.0, 1.0);
        break;

    case ROTAT_LEFT://原地左转
        Change_The_Gains_Of_PD(leg_gain);
        gait_detached(detached_param, 0.0, 0.5, 0.5, 0.0, -1.0, 1.0, -1.0, 1.0);
        break;

    case ROTAT_RIGHT://原地右转
        Change_The_Gains_Of_PD(leg_gain);
        gait_detached(detached_param, 0.0, 0.5, 0.5, 0.0, 1.0, -1.0, 1.0, -1.0);
        break;

    case ROTAT_LEFTl://原地左转
        Change_The_Gains_Of_PD(leg_gain);
        gait_detached(detached_param, 0.0, 0.5, 0.5, 0.0, -1.0, 1.0, -1.0, 1.0);
        break;

    case ROTAT_RIGHTl://原地右转
        Change_The_Gains_Of_PD(leg_gain);
        gait_detached(detached_param, 0.0, 0.5, 0.5, 0.0, 1.0, -1.0, 1.0, -1.0);
        break;

    case JUMP1://跳跃
        ExecuteJump1();
        break;

    case JUMP2://跳跃
        ExecuteJump2();
        break;

    case TROT://对角小跑
        Change_The_Gains_Of_PD(leg_gain);
        if(rc_ctrl.sw1 == DOWN)
        {
            if(rc_ctrl.ch2<0)
            {
                detached_param.detached_params_0.step_length = 15;
                detached_param.detached_params_2.step_length = 15;
            }
            else if(rc_ctrl.ch2>0)
            {
                detached_param.detached_params_1.step_length = 15;
                detached_param.detached_params_3.step_length = 15;
            }
        }
        gait_detached(detached_param,  0.5, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0);
        break;

    case TROTl://对角小跑
        Change_The_Gains_Of_PD(leg_gain);
        gait_detached(detached_param,  0.5, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0);
        break;

    case CLIMBINGUP://爬坡
        Change_The_Gains_Of_PD(leg_gain);
        gait_detached(detached_param, 0.0, 0.5, 0.5, 0.0, 1.0, 1.0, 1.0, 1.0);
        break;

    case ROLLLEFT:
        Change_The_Gains_Of_PD(leg_gain);
        gait_detached(detached_param, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0);
        break;

    case ROLLRIGHT:
        Change_The_Gains_Of_PD(leg_gain);
        gait_detached(detached_param, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0);
        break;

    case FORWARD:
        Change_The_Gains_Of_PD(leg_gain);
        gait_detached(detached_param, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0);
        break;

    case BACK:
        Change_The_Gains_Of_PD(leg_gain);
        gait_detached(detached_param, 0.0, 0.0, 0.0, 0.0, 1.0, 1.0, 1.0, 1.0);
        break;
    }
    vTaskDelay(3);

}

