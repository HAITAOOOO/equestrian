#ifndef PID_H
#define PID_H

#include "stdint.h"

typedef struct _PID_TypeDef
{
    float target;							//目标值
    float kp;
    float ki;
    float kd;

    float   measure;					//测量值
    float   err;							//误差
    float   last_err;      		//上次误差

    float pout;
    float iout;
    float dout;

    float output;						//本次输出
    float last_output;			//上次输出

    float MaxOutput;				//输出限幅
    float IntegralLimit;		//积分限幅
    float DeadBand;			  //死区（绝对值）
	
		float ref_angle;     //参考角度

    void (*f_param_init)(struct _PID_TypeDef *pid,  //PID参数初始化
												 float deadband,
												 int16_t  target,
                         uint16_t maxOutput,
                         uint16_t integralLimit,
                         float kp,
                         float ki,
                         float kd);

    void (*f_pid_reset)(struct _PID_TypeDef *pid, float kp,float ki, float kd);		//pid三个参数修改
    float (*f_cal_pid)(struct _PID_TypeDef *pid, float measure,float target);   //pid计算
} PID_TypeDef;


/*****************模糊pid部分******************/
typedef struct
{
    float dKp;          //PID变化量
    float dKi;
    float dKd;

    float stair ;	      //动态调整梯度   //0.25f
    float Kp_stair;                      //0.015f
    float Ki_stair;                      //0.0005f
    float Kd_stair;                      //0.001f

} FuzzyPID;

void  pid_init(PID_TypeDef* pid);
void pid_reset(PID_TypeDef * pid, float kp, float ki, float kd);

double fuzzy(PID_TypeDef * pid);

#endif
