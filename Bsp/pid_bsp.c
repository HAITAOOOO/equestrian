#include "pid_bsp.h"
#include "stm32f4xx.h"

#define ABS(x)  ((x>0)? x: -x)

/*--------对PID_TypeDef pid初始化--------*/
//例：f_param_init(&motor_pid2[i],PID_Speed,16383,5000,10,0,0,3.0,0.05,0.27);
static void pid_param_init(
    PID_TypeDef * pid,
    float deadband,
    int16_t  target,
    uint16_t maxout,
    uint16_t intergral_limit,


    float 	kp,
    float 	ki,
    float 	kd)
{
    pid->DeadBand = deadband;
    pid->target = target;
    pid->IntegralLimit = intergral_limit;	//积分极限
    pid->MaxOutput = maxout;

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;

    pid->output = 0;
}

/*中途更改参数设定--------------------------------------------------------------*/
void pid_reset(PID_TypeDef * pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

/*-----------------pid计算---------------------------*/
//例：f_cal_pid(&motor_pid2[i],can_motor_message[i].speed_rpm);
static float pid_calcuate(PID_TypeDef * pid,float measure,float target)
{
    pid->measure = measure;
    pid->target = target;

    pid->err = pid->target - pid->measure;

    //是否进入死区
    if((ABS(pid->err)) > pid->DeadBand)
    {
        pid->pout = pid->kp*pid->err;
        pid->iout += (pid->ki*pid->err);
        pid->dout = pid->kd*(pid->err - pid->last_err);

        //积分是否超出限制
        if(pid->iout > pid->IntegralLimit)
        {
            pid->iout = pid->IntegralLimit;
        }
        if(pid->iout < -pid->IntegralLimit)
        {
            pid->iout = -pid->IntegralLimit;
        }

        //pid输出
        pid->output = pid->pout + pid->iout + pid->dout;

        //滤波
        //pid->output = pid->output*0.7f + pid->last_output*0.3f;

        //输出是否超出限制
        if(pid->output>pid->MaxOutput)
        {
            pid->output = pid->MaxOutput;
        }
        if(pid->output < -(pid->MaxOutput))
        {
            pid->output = -(pid->MaxOutput);
        }
        pid->last_err  = pid->err;      //err - last_err计算dout
    }
    return pid->output;
}



/*-------pid结构体初始化，每一个pid参数需要调用一次----*/
//例：pid_init(&motor_pid2[i]);
void pid_init(PID_TypeDef* pid)        //用结构体内的函数定义结构体其他成员的值
{
    pid->f_param_init = pid_param_init;    //给f_param_init定义成员
    pid->f_cal_pid = pid_calcuate;			 	 //单级初始化
    pid->f_pid_reset = pid_reset;					 //单级参数修改

}

/*******规则表 参数根据实际整定 *******/
#define ERR_L   500
#define ERR_M	 	250
#define ERR_S	 	125
#define ERR_ZE	0

#define ERR_EIGHT   1600
#define ERR_SEVEN   1400
#define ERR_SIX	 		1200
#define ERR_FIVE 		1000
#define ERR_FOUR		800
#define ERR_THREE   600
#define ERR_TWO	 		400
#define ERR_ONE	 		200
#define ERR_ZERO		0

#define KP_EIGHT   	-3
#define KP_SEVEN   	-2.5
#define KP_SIX	 		-2.0
#define KP_FIVE 		0
#define KP_FOUR			0
#define KP_THREE   	0
#define KP_TWO	 		0
#define KP_ONE	 		0.5
#define KP_ZERO			1

int16_t E[9]= {ERR_ZERO,ERR_ONE,ERR_TWO,ERR_THREE,ERR_FOUR,ERR_FIVE,ERR_SIX,ERR_SEVEN,ERR_EIGHT};
float 	KP[9]= {KP_ZERO,KP_ONE,KP_TWO,KP_THREE,KP_FOUR,KP_FIVE,KP_SIX,KP_SEVEN,KP_EIGHT};

/*以下需为全局变量  ？*/
double eLmembership,eRmembership;  //隶属度
double eLmembership_val,eRmembership_val; //隶属值

double fuzzy(PID_TypeDef * pid)
{
    //short etemp;
    if(pid->err>ERR_EIGHT)
    {
				 
        eLmembership = (pid->err - ERR_SEVEN)*1.0f/(ERR_EIGHT-ERR_SEVEN);
        eLmembership_val = KP_EIGHT * eLmembership;

    }
    else if(ERR_EIGHT>pid->err&&pid->err>ERR_SEVEN)
    {
        eLmembership = (pid->err - ERR_SEVEN)*1.0f/(ERR_EIGHT-ERR_SEVEN);
        eRmembership = (ERR_EIGHT - pid->err)*1.0f/(ERR_EIGHT-ERR_SEVEN);
        eLmembership_val = ERR_EIGHT * eLmembership;
        eRmembership_val = KP_SEVEN * eRmembership;
    }
    else if(ERR_SEVEN>pid->err&&pid->err>ERR_SIX)
    {
        eLmembership = (pid->err - ERR_SIX)*1.0f/(ERR_SEVEN-ERR_SIX);
        eRmembership = (ERR_SEVEN -  pid->err)*1.0f/(ERR_SEVEN-ERR_SIX);
        eLmembership_val = ERR_SEVEN * eLmembership;
        eRmembership_val = ERR_SIX * eRmembership;
    }
    else if(ERR_SIX>pid->err&&pid->err>ERR_FIVE)
    {
        eLmembership = (pid->err - ERR_FIVE)*1.0f/(ERR_SIX-ERR_FIVE);
        eRmembership = (ERR_SIX -  pid->err)*1.0f/(ERR_SIX-ERR_FIVE);
        eLmembership_val = ERR_SIX * eLmembership;
        eRmembership_val = ERR_FIVE * eRmembership;

    }
    else if(ERR_FIVE>pid->err&&pid->err>ERR_FOUR)
    {
        eLmembership = (pid->err - ERR_FOUR)*1.0f/(ERR_FIVE-ERR_FOUR);
        eRmembership = (ERR_FIVE -  pid->err)*1.0f/(ERR_FIVE-ERR_FOUR);
        eLmembership_val = ERR_FIVE * eLmembership;
        eRmembership_val = ERR_FOUR * eRmembership;

    }
    else if(ERR_FOUR>pid->err&&pid->err>ERR_THREE)
    {
        eLmembership = (pid->err - ERR_THREE)*1.0f/(ERR_FOUR-ERR_THREE);
        eRmembership = (ERR_FOUR -  pid->err)*1.0f/(ERR_FOUR-ERR_THREE);
        eLmembership_val = ERR_FOUR * eLmembership;
        eRmembership_val = ERR_THREE * eRmembership;

    }
    else if(ERR_THREE>pid->err&&pid->err>ERR_TWO)
    {
        eLmembership = (pid->err - ERR_TWO)*1.0f/(ERR_THREE-ERR_TWO);
        eRmembership = (ERR_THREE -  pid->err)*1.0f/(ERR_THREE-ERR_TWO);
        eLmembership_val =ERR_THREE * eLmembership;
        eRmembership_val = ERR_TWO * eRmembership;

    }
    else if(ERR_TWO>pid->err&&pid->err>ERR_ONE)
    {
        eLmembership = (pid->err - ERR_ONE)*1.0f/(ERR_TWO-ERR_ONE);
        eRmembership = (ERR_TWO -  pid->err)*1.0f/(ERR_TWO-ERR_ONE);
        eLmembership_val = ERR_TWO * eLmembership;
        eRmembership_val = ERR_ONE * eRmembership;

    }
    else if(ERR_ONE>pid->err&&pid->err>ERR_ZERO)
    {
        eLmembership = (pid->err - ERR_ZERO)*1.0f/(ERR_ONE-ERR_ZERO);
        eRmembership = (ERR_ONE -  pid->err)*1.0f/(ERR_ONE-ERR_ZERO);
        eLmembership_val = ERR_ONE * eLmembership;
        eRmembership_val = ERR_ZERO * eRmembership;
    }
    return (eLmembership_val + eRmembership_val);
}
