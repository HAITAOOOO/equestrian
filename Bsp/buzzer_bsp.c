#include "buzzer_bsp.h"
#include "tim.h"
/**
	* @brief          爱在西元前
  * @param[in]      none
  * @retval         none
  */
void music()
{
    int music[]=
    {
//        L1,125,L5,125,M1,150,L5,100,M1,100,M2,100,M2,100,L5,100,M1,100,L5,100,M5,100,
//				L7,100,M1,100,L6,100,L5,100,M1,100,L5,100,M5,100,L5,100,M1,100,M2,100,L6,100,
//				L5,100,M1,100,L5,100,L7,100,M1,100
//					L7,100,M1,100,L7,100,L5,100,Z0,100,L3,100,M2,100,M1,175,Z0,100,

//					M1,100,M2,100,M3,100,M1,100,Z0,100,M1,100,M2,100,M3,100,M1,200,M1,25,Z0,100,
//					L7,100,M1,100, L7,100,L6,100,L5,100,L6,100,L5,100,Z0,100,
//					L7,100,M1,100,L7,100,L5,100,Z0,5,L5,100,L3,100,
//					M2,150,M1,100,M1,100,M2,100,M3,100,M3,100,Z0,100,
//					L5,100,L6,100,M1,100,M3,200,M4,100,M3,100,M1,100,L6,100,Z0,100,//他不止一次骗了你
        M5,100,Z0,1,M5,100,M3,100,M2,100,M3,100,M5,100,Z0,1,M3,100,M2,100,M3,100,M5,100,Z0,1,
        M5,100,Z0,1,M5,100,M3,100,M2,100,M3,100,M2,100,M3,100,M2,100,L6,100,M1,100,Z0,1,
        M5,100,M5,100,M3,100,M2,100,M3,100,M2,100,M3,100,M2,100,M3,100,L6,100,L6,100,Z0,1,
        L6,100,L6,100,M1,100,M2,100,M3,100,M2,100,M1,100,M2,100,M3,100,M5,100,Z0,1
    };
    int length = sizeof(music)/sizeof(music[0]);//计算数组长度
    for(int i=0; i<(length/2); i++)//取数组数据
    {
        buzzer_on(music[2*i], 200);
        HAL_Delay(4*music[2*i+1]);//音长的时间都乘以5即一拍为500微秒，此值"5"可调整，只是播放的整个快慢而已，有点类似于视频快进和后退
    }
    buzzer_off();
}
/**
	* @brief          开机提示音
  * @param[in]      none
  * @retval         none
  */
// 例:BeginWarnBuzzer();
void BeginWarnBuzzer(void)
{
    int sound[]=
    {
        M1,50,M2,50,M3,50,M4,50,Z0,100
    };

    int length = sizeof(sound)/sizeof(sound[0]);//计算数组长度
    for(int i=0; i<(length/2); i++)//取数组数据
    {
        buzzer_on(sound[2*i], 200);
        HAL_Delay(4*sound[2*i+1]);//音长的时间都乘以5即一拍为500微秒，此值"5"可调整，只是播放的整个快慢而已，有点类似于视频快进和后退
    }
    buzzer_off();
}
/**
  * @brief          祝你生日快乐！！！
  * @param[in]      none
  * @retval         none
  */
// 例:happy_time();
void happy_time(void)
{
    int happy_birthday[]=
    {
        M5,50,M5,25,M5,25,
        M6,100,M5,100,H1,100,
        M7,100,M7,100,M5,50,M5,25,M5,25,
        M6,100,M5,100,H2,100,
        H1,100,H1,100,M5,50,M5,25,M5,25,
        H5,100,H3,100,H1,100,
        M7,100,M6,100,H4,50,H4,25,H4,25,
        H3,100,H1,100,H2,100,H1,100,H1,100,Z0,150
    };

    int length = sizeof(happy_birthday)/sizeof(happy_birthday[0]);//计算数组长度
    for(int i=0; i<(length/2); i++)//取数组数据
    {
        buzzer_on(happy_birthday[2*i], 200);
        HAL_Delay(5*happy_birthday[2*i+1]);//音长的时间都乘以5即一拍为500微秒，此值"5"可调整，只是播放的整个快慢而已，有点类似于视频快进和后退
    }
    buzzer_off();
}

/**
  * @brief          蜂鸣器响一段时间
  * @param[in]      psc：	预分频值
	* @param[in] 			arr：	自动重装周期
	* @param[in] 			time：时间，单位ms
  * @retval         none
  */
// 例:buzzer_time(900,500,1000);
void buzzer_time(uint16_t psc, uint16_t arr,uint16_t time)
{
    buzzer_on(psc,arr);
    HAL_Delay(time);
    buzzer_off();
}

/**
  * @brief          蜂鸣器启动函数
  * @param[in]      psc：	预分频值
	* @param[in] 			arr：	自动重装周期
  * @retval         none
  */
//例:buzzer_on();
void buzzer_on(uint16_t psc, uint16_t arr)
{
    TIM12->PSC = psc-1;
    TIM12->ARR = arr-1;
    TIM12->CCR1 = arr*0.5;
    HAL_TIM_PWM_Start(&htim12,TIM_CHANNEL_1);//开启PWM通道1
}

/**
  * @brief          蜂鸣器关闭函数
  * @param[in]      none
  * @retval         none
  */
//例:buzzer_off();
void buzzer_off(void)
{
    TIM12->CCR1=0;
}

