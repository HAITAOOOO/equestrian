#ifndef __BUZZER_BSP_H_
#define __BUZZER_BSP_H_

#include "main.h"

#define  proport          100000 //Tclk/(psc+1)=180000000/(89+1)
#define  L1       ((proport/131)-1)//低调　do 的周期根据Tout= ((arr+1)*(psc+1))/Tclk推出arr值就是本句define定义的值，Tout为音调频率131Hz的倒数，Tclk=72MHz
#define  L2       ((proport/147)-1)//低调　re 的周期
#define  L3       ((proport/165)-1)//低调　mi 的周期
#define  L4       ((proport/176)-1)//低调　fa 的周期
#define  L5       ((proport/196)-1)//低调　sol的周期
#define  L6       ((proport/220)-1)//低调　la 的周期
#define  L7       ((proport/247)-1)//低调　si 的周期

#define  M1       ((proport/262)-1)//中调　do 的周期
#define  M2       ((proport/296)-1)//中调　re 的周期
#define  M3       ((proport/330)-1)//中调　mi 的周期 
#define  M4       ((proport/349)-1)//中调　fa 的周期
#define  M5       ((proport/392)-1)//中调　sol的周期
#define  M6       ((proport/440)-1)//中调　la 的周期
#define  M7       ((proport/494)-1)//中调　si 的周期

#define  H1       ((proport/523)-1)//高调　do 的周期
#define  H2       ((proport/587)-1)//高调　re 的周期
#define  H3       ((proport/659)-1)//高调　mi 的周期
#define  H4       ((proport/699)-1)//高调　fa 的周期
#define  H5       ((proport/784)-1)//高调　sol的周期
#define  H6       ((proport/880)-1)//高调　la 的周期
#define  H7       ((proport/988)-1)//高调　si 的周期

#define  Z0       0//

void buzzer_time(uint16_t psc, uint16_t arr,uint16_t time);
void buzzer_on(uint16_t psc, uint16_t arr);
void buzzer_off(void);
void happy_time(void);
void BeginWarnBuzzer(void);
void music(void);

#endif
