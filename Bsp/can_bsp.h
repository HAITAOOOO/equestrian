#ifndef CAN_BSP_H
#define CAN_BSP_H
#include "main.h"
//3508电机控制电流, 范围 [-16384,16384]
typedef enum
{
    CAN_3508_ALL_ID = 0x200,
    CAN_3508_M1_ID = 0x201,
    CAN_3508_M2_ID = 0x202,
    CAN_3508_M3_ID = 0x203,
    CAN_3508_M4_ID = 0x204,

    CAN_2006_ALL_ID = 0x1FF,
    CAN_2006_M1_ID = 0x205,
    CAN_2006_M2_ID = 0x206,
    CAN_2006_M3_ID = 0x207,
    CAN_2006_M4_ID = 0x208,

} can_motor_id;

//rm motor data
typedef struct
{
    uint16_t ecd;							//电机转子角度
    int real_ecd;							//电机转子绝对角度（360）
    int16_t speed_rpm;				//转速
    int16_t given_current;		//给定电流（电流控制）
    uint8_t temperate;				//温度
    int16_t last_ecd;					//转子上一瞬角度
    int16_t	offset_angle;			//补偿角度
    int32_t	round_cnt;				//累计圈数
    int32_t	total_angle;			//累计角度

} motor_measure_t;

extern motor_measure_t can_motor_message[8];              //存储can上对应电机返回解码后的信息 can1 can2不可共用ID 

void can_filter_init(void);
void CAN1_Send1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN1_Send2(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);
void CAN2_Send1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4);
void CAN2_Send2(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8);


#endif
