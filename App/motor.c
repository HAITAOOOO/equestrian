#include "motor.h"
#include "can_bsp.h"
#include "cmsis_os.h"
#include "posture_ctrl.h"

PID_TypeDef motor_pid_speed[8];
PID_TypeDef motor_pid_pos[8];

/**
	* @brief          pid初始化
	* @param[in]      none
  * @retval         none
  */
void PID_init()
{
	  for (int i = 0; i < 8; i++)
    {
			pid_init(&motor_pid_speed[i]);
			motor_pid_speed[i].f_param_init(&motor_pid_speed[i],10,0, 16383,5000,0,0.05,0); // 结构体 死区 初始期望 输出限幅 积分限幅 kp ki kd
    }
    for (int i = 0; i < 8; i++)
    {
			pid_init(&motor_pid_pos[i]);
			motor_pid_pos[i].f_param_init(&motor_pid_pos[i],10,0, 16383,5000,0,0.0,0.0);
    }
}

/**
	* @brief          can发送函数pid版
	* @param[in]      none
  * @retval         none
  */
float ref_angle[8]; 
void CAN_Send_PID()
{
			if(Is_Motor_Ready== Is_Ready) 
			{
        for(int i=0; i<8; i++)
            ref_angle[i]=motor_pid_pos[i].ref_angle;
        Is_Motor_Ready= Not_Ready;
    }
			
		for(int i=0;i<8;i++)
		{
			if(state==JUMP2)
			{
				for(int i = 0;i<8;i++)
				{
					motor_pid_pos[i].MaxOutput=6770;
				}
			}
			else
			{
				for(int i = 0;i<8;i++)
				{
					motor_pid_pos[i].MaxOutput=16383;
				}
			}
			motor_pid_pos[i].f_cal_pid(&motor_pid_pos[i],can_motor_message[i].total_angle/100,ref_angle[i]/100);
			motor_pid_speed[i].f_cal_pid(&motor_pid_speed[i],can_motor_message[i].speed_rpm,motor_pid_pos[i].output);
		}
		
		CAN2_Send1(motor_pid_speed[0].output,motor_pid_speed[1].output,motor_pid_speed[2].output,motor_pid_speed[3].output);
		CAN1_Send2(motor_pid_speed[4].output,motor_pid_speed[5].output,motor_pid_speed[6].output,motor_pid_speed[7].output);

    osDelay(10);
}
