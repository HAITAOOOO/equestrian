#include "task_user.h"
#include "motor.h"
#include "posture_ctrl.h"
#include "rc_bsp.h"

#define START_TASK_PRIO 1
#define START_STK_SIZE 256
static  TaskHandle_t StartTask_Handler;

#define POSTURE_TASK_PRIO 13
#define POSTURE_STK_SIZE 512
static  TaskHandle_t PostureTask_Handler;

#define MOTOR_TASK_PRIO 13
#define MOTOR_STK_TASK 512
static  TaskHandle_t MotorTask_Handler;


void POSTURE_TASK(void * param)
{
    for(;;)
    {
        Posture_Control();
        osDelay(1);
    }
}

void MOTOR_TASK(void * param)
{
    for(;;)
    {
        CAN_Send_PID();
        osDelay(1);
    }
}

void start_task(void *pvParameters)
{
    taskENTER_CRITICAL();//进入临界保护区
    xTaskCreate((TaskFunction_t)POSTURE_TASK,(const char *)"POSTURE_TASK",(uint16_t)POSTURE_STK_SIZE,(void *)NULL,(UBaseType_t)POSTURE_TASK_PRIO,(TaskHandle_t *)&PostureTask_Handler);
    xTaskCreate((TaskFunction_t)MOTOR_TASK,(const char *)"MOTOR_TASK",(uint16_t)MOTOR_STK_TASK,(void *)NULL,(UBaseType_t)MOTOR_TASK_PRIO,(TaskHandle_t *)&MotorTask_Handler);

    vTaskDelete(StartTask_Handler); //删除开始任务
    taskEXIT_CRITICAL();            //退出临界区
}
//创建开始任务
void startTast(void)
{
    xTaskCreate((TaskFunction_t)start_task,          //任务函数
                (const char *)"start_task",          //任务名称
                (uint16_t)START_STK_SIZE,            //任务堆栈大小
                (void *)NULL,                        //传递给任务函数的参数
                (UBaseType_t)START_TASK_PRIO,        //任务优先级
                (TaskHandle_t *)&StartTask_Handler); //任务句柄

}


