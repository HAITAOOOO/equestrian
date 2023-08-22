#include "can_bsp.h"
#include "can.h"
#include "rc_bsp.h"
#include "math.h"
#include "string.h"

static CAN_TxHeaderTypeDef  can1_tx1_message;					 //存储can1报文ID,格式
static CAN_TxHeaderTypeDef  can1_tx2_message;       	 //存储can1报文ID,格式
static CAN_TxHeaderTypeDef  can2_tx1_message;       	 //存储can2报文ID,格式
static CAN_TxHeaderTypeDef  can2_tx2_message;       	 //存储can2报文ID,格式

static uint8_t              can1_send1_data[8]; 		//存储can1发送数据
static uint8_t              can1_send2_data[8];		  //存储can1发送数据
static uint8_t              can2_send1_data[8];		  //存储can2发送数据
static uint8_t  						can2_send2_data[8]; 		//存储can2发送数据

int k=3;

motor_measure_t can_motor_message[8];              //存储can上对应电机返回解码后的信息 can1 can2不可共用ID 

/**
  * @brief          解码函数（3508/2006），将data解码到ptr 宏定义执行速度快于函数
  * @param[in]      ptr:数组首地址
	* @param[in]      data:数据
  * @retval         none
  */
#define get_motor_measure(ptr, data)/* “ \ ” 为续行符，表示下面一行是紧接着当前行，\后不可含有任何符号，包括空格和空行，此处必须使用 */    	\
{                                                                 		  \
  (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]); /*转子机械角度*/ \
	if(k)																																	\
	{ 																																		\
		(ptr)->offset_angle =(ptr)->last_ecd = (ptr)->ecd;                  \
	   k--;																																\
	}																																			\
	(ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);     			  \
	(ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]); 			  \
	(ptr)->temperate = (data)[6];                                  			  \
	if((ptr)->ecd - (ptr)->last_ecd > 4096)																\
		(ptr)->round_cnt --;																								\
	else if ((ptr)->ecd - (ptr)->last_ecd < -4096)												\
		(ptr)->round_cnt ++;																								\
	(ptr)->last_ecd = (ptr)->ecd;																					\
	(ptr)->total_angle = (ptr)->round_cnt * 8192 + ((ptr)->ecd - (ptr)->offset_angle);	\
  (ptr)->real_ecd = (ptr)->total_angle * 360.0/19/ 8192 ;          	  			\
}

/**
  * @brief          解码函数（6020），将data解码到ptr 宏定义执行速度快于函数
  * @param[in]      ptr:数组首地址
	* @param[in]      data:数据
  * @retval         none
  */
#define get_motor_info(ptr, data)/* “ \ ” 为续行符，表示下面一行是紧接着当前行，\后不可含有任何符号，包括空格和空行，此处必须使用 */    	\
{                                                                  		  \
  (ptr)->rotor_angle 			= (uint16_t)((data)[0]<<8 | (data)[1]);    		\
	if(k)																																	\
	{ 																																		\
		(ptr)->offset_angle =(ptr)->last_ecd = (ptr)->rotor_angle;          \
	   k--;																																\
	}																																			\
	(ptr)->rotor_speed		  = (uint16_t)((data)[2]<<8 | (data)[3]);				\
	(ptr)->torque_current 	= (uint16_t)((data)[4]<<8 | (data)[5]);				\
	(ptr)->temp						 	= (data)[6];																	\
	if((ptr)->rotor_angle - (ptr)->last_ecd > 4096)												\
		(ptr)->round_cnt --;																								\
	else if ((ptr)->rotor_angle - (ptr)->last_ecd < -4096)								\
		(ptr)->round_cnt ++;																								\
	(ptr)->last_ecd = (ptr)->rotor_angle;																	\
	(ptr)->total_angle = (ptr)->round_cnt * 8192 + ((ptr)->rotor_angle - (ptr)->offset_angle);	\
  (ptr)->real_ecd = (ptr)->total_angle * 10.0/ 8192 ;      \
}

/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
  */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    static uint8_t i = 0;
    uint8_t rx_data[8];
    //找到can总线上返回信息rx_data
    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data);
		//get motor id
		i = rx_header.StdId - 0x201;
		//将rx_data解码到motor_message[i]
			get_motor_measure(&can_motor_message[i], rx_data);//can1 can2不可共用ID 
}

/**
  * @brief          CAN过滤函数
  * @param[in]      none
  * @retval         none
  */
//例：can_filter_init();  放在main函数中，while前
void can_filter_init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK; 	//掩码模式
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT; 	//32位  给can2 16位
    can_filter_st.FilterIdHigh = 0x0000;								//存储要筛选的ID	高16
    can_filter_st.FilterIdLow = 0x0000; 								//存储要筛选的ID	低16
    can_filter_st.FilterMaskIdHigh = 0x0000; 						//储存要筛选的掩码
    can_filter_st.FilterMaskIdLow = 0x0000;  						//储存要筛选的掩码
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;

    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
	
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
    can_filter_st.SlaveStartFilterBank = 14;						//过滤器开始编号
    can_filter_st.FilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan2, &can_filter_st);
    HAL_CAN_Start(&hcan2);
    HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/**
  * @brief          发送电机控制电流
  * @param[in]      motor1: (0x201)
  * @param[in]      motor2: (0x202)
  * @param[in]      motor3: (0x203)
  * @param[in]      motor4: (0x204)
  * @retval         none
  */
//例：CAN1_Send1(666,666,666,666);
void CAN1_Send1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    can1_tx1_message.StdId = 0X200; 				//ALL_ID;
    can1_tx1_message.IDE = CAN_ID_STD;		 	//标准格式 || 扩展格式
    can1_tx1_message.RTR = CAN_RTR_DATA;   //数据帧 || 遥控帧
    can1_tx1_message.DLC = 0x08;
    can1_send1_data[0] = motor1 >> 8;  		//高八位
    can1_send1_data[1] = motor1;
    can1_send1_data[2] = motor2 >> 8;
    can1_send1_data[3] = motor2;
    can1_send1_data[4] = motor3 >> 8;
    can1_send1_data[5] = motor3;
    can1_send1_data[6] = motor4 >> 8;
    can1_send1_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan1, &can1_tx1_message, can1_send1_data, &send_mail_box);
}

/**
  * @brief          发送电机控制电流
  * @param[in]      motor5: (0x205)
  * @param[in]      motor6: (0x206)
  * @param[in]      motor7: (0x207)
  * @param[in]      motor8: (0x208)
  * @retval         none
  */
//例：CAN1_Send2(666,666,666,666);
void CAN1_Send2(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
    uint32_t send_mail_box;
    can1_tx2_message.StdId = 0X1FF; 				//ALL_ID;
    can1_tx2_message.IDE = CAN_ID_STD;		 	//标准格式 || 扩展格式
    can1_tx2_message.RTR = CAN_RTR_DATA;   //数据帧 || 遥控帧
    can1_tx2_message.DLC = 0x08;
    can1_send2_data[0] = motor5 >> 8;  		//高八位
    can1_send2_data[1] = motor5;
    can1_send2_data[2] = motor6 >> 8;
    can1_send2_data[3] = motor6;
    can1_send2_data[4] = motor7 >> 8;
    can1_send2_data[5] = motor7;
    can1_send2_data[6] = motor8 >> 8;
    can1_send2_data[7] = motor8;

    HAL_CAN_AddTxMessage(&hcan1, &can1_tx2_message, can1_send2_data, &send_mail_box);
}

/**
  * @brief          发送电机控制电流
  * @param[in]      motor1: (0x201)
  * @param[in]      motor2: (0x202)
  * @param[in]      motor3: (0x203)
  * @param[in]      motor4: (0x204)
  * @retval         none
  */
//例：CAN2_Send2(666,666,666,666);
void CAN2_Send1(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
    uint32_t send_mail_box;
    can2_tx1_message.StdId = 0X200; 				//ALL_ID;
    can2_tx1_message.IDE = CAN_ID_STD;		 	//标准格式 || 扩展格式
    can2_tx1_message.RTR = CAN_RTR_DATA;   //数据帧 || 遥控帧
    can2_tx1_message.DLC = 0x08;
    can2_send1_data[0] = motor1 >> 8;  		//高八位
    can2_send1_data[1] = motor1;
    can2_send1_data[2] = motor2 >> 8;
    can2_send1_data[3] = motor2;
    can2_send1_data[4] = motor3 >> 8;
    can2_send1_data[5] = motor3;
    can2_send1_data[6] = motor4 >> 8;
    can2_send1_data[7] = motor4;

    HAL_CAN_AddTxMessage(&hcan2, &can2_tx1_message, can2_send1_data, &send_mail_box);
}
/**
  * @brief          发送电机控制电流
  * @param[in]      motor5: (0x205)
  * @param[in]      motor6: (0x206)
  * @param[in]      motor7: (0x207)
  * @param[in]      motor8: (0x208)
  * @retval         none
  */
//例：CAN2_Send2(666,666,666,666);
void CAN2_Send2(int16_t motor5, int16_t motor6, int16_t motor7, int16_t motor8)
{
    uint32_t send_mail_box;
    can2_tx2_message.StdId = 0X1FF; 				//ALL_ID;
    can2_tx2_message.IDE = CAN_ID_STD;		 	//标准格式 || 扩展格式
    can2_tx2_message.RTR = CAN_RTR_DATA;   //数据帧 || 遥控帧
    can2_tx2_message.DLC = 0x08;
    can2_send2_data[0] = motor5 >> 8;  		//高八位
    can2_send2_data[1] = motor5;
    can2_send2_data[2] = motor6 >> 8;
    can2_send2_data[3] = motor6;
    can2_send2_data[4] = motor7 >> 8;
    can2_send2_data[5] = motor7;
    can2_send2_data[6] = motor8 >> 8;
    can2_send2_data[7] = motor8;

    HAL_CAN_AddTxMessage(&hcan2, &can2_tx2_message, can2_send2_data, &send_mail_box);
}
