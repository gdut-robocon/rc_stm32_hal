//
//LLY 2023.3.9
//
#include "moto.h"
#include "stm32f4xx_hal.h"
#include "can.h"
//电机
/*******************************控制驱动器命令************************************/
/**
* @brief  电机驱动器初始化
* @param  CANx：所使用的CAN通道编号
* @author ACTION
*/

M3508_REAL_INFO M3508_CHASSIS_MOTOR_REAL_INFO[6] = {0}; // 1-4分别对应顺时针方向的底盘电机
//M3508_REAL_INFO M3508_CAST_MOTOR_REAL_INFO 			 = {0};	// 射箭机构电机
M3508_REAL_INFO M2006_CHASSIS_MOTOR_REAL_INFO[2] = {0}; //2006电机


PID M3508_CHASSIS_MOTOR_PID_RPM[6];	// 1-4底盘电机 5-6发射电机
PID M3508_CHASSIS_MOTOR_PID_POS[2];			  //位置信息
PID M2006_CHASSIS_MOTOR_PID_RPM[2];	// 2个M2006电机


  
//M3580初始化
void M3508_Motor_Init(void)
{
	// 底盘电机PID初始化
	PID_parameter_init(&M3508_CHASSIS_MOTOR_PID_RPM[0], 10.0, 1.0, 0.0, 16384, 16384, -1);
	PID_parameter_init(&M3508_CHASSIS_MOTOR_PID_RPM[1], 10.0, 1.0, 0.0, 16384, 16384, -1);
	PID_parameter_init(&M3508_CHASSIS_MOTOR_PID_RPM[2], 10.0, 1.0, 0.0, 16384, 16384, -1);
	// 发射电机PID初始化
	PID_parameter_init(&M3508_CHASSIS_MOTOR_PID_RPM[3], 10.0, 1.0, 0.0, 16384, 16384, -1);
	PID_parameter_init(&M3508_CHASSIS_MOTOR_PID_RPM[4], 10.0, 1.0, 0.0, 16384, 16384, -1);
	
	// 夹爪电机PID初始化
	PID_parameter_init(&M2006_CHASSIS_MOTOR_PID_RPM[0], 50, 0, 0.1, 7000, 7000, 10);
	// 抬升机构电机PID初始化 
	PID_parameter_init(&M3508_CHASSIS_MOTOR_PID_RPM[5] , 10.0, 1.0, 0.0, 16384, 16384, -1);
	PID_parameter_init(&M3508_CHASSIS_MOTOR_PID_POS[1] , 1000, 0, 1, 7000, 7000, 0.05);
}


// 利用电机通过CAN反馈的数据更新m3508的状态信息
// 接受频率：1kHz
//2023/3/15LLy尝试更改
void m3508_update_m3508_info(CAN_RxHeaderTypeDef *msg,uint8_t	can1_RxData[8])
{
	switch(msg -> StdId)  // 检测标准ID
	{
    case M3508_CHASSIS_MOTOR_ID_1:
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[0].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];  // 转子机械角度
			M3508_CHASSIS_MOTOR_REAL_INFO[0].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // 实际转子转速
			M3508_CHASSIS_MOTOR_REAL_INFO[0].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // 实际转矩电流
		}; break;
		
		case M3508_CHASSIS_MOTOR_ID_2:
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[1].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];  // 转子机械角度
			M3508_CHASSIS_MOTOR_REAL_INFO[1].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // 实际转子转速
			M3508_CHASSIS_MOTOR_REAL_INFO[1].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // 实际转矩电流
		}; break;
		
		case M3508_CHASSIS_MOTOR_ID_3:
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[2].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];  // 转子机械角度
			M3508_CHASSIS_MOTOR_REAL_INFO[2].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // 实际转子转速
			M3508_CHASSIS_MOTOR_REAL_INFO[2].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // 实际转矩电流
		}; break;	
		
		case M3508_CHASSIS_MOTOR_ID_4:
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[3].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];  // 转子机械角度
			M3508_CHASSIS_MOTOR_REAL_INFO[3].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // 实际转子转速
			M3508_CHASSIS_MOTOR_REAL_INFO[3].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // 实际转矩电流
		}; break;
		
//		case M3508_CAST_MOTOR_ID:
//		{ 
//			M3508_CAST_MOTOR_REAL_INFO.ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // 转子机械角度
//			M3508_CAST_MOTOR_REAL_INFO.RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // 实际转子转速
//			M3508_CAST_MOTOR_REAL_INFO.CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // 实际转矩电流
//		}; break;
		case M3508_CHASSIS_MOTOR_ID_5:
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[4].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];  // 转子机械角度
			M3508_CHASSIS_MOTOR_REAL_INFO[4].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // 实际转子转速
			M3508_CHASSIS_MOTOR_REAL_INFO[4].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // 实际转矩电流
		}; break;
		case M3508_CHASSIS_MOTOR_ID_6:
		{ 
		M3508_CHASSIS_MOTOR_REAL_INFO[5].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];  // 转子机械角度
			M3508_CHASSIS_MOTOR_REAL_INFO[5].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // 实际转子转速
			M3508_CHASSIS_MOTOR_REAL_INFO[5].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // 实际转矩电流
		}; break;
		
		case M2006_CHASSIS_MOTOR_ID_0://2006电机（右边）
		{ 
			M2006_CHASSIS_MOTOR_REAL_INFO[0].ANGLE= (can1_RxData[0] << 8) | can1_RxData[1];  // 转子机械角度
			M2006_CHASSIS_MOTOR_REAL_INFO[0].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // 实际转子转速
			M2006_CHASSIS_MOTOR_REAL_INFO[0].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // 实际转矩电流
//			M2006AngleIntegral_special_left_turn(&M2006_CHASSIS_MOTOR_REAL_INFO[0]);			//??????????????为啥这里用的是普通初始化？？？？？
		}; break;

		case M2006_CHASSIS_MOTOR_ID_1://2006电机（左边）
		{ 
			M2006_CHASSIS_MOTOR_REAL_INFO[1].ANGLE= (can1_RxData[0] << 8) | can1_RxData[1];  // 转子机械角度
			M2006_CHASSIS_MOTOR_REAL_INFO[1].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // 实际转子转速
			M2006_CHASSIS_MOTOR_REAL_INFO[1].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // 实际转矩电流
//			M2006AngleIntegral_special_right_turn(&M2006_CHASSIS_MOTOR_REAL_INFO[1]);			
		}; break;

		default: break;
	}
}



//发送电流
void chassis_m3508_send_motor_currents(void)
{
	
	/***********************************用于ID为 1 2 3 4 的电机*********************************/
	CAN_TxHeaderTypeDef tx_message_1;
  uint8_t send_buf1[8] = {0};
	uint32_t msg_box1;
	// 配置控制段
	tx_message_1.IDE = CAN_ID_STD;//报文的11位标准标识符CAN_ID_STD表示本报文是标准帧
	tx_message_1.RTR = CAN_RTR_DATA;//报文类型标志RTR位CAN_ID_STD表示本报文的数据帧
	tx_message_1.DLC = 0x08;//数据段长度
	tx_message_1.TransmitGlobalTime = DISABLE;
	// 配置仲裁段和数据段	
	tx_message_1.StdId = 0x200;  // 用于ID为 1 2 3 4 的电机
	
	send_buf1[0] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[0].TARGET_CURRENT >> 8);
	send_buf1[1] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[0].TARGET_CURRENT;
	send_buf1[2] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[1].TARGET_CURRENT >> 8);
	send_buf1[3] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[1].TARGET_CURRENT;
	send_buf1[4] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[2].TARGET_CURRENT >> 8);
	send_buf1[5] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[2].TARGET_CURRENT;
	send_buf1[6] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[3].TARGET_CURRENT >> 8);
	send_buf1[7] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[3].TARGET_CURRENT;

	                                                                                                                                                                                          
	    if (HAL_CAN_AddTxMessage(&hcan1,&tx_message_1,send_buf1,&msg_box1)!= HAL_OK) {
        // Failed to add message to the transmit mailbox
    }
	
		
	/***********************************用于ID为 5 6 7 8 的电机*********************************/
	CAN_TxHeaderTypeDef tx_message_2;
	uint8_t send_buf2[8] = {0};
	uint32_t msg_box2;
	
	// 配置控制段
	tx_message_2.IDE = CAN_ID_STD;//报文的11位标准标识符CAN_ID_STD表示本报文是标准帧
	tx_message_2.RTR = CAN_RTR_DATA;//报文类型标志RTR位CAN_ID_STD表示本报文的数据帧
	tx_message_2.DLC = 0x08;//数据段长度
	tx_message_2.TransmitGlobalTime = DISABLE;
	
	// 配置仲裁段和数据段
	tx_message_2.StdId = 0x1FF;  // 用于ID为 5 6 7 8 的电机(不论是3508还是2006)
	send_buf2[0] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[4].TARGET_CURRENT >> 8);
	send_buf2[1] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[4].TARGET_CURRENT;	      //ID 5 
	send_buf2[2] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[5].TARGET_CURRENT >> 8);
	send_buf2[3] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[5].TARGET_CURRENT;	      //ID 6 cast
	send_buf2[4] = (uint8_t)( M2006_CHASSIS_MOTOR_REAL_INFO[0].TARGET_CURRENT >> 8 ) ;
  send_buf2[5] = (uint8_t) M2006_CHASSIS_MOTOR_REAL_INFO[0].TARGET_CURRENT; //ID 7
  send_buf2[6] = (uint8_t)( M2006_CHASSIS_MOTOR_REAL_INFO[1].TARGET_CURRENT >> 8 ) ;
  send_buf2[7] = (uint8_t) M2006_CHASSIS_MOTOR_REAL_INFO[1].TARGET_CURRENT; //ID 8
	
	
	if (HAL_CAN_AddTxMessage(&hcan1,&tx_message_2,send_buf2,&msg_box2)!= HAL_OK) {
        // Failed to add message to the transmit mailbox
    }
}

////控制电流
//void shoot_m3508_send_motor_currents(void)
//{
//	CanTxMsg tx_message_2;
//	
//	// 配置控制段
//	tx_message_2.IDE = CAN_Id_Standard;//标准ID
//	tx_message_2.RTR = CAN_RTR_Data; //数据帧
//	tx_message_2.DLC = 0x08;
//	
//	// 配置仲裁段和数据段
//	tx_message_2.StdId = 0x1FF;  // 用于ID为 5 6 7 8 的电机
//	tx_message_2.Data[0] = (uint8_t)(M3508_CAST_MOTOR_REAL_INFO.TARGET_CURRENT >> 8);
//	tx_message_2.Data[1] = (uint8_t) M3508_CAST_MOTOR_REAL_INFO.TARGET_CURRENT;	
//	
//	CAN_Transmit(CAN1, &tx_message_2);
//}


void chassis_m3508_m2006_send_motor_currents_can1(void)
{
  CAN_TxHeaderTypeDef tx_message_2;
	uint8_t send_buf2[8] = {0};
	uint32_t *msg_box;
	
	// 配置控制段
	tx_message_2.IDE = CAN_ID_STD;//报文的11位标准标识符CAN_ID_STD表示本报文是标准帧
	tx_message_2.RTR = CAN_RTR_DATA;//报文类型标志RTR位CAN_ID_STD表示本报文的数据帧
	tx_message_2.DLC = 0x08;//数据段长度
	tx_message_2.TransmitGlobalTime = DISABLE;
	
	// 配置仲裁段和数据段
	tx_message_2.StdId = 0x1FF;  // 用于ID为 5 6 7 8 的电机(不论是3508还是2006)
	send_buf2[0] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[4].TARGET_CURRENT >> 8);
	send_buf2[1] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[4].TARGET_CURRENT;	      //ID 5 
	send_buf2[2] = (uint8_t)(M3508_CHASSIS_MOTOR_REAL_INFO[5].TARGET_CURRENT >> 8);
	send_buf2[3] = (uint8_t) M3508_CHASSIS_MOTOR_REAL_INFO[5].TARGET_CURRENT;	      //ID 6 cast
	send_buf2[4] = (uint8_t)( M2006_CHASSIS_MOTOR_REAL_INFO[0].TARGET_CURRENT >> 8 ) ;
  send_buf2[5] = (uint8_t) M2006_CHASSIS_MOTOR_REAL_INFO[0].TARGET_CURRENT; //ID 7
  send_buf2[6] = (uint8_t)( M2006_CHASSIS_MOTOR_REAL_INFO[1].TARGET_CURRENT >> 8 ) ;
  send_buf2[7] = (uint8_t) M2006_CHASSIS_MOTOR_REAL_INFO[1].TARGET_CURRENT; //ID 8
	
	
	if (HAL_CAN_AddTxMessage(&hcan1,&tx_message_2,send_buf2,msg_box)!= HAL_OK) {
        // Failed to add message to the transmit mailbox
    }
}



//M3508电机角度积分
// M3508电机角度积分
void M3508AngleIntegral(M3508_REAL_INFO *M3508_MOTOR)
{
	float delta_pos = 0;
	
	// 记录第一次进入时的数据
	if(!M3508_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG)
	{
		M3508_MOTOR->LAST_ANGLE = M3508_MOTOR->ANGLE;
		M3508_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG = 1;
		return;
	}
	
	// 计算变化的角度
	if(M3508_MOTOR->RPM >= 0)
	{
		if(M3508_MOTOR->ANGLE < M3508_MOTOR->LAST_ANGLE)
		{
			if(ABS(8191 + M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) < 1250)  // 利用两次CAN接收时间电机最大转动角度进行滤波
			{
				delta_pos = ((float)(8191 + M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
				delta_pos = delta_pos / 3591.0f * 187.0f;	//减速比
			}
		}
		else
		{
			delta_pos = ((float)(M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
			delta_pos = delta_pos / 3591.0f * 187.0f;	//减速比
		}
		
		// 滤波
		if(delta_pos > 0) 
			M3508_MOTOR->REAL_ANGLE += delta_pos;  // 积分	
	}
	else
	{
		if(M3508_MOTOR->ANGLE > M3508_MOTOR->LAST_ANGLE)
		{
			if(ABS(8191 - M3508_MOTOR->ANGLE + M3508_MOTOR->LAST_ANGLE) < 1250)  // 利用两次CAN接收时间电机最大转动角度进行滤波			
			{
				delta_pos = ((float)(8191 - M3508_MOTOR->ANGLE + M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
				delta_pos = delta_pos / 3591.0f * 187.0f;	//减速比
			}
		}	
		else
		{
			delta_pos = ((float)(M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
			delta_pos = delta_pos / 3591.0f * 187.0f;	//减速比
		}
		
		// 滤波
		if(delta_pos < 0) 
			M3508_MOTOR->REAL_ANGLE += delta_pos;  // 积分
	}

	// 存储角度值 
	M3508_MOTOR->LAST_ANGLE = M3508_MOTOR->ANGLE;
}