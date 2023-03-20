//
//LLY 2023.3.9
//
#include "moto.h"
#include "stm32f4xx_hal.h"
#include "can.h"
//���
/*******************************��������������************************************/
/**
* @brief  �����������ʼ��
* @param  CANx����ʹ�õ�CANͨ�����
* @author ACTION
*/

M3508_REAL_INFO M3508_CHASSIS_MOTOR_REAL_INFO[6] = {0}; // 1-4�ֱ��Ӧ˳ʱ�뷽��ĵ��̵��
//M3508_REAL_INFO M3508_CAST_MOTOR_REAL_INFO 			 = {0};	// ����������
M3508_REAL_INFO M2006_CHASSIS_MOTOR_REAL_INFO[2] = {0}; //2006���


PID M3508_CHASSIS_MOTOR_PID_RPM[6];	// 1-4���̵�� 5-6������
PID M3508_CHASSIS_MOTOR_PID_POS[2];			  //λ����Ϣ
PID M2006_CHASSIS_MOTOR_PID_RPM[2];	// 2��M2006���


  
//M3580��ʼ��
void M3508_Motor_Init(void)
{
	// ���̵��PID��ʼ��
	PID_parameter_init(&M3508_CHASSIS_MOTOR_PID_RPM[0], 10.0, 1.0, 0.0, 16384, 16384, -1);
	PID_parameter_init(&M3508_CHASSIS_MOTOR_PID_RPM[1], 10.0, 1.0, 0.0, 16384, 16384, -1);
	PID_parameter_init(&M3508_CHASSIS_MOTOR_PID_RPM[2], 10.0, 1.0, 0.0, 16384, 16384, -1);
	// ������PID��ʼ��
	PID_parameter_init(&M3508_CHASSIS_MOTOR_PID_RPM[3], 10.0, 1.0, 0.0, 16384, 16384, -1);
	PID_parameter_init(&M3508_CHASSIS_MOTOR_PID_RPM[4], 10.0, 1.0, 0.0, 16384, 16384, -1);
	
	// ��צ���PID��ʼ��
	PID_parameter_init(&M2006_CHASSIS_MOTOR_PID_RPM[0], 50, 0, 0.1, 7000, 7000, 10);
	// ̧���������PID��ʼ�� 
	PID_parameter_init(&M3508_CHASSIS_MOTOR_PID_RPM[5] , 10.0, 1.0, 0.0, 16384, 16384, -1);
	PID_parameter_init(&M3508_CHASSIS_MOTOR_PID_POS[1] , 1000, 0, 1, 7000, 7000, 0.05);
}


// ���õ��ͨ��CAN���������ݸ���m3508��״̬��Ϣ
// ����Ƶ�ʣ�1kHz
//2023/3/15LLy���Ը���
void m3508_update_m3508_info(CAN_RxHeaderTypeDef *msg,uint8_t	can1_RxData[8])
{
	switch(msg -> StdId)  // ����׼ID
	{
    case M3508_CHASSIS_MOTOR_ID_1:
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[0].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];  // ת�ӻ�е�Ƕ�
			M3508_CHASSIS_MOTOR_REAL_INFO[0].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // ʵ��ת��ת��
			M3508_CHASSIS_MOTOR_REAL_INFO[0].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // ʵ��ת�ص���
		}; break;
		
		case M3508_CHASSIS_MOTOR_ID_2:
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[1].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];  // ת�ӻ�е�Ƕ�
			M3508_CHASSIS_MOTOR_REAL_INFO[1].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // ʵ��ת��ת��
			M3508_CHASSIS_MOTOR_REAL_INFO[1].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // ʵ��ת�ص���
		}; break;
		
		case M3508_CHASSIS_MOTOR_ID_3:
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[2].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];  // ת�ӻ�е�Ƕ�
			M3508_CHASSIS_MOTOR_REAL_INFO[2].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // ʵ��ת��ת��
			M3508_CHASSIS_MOTOR_REAL_INFO[2].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // ʵ��ת�ص���
		}; break;	
		
		case M3508_CHASSIS_MOTOR_ID_4:
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[3].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];  // ת�ӻ�е�Ƕ�
			M3508_CHASSIS_MOTOR_REAL_INFO[3].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // ʵ��ת��ת��
			M3508_CHASSIS_MOTOR_REAL_INFO[3].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // ʵ��ת�ص���
		}; break;
		
//		case M3508_CAST_MOTOR_ID:
//		{ 
//			M3508_CAST_MOTOR_REAL_INFO.ANGLE   = (msg -> Data[0] << 8) | msg -> Data[1];  // ת�ӻ�е�Ƕ�
//			M3508_CAST_MOTOR_REAL_INFO.RPM     = (msg -> Data[2] << 8) | msg -> Data[3];  // ʵ��ת��ת��
//			M3508_CAST_MOTOR_REAL_INFO.CURRENT = (msg -> Data[4] << 8) | msg -> Data[5];  // ʵ��ת�ص���
//		}; break;
		case M3508_CHASSIS_MOTOR_ID_5:
		{ 
			M3508_CHASSIS_MOTOR_REAL_INFO[4].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];  // ת�ӻ�е�Ƕ�
			M3508_CHASSIS_MOTOR_REAL_INFO[4].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // ʵ��ת��ת��
			M3508_CHASSIS_MOTOR_REAL_INFO[4].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // ʵ��ת�ص���
		}; break;
		case M3508_CHASSIS_MOTOR_ID_6:
		{ 
		M3508_CHASSIS_MOTOR_REAL_INFO[5].ANGLE   = (can1_RxData[0] << 8) | can1_RxData[1];  // ת�ӻ�е�Ƕ�
			M3508_CHASSIS_MOTOR_REAL_INFO[5].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // ʵ��ת��ת��
			M3508_CHASSIS_MOTOR_REAL_INFO[5].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // ʵ��ת�ص���
		}; break;
		
		case M2006_CHASSIS_MOTOR_ID_0://2006������ұߣ�
		{ 
			M2006_CHASSIS_MOTOR_REAL_INFO[0].ANGLE= (can1_RxData[0] << 8) | can1_RxData[1];  // ת�ӻ�е�Ƕ�
			M2006_CHASSIS_MOTOR_REAL_INFO[0].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // ʵ��ת��ת��
			M2006_CHASSIS_MOTOR_REAL_INFO[0].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // ʵ��ת�ص���
//			M2006AngleIntegral_special_left_turn(&M2006_CHASSIS_MOTOR_REAL_INFO[0]);			//??????????????Ϊɶ�����õ�����ͨ��ʼ������������
		}; break;

		case M2006_CHASSIS_MOTOR_ID_1://2006�������ߣ�
		{ 
			M2006_CHASSIS_MOTOR_REAL_INFO[1].ANGLE= (can1_RxData[0] << 8) | can1_RxData[1];  // ת�ӻ�е�Ƕ�
			M2006_CHASSIS_MOTOR_REAL_INFO[1].RPM     = (can1_RxData[2] << 8) | can1_RxData[3];  // ʵ��ת��ת��
			M2006_CHASSIS_MOTOR_REAL_INFO[1].CURRENT = (can1_RxData[4] << 8) | can1_RxData[5];  // ʵ��ת�ص���
//			M2006AngleIntegral_special_right_turn(&M2006_CHASSIS_MOTOR_REAL_INFO[1]);			
		}; break;

		default: break;
	}
}



//���͵���
void chassis_m3508_send_motor_currents(void)
{
	
	/***********************************����IDΪ 1 2 3 4 �ĵ��*********************************/
	CAN_TxHeaderTypeDef tx_message_1;
  uint8_t send_buf1[8] = {0};
	uint32_t msg_box1;
	// ���ÿ��ƶ�
	tx_message_1.IDE = CAN_ID_STD;//���ĵ�11λ��׼��ʶ��CAN_ID_STD��ʾ�������Ǳ�׼֡
	tx_message_1.RTR = CAN_RTR_DATA;//�������ͱ�־RTRλCAN_ID_STD��ʾ�����ĵ�����֡
	tx_message_1.DLC = 0x08;//���ݶγ���
	tx_message_1.TransmitGlobalTime = DISABLE;
	// �����ٲöκ����ݶ�	
	tx_message_1.StdId = 0x200;  // ����IDΪ 1 2 3 4 �ĵ��
	
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
	
		
	/***********************************����IDΪ 5 6 7 8 �ĵ��*********************************/
	CAN_TxHeaderTypeDef tx_message_2;
	uint8_t send_buf2[8] = {0};
	uint32_t msg_box2;
	
	// ���ÿ��ƶ�
	tx_message_2.IDE = CAN_ID_STD;//���ĵ�11λ��׼��ʶ��CAN_ID_STD��ʾ�������Ǳ�׼֡
	tx_message_2.RTR = CAN_RTR_DATA;//�������ͱ�־RTRλCAN_ID_STD��ʾ�����ĵ�����֡
	tx_message_2.DLC = 0x08;//���ݶγ���
	tx_message_2.TransmitGlobalTime = DISABLE;
	
	// �����ٲöκ����ݶ�
	tx_message_2.StdId = 0x1FF;  // ����IDΪ 5 6 7 8 �ĵ��(������3508����2006)
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

////���Ƶ���
//void shoot_m3508_send_motor_currents(void)
//{
//	CanTxMsg tx_message_2;
//	
//	// ���ÿ��ƶ�
//	tx_message_2.IDE = CAN_Id_Standard;//��׼ID
//	tx_message_2.RTR = CAN_RTR_Data; //����֡
//	tx_message_2.DLC = 0x08;
//	
//	// �����ٲöκ����ݶ�
//	tx_message_2.StdId = 0x1FF;  // ����IDΪ 5 6 7 8 �ĵ��
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
	
	// ���ÿ��ƶ�
	tx_message_2.IDE = CAN_ID_STD;//���ĵ�11λ��׼��ʶ��CAN_ID_STD��ʾ�������Ǳ�׼֡
	tx_message_2.RTR = CAN_RTR_DATA;//�������ͱ�־RTRλCAN_ID_STD��ʾ�����ĵ�����֡
	tx_message_2.DLC = 0x08;//���ݶγ���
	tx_message_2.TransmitGlobalTime = DISABLE;
	
	// �����ٲöκ����ݶ�
	tx_message_2.StdId = 0x1FF;  // ����IDΪ 5 6 7 8 �ĵ��(������3508����2006)
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



//M3508����ǶȻ���
// M3508����ǶȻ���
void M3508AngleIntegral(M3508_REAL_INFO *M3508_MOTOR)
{
	float delta_pos = 0;
	
	// ��¼��һ�ν���ʱ������
	if(!M3508_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG)
	{
		M3508_MOTOR->LAST_ANGLE = M3508_MOTOR->ANGLE;
		M3508_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG = 1;
		return;
	}
	
	// ����仯�ĽǶ�
	if(M3508_MOTOR->RPM >= 0)
	{
		if(M3508_MOTOR->ANGLE < M3508_MOTOR->LAST_ANGLE)
		{
			if(ABS(8191 + M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) < 1250)  // ��������CAN����ʱ�������ת���ǶȽ����˲�
			{
				delta_pos = ((float)(8191 + M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
				delta_pos = delta_pos / 3591.0f * 187.0f;	//���ٱ�
			}
		}
		else
		{
			delta_pos = ((float)(M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
			delta_pos = delta_pos / 3591.0f * 187.0f;	//���ٱ�
		}
		
		// �˲�
		if(delta_pos > 0) 
			M3508_MOTOR->REAL_ANGLE += delta_pos;  // ����	
	}
	else
	{
		if(M3508_MOTOR->ANGLE > M3508_MOTOR->LAST_ANGLE)
		{
			if(ABS(8191 - M3508_MOTOR->ANGLE + M3508_MOTOR->LAST_ANGLE) < 1250)  // ��������CAN����ʱ�������ת���ǶȽ����˲�			
			{
				delta_pos = ((float)(8191 - M3508_MOTOR->ANGLE + M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
				delta_pos = delta_pos / 3591.0f * 187.0f;	//���ٱ�
			}
		}	
		else
		{
			delta_pos = ((float)(M3508_MOTOR->ANGLE - M3508_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
			delta_pos = delta_pos / 3591.0f * 187.0f;	//���ٱ�
		}
		
		// �˲�
		if(delta_pos < 0) 
			M3508_MOTOR->REAL_ANGLE += delta_pos;  // ����
	}

	// �洢�Ƕ�ֵ 
	M3508_MOTOR->LAST_ANGLE = M3508_MOTOR->ANGLE;
}