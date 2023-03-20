//
//JIAlonglong 2023.2.17
//
#include "HareWare.h"
#include "main.h"
#include "tim.h"
#include "gpio.h"
//�����ʼ��
void MotorInit(void)
{
	/*********���ֳ�ʼ��*********/
	//��������ʼ��
	ElmoInit(&hcan1);
	ElmoInit(&hcan2);

	//���������ֵ���ٶȻ� acc/dec 100,000,000
	VelLoopCfg(&hcan1, LEFT_FRONT_ID, 100000000, 100000000);
	VelLoopCfg(&hcan1, RIGHT_FRONT_ID, 100000000, 100000000);
	VelLoopCfg(&hcan1, LEFT_REAR_ID, 100000000, 100000000);
	VelLoopCfg(&hcan1, RIGHT_REAR_ID, 100000000, 100000000);
	
	//����ת����λ�û� acc/dec 1,500,000,000 vel 430*32768 14,000,000
	PosLoopCfg(&hcan1, LEFT_FRONT_TURNING_ID, 1500000000, 1500000000,14000000);
	PosLoopCfg(&hcan1, RIGHT_FRONT_TURNING_ID, 1500000000, 1500000000,14000000);
	PosLoopCfg(&hcan1, LEFT_REAR_TURNING_ID, 1500000000, 1500000000,14000000);
	PosLoopCfg(&hcan1, RIGHT_REAR_TURNING_ID, 1500000000, 1500000000,14000000);
	
		//���ʹ��
	MotorOn(&hcan1,LEFT_FRONT_ID);
	MotorOn(&hcan1,RIGHT_FRONT_ID);
	MotorOn(&hcan1,LEFT_REAR_ID);
	MotorOn(&hcan1,RIGHT_REAR_ID);
	MotorOn(&hcan1,LEFT_FRONT_TURNING_ID);
	MotorOn(&hcan1,RIGHT_FRONT_TURNING_ID);
	MotorOn(&hcan1,LEFT_REAR_TURNING_ID);
	MotorOn(&hcan1,RIGHT_REAR_TURNING_ID);
}

/***********************************************��ģң��<PPM>************************************/
//��ģ�ṹ��ʵ��
Air_Contorl  Device;
//�������
static uint16_t PPM_buf[10]={0};
//uint16_t PPM_Databuf[10]={0};
uint8_t ppm_update_flag=0;
uint32_t now_ppm_time_send=0;
uint32_t TIME_ISR_CNT=0,LAST_TIME_ISR_CNT=0;
//TIM2_IRQHandler
uint16_t Time_Sys[4]={0};
uint16_t Microsecond_Cnt=0;

uint16_t PPM_Sample_Cnt = 0;//ͨ��
uint8_t PPM_Chn_Max = 8;//���ͨ����
uint32_t PPM_Time = 0;//��ȡͨ��ʱ��
uint16_t PPM_Okay = 0;//��һ�ν���״̬
uint16_t PPM_Databuf[8] = {0};//����ͨ��������

#define Hour         3
#define Minute       2
#define Second       1
#define MicroSecond  0

/**
  * ��������: �����ⲿ�жϻص�����
  * �������: GPIO_Pin���ж�����
  * �� �� ֵ: ��
  * ˵    ��: ��
 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
//	
	if(GPIO_Pin==GPIO_PIN_7)//�ж��Ƿ�Ϊ�������������жϣ���������ΪPIN8
    {
        PPM_Time = TIM2 ->CNT;//����ʱ��ת��
        TIM2 -> CNT = 0;//����������
        if (PPM_Okay == 1)//�ж��Ƿ����µ�һ�ֽ���
        {
            PPM_Sample_Cnt++;//ͨ����+1
            PPM_Databuf[PPM_Sample_Cnt - 1] = PPM_Time;//��ÿһ��ͨ������ֵ��������
            if (PPM_Sample_Cnt >= PPM_Chn_Max)//�ж��Ƿ񳬹��ͨ����
                PPM_Okay = 0;
        }
        if (PPM_Time >= 2050)//��ʱ�����½��ؼ���ͨ�����ݣ�������һ�ֽ���
        {
            PPM_Okay = 1;
            PPM_Sample_Cnt = 0;
        }
    }
	}




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)  // ����Ƿ���TIM2��ʱ���ж�
  {
    LAST_TIME_ISR_CNT = TIME_ISR_CNT;
    TIME_ISR_CNT++;
    Microsecond_Cnt++;
    if (Microsecond_Cnt >= 100)
    {
      Microsecond_Cnt = 0;
      Time_Sys[Second]++;
      if (Time_Sys[Second] >= 60)
      {
        Time_Sys[Second] = 0;
        Time_Sys[Minute]++;
        if (Time_Sys[Minute] >= 60)
        {
          Time_Sys[Minute] = 0;
          Time_Sys[Hour]++;
        }
      }
    }
    Time_Sys[MicroSecond] = Microsecond_Cnt;
  }
}



/***********************************************action************************************/
//action���ݽṹ��
ACTION_GL_POS ACTION_GL_POS_DATA;
ROBOT_REAL_POS ROBOT_REAL_POS_DATA = {0, 0, 0};
unsigned char aRxBuffer[1];//HAL��USART����Buffer
float OFFSET_YAW = 0;

//����4�жϺ���
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)

{
	if(huart->Instance==UART4)//����Ǵ���4
	{
	/***************************************/
	static uint8_t ch;
	static union {
		uint8_t data[24];
		float ActVal[6];
	} posture;

	static uint8_t count = 0;
	static uint8_t i = 0;
if(__HAL_UART_GET_FLAG(&huart4, UART_FLAG_RXNE) != RESET)

	{
		ch = (uint8_t)(huart4.Instance->DR & 0xFF);


		switch (count)
		{
			case 0:
			{
				if (ch == 0x0d)
					count++;
				else
					count = 0;
			}
			break;
			case 1:
			{
				if (ch == 0x0a)
				{
					i = 0;
					count++;
				}
				else if (ch == 0x0d);
				else
					count = 0;
			}
			break;
			case 2:
			{
				posture.data[i] = ch;
				i++;
				if (i >= 24)
				{
					i = 0;
					count++;
				}
			}
			break;
			case 3:
			{
				if (ch == 0x0a)
					count++;
				else
					count = 0;
			}
			break;
			case 4:
			{
				if (ch == 0x0d)
				{
					//���´���������
					//��ֱ�Ӹ��£����в������
					
				
					
	Update_Action(posture.ActVal);
				}
				count = 0;
			}
			break;
			default:
				count = 0;
				break;
		}
	}
	}
	
	HAL_UART_Receive_IT(&huart4, (uint8_t *)&aRxBuffer, 1);   //�ٿ��������ж�
}



//����actionȫ����λ��ֵ
//��ֱ�Ӹ���
void Update_Action(float value[6])
{
	float error;
//������һ�ε�ֵ
	ACTION_GL_POS_DATA.LAST_POS_X = ACTION_GL_POS_DATA.POS_X;
	ACTION_GL_POS_DATA.LAST_POS_Y = ACTION_GL_POS_DATA.POS_Y;
// ��¼��ε�ֵ
	ACTION_GL_POS_DATA.ANGLE_Z = value[0];  // ����
	ACTION_GL_POS_DATA.ANGLE_X = value[1];
	ACTION_GL_POS_DATA.ANGLE_Y = value[2];
	ACTION_GL_POS_DATA.POS_X   = value[3];  // ����
	ACTION_GL_POS_DATA.POS_Y   = value[4];  // ����
	ACTION_GL_POS_DATA.W_Z     = value[5];
	
	// �������
	ACTION_GL_POS_DATA.DELTA_POS_X = ACTION_GL_POS_DATA.POS_X - ACTION_GL_POS_DATA.LAST_POS_X;
	ACTION_GL_POS_DATA.DELTA_POS_Y = ACTION_GL_POS_DATA.POS_Y - ACTION_GL_POS_DATA.LAST_POS_Y;
	
	ACTION_GL_POS_DATA.REAL_X += (ACTION_GL_POS_DATA.DELTA_POS_X);
	ACTION_GL_POS_DATA.REAL_Y += (ACTION_GL_POS_DATA.DELTA_POS_Y);
	

	// ƫ����ֱ�Ӹ�ֵ����ʱ��Ϊ����˳ʱ��Ϊ����
  ROBOT_REAL_POS_DATA.POS_YAW = ACTION_GL_POS_DATA.ANGLE_Z - OFFSET_YAW;
	
	//������е���,��ֵX��Y
	ROBOT_REAL_POS_DATA.POS_X = ACTION_GL_POS_DATA.REAL_X + INSTALL_ERROR_Y * sin(ROBOT_REAL_POS_DATA.POS_YAW * PI / 180.0f);
	ROBOT_REAL_POS_DATA.POS_Y = ACTION_GL_POS_DATA.REAL_Y - INSTALL_ERROR_Y * (cos(ROBOT_REAL_POS_DATA.POS_YAW * PI / 180.0f)-1);
	 
}





void UART_SendString(USART_TypeDef* USARTx, char *DataString)
{
	int i = 0;
	while(DataString[i] != '\0')												//�ַ���������
	{
		HAL_UART_Transmit(&huart4, (uint8_t*)&DataString[i], 1, HAL_MAX_DELAY);//ÿ�η����ַ�����һ���ַ�
		while(__HAL_UART_GET_FLAG(&huart4, UART_FLAG_TC) == RESET);					//�ȴ����ݷ��ͳɹ�
		i++;
	}
}


