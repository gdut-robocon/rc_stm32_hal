//
//JIAlonglong 2023.2.17
//
#include "HareWare.h"
#include "main.h"
#include "tim.h"
#include "gpio.h"

/***********************电机初始化********************/
void MotorInit(void)
{
	/*********舵轮初始化*********/
	//驱动器初始化
	ElmoInit(&hcan1);
	ElmoInit(&hcan2);

		//配置驱动轮电机速度环 acc/dec 100,000,000
	VelLoopCfg(&hcan1, LEFT_FRONT_ID, 100000000, 100000000);
	VelLoopCfg(&hcan1, RIGHT_FRONT_ID, 100000000, 100000000);
	VelLoopCfg(&hcan1, LEFT_REAR_ID, 100000000, 100000000);
	VelLoopCfg(&hcan1, RIGHT_REAR_ID, 100000000, 100000000);
	
	//配置转向电机位置环 acc/dec 1,500,000,000 vel 430*32768 14,000,000
	PosLoopCfg(&hcan1, LEFT_FRONT_TURNING_ID, 1500000000, 1500000000,14000000);
	PosLoopCfg(&hcan1, RIGHT_FRONT_TURNING_ID, 1500000000, 1500000000,14000000);
	PosLoopCfg(&hcan1, LEFT_REAR_TURNING_ID, 1500000000, 1500000000,14000000);
	PosLoopCfg(&hcan1, RIGHT_REAR_TURNING_ID, 1500000000, 1500000000,14000000);
	
		//电机使能
	MotorOn(&hcan1,LEFT_FRONT_ID);
	MotorOn(&hcan1,RIGHT_FRONT_ID);
	MotorOn(&hcan1,LEFT_REAR_ID);
	MotorOn(&hcan1,RIGHT_REAR_ID);
	MotorOn(&hcan1,LEFT_FRONT_TURNING_ID);
	MotorOn(&hcan1,RIGHT_FRONT_TURNING_ID);
	MotorOn(&hcan1,LEFT_REAR_TURNING_ID);
	MotorOn(&hcan1,RIGHT_REAR_TURNING_ID);
}

/***********************************************航模遥控<PPM>************************************/
//航模结构体实例
Air_Contorl  Device;
//定义变量
static uint16_t PPM_buf[10]={0};
uint8_t ppm_update_flag=0;
uint32_t now_ppm_time_send=0;
uint32_t TIME_ISR_CNT=0,LAST_TIME_ISR_CNT=0;
//TIM2_IRQHandler
uint16_t Time_Sys[4]={0};
uint16_t Microsecond_Cnt=0;

uint16_t PPM_Sample_Cnt = 0;//通道
uint8_t PPM_Chn_Max = 8;//最大通道数
uint32_t PPM_Time = 0;//获取通道时间
uint16_t PPM_Okay = 0;//下一次解析状态
uint16_t PPM_Databuf[8] = {0};//所有通道的数组

#define Hour         3
#define Minute       2
#define Second       1
#define MicroSecond  0

/**
  * 函数功能: 按键外部中断回调函数
  * 输入参数: GPIO_Pin：中断引脚
  * 返 回 值: 无
  * 说    明: 无
 */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	
//	
	if(GPIO_Pin==GPIO_PIN_7)//判断是否为接收器产生的中断，例程设置为PIN8
    {
        PPM_Time = TIM2 ->CNT;//将定时数转存
        TIM2 -> CNT = 0;//计数器归零
        if (PPM_Okay == 1)//判断是否是新的一轮解析
        {
            PPM_Sample_Cnt++;//通道数+1
            PPM_Databuf[PPM_Sample_Cnt - 1] = PPM_Time;//把每一个通道的数值存入数组
            if (PPM_Sample_Cnt >= PPM_Chn_Max)//判断是否超过额定通道数
                PPM_Okay = 0;
        }
        if (PPM_Time >= 2050)//长时间无下降沿即无通道数据，进入下一轮解析
        {
            PPM_Okay = 1;
            PPM_Sample_Cnt = 0;
        }
    }
	}




void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM2)  // 检查是否是TIM2定时器中断
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
uint8_t * kRxBuffer;
uint8_t ch;
	static union {
		uint8_t data[24];
		float ActVal[6];
	} posture;

	static uint8_t count = 0;
	static uint8_t i = 0;
//action数据结构体
ACTION_GL_POS ACTION_GL_POS_DATA;
ROBOT_REAL_POS ROBOT_REAL_POS_DATA = {0, 0, 0};

float OFFSET_YAW = 0;



//串口4中断函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)

{
	
	if(huart->Instance==UART4)//如果是串口4.
	{
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
					//更新传感器数据
					//不直接跟新，进行差分运算
					Update_Action(posture.ActVal);
				}
				count = 0;
			}
			break;
			default:
				count = 0;
				break;
		}
	HAL_UART_Receive_IT(&huart4, &ch, 1);
	}
	

}



//更新action全场定位的值
//不直接跟新
void Update_Action(float value[6])
{
	float error;
//储存上一次的值
	ACTION_GL_POS_DATA.LAST_POS_X = ACTION_GL_POS_DATA.POS_X;
	ACTION_GL_POS_DATA.LAST_POS_Y = ACTION_GL_POS_DATA.POS_Y;
// 记录这次的值
	ACTION_GL_POS_DATA.ANGLE_Z = value[0];  // 有用
	ACTION_GL_POS_DATA.ANGLE_X = value[1];
	ACTION_GL_POS_DATA.ANGLE_Y = value[2];
	ACTION_GL_POS_DATA.POS_X   = value[3];  // 有用
	ACTION_GL_POS_DATA.POS_Y   = value[4];  // 有用
	ACTION_GL_POS_DATA.W_Z     = value[5];
	
	// 差分运算
	ACTION_GL_POS_DATA.DELTA_POS_X = ACTION_GL_POS_DATA.POS_X - ACTION_GL_POS_DATA.LAST_POS_X;
	ACTION_GL_POS_DATA.DELTA_POS_Y = ACTION_GL_POS_DATA.POS_Y - ACTION_GL_POS_DATA.LAST_POS_Y;
	
	ACTION_GL_POS_DATA.REAL_X += (ACTION_GL_POS_DATA.DELTA_POS_X);
	ACTION_GL_POS_DATA.REAL_Y += (ACTION_GL_POS_DATA.DELTA_POS_Y);
	

	// 偏航角直接赋值（逆时针为正，顺时针为负）
  ROBOT_REAL_POS_DATA.POS_YAW = ACTION_GL_POS_DATA.ANGLE_Z - OFFSET_YAW;
	
	//消除机械误差,赋值X、Y
	ROBOT_REAL_POS_DATA.POS_X = ACTION_GL_POS_DATA.REAL_X + INSTALL_ERROR_Y * sin(ROBOT_REAL_POS_DATA.POS_YAW * PI / 180.0f);
	ROBOT_REAL_POS_DATA.POS_Y = ACTION_GL_POS_DATA.REAL_Y - INSTALL_ERROR_Y * (cos(ROBOT_REAL_POS_DATA.POS_YAW * PI / 180.0f)-1);
	 
}





void UART_SendString(USART_TypeDef* USARTx, char *DataString)
{
	int i = 0;
	while(DataString[i] != '\0')												//字符串结束符
	{
		HAL_UART_Transmit(&huart4, (uint8_t*)&DataString[i], 1, HAL_MAX_DELAY);//每次发送字符串的一个字符
		while(__HAL_UART_GET_FLAG(&huart4, UART_FLAG_TC) == RESET);					//等待数据发送成功
		i++;
	}
}


