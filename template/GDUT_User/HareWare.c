//
//JIAlonglong 2023.2.17
//
#include "HareWare.h"
#include "main.h"
#include "tim.h"
#include "gpio.h"
//电机初始化
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

/******航模遥控<PPM>*******/
//航模结构体实例
Air_Contorl  Device;
//定义变量
static uint16_t PPM_buf[10]={0};
uint16_t PPM_Databuf[10]={0};
uint8_t ppm_update_flag=0;
uint32_t now_ppm_time_send=0;
uint32_t TIME_ISR_CNT=0,LAST_TIME_ISR_CNT=0;
//TIM2_IRQHandler
uint16_t Time_Sys[4]={0};
uint16_t Microsecond_Cnt=0;;

#define Hour         3
#define Minute       2
#define Second       1
#define MicroSecond  0

void bsp_air_Callback_EXTI_IRQHandler(void)
{
	static uint32_t last_ppm_time=0,now_ppm_time=0;
	static uint8_t ppm_ready=0,ppm_sample_cnt=0;
	static uint16_t ppm_time_delta=0;//得到上升沿与下降沿的时间
	 if (bsp_air_Callback_Pin == GPIO_PIN_7)
  {
    // 获取系统运行时间，单位us
    last_ppm_time = now_ppm_time;
    now_ppm_time = 10000 * HAL_GetTick() + __HAL_TIM_GET_COUNTER(&htim2);
    ppm_time_delta = now_ppm_time - last_ppm_time;

    // PPM解析开始
    if (ppm_ready == 1)
    {
      if (ppm_time_delta >= 2200)
      {
        ppm_ready = 1;
        ppm_sample_cnt = 0;
        ppm_update_flag=1;
      }
      else if (ppm_time_delta >= 950 && ppm_time_delta <= 2050)
      {
        PPM_buf[ppm_sample_cnt++] = ppm_time_delta;
        if (ppm_sample_cnt >= 8)
        {
          memcpy(PPM_Databuf, PPM_buf, ppm_sample_cnt * sizeof(uint16_t));
          ppm_sample_cnt = 0;
        }
      }
      else
      {
        ppm_ready = 0;
      }
    }
    else if (ppm_time_delta >= 2200)
    {
      ppm_ready = 1;
      ppm_sample_cnt = 0;
			ppm_update_flag=0;
    }
  }
}
//无需声明,有weak
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



