#ifndef __HAREWARE_H
#define __HAREWARE_H

#include "elmo.h"
//Functions
void MotorInit(void);
void bsp_air_Callback_EXTI_IRQHandler(void);
//struct
typedef struct
{
	struct  //遥控原始数据，8通道
	{
	 uint16_t roll;			//右摇杆
	 uint16_t pitch;		//
	 uint16_t thr;
	 uint16_t yaw;
	 uint16_t AUX1;
	 uint16_t AUX2;
	 uint16_t AUX3;
	 uint16_t AUX4; 
	 uint16_t	BUX1;
	 uint16_t	BUX2;		
	}Remote; 

}Air_Contorl;
//extern
extern Air_Contorl  Device;
//define
#define AIR_L_SHORT		PPM_Databuf[4]				//AUX4 1000~2000//没用
#define AIR_L_LONG		PPM_Databuf[5]				//AUX2 1000-1500-2000
#define AIR_R_SHORT		PPM_Databuf[7]			//AUX1 1000~2000
#define AIR_R_LONG		PPM_Databuf[6]				//AUX3 1000-1500-2000
#define ROCK_R_X			PPM_Databuf[0]					//YAW  1000-1500-2000
#define ROCK_R_Y			PPM_Databuf[1]					//THR  1000-1500-2000
#define ROCK_L_Y			PPM_Databuf[2]				//ROLL 1000-1500-2000//未知bug
#define	ROCK_L_X		  PPM_Databuf[3]				//PITCH 1000-1500-2000P
#define LEFT_BUTTON		Device.Remote.BUX1
#define RIGHT_BUTTON	Device.Remote.BUX2				//注意！！这里修改了两个按键的宏定义

#endif
