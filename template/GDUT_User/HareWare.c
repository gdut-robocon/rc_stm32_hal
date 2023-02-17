#include "HareWare.h"
#include "main.h"
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
