//
//JIAlonglong 2023.2.17
//
#include "MoveBase.h"
/********角度转换********/
/**
* @brief  WheelAngle2PositionTransform将轮子朝向角度转化为脉冲
  * @note
* @param  angle:轮子朝向角度
* @retval 对应的电机脉冲位置
  */
int WheelAngle2PositionTransform(float angle , int32_t loopShift)
{
	return (int)(((angle / 360.0f)* WHEEL_TURNING_REDUCTION_RATIO + loopShift)* COUNTS_PER_ROUND );
}
/**
* @brief  WheelAngle2PositionInverseTransform将轮子脉冲位置转化为角度
  * @note
* @param  position:轮子脉冲位置
* @retval 轮子朝向角度
  */
float WheelAngle2PositionInverseTransform(int position , int32_t loopShift)
{
	return (float)(((float)position / COUNTS_PER_ROUND - loopShift)/ WHEEL_TURNING_REDUCTION_RATIO * 360.0f);
}
/********速度转换********/
/**
* @brief  Vel2Pulse将速度转换为脉冲
  * @note
  * @param  vel:速度（mm/s）
  * @retval 脉冲速度
  */
int Vel2Pulse(float vel)
{
	return (int)(vel/(PI*WHEEL_DIAMETER)*COUNTS_PER_ROUND*WHEEL_REDUCTION_RATIO);
}

/**
* @brief  Pulse2Vel将速度转换为脉冲
  * @note
* @param  pulse:脉冲速度
  * @retval 速度（mm/s）
  */
float Pulse2Vel(int pulse)
{
	return ((float)pulse/COUNTS_PER_ROUND)/WHEEL_REDUCTION_RATIO*PI*WHEEL_DIAMETER;
}
/********底盘类型以及解算********/

//三轮全向轮底盘<mm/s>
void World_3wheels_onmi(float Vx,float Vy,float W,float theta)
{
	float A_wheel_target_RPM,B_wheel_target_RPM,C_wheel_target_RPM;
	theta = theta* PI / 180.0f;
	A_wheel_target_RPM = Vel2Pulse(-( -cos(theta) * Vx -sin(theta) * Vy + Robot_R*W));
	B_wheel_target_RPM = Vel2Pulse(-(+sin(theta+PI/6.0f) * Vx - cos(theta+PI/6.0f) * Vy + Robot_R*W));
	C_wheel_target_RPM = Vel2Pulse(-(+cos(theta+PI/3.0f) * Vx + sin(theta+PI/3.0f) * Vy + Robot_R*W));
	//remember to change ID when you use!
	VelCrl(&hcan1,1,A_wheel_target_RPM);
	VelCrl(&hcan1,2,B_wheel_target_RPM);
	VelCrl(&hcan1,3,C_wheel_target_RPM);
}

//四轮全向轮底盘<麦轮可用>
void World_4wheels_onmi(float Vx,float Vy,float W,float theta,float width,float length)
{
	float LF_wheel_target_RPM,LB_wheel_target_RPM,RB_wheel_target_RPM,RF_wheel_target_RPM,vx,vy,w;
	theta = theta* PI / 180.0f;
	vx=Vx*cos(theta)+Vy*sin(theta);
	vy=-Vx*sin(theta)+Vy*cos(theta);
	w=W;
	LF_wheel_target_RPM = Vel2Pulse((vy+vx-w*sqrt((width/2+length/2))));
	LB_wheel_target_RPM = Vel2Pulse((vy-vx-w*sqrt((width/2+length/2))));
	RB_wheel_target_RPM = Vel2Pulse((vy+vx+w*sqrt((width/2+length/2))));
	RF_wheel_target_RPM = Vel2Pulse((vy-vx+w*sqrt((width/2+length/2))));
	//remember to change ID when you use!
	VelCrl(&hcan1,1,LF_wheel_target_RPM);
	VelCrl(&hcan1,2,LB_wheel_target_RPM);
	VelCrl(&hcan1,3,RB_wheel_target_RPM);
	VelCrl(&hcan1,4,RF_wheel_target_RPM);
}

//三舵轮底盘
void World_3_AGVwheel(float Vx,float Vy,float W,float theta,float width,float length)
{
float vx,vy,w,AWheelAng,BWheelAng,CWheelAng;
	//3508目标速度
	int16_t wheel_rpm[3],out_speed[3];
	//3508目标角度
	int16_t wheel_pos[3],out_pos[3];
	int8_t drct=1;//决定驱动电机正反转
	theta = theta* PI / 180.0f;
	vx=Vx*cos(theta)+Vy*sin(theta);
	vy=-Vx*sin(theta)+Vy*cos(theta);
	w=W;
	//计算目标速度
	wheel_rpm[0]=Vel2Pulse((sqrt(pow(vy+w*sqrt(length/2)*1.0f,2)+pow(vx-w*sqrt(length/2)*0.0f,2))));//A LF
	wheel_rpm[1]=Vel2Pulse((sqrt(pow(vy+w*sqrt(width/2+length/2)*sin(60),2)+pow(vx+w*sqrt(width/2+length/2)*cos(60),2))));//B RB
	wheel_rpm[2]=Vel2Pulse((sqrt(pow(vy-w*sqrt(width/2+length/2)*sin(60),2)+pow(vx+w*sqrt(width/2+length/2)*cos(60),2))));//C LB
	for(int i=0;i<3;i++)
       out_speed[i] = drct * wheel_rpm[i];
	//计算目标角度
	if(! (vx == 0 && vy == 0&& w == 0))
	{
		wheel_pos[0]=atan2((vx-w*sqrt(length/2)*1.0f),(vy+w*sqrt(length/2)*0.0f))*180.0f/PI;//A
		wheel_pos[1]=atan2((vx+w*sqrt(width/2+length/2)*sin(60)),(vy+w*sqrt(width/2+length/2)*cos(60)))*180.0f/PI;//B
		wheel_pos[2]=atan2((vx+w*sqrt(width/2+length/2)*sin(60)),(vy-w*sqrt(width/2+length/2)*cos(60)))*180.0f/PI;//C
	}
	//发送速度信息
		//VelCrl(CAN_HandleTypeDef* hcan, uint8_t ElmoNum,int32_t vel);
	//remember to change ID when you use!
	VelCrl(&hcan1,1,out_speed[0]);
	VelCrl(&hcan1,2,out_speed[1]);
	VelCrl(&hcan1,3,out_speed[2]);
	
	//保证旋转为劣弧
	AWheelAng  = TurnInferiorArc(wheel_pos[0] , AWheelAng);
	BWheelAng = TurnInferiorArc(wheel_pos[1] , BWheelAng);
	CWheelAng   = TurnInferiorArc(wheel_pos[2] , CWheelAng);
	
	//角度限制控制
	AngleLimit(&AWheelAng);
	AngleLimit(&BWheelAng);
	AngleLimit(&CWheelAng);
	
	//发送位置信息
	//remember to change ID when you use!
	PosCrl(&hcan1 , 4 , ABSOLUTE_MODE , WheelAngle2PositionTransform(AWheelAng , out_pos[0]));
	PosCrl(&hcan1 , 5 , ABSOLUTE_MODE , WheelAngle2PositionTransform(BWheelAng , out_pos[1]));
	PosCrl(&hcan1 , 6 , ABSOLUTE_MODE , WheelAngle2PositionTransform(CWheelAng , out_pos[2]));
}

//四舵轮底盘
void World_4_AGVwheel(float Vx,float Vy,float W,float theta,float width,float length)
{
	float vx,vy,w,leftFrontAng,rightFrontAng,leftRearAng,rightRearAng;
	//3508目标速度
	int16_t wheel_rpm[4],out_speed[4];
	//3508目标角度
	int16_t wheel_pos[4],out_pos[4];
	int8_t drct=1;//决定驱动电机正反转
	theta = theta* PI / 180.0f;
	vx=Vx*cos(theta)+Vy*sin(theta);
	vy=-Vx*sin(theta)+Vy*cos(theta);
	w=W;
	//计算目标速度
	wheel_rpm[0]=Vel2Pulse((sqrt(pow(vy+w*sqrt(width/2+length/2)*0.707107f,2)+pow(vx-w*sqrt(width/2+length/2)*0.707107f,2))));//LF
	wheel_rpm[1]=Vel2Pulse((sqrt(pow(vy-w*sqrt(width/2+length/2)*0.707107f,2)+pow(vx-w*sqrt(width/2+length/2)*0.707107f,2))));//RF
	wheel_rpm[2]=Vel2Pulse((sqrt(pow(vy-w*sqrt(width/2+length/2)*0.707107f,2)+pow(vx+w*sqrt(width/2+length/2)*0.707107f,2))));//LB
	wheel_rpm[3]=Vel2Pulse((sqrt(pow(vy+w*sqrt(width/2+length/2)*0.707107f,2)+pow(vx+w*sqrt(width/2+length/2)*0.707107f,2))));//RB
	for(int i=0;i<4;i++)
       out_speed[i] = drct * wheel_rpm[i];
	//计算目标角度
	if(! (vx == 0 && vy == 0&& w == 0))
	{
		wheel_pos[0]=atan2((vx-w*sqrt(width/2+length/2)*0.707107f),(vy+w*sqrt(width/2+length/2)*0.707107f))*180.0f/PI;//LF
		wheel_pos[1]=atan2((vx-w*sqrt(width/2+length/2)*0.707107f),(vy-w*sqrt(width/2+length/2)*0.707107f))*180.0f/PI;//RF
		wheel_pos[2]=atan2((vx+w*sqrt(width/2+length/2)*0.707107f),(vy-w*sqrt(width/2+length/2)*0.707107f))*180.0f/PI;//LB
		wheel_pos[3]=atan2((vx+w*sqrt(width/2+length/2)*0.707107f),(vy+w*sqrt(width/2+length/2)*0.707107f))*180.0f/PI;//RB
	}
	//发送速度信息
	//remember to change ID when you use!
	VelCrl(&hcan1,1,out_speed[0]);
	VelCrl(&hcan1,2,out_speed[1]);
	VelCrl(&hcan1,3,out_speed[2]);
	VelCrl(&hcan1,4,out_speed[3]);
	
	//保证旋转为劣弧
	leftFrontAng  = TurnInferiorArc(wheel_pos[0] , leftFrontAng);
	rightFrontAng = TurnInferiorArc(wheel_pos[1] , rightFrontAng);
	leftRearAng   = TurnInferiorArc(wheel_pos[2] , leftRearAng);
	rightRearAng  = TurnInferiorArc(wheel_pos[3] , rightRearAng);
	
	//角度限制控制
	AngleLimit(&leftFrontAng);
	AngleLimit(&rightFrontAng);
	AngleLimit(&leftRearAng);
	AngleLimit(&rightRearAng);
	
	//发送位置信息
	//remember to change ID when you use!
	PosCrl(&hcan1 , 5 , ABSOLUTE_MODE , WheelAngle2PositionTransform(leftFrontAng , out_pos[0]));
	PosCrl(&hcan1 , 6 , ABSOLUTE_MODE , WheelAngle2PositionTransform(rightFrontAng , out_pos[1]));
	PosCrl(&hcan1 , 7 , ABSOLUTE_MODE , WheelAngle2PositionTransform(leftRearAng , out_pos[2]));
	PosCrl(&hcan1 , 8 , ABSOLUTE_MODE , WheelAngle2PositionTransform(rightRearAng , out_pos[3]));
}

/**
* @brief  TurnInferiorArc确保旋转角度为劣弧
  * @note
* @param  targetAngle:目标角度
		  actualAngle：当前角度
* @retval 
  */
float TurnInferiorArc(float targetAngle , float actualAngle)
{
	if(targetAngle - actualAngle>180.0f)
	{
		return (targetAngle - 360.0f);
	}
	else if(targetAngle - actualAngle<-180.0f)
	{
		return (targetAngle + 360.0f);
	}
	else
	{
		return targetAngle;
	}	
}
/**
* @brief  AngleLimit角度限幅，将角度限制在-180°到180°
  * @note
* @param  angle:要限制的值
* @retval 
  */
void AngleLimit(float *angle)
{
	static uint8_t recursiveTimes = 0;
	
	recursiveTimes++;
	
	if(recursiveTimes<100)
	{
		if(*angle>180.0f)
		{
			*angle-=360.0f;
			AngleLimit(angle);
		}
		else if(*angle<-180.0f)
		{
			*angle+=360.0f;
			AngleLimit(angle);
		}
	}
	
	recursiveTimes--;
}
/**
* @brief  ReturnLimitAngle返回限制后的角度值
  * @note
* @param  angle:要限制的值
* @retval 
  */
float ReturnLimitAngle(float angle)
{
	static uint8_t recursiveTimes = 0;
	
	recursiveTimes++;
	
	if(recursiveTimes<100)
	{
		if(angle>180.0f)
		{
			angle = ReturnLimitAngle(angle - 360.0f);
		}
		else if(angle<-180.0f)
		{
			angle = ReturnLimitAngle(angle + 360.0f);
		}
	}
	
	recursiveTimes--;
	
	return angle;
}