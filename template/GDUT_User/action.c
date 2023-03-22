//
//JIAlonglong 2023.2.17
//
#include "MoveBase.h"
/********�Ƕ�ת��********/
/**
* @brief  WheelAngle2PositionTransform�����ӳ���Ƕ�ת��Ϊ����
  * @note
* @param  angle:���ӳ���Ƕ�
* @retval ��Ӧ�ĵ������λ��
  */
int WheelAngle2PositionTransform(float angle , int32_t loopShift)
{
	return (int)(((angle / 360.0f)* WHEEL_TURNING_REDUCTION_RATIO + loopShift)* COUNTS_PER_ROUND );
}
/**
* @brief  WheelAngle2PositionInverseTransform����������λ��ת��Ϊ�Ƕ�
  * @note
* @param  position:��������λ��
* @retval ���ӳ���Ƕ�
  */
float WheelAngle2PositionInverseTransform(int position , int32_t loopShift)
{
	return (float)(((float)position / COUNTS_PER_ROUND - loopShift)/ WHEEL_TURNING_REDUCTION_RATIO * 360.0f);
}
/********�ٶ�ת��********/
/**
* @brief  Vel2Pulse���ٶ�ת��Ϊ����
  * @note
  * @param  vel:�ٶȣ�mm/s��
  * @retval �����ٶ�
  */
int Vel2Pulse(float vel)
{
	return (int)(vel/(PI*WHEEL_DIAMETER)*COUNTS_PER_ROUND*WHEEL_REDUCTION_RATIO);
}

/**
* @brief  Pulse2Vel���ٶ�ת��Ϊ����
  * @note
* @param  pulse:�����ٶ�
  * @retval �ٶȣ�mm/s��
  */
float Pulse2Vel(int pulse)
{
	return ((float)pulse/COUNTS_PER_ROUND)/WHEEL_REDUCTION_RATIO*PI*WHEEL_DIAMETER;
}
/********���������Լ�����********/

//����ȫ���ֵ���<mm/s>
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

//����ȫ���ֵ���<���ֿ���>
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

//�����ֵ���
void World_3_AGVwheel(float Vx,float Vy,float W,float theta,float width,float length)
{
float vx,vy,w,AWheelAng,BWheelAng,CWheelAng;
	//3508Ŀ���ٶ�
	int16_t wheel_rpm[3],out_speed[3];
	//3508Ŀ��Ƕ�
	int16_t wheel_pos[3],out_pos[3];
	int8_t drct=1;//���������������ת
	theta = theta* PI / 180.0f;
	vx=Vx*cos(theta)+Vy*sin(theta);
	vy=-Vx*sin(theta)+Vy*cos(theta);
	w=W;
	//����Ŀ���ٶ�
	wheel_rpm[0]=Vel2Pulse((sqrt(pow(vy+w*sqrt(length/2)*1.0f,2)+pow(vx-w*sqrt(length/2)*0.0f,2))));//A LF
	wheel_rpm[1]=Vel2Pulse((sqrt(pow(vy+w*sqrt(width/2+length/2)*sin(60),2)+pow(vx+w*sqrt(width/2+length/2)*cos(60),2))));//B RB
	wheel_rpm[2]=Vel2Pulse((sqrt(pow(vy-w*sqrt(width/2+length/2)*sin(60),2)+pow(vx+w*sqrt(width/2+length/2)*cos(60),2))));//C LB
	for(int i=0;i<3;i++)
       out_speed[i] = drct * wheel_rpm[i];
	//����Ŀ��Ƕ�
	if(! (vx == 0 && vy == 0&& w == 0))
	{
		wheel_pos[0]=atan2((vx-w*sqrt(length/2)*1.0f),(vy+w*sqrt(length/2)*0.0f))*180.0f/PI;//A
		wheel_pos[1]=atan2((vx+w*sqrt(width/2+length/2)*sin(60)),(vy+w*sqrt(width/2+length/2)*cos(60)))*180.0f/PI;//B
		wheel_pos[2]=atan2((vx+w*sqrt(width/2+length/2)*sin(60)),(vy-w*sqrt(width/2+length/2)*cos(60)))*180.0f/PI;//C
	}
	//�����ٶ���Ϣ
		//VelCrl(CAN_HandleTypeDef* hcan, uint8_t ElmoNum,int32_t vel);
	//remember to change ID when you use!
	VelCrl(&hcan1,1,out_speed[0]);
	VelCrl(&hcan1,2,out_speed[1]);
	VelCrl(&hcan1,3,out_speed[2]);
	
	//��֤��תΪ�ӻ�
	AWheelAng  = TurnInferiorArc(wheel_pos[0] , AWheelAng);
	BWheelAng = TurnInferiorArc(wheel_pos[1] , BWheelAng);
	CWheelAng   = TurnInferiorArc(wheel_pos[2] , CWheelAng);
	
	//�Ƕ����ƿ���
	AngleLimit(&AWheelAng);
	AngleLimit(&BWheelAng);
	AngleLimit(&CWheelAng);
	
	//����λ����Ϣ
	//remember to change ID when you use!
	PosCrl(&hcan1 , 4 , ABSOLUTE_MODE , WheelAngle2PositionTransform(AWheelAng , out_pos[0]));
	PosCrl(&hcan1 , 5 , ABSOLUTE_MODE , WheelAngle2PositionTransform(BWheelAng , out_pos[1]));
	PosCrl(&hcan1 , 6 , ABSOLUTE_MODE , WheelAngle2PositionTransform(CWheelAng , out_pos[2]));
}

//�Ķ��ֵ���
void World_4_AGVwheel(float Vx,float Vy,float W,float theta,float width,float length)
{
	float vx,vy,w,leftFrontAng,rightFrontAng,leftRearAng,rightRearAng;
	//3508Ŀ���ٶ�
	int16_t wheel_rpm[4],out_speed[4];
	//3508Ŀ��Ƕ�
	int16_t wheel_pos[4],out_pos[4];
	int8_t drct=1;//���������������ת
	theta = theta* PI / 180.0f;
	vx=Vx*cos(theta)+Vy*sin(theta);
	vy=-Vx*sin(theta)+Vy*cos(theta);
	w=W;
	//����Ŀ���ٶ�
	wheel_rpm[0]=Vel2Pulse((sqrt(pow(vy+w*sqrt(width/2+length/2)*0.707107f,2)+pow(vx-w*sqrt(width/2+length/2)*0.707107f,2))));//LF
	wheel_rpm[1]=Vel2Pulse((sqrt(pow(vy-w*sqrt(width/2+length/2)*0.707107f,2)+pow(vx-w*sqrt(width/2+length/2)*0.707107f,2))));//RF
	wheel_rpm[2]=Vel2Pulse((sqrt(pow(vy-w*sqrt(width/2+length/2)*0.707107f,2)+pow(vx+w*sqrt(width/2+length/2)*0.707107f,2))));//LB
	wheel_rpm[3]=Vel2Pulse((sqrt(pow(vy+w*sqrt(width/2+length/2)*0.707107f,2)+pow(vx+w*sqrt(width/2+length/2)*0.707107f,2))));//RB
	for(int i=0;i<4;i++)
       out_speed[i] = drct * wheel_rpm[i];
	//����Ŀ��Ƕ�
	if(! (vx == 0 && vy == 0&& w == 0))
	{
		wheel_pos[0]=atan2((vx-w*sqrt(width/2+length/2)*0.707107f),(vy+w*sqrt(width/2+length/2)*0.707107f))*180.0f/PI;//LF
		wheel_pos[1]=atan2((vx-w*sqrt(width/2+length/2)*0.707107f),(vy-w*sqrt(width/2+length/2)*0.707107f))*180.0f/PI;//RF
		wheel_pos[2]=atan2((vx+w*sqrt(width/2+length/2)*0.707107f),(vy-w*sqrt(width/2+length/2)*0.707107f))*180.0f/PI;//LB
		wheel_pos[3]=atan2((vx+w*sqrt(width/2+length/2)*0.707107f),(vy+w*sqrt(width/2+length/2)*0.707107f))*180.0f/PI;//RB
	}
	//�����ٶ���Ϣ
	//remember to change ID when you use!
	VelCrl(&hcan1,1,out_speed[0]);
	VelCrl(&hcan1,2,out_speed[1]);
	VelCrl(&hcan1,3,out_speed[2]);
	VelCrl(&hcan1,4,out_speed[3]);
	
	//��֤��תΪ�ӻ�
	leftFrontAng  = TurnInferiorArc(wheel_pos[0] , leftFrontAng);
	rightFrontAng = TurnInferiorArc(wheel_pos[1] , rightFrontAng);
	leftRearAng   = TurnInferiorArc(wheel_pos[2] , leftRearAng);
	rightRearAng  = TurnInferiorArc(wheel_pos[3] , rightRearAng);
	
	//�Ƕ����ƿ���
	AngleLimit(&leftFrontAng);
	AngleLimit(&rightFrontAng);
	AngleLimit(&leftRearAng);
	AngleLimit(&rightRearAng);
	
	//����λ����Ϣ
	//remember to change ID when you use!
	PosCrl(&hcan1 , 5 , ABSOLUTE_MODE , WheelAngle2PositionTransform(leftFrontAng , out_pos[0]));
	PosCrl(&hcan1 , 6 , ABSOLUTE_MODE , WheelAngle2PositionTransform(rightFrontAng , out_pos[1]));
	PosCrl(&hcan1 , 7 , ABSOLUTE_MODE , WheelAngle2PositionTransform(leftRearAng , out_pos[2]));
	PosCrl(&hcan1 , 8 , ABSOLUTE_MODE , WheelAngle2PositionTransform(rightRearAng , out_pos[3]));
}

/**
* @brief  TurnInferiorArcȷ����ת�Ƕ�Ϊ�ӻ�
  * @note
* @param  targetAngle:Ŀ��Ƕ�
		  actualAngle����ǰ�Ƕ�
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
* @brief  AngleLimit�Ƕ��޷������Ƕ�������-180�㵽180��
  * @note
* @param  angle:Ҫ���Ƶ�ֵ
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
* @brief  ReturnLimitAngle�������ƺ�ĽǶ�ֵ
  * @note
* @param  angle:Ҫ���Ƶ�ֵ
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