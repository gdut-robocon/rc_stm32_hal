#ifndef __MOVEBASE_H
#define __MOVEBASE_H
/* Includes ------------------------------------------------------------------*/

#include "arm_math.h"
#include "can.h"
/* function ------------------------------------------------------------------*/
void World_3wheels_onmi(float Vx,float Vy,float W,float theta);
void World_4wheels_onmi(float Vx,float Vy,float W,float theta,float width,float length);
//�����ֵ���
void World_3_AGVwheel(float Vx,float Vy,float W,float theta,float width,float length);
//�Ķ��ֵ���
void World_4_AGVwheel(float Vx,float Vy,float W,float theta,float width,float length);
/**
* @brief  TurnInferiorArcȷ����ת�Ƕ�Ϊ�ӻ�
  * @note
* @param  targetAngle:Ŀ��Ƕ�
		  actualAngle����ǰ�Ƕ�
* @retval 
  */
float TurnInferiorArc(float targetAngle , float actualAngle);
/**
* @brief  WheelAngle2PositionTransform�����ӳ���Ƕ�ת��Ϊ����
  * @note
* @param  angle:���ӳ���Ƕ�
* @retval ��Ӧ�ĵ������λ��
  */
int WheelAngle2PositionTransform(float angle , int32_t loopShift);
/**
* @brief  WheelAngle2PositionInverseTransform����������λ��ת��Ϊ�Ƕ�
  * @note
* @param  position:��������λ��
* @retval ���ӳ���Ƕ�
  */
float WheelAngle2PositionInverseTransform(int position , int32_t loopShift);
/**
* @brief  Vel2Pulse���ٶ�ת��Ϊ����
  * @note
  * @param  vel:�ٶȣ�mm/s��
  * @retval �����ٶ�
  */
int Vel2Pulse(float vel);
/**
* @brief  Pulse2Vel���ٶ�ת��Ϊ����
  * @note
* @param  pulse:�����ٶ�
  * @retval �ٶȣ�mm/s��
  */
float Pulse2Vel(int pulse);
/**
* @brief  AngleLimit�Ƕ��޷������Ƕ�������-180�㵽180��
  * @note
* @param  angle:Ҫ���Ƶ�ֵ
* @retval 
  */
void AngleLimit(float *angle);
/**
* @brief  ReturnLimitAngle�������ƺ�ĽǶ�ֵ
  * @note
* @param  angle:Ҫ���Ƶ�ֵ
* @retval 
  */
float ReturnLimitAngle(float angle);

/* define ------------------------------------------------------------------*/
//��ȫ���ֵ��̵Ĳ���
#define X_PARAMETER          (0.5f)               
#define Y_PARAMETER           (sqrt(3)/2.f)      
#define L_PARAMETER            (1.0f)

// Chassis Config
#define WHEEL_R            0.076f	                  //���Ӱ뾶(��λ��m)
#define Robot_R            0.406f                  	//���ֵ����ľ���(��λ��m)
#define M3508_RM_To_MS     (PI*WHEEL_R)/570.0f      //ת�����ٶȵ�ת�� (��λ��m/s) 
#define M3508_MS_To_RM     1.0f/(PI*WHEEL_R)      //�ٶ���ת�ٵ�ת�� (��λ��m/s)  
#define MS_TO_RPM          21*60/(PI*WHEEL_R*2)     //����ֱ��152mm��������ٱ�1:21������һȦpi*152mm
#define RM_transition_MS (PI * WHEEL_R) / 570.0f //ת�����ٶȵ�ת��
#define MS_transition_RM 1.0f / (PI * WHEEL_R) //�ٶ���ת�ٵ�ת��
																										// ���㹫ʽ��1/��pi*����ֱ����*���ٱ�*60

//�����תһ�ܵ�������
#define COUNTS_PER_ROUND (32768)
//������ת��
#define MAX_MOTOR_SPEED (COUNTS_PER_ROUND*100)
//����ֱ������λ��mm��
#define WHEEL_DIAMETER (70.0f)
//��λϵͳX�᷽�����ľ���
#define DISX_OPS2CENTER (0.0f)
//��λϵͳY�᷽�����ľ���
#define DISY_OPS2CENTER (-307.0f)
//�����ּ��ٱ�
#define WHEEL_REDUCTION_RATIO (2.0f/1.0f)
//M2006���ٱ�
#define M2006_REDUCTION_RATIO (36.0f/1.0f)
//ת����ּ��ٱ�
#define TURNING_REDUCTION_RATIO (4.0f/1.0f)
//����ת����ٱ�
#define WHEEL_TURNING_REDUCTION_RATIO (M2006_REDUCTION_RATIO*TURNING_REDUCTION_RATIO)
//������ת�뾶
#define MOVEBASE_RADIUS (362.039f)
//�Ƕ���ת��Ϊ������
#define ANGLE2RAD(x) (x/180.0f*PI)
//������ת��Ϊ�Ƕ���
#define RAD2ANGLE(x) (x/PI*180.0f)
																				
//��ǰ��ID��
#define LEFT_FRONT_ID (1)
//��ǰ��ID��
#define RIGHT_FRONT_ID (2)
//�����ID��
#define LEFT_REAR_ID (3)
//�Һ���ID��
#define RIGHT_REAR_ID (4)
//��ǰ��ת��ID��
#define LEFT_FRONT_TURNING_ID (5)
//��ǰ��ת��ID��
#define RIGHT_FRONT_TURNING_ID (6)
//�����ת��ID��
#define LEFT_REAR_TURNING_ID (7)
//�Һ���ת��ID��
#define RIGHT_REAR_TURNING_ID (8)

#endif
