#ifndef __MOVEBASE_H
#define __MOVEBASE_H
/* Includes ------------------------------------------------------------------*/

#include "arm_math.h"
#include "can.h"
/* function ------------------------------------------------------------------*/
void World_3wheels_onmi(float Vx,float Vy,float W,float theta);
void World_4wheels_onmi(float Vx,float Vy,float W,float theta,float width,float length);
//三舵轮底盘
void World_3_AGVwheel(float Vx,float Vy,float W,float theta,float width,float length);
//四舵轮底盘
void World_4_AGVwheel(float Vx,float Vy,float W,float theta,float width,float length);
/**
* @brief  TurnInferiorArc确保旋转角度为劣弧
  * @note
* @param  targetAngle:目标角度
		  actualAngle：当前角度
* @retval 
  */
float TurnInferiorArc(float targetAngle , float actualAngle);
/**
* @brief  WheelAngle2PositionTransform将轮子朝向角度转化为脉冲
  * @note
* @param  angle:轮子朝向角度
* @retval 对应的电机脉冲位置
  */
int WheelAngle2PositionTransform(float angle , int32_t loopShift);
/**
* @brief  WheelAngle2PositionInverseTransform将轮子脉冲位置转化为角度
  * @note
* @param  position:轮子脉冲位置
* @retval 轮子朝向角度
  */
float WheelAngle2PositionInverseTransform(int position , int32_t loopShift);
/**
* @brief  Vel2Pulse将速度转换为脉冲
  * @note
  * @param  vel:速度（mm/s）
  * @retval 脉冲速度
  */
int Vel2Pulse(float vel);
/**
* @brief  Pulse2Vel将速度转换为脉冲
  * @note
* @param  pulse:脉冲速度
  * @retval 速度（mm/s）
  */
float Pulse2Vel(int pulse);
/**
* @brief  AngleLimit角度限幅，将角度限制在-180°到180°
  * @note
* @param  angle:要限制的值
* @retval 
  */
void AngleLimit(float *angle);
/**
* @brief  ReturnLimitAngle返回限制后的角度值
  * @note
* @param  angle:要限制的值
* @retval 
  */
float ReturnLimitAngle(float angle);

/* define ------------------------------------------------------------------*/
//三全向轮底盘的参数
#define X_PARAMETER          (0.5f)               
#define Y_PARAMETER           (sqrt(3)/2.f)      
#define L_PARAMETER            (1.0f)

// Chassis Config
#define WHEEL_R            0.076f	                  //轮子半径(单位：m)
#define Robot_R            0.406f                  	//车轮到中心距离(单位：m)
#define M3508_RM_To_MS     (PI*WHEEL_R)/570.0f      //转速与速度的转换 (单位：m/s) 
#define M3508_MS_To_RM     1.0f/(PI*WHEEL_R)      //速度与转速的转换 (单位：m/s)  
#define MS_TO_RPM          21*60/(PI*WHEEL_R*2)     //轮子直径152mm，电机减速比1:21，轮子一圈pi*152mm
#define RM_transition_MS (PI * WHEEL_R) / 570.0f //转速与速度的转换
#define MS_transition_RM 1.0f / (PI * WHEEL_R) //速度与转速的转换
																										// 计算公式：1/（pi*轮子直径）*减速比*60

//电机旋转一周的脉冲数
#define COUNTS_PER_ROUND (32768)
//电机最大转速
#define MAX_MOTOR_SPEED (COUNTS_PER_ROUND*100)
//轮子直径（单位：mm）
#define WHEEL_DIAMETER (70.0f)
//定位系统X轴方向到中心距离
#define DISX_OPS2CENTER (0.0f)
//定位系统Y轴方向到中心距离
#define DISY_OPS2CENTER (-307.0f)
//驱动轮减速比
#define WHEEL_REDUCTION_RATIO (2.0f/1.0f)
//M2006减速比
#define M2006_REDUCTION_RATIO (36.0f/1.0f)
//转向齿轮减速比
#define TURNING_REDUCTION_RATIO (4.0f/1.0f)
//轮子转向减速比
#define WHEEL_TURNING_REDUCTION_RATIO (M2006_REDUCTION_RATIO*TURNING_REDUCTION_RATIO)
//底盘旋转半径
#define MOVEBASE_RADIUS (362.039f)
//角度制转化为弧度制
#define ANGLE2RAD(x) (x/180.0f*PI)
//弧度制转换为角度制
#define RAD2ANGLE(x) (x/PI*180.0f)
																				
//左前轮ID号
#define LEFT_FRONT_ID (1)
//右前轮ID号
#define RIGHT_FRONT_ID (2)
//左后轮ID号
#define LEFT_REAR_ID (3)
//右后轮ID号
#define RIGHT_REAR_ID (4)
//左前轮转向ID号
#define LEFT_FRONT_TURNING_ID (5)
//右前轮转向ID号
#define RIGHT_FRONT_TURNING_ID (6)
//左后轮转向ID号
#define LEFT_REAR_TURNING_ID (7)
//右后轮转向ID号
#define RIGHT_REAR_TURNING_ID (8)

#endif
