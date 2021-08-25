/*
  ******************************************************************************
  * @file    .h
  * @author  ACTION_2017
  * @version V0.0.0._alpha
  * @date    2017//
  * @brief   This file contains all the functions prototypes for 
  *          
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOVEBASE_H
#define __MOVEBASE_H

/* Includes ------------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
//机器人速度结构体
typedef struct
{
	//速度大小
	float vel;
	//速度方向
	float direction;
	//角速度大小
	float omega;
}robotVel_t;

//轮子速度和方向结构体
typedef struct
{
	//轮子速度大小
	float vel;
	//轮子速度方向
	float direction;
}wheelVel_t;

//轮子结构体
typedef struct
{
	//左前轮
	wheelVel_t one;
	//右前轮
	wheelVel_t two;
	//左后轮
	wheelVel_t three;
}wheel_t;

//轮子状态结构体
typedef struct
{
	//左前轮目标速度与方向
	wheelVel_t oneTarget;
	//右前轮目标速度与方向
	wheelVel_t twoTarget;
	//左后轮目标速度与方向
	wheelVel_t threeTarget;
	
}wheelState_t;

/** 
  * @brief  
  */


 
/* Exported constants --------------------------------------------------------*/



/** @defgroup 
  * @{
  */

//#define SEND_MOVEBASE_DEBUGINFO

#ifndef PI

#define PI (3.1415926f)

#endif


//电机旋转一周的脉冲数
#define COUNTS_PER_ROUND (4096)
//电机最大转速 脉冲为单位
#define MAX_MOTOR_SPEED (COUNTS_PER_ROUND*120)
//轮子直径（单位：mm）
#define WHEEL_DIAMETER (60.0f)
//定位系统到中心距离
#define DIS_OPS2CENTER (135.0f)
//定位系统X轴方向到中心距离 135*cos(33.92265°) = 112.0f
#define DISX_OPS2CENTER (-112.0f)
//定位系统Y轴方向到中心距离 135*sin(33.92265°) = 75.34f
#define DISY_OPS2CENTER (-75.34f)
//驱动轮减速比
#define WHEEL_REDUCTION_RATIO (2.0f/1.0f)
//轮子转向减速比
#define WHEEL_TURNING_REDUCTION_RATIO (110.0f/46.0f)
//底盘旋转半径
#define MOVEBASE_RADIUS (500.0f/sqrtf(3.0f))
//角度制转化为弧度制
#define ANGLE2RAD(x) (x/180.0f*PI)
//弧度制转换为角度制
#define RAD2ANGLE(x) (x/PI*180.0f)

//前轮ID号
#define ONE_ID (1)
//左后轮ID号
#define TWO_ID (2)
//右前轮ID号
#define THREE_ID (3)
//左前轮转向ID号
#define ONE_TURNING_ID (4)
//右前轮转向ID号
#define TWO_TURNING_ID (5)
//左后轮转向ID号
#define THREE_TURNING_ID (6)

//左前轮与中心连线切线方向
#define ONE_VERTICAL_ANG (-90.0f)
//右前轮与中心连线切线方向
#define TWO_VERTICAL_ANG (30.0f)
//后轮与中心连线切线方向
#define THREE_VERTICAL_ANG (150.0f)

//
/**
  * @}
  */


/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/
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
* @brief  OutputVel2Wheel计算每个轮子的速度和朝向并输出到电机上
  * @note
* @param  vel:平移速度大小（mm/s）
		  direction:平移速度方向（-180°到180°）
		  omega:绕底盘中心旋转角速度(度/s),顺时针为负逆时针为正
  * @retval 
  */
void OutputVel2Wheel(float vel, float direction, float omega);

/**
* @brief  WheelVelControl控制电机速度和位置
  * @note
* @param  wheelVel:四个轮子速度大小和方向结构体
  * @retval 
  */
void WheelVelControl(wheel_t wheelVel);

/**
* @brief  SendCmd2Driver向电机发送速度和位置命令
  * @note
* @param  lfVel:左前轮速度
		  lfDir：左前轮方向
		  rfVel:右前轮速度
		  rfDir：右前轮方向
		  lrVel:左后轮速度
		  lrDir：左后轮方向
		  rrVel:右后轮速度
		  rrDir：右后轮方向

  * @retval 
  */
void SendCmd2Driver(float frontVel , float frontDir , float lrVel , float lrDir,
					float rrVel , float rrDir);
/**
* @brief  WheelAngle2PositionTransform将轮子朝向角度转化为脉冲
  * @note
* @param  angle:轮子朝向角度
* @retval 对应的电机脉冲位置
  */
int WheelAngle2PositionTransform(float angle);
/**
* @brief  WheelAngle2PositionTransform将轮子朝向角度转化为脉冲
  * @note
* @param  angle:轮子朝向角度
* @retval 对应的电机脉冲位置
  */
float WheelAngle2PositionInverseTransform(int position);
/**
* @brief  Transform2RobotCoodinate将世界坐标系下轮子朝向转换到机器人坐标系下
  * @note
* @param  wheelVel:要转换的轮子速度结构体指针
* @retval
  */
void Transform2RobotCoodinate(wheel_t * wheelVel);
/**
* @brief  Transform2WheelCoodinate将机器人坐标系下轮子朝向转换到轮子坐标系下
  * @note
* @param  wheelVel:要转换的轮子速度结构体指针
* @retval 
  */
void Transform2WheelCoodinate(wheel_t * wheelVel);
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
/**
* @brief  JudgeVelDirection判断轮子是否需要反转
  * @note
* @param  targetVel:目标速度大小和方向
		  actualAngle：当前轮子角度
* @retval 
  */
void JudgeVelDirection(wheelVel_t *targetVel , float actualAngle);
/**
* @brief  TurnInferiorArc确保旋转角度为劣弧
  * @note
* @param  targetAngle:目标角度
		  actualAngle：当前角度
* @retval 
  */
float TurnInferiorArc(float targetAngle , float actualAngle);
/**
* @brief  CalcWheelSpeed计算轮子的和速度大小和方向
  * @note
* @param  vel:平移速度大小（mm/s）
		  direction:平移速度方向（-180°到180°）
		  omega:绕底盘中心旋转角速度(度/s),顺时针为负逆时针为正
		  angleN:在机器人坐标系下旋转线速度的正方向(单位：度)
		  postureAngle：机器人姿态角
* @retval 
  */
wheelVel_t CalcWheelSpeed(float vel , float direction , float omega , float angleN, float postureAngle);
#endif /* ___H */

/************************ (C) COPYRIGHT NEU_ACTION_2017 *****END OF FILE****/
