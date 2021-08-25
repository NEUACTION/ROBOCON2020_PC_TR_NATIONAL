#include "posSystem.h"
#include "MotionCard.h"
#include <math.h>
#include <stdio.h>
#include "Bspline.h"
#include "robot.h"
#include "pathFollowing.h"
#include "send.h"

/***********************************************************************************
* @name 		CaculatePath
* @brief  	计算走过的里程
* @param  	
* @retval 	
**********************************************************************************/
static float lengthTwoWheel = 0.0f;
//0不记录
//1记录距离
//默认初始就开始记录距离
static _Bool CaculateLenFlag = 1;
float posXOld = 860.f,posYOld = 8903.6f;
//通过定位系统计算机器人行走的路径长度
void CaculatePath(void)
{
	float err = -0.02f;
	float errDirection = 0.0f;
	float reducePath = 0.0f;
	PointU_t virtualPosition = {0.0f};
	
	if(CaculateLenFlag == 1)
	{
		err = sqrt((GetX() - posXOld)*(GetX() - posXOld) + (GetY() - posYOld)*(GetY() - posYOld));
		if(fabs(err) > 500.f)
			gRobot.walkStatus = stop;
		if(err> 0.2f)
		{
			errDirection = CHANGE_TO_ANGLE*atan2f((GetY() - posYOld) , (GetX() - posXOld));
			virtualPosition = SerchVirtualPoint2(GetPath());
			err = err * cosf(CHANGE_TO_RADIAN * (errDirection - virtualPosition.direction));
	
			if(err>=0.0f)
			{
				if(err<50.0f)
				{
					lengthTwoWheel += err;
				}
				else
				{
					printf("ERR_TOO_BIG %d\r\n",(int)err);					
				}
			}
			if(lengthTwoWheel<=0.0f)
			{
				lengthTwoWheel = 0.0f;
			}
			posXOld = GetX();
			posYOld = GetY();
		}
	}
}

//运行路径长度记录
void UpdateLenBegin(void)
{
	CaculateLenFlag = 1;
}

//停止路径长度记录
void UpdateLenStop(void)
{
	CaculateLenFlag = 0;
}

//增加距离
void AddPath(float dis)
{
	lengthTwoWheel += dis;
}

//减小距离
void ReducePath(float dis)
{
	lengthTwoWheel -= (dis);
}

/***********************************************************************************
* @name 		GetPath
* @brief  	返回里程数
* @param  	无
* @retval 	lengthTwoWheel
**********************************************************************************/
float GetPath(void)
{
	return lengthTwoWheel;
}

/***********************************************************************************
* @name 		GetPosPresent
* @brief  	返回当前姿态
* @param  	无
* @retval 	
**********************************************************************************/
Pose_t GetPosPresent(void)
{
	Pose_t pos;
	pos.point.x = GetX();
	pos.point.y = GetY();
	pos.direction   = GetAngle();
	pos.vel = sqrt(GetSpeedWithoutOmega().x*GetSpeedWithoutOmega().x + GetSpeedWithoutOmega().y*GetSpeedWithoutOmega().y);
	return pos;
}

void ClearPathLen(void)
{
	posXOld = GetX();
	posYOld = GetY();
	lengthTwoWheel = 0.0f;
	UpdateLenBegin();
}
