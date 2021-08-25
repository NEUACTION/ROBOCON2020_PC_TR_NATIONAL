#ifndef __ROBOT_H
#define __ROBOT_H

#include <stdint.h>
#include "calculate.h"
#include "moveBase.h"

//红场
#define RED_COURT (0)
//蓝场
#define BLUE_COURT (1)

#define NO_PLANNING (0)   
#define IS_PLANNING (1)

//调试数据结构体
typedef struct
{
	//PathFollowing轨迹跟随变量
	float VIEW_L;
	float robotlenVP;
	float robotlenVT;
	PointU_t virtualPos;
	PointU_t virtualTarget;
	float disRealPos2VirPos;
	float disRealPos2VirTarget;
	float disRealPos2VirPos2;
	float disAdd;
	float posAngleVP;
	float posAngleVT;
	float omega;
	float originVel;
	float originVelDir;
	float fixedVel;
	robotVel_t adjustVel;
	float sumVel;
	float sumVelDir;
	
	//VelControl速度环变量
	float velXErr;
	float velYErr;
	float outputVel;
	float outputDirection;
	float outputOmega;
	
}debugInfo_t;
//走行状态变量枚举类型变量
typedef enum
{
	waitForStart,
	
	goForFirstPath,

	waitForSecondPathGo,

	goForSecondPath,

	waitForSecondPathBack,

	backForSecondPath,
	
	waitForThirdPathGo,

	goForThirdPath,

	waitForThirdPathBack,

	backForThirdPath,

	waitForForthPathGo,

	goForForthPath,

	waitForForthPathBack,

	backForForthPath,

	waitForFifthPath,

	goForFifthPath,

	waitForSixthPath,

	goForSixthPath,

	waitForSeventhPath,

	goForSeventhPath,

	testPara,
	
	stop
}walkStatus_t;
//全局变量结构体
typedef struct
{
	//轮子状态
	wheelState_t wheelState;
	//调试数据
	debugInfo_t debugInfomation;
	//走行状态
	walkStatus_t walkStatus;
	//mcu通信心跳包
	int mcuHeart;
	//红蓝场
	uint8_t courdID;
	uint8_t pathPlanFlag;
	uint8_t nextFlag;
	uint8_t emergencyStopFlag;
}gRobot_t;

extern gRobot_t gRobot;
extern uint8_t ballCnt;
extern float posXOld,posYOld;
int GetTimeCounter(void);

void CountTime(void);

void SetCountTimeFlag(void);

void Walk(void);

uint8_t JudgeSpeedLessEqual(float speedCompared);

void VelControl(robotVel_t actVel);

void pathPlan(Pose_t *pathPoints, uint8_t *pathNum);
void adjustXPosition(void);
#endif
