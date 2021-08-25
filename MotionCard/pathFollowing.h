#ifndef _PATHFOLLOWING_H
#define _PATHFOLLOWING_H
#include "calculate.h"
#include "moveBase.h"

float AngleControl(float anglePresent,float angleTarget,float kp,float kd);

robotVel_t GetAdjustVel(Point_t robotPos,PointU_t adjustTarget,float vell);

void AdjustVel(float *carVel,float *velAngle,robotVel_t adjustVel);
void GoStraight(Point_t endPoint, float posTargetAngle);

#endif
