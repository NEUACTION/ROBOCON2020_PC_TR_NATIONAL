/********************************************************************
*Copyright(C）2014-2016,沈阳艾克申机器人技术开发有限责任公司
*FileName：	   roundView.c
*Author：      Peng Xu
*Date：        2016/10/28
*Description： 圆形视野巡迹函数
*
*
*Version：     V1.0
*
********************************************************************/
#include "pathFollowing.h"
#include <math.h>
#include "ringbuffer.h"
#include "Bspline.h"
#include "posSystem.h"
#include "MotionCard.h"
#include "Move.h"
#include "stdint.h"
#include "robot.h"
#include "SpeedPlaning.h"
#include "send.h"
/*********************************************************************************
* @name 	PathFollowingNew
* @brief	路径跟随函数
* @param	percent 速度的百分比，若为1代表100%所规划的速度运行。范围为大于0,如果超过1，会超过机器人承受速度
* @retval	无
**********************************************************************************/
//调节量：按当前点到虚拟位置点作为偏差 提前30m到终点不调
//降速的量：TR第一段直线：垂直于速度方向的误差作为偏移量降速
//提前量：TR第一段直线:会减去 垂直于速度方向的误差 这个偏移量
int PathFollowing(float percent)
{
	static float vell = 150.0f;
	float angle1 = 0.0f;
	float angularVel = 0.0f;
	float angleErr = 0.0f;
	float posAngleVP = 0.0f;
	float posAngleVT = 0.0f;
	float robotlen = 0.0f;
	float disRealPos2VirTarget = 0.0f;
	float disRealPos2VirPos = 0.0f;
	float disRealPos2VirPos2 = 0.0f;
	robotVel_t adjustVel = {0.0f};
	PointU_t virtualPos,virtualTarget;

	//当前点与虚拟位置点距离 和 虚拟位置点到虚拟目标点距离之和
	float VIEW_L = 0.0f;
	if(gRobot.walkStatus == goForFirstPath)
	{
		//提前量
		VIEW_L = GetPosPresent().vel/50.0f*4.0f;
		// VIEW_L = vell/50.0f*5.9f;
	}
	else if(gRobot.walkStatus == goForSecondPath)
	{
		//提前量
		// VIEW_L = GetPosPresent().vel/50.0f*8.0f;
		//VIEW_L = GetPosPresent().vel/50.0f*4.8f;
		VIEW_L = vell/50.0f*2.6f;
	}
	else if(gRobot.walkStatus == backForSecondPath)
	{
		//提前量
		// VIEW_L = GetPosPresent().vel/50.0f*8.0f;
		//VIEW_L = GetPosPresent().vel/50.0f*4.8f;
		VIEW_L = vell/50.0f*2.5f;
	}
	else if(gRobot.walkStatus == goForThirdPath)
	{
		//提前量
		// VIEW_L = GetPosPresent().vel/50.0f*8.0f;
		VIEW_L = GetPosPresent().vel/50.0f*5.9f;
		// VIEW_L = vell/50.0f*5.9f;
	}
	else if(gRobot.walkStatus == backForThirdPath)
	{
		//提前量
		// VIEW_L = GetPosPresent().vel/50.0f*8.0f;
		VIEW_L = GetPosPresent().vel/50.0f*5.9f;
		// VIEW_L = vell/50.0f*5.9f;
	}
	else if(gRobot.walkStatus == goForForthPath)
	{
		//提前量
		// VIEW_L = GetPosPresent().vel/50.0f*8.0f;
		VIEW_L = GetPosPresent().vel/50.0f*5.8f;
		// VIEW_L = vell/50.0f*5.9f;
	}
	else if(gRobot.walkStatus == backForForthPath)
	{
		//提前量
		// VIEW_L = GetPosPresent().vel/50.0f*8.0f;
		VIEW_L = GetPosPresent().vel/50.0f*5.8f;
		// VIEW_L = vell/50.0f*5.9f;
	}
	else if(gRobot.walkStatus == goForFifthPath)
	{
		//提前量
		// VIEW_L = GetPosPresent().vel/50.0f*8.0f;
		VIEW_L = GetPosPresent().vel/50.0f*5.4f;
		// VIEW_L = vell/50.0f*5.9f;
	}
	else if(gRobot.walkStatus == goForSixthPath)
	{
		//提前量
		// VIEW_L = GetPosPresent().vel/50.0f*8.0f;
		VIEW_L = GetPosPresent().vel/50.0f*5.0f;
		// VIEW_L = vell/50.0f*5.9f;
	}
	else if(gRobot.walkStatus == goForSeventhPath)
	{
		//提前量
		VIEW_L = GetPosPresent().vel/50.0f*5.0f;
		// VIEW_L = vell/50.0f*5.9f;
	}
	else
	{
		//提前量
		VIEW_L = GetPosPresent().vel/50.0f*6.3f;
		// VIEW_L = vell/50.0f*5.9f;
	}
	
	if(VIEW_L < 10.0f)
	{
		VIEW_L = 10.0f;
	}

	if(percent < 0.0f || percent > 1.2f)
	{
		return -1;
	}
	
	//获取定位系统所计算的机器人实际行走路径长度
	robotlen = GetPath();
	
	gRobot.debugInfomation.robotlenVP = robotlen;

	//虚拟位置点
	virtualPos = SerchVirtualPoint(robotlen);
	
	//计算当前点到虚拟位置点的距离(直线距离)	
	disRealPos2VirPos = CalculatePoint2PointDistance(GetPosPresent().point,virtualPos.point);

	// if(gRobot.walkStatus == goForFirstPath)
	// {
	// 	disRealPos2VirPos2 = fabs(CalculateDisPointToLine3(GetPosPresent().point, virtualPos));
	// }
	// else
	// {
		disRealPos2VirPos2 = disRealPos2VirPos;
	// }
	
	gRobot.debugInfomation.disRealPos2VirPos2 == disRealPos2VirPos2;

	if(VIEW_L - disRealPos2VirPos2 >= 0.0f)
	{
		//加上 提前量与偏移量的差值
		robotlen = GetPath() + VIEW_L - disRealPos2VirPos2;
		if((gRobot.walkStatus == goForSecondPath || gRobot.walkStatus == backForSecondPath) && robotlen < gRobot.debugInfomation.robotlenVT)
		{
			robotlen = gRobot.debugInfomation.robotlenVT;
		}
	}
	
	gRobot.debugInfomation.robotlenVT = robotlen;
	
	//求取虚拟目标点
	virtualTarget = SerchVirtualPoint2(robotlen);
	
	//计算实际位置距离虚拟目标点距离
	disRealPos2VirTarget = CalculatePoint2PointDistance(GetPosPresent().point,virtualTarget.point);

	float disAdd = (VIEW_L - disRealPos2VirPos) - disRealPos2VirTarget;
	
	if(GetPath() < GetLength())
	{
		if(disAdd > 0)
		{
			AddPath(2*disAdd);
		}
	}
	else
	{
		//当记录的路程大于轨迹总路程后停止记录路程
		UpdateLenStop();
	}
	
	//虚拟位置点姿态角
	posAngleVP = CalculateAngleAdd(GetRingBufferPointPoseAngle(1),CalculateAngleSub(GetRingBufferPointPoseAngle(2) , GetRingBufferPointPoseAngle(1))*virtualPos.u);
	
	//两端点之间角度的插值
	angleErr = CalculateAngleSub(GetRingBufferPointPoseAngle(virtualTarget.endPtr) , GetRingBufferPointPoseAngle(virtualTarget.startPtr));
	
	//虚拟目标点姿态角
	posAngleVT = CalculateAngleAdd(GetRingBufferPointPoseAngle(virtualTarget.startPtr),angleErr*virtualTarget.u);

	if(gRobot.walkStatus == goForFirstPath)
	{
		if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT) < 0.5))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT, 3.5f, 2.5f);
		}
		else if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT) < 1))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT, 4.5f, 2.5f);
		}
		else if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<2))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT, 5.f, 2.5f);
		}
		else if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT) < 4))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT, 6.f, 2.5f);
		}
		else
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT, 8.0f, 2.5f);
	}
	else if(gRobot.walkStatus == goForSecondPath)
	 {
	// 	if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<1))
	// 	{
	// 		angularVel = AngleControl(GetPosPresent().direction,posAngleVT, 10.f, 8.5f);
	// 	}
	// 	else if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<2))
	// 	{
	// 		angularVel = AngleControl(GetPosPresent().direction,posAngleVT,12.5f, 11.5f);
	// 	}
	// 	else if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<4))
	// 	{
	// 		angularVel = AngleControl(GetPosPresent().direction,posAngleVT, 6.5f, 4.5f);
	// 	}
	// 	else
	// 	{
	// 		angularVel = AngleControl(GetPosPresent().direction,posAngleVT,6.5f, 4.5f);
	// 	}
		angularVel = AngleControl(GetPosPresent().direction,posAngleVT, 16.5f, 14.5f);		

		
	}
	else if(gRobot.walkStatus == backForSecondPath)
	{
		
		// if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<1))
		// {
		// 	angularVel = AngleControl(GetPosPresent().direction,posAngleVT, 10.f, 8.5f);
		// }
		// else if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<2))
		// {
		// 	angularVel = AngleControl(GetPosPresent().direction,posAngleVT,12.5f, 11.5f);
		// }
		// else if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<4))
		// {
		// 	angularVel = AngleControl(GetPosPresent().direction,posAngleVT, 6.5f, 4.5f);
		// }
		// else
		// {
		// 	angularVel = AngleControl(GetPosPresent().direction,posAngleVT,6.5f, 4.5f);
		// }
		// if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<3))
		// {
		// 	angularVel = AngleControl(GetPosPresent().direction,posAngleVT, 13.5f, 8.5f);
		// }
		// else
		// {
		// 	angularVel = AngleControl(GetPosPresent().direction,posAngleVT, 6.5f, 4.5f);
		// }
		angularVel = AngleControl(GetPosPresent().direction,posAngleVT, 16.5f, 14.5f);			
	}
	else if(gRobot.walkStatus == goForThirdPath)
	{
		if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<5))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,2.5f,5.5f);
		}
		else if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<10))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,3.5f,4.);
		}
		else if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<12))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,5.f,3.5f);
		}
		else if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<15))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,6.5f,3.0f);
		}
		else if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<20))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,10.f,3.0f);
		}
		else if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<25))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,13.f,3.0f);
		}
		else
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,16.0f,2.0f);
		// if(posAngleVT > 30.f && posAngleVT <35.0f) //终终点调节迅速终点点调节迅速
		// {
		// 	angularVel = AngleControl(GetPosPresent().direction,posAngleVT,9.5f,3.0f);
		// } 
		if(GetX()>4600.0f)
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,2.f,5.5f);
		}	
	}
	else if(gRobot.walkStatus == backForThirdPath)
	{
		if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<5))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,3.f,4.5f);
		}
		else if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<10))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,7.f,3.5f);
		}
		else if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<12))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,8.f,3.0f);
		}
		else if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<15))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,9.f,3.0f);
		}
		else if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<20))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,10.f,3.0f);
		}
		else if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<25))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,12.f,3.0f);
		}
		else
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,15.0f,2.0f);
		if(posAngleVT > 27.f && posAngleVT <37.0f) //终终点调节迅速终点点调节迅速 32
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,3.5f,5.0f);
		} 
		// if(GetX()>4600.0f)
		// {
		// 	angularVel = AngleControl(GetPosPresent().direction,posAngleVT,4.f,5.f);
		// }
		//第三段旋转
		//angularVel = AngleControl(GetPosPresent().direction,posAngleVT,4.0f,2.0f);		
	}
	else if(gRobot.walkStatus == goForForthPath)
	{
		if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<5))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,3.5f,6.5f);
		}
		else if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<10))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,4.5f,4.5f);
		}
		else if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<12))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,5.f,3.f);
		}
		else if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<15))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,6.f,2.5f);
		}
		else if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<25))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,10.f,2.f);
		}
		else
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,14.0f,2.0f);
		if(GetX()>4600.0f)
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,2.f,5.5f);
		}
	}
	else if(gRobot.walkStatus == backForForthPath)
	{
		if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<5))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,3.5f,5.5f);
		}
		else if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<10))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,4.5f,4.5f);
		}
		else if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<12))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,6.f,3.5f);
		}
		else if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<15))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,8.f,2.5f);
		}
		else if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<25))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,8.5f,2.5f);
		}
		else
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,12.0f,2.0f);
		if(posAngleVT > 25.f && posAngleVT <36.0f) //29.32
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,5.f,4.0f);
		} 
	}
	else if(gRobot.walkStatus == goForFifthPath)
	{
		if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<1.0f))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,3.0f,3.f);
		}
		else if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<3.0f))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,4.f,4.0f);
		}
		else if(fabsf(CalculateAngleSub(GetPosPresent().direction,posAngleVT)<5.0f))
		{
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,8.f,1.0f);
		}
		else
			angularVel = AngleControl(GetPosPresent().direction,posAngleVT,12.0f,1.f);
	}
	else if(gRobot.walkStatus == goForSixthPath)
	{
		angularVel = AngleControl(GetPosPresent().direction,posAngleVT,3.0f,1.f);
	}
	else if(gRobot.walkStatus == goForSeventhPath)
	{
		angularVel = AngleControl(GetPosPresent().direction,posAngleVT,3.0f,1.f);
	}
	else
	{
		angularVel = AngleControl(GetPosPresent().direction,posAngleVT,10.0f,3.0f);
	}	
	
	//目标方向
	angle1 = virtualTarget.direction;
	
	AngleLimit(&angle1);
	
	//目标速度
	vell = GetRingBufferPointVell(virtualTarget.startPtr)+(GetRingBufferPointVell(virtualTarget.endPtr) - GetRingBufferPointVell(virtualTarget.startPtr))*virtualTarget.u;
	
	vell = vell*percent;
	
	gRobot.debugInfomation.originVel = vell;
	gRobot.debugInfomation.originVelDir = angle1;
	
	//fix me 直线段应只将垂直于轨迹方向的分量作为偏移量
	//偏移量过大修正速度(适当降速)
	if(gRobot.walkStatus == goForFirstPath)
	{
		if(disRealPos2VirPos2>50.0f && disRealPos2VirPos2 < 130.0f)
		{
			if(vell*vell > 2.0f*(0.5f*GetAccLimit())*(disRealPos2VirPos2 - 50.0f))
			{			
				vell = vell*(1-0.005f*(disRealPos2VirPos2 - 50.f));
			}
			else
			{
				vell = 0.85f*vell;
			}
		}
		else if(disRealPos2VirPos2 >= 130.0f)
		{
			if(vell*vell > 2.0f*(0.5f*GetAccLimit())*(disRealPos2VirPos2 - 130.0f))
			{		
				vell = 0.6f*vell;
			}
			else
			{
				vell = 0.85f*vell;
			}
		}
	}
	else
	{
		if(disRealPos2VirPos2>80.0f && disRealPos2VirPos2 < 130.0f)
		{
			if(vell*vell > 2.0f*(0.5f*GetAccLimit())*(disRealPos2VirPos2 - 80.0f))
			{			
				vell = vell*(1-0.005f*(disRealPos2VirPos2 - 80.f));
			}
			else
			{
				vell = 0.85f*vell;
			}
		}
		else if(disRealPos2VirPos2 >= 130.0f)
		{
			if(vell*vell > 2.0f*(0.5f*GetAccLimit())*(disRealPos2VirPos2 - 130.0f))
			{		
				vell = 0.75f*vell;
			}
			else
			{
				vell = 0.85f*vell;
			}
		}
	}
	gRobot.debugInfomation.fixedVel = vell;
	
//	static float lastVell = 0.0f;

//	if(GetPath() < 15.f)
//	{
//		if(vell < lastVell)
//		{
//			vell = lastVell;
//		}
//	}
//	
//	lastVell = vell;

	//计算当前点到目标点的位置调节量
	adjustVel = GetAdjustVel(GetPosPresent().point,virtualPos,vell);

	//调节量与目标速度合成 距离终点很近时不调节
	if(GetPath() < (GetLength()-30.0f))
	{		
		AdjustVel(&vell,&angle1,adjustVel);
	}
	else
	{
		if(disRealPos2VirTarget > 50.0f)
		{
			AdjustVel(&vell,&angle1,adjustVel);
		}
	}

	float adjustPath = 0.0f;
	if(fabs(ReturnLimitAngle(virtualTarget.direction - adjustVel.direction)) > 90.0f)
	{
		adjustPath = (fabs(ReturnLimitAngle(virtualTarget.direction - adjustVel.direction)) - 90.0f) / 90.0f * 50.0f;
		AddPath(adjustPath);
	}

//	float time;
//	//该段样条曲线首尾的角度差
//	float angErr = GetRingBufferPointPoseAngle(2) - GetRingBufferPointPoseAngle(1);
//	angErr = angErr > 180.0f ? angErr - 360.0f : angErr; 
//	angErr = angErr < -180.0f ? 360.0f + angErr : angErr;
//	//粗略计算每两示教点之间的运动的时间
//	time = (GetRingBufferPointLen(2) - GetRingBufferPointLen(1)) / (GetRingBufferPointVell(2) + GetRingBufferPointVell(1)) * 2;

	SetTargetVel(vell,angle1,angularVel);
			
	gRobot.debugInfomation.VIEW_L = VIEW_L;
	gRobot.debugInfomation.virtualPos = virtualPos;
	gRobot.debugInfomation.virtualTarget = virtualTarget;
	gRobot.debugInfomation.disRealPos2VirPos = disRealPos2VirPos;
	gRobot.debugInfomation.disRealPos2VirPos2 = disRealPos2VirPos2;
	gRobot.debugInfomation.disRealPos2VirTarget = disRealPos2VirTarget;
	gRobot.debugInfomation.disAdd = disAdd;
	gRobot.debugInfomation.adjustVel = adjustVel;
	gRobot.debugInfomation.sumVel = vell;
	gRobot.debugInfomation.sumVelDir = angle1;
	gRobot.debugInfomation.posAngleVP = posAngleVP;
	gRobot.debugInfomation.posAngleVT = posAngleVT;
	//gRobot.debugInfomation.omega = angularVel;	
	
	return 1;
}
/*********************************************************************************
* @name 	AngleControl
* @brief	角度闭环控制程序
* @param	anglePresent 当前的角度 单位 度
* @param  angleTarget  目标角度   单位 度
* @retval	无
**********************************************************************************/
float AngleControl(float anglePresent,float angleTarget,float kp,float kd)
{
	//PD控制器
	float angleErr = 0.0f,angularVel = 0.0f, angularVelErr = 0.0f;
	static float angleErrRecord = 0.0f , dTermRecord = 0.0f;
	float dTerm = 0.0f;
	
	//目标角度减去当前角度
	angleErr = CalculateAngleSub(angleTarget,anglePresent);
	
	dTerm = (angleErr - angleErrRecord) * kd;

	//低通滤波
	angularVel = angleErr * kp + 0.5f*dTerm + 0.5f*dTermRecord;
	
	dTermRecord = dTerm;
	angleErrRecord = angleErr;
	gRobot.debugInfomation.omega = angularVel;
	angularVelErr = angularVel - GetWZ();
	if(gRobot.walkStatus == goForFirstPath)
	{
		angularVel = angularVel + angularVelErr * 0.22f;
	}
	else if(gRobot.walkStatus == goForSecondPath)
	{
		angularVel = angularVel + angularVelErr * 0.f;
	}
	else if(gRobot.walkStatus == backForSecondPath)
	{
		angularVel = angularVel + angularVelErr * 0.f;
	}
	else if(gRobot.walkStatus == goForThirdPath)
	{
		angularVel = angularVel + angularVelErr * 0.15f;
	}
	else if(gRobot.walkStatus == backForThirdPath)
	{
		angularVel = angularVel + angularVelErr * 0.15f;
	}
	else if(gRobot.walkStatus == goForForthPath)
	{
		angularVel = angularVel + angularVelErr * 0.18f;
	}
	else if(gRobot.walkStatus == backForForthPath)
	{
		angularVel = angularVel + angularVelErr * 0.18f;
	}
	//限幅
	if(angularVel>31.0f)
	{
		angularVel = 31.0f;
	}
	else if(angularVel < -31.0f)
	{
		angularVel = -31.0f;
	}
	
	gRobot.debugInfomation.outputOmega = angularVel;
	
	return angularVel;
}

robotVel_t GetAdjustVel(Point_t robotPos,PointU_t adjustTarget,float vell)
{
	#define MAX_ADJUST_VEL (500.0f)//限幅给小防止校正后震荡,former 1500
	robotVel_t adjustVel = {0};
	float distance = 0.0f;
	float angle = 0.0f;
	static float distanceRecord = 0.0f;
	float velX = 0, velY= 0;
	
	// //计算实际坐标点与调节目标点之间的距离 角度
	// if(gRobot.walkStatus == goForFirstPath)
	// {
	// 	distance = CalculateDisPointToLine3(robotPos,adjustTarget);		

	// 	if( (adjustTarget.direction == 90.0f && distance < 0.0f) || \
	// 		(adjustTarget.direction == -90.0f && distance > 0.0f) || \
	// 		(fabs(adjustTarget.direction) < 90.0f && distance > 0) || \
	// 		(fabs(adjustTarget.direction) > 90.0f && distance < 0) )
	// 	{
	// 		angle = adjustTarget.direction - 90.0f;
	// 	}
	// 	else
	// 	{
	// 		angle = adjustTarget.direction + 90.0f;
	// 	}

	// 	gRobot.debugInfomation.distance = distance;

	// 	distance = fabs(distance);
	// }
	// else
	// {
	// 	distance = CalculatePoint2PointDistance(robotPos,adjustTarget.point);

	// 	angle = CalculateLineAngle(robotPos,adjustTarget.point);
	// }

	angle = CalculateLineAngle(robotPos,adjustTarget.point);
	
	distance = CalculatePoint2PointDistance(robotPos,adjustTarget.point);
	
	//死区
	if(distance<5.0f)
	{
		distance = 0.0f;
	}
	else
	{
		distance-=5.0f;
	}
	
	if(distance<=0.0f)
	{
		distance = 0.0f;
	}

	distance = 0.5f*distance + 0.5f*distanceRecord;
	distanceRecord = distance;

	//计算调节速度大小和方向
	if(distance<=40.0f)
	{
		adjustVel.vel = distance * 2.5f;
//		adjustVel.vel = distance * 7.5f;
	}
	else if(distance<=200.0f)
	{
		adjustVel.vel = distance * 3.5f - 40.0f;
//		adjustVel.vel = distance * 4.5f + 120.0f;
	}
	else
	{
		adjustVel.vel = distance * 2.5f + 160.0f;
//		adjustVel.vel = distance * 4.0f + 320.0f;
	}
	
	if(gRobot.walkStatus == goForFirstPath)
	{
		adjustVel.vel = adjustVel.vel*1.5f;
	}
	else if(gRobot.walkStatus == goForSecondPath)
	{
		if(GetY()<7950.0f)
		{
			adjustVel.vel = adjustVel.vel*2.2f;
		}
		else
		{
			adjustVel.vel = adjustVel.vel*2.5f;	
		}
	}
	else if(gRobot.walkStatus == backForSecondPath)
	{
		if(GetY()>8850.0f)
		{
			adjustVel.vel = adjustVel.vel*2.2f;
		}		
		else
		{
			adjustVel.vel = adjustVel.vel*2.5f;	
		}
	}
	else if(gRobot.walkStatus == goForThirdPath)
	{
		if(GetX()>4600.0f)
		{
			adjustVel.vel = adjustVel.vel*1.2f;
		}
		else
		{
			adjustVel.vel = adjustVel.vel*2.4f;	
		}
		if(GetX()>2500.0f && GetX()<4400.0f && GetY()>2850.0f && GetY()<3200.0f)//坐标矫正的时候误差会突变，减小KP使轨迹变化平滑
		{
			adjustVel.vel = adjustVel.vel*1.0f;
		}

	}
	else if(gRobot.walkStatus == backForThirdPath)
	{
		if(GetX()>4600.0f)
		{
			adjustVel.vel = adjustVel.vel*1.2f;
		}
		else
		{
			adjustVel.vel = adjustVel.vel*2.4f;	
		}
		if(GetX()>2500.0f && GetX()<4400.0f && GetY()>2850.0f && GetY()<3200.0f)//坐标矫正的时候误差会突变，减小KP使轨迹变化平滑
		{
			adjustVel.vel = adjustVel.vel*1.0f;
		}

	}
	else if(gRobot.walkStatus == goForForthPath)
	{
		if(GetX()>4600.0f)
		{
			adjustVel.vel = adjustVel.vel*3.0f;
		}
		else
		{
			adjustVel.vel = adjustVel.vel*2.5f;	
		}
	}
	else if(gRobot.walkStatus == backForForthPath)
	{
		if(GetX()>4600.0f)
		{
			adjustVel.vel = adjustVel.vel*1.6f;
		}
		else
		{
			adjustVel.vel = adjustVel.vel*2.5f;	
		}
	}
	else if(gRobot.walkStatus == goForFifthPath)
	{
		if(GetX()>4600.0f)
		{
			adjustVel.vel = adjustVel.vel*1.5f;
		}
		else
		{
			adjustVel.vel = adjustVel.vel*1.8f;	
		}
	}
	else if(gRobot.walkStatus == goForSixthPath)
	{
		
		adjustVel.vel = adjustVel.vel*1.8f;	
	}
	else if(gRobot.walkStatus == goForSeventhPath)
	{
		
		adjustVel.vel = adjustVel.vel*1.1f;	
	}
	else
	{
		adjustVel.vel = adjustVel.vel*2.5f;
	}
	//暂时屏蔽
	//规划速度小于0.5m/s 不调节
	//if(vell < 500.f)
	// if(vell < 200.f)
	// {
	// 	adjustVel.vel = 0.0f;
	// }
	/*else if(vell < 1000.f)
	{
		adjustVel.vel = adjustVel.vel * 0.5f;
//		adjustVel.vel = 0.0f;
	}*/	
	adjustVel.direction = angle;
	//最后坐标矫正时让车的y方向不调节
	velX = adjustVel.vel * cosf(angle*CHANGE_TO_RADIAN);
	velY = adjustVel.vel * sinf(angle*CHANGE_TO_RADIAN);
	//其余几段不调
	if(robotPos.x < 800 && gRobot.walkStatus != goForFirstPath)
	{
		velY = 0.0 * velY;
		adjustVel.vel = sqrt(velX * velX + velY * velY);
		adjustVel.direction = atan2(velY ,velX) * CHANGE_TO_ANGLE; 
	} 
	if(robotPos.x < 520 && gRobot.walkStatus != goForFirstPath)
	{
		velX = 0.0 * velX;
		adjustVel.vel = sqrt(velX * velX + velY * velY);
		adjustVel.direction = atan2(velY ,velX) * CHANGE_TO_ANGLE; 
	} 
	
	//对调节速度大小进行限制
	if(adjustVel.vel>=MAX_ADJUST_VEL)
	{
		adjustVel.vel = MAX_ADJUST_VEL;
	}
	return adjustVel;
}

void AdjustVel(float *carVel,float *velAngle,robotVel_t adjustVel)
{
	#define ADJUST_KP (1.f)
	
	float angleErr = 0.0f;
	float projectOnvel = 0.0f;
	vector_t oriVel = {0} , adjust = {0} , result = {0};
	
	oriVel.module = *carVel;
	oriVel.direction = *velAngle;
	
	//计算调节目标速度的大小
	adjust.module = ADJUST_KP * adjustVel.vel;
	
//	if(oriVel.module>(10.0f * adjust.module))
//	{
//		adjust.module = oriVel.module/10.0f;
//	}
	
//	if(adjust.module>*carVel)
//	{
//		adjust.module = *carVel;
//	}
	
	//计算速度方向的调节量并进行限制
	angleErr = adjustVel.direction - *velAngle;
	
	angleErr = angleErr > 180.0f ? angleErr - 360.0f : angleErr; 
	angleErr = angleErr < -180.0f ? 360.0f + angleErr : angleErr;

	angleErr =  ADJUST_KP * angleErr;
	if(angleErr>180.0f)
	{
		angleErr = 180.0f;
	}
	else if(angleErr<-180.0f)
	{
		angleErr = -180.0f;
	}
	
	adjust.direction = *velAngle + angleErr;
	
	adjust.direction = adjust.direction > 180.0f ? adjust.direction - 360.0f : adjust.direction; 
	adjust.direction = adjust.direction < -180.0f ? 360.0f + adjust.direction : adjust.direction;
	
	
	//计算调节量在原速度上的投影大小
	projectOnvel = CalculateVectorProject(adjust,oriVel);
	
	// //根据投影大小限制调节后的速度方向
	// if(projectOnvel<=-oriVel.module)
	// {
	// 	adjust = CalculateVectorFromProject(-oriVel.module,adjust.direction,oriVel.direction);
	// }
	
	result = CalculateVectorAdd(oriVel , adjust);
	//对调整结果的速度大小进行限制
	if(result.module>=GetVelMax())
	{
		result.module=GetVelMax();
	}
	
	*carVel = result.module;
	*velAngle = result.direction;
}

//三轮直线
void GoStraight(Point_t endPoint, float posTargetAngle)
{
	PointU_t virtualPos = {0};
	robotVel_t adjustVel;
	//末端速度设为0
	float vell=0;// = sqrt((endPoint.x - GetPosPresent().point.x) * (endPoint.x - GetPosPresent().point.x) + (endPoint.y - GetPosPresent().point.y) * (endPoint.y - GetPosPresent().point.y));
	//计算角速度
	float angularVel = AngleControl(GetPosPresent().direction,posTargetAngle,4.0f,1.f);
	//目标速度方向
	float targetDir = atan2f(endPoint.y-GetY(),endPoint.x-GetX())* CHANGE_TO_ANGLE;

	virtualPos.point.x = endPoint.x;
	virtualPos.point.y = endPoint.y;
	virtualPos.direction = targetDir;

	//位置调节量
	adjustVel = GetAdjustVel(GetPosPresent().point,virtualPos,vell);
	//速度合成
	AdjustVel(&vell,&targetDir,adjustVel);
	SetTargetVel(vell,targetDir,angularVel);
}
