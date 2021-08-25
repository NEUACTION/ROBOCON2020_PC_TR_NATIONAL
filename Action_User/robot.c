#include "robot.h"
#include "posSystem.h"
#include "path.h"
#include "send.h"
#include "pathFollowing.h"
#include "ringbuffer.h"
#include <math.h>
#include <unistd.h>
#include <stdio.h>
/*  ********************************************	计时	***************************************************** */
static int timeCounter = 0;
static uint8_t countTimeFlag = 0;
uint8_t pathFinishFlag = 0; 
uint8_t remoteFlag = 1;//遥控模式
uint8_t continuousNextFlag = 0;
uint8_t ballCnt = 0;
//float velPersent[10] = {0.8f, 0.78f, 0.75f, 0.81f ,0.79f, 0.76f, 0.73f, 0.8f, 0.58f,0.58f};
float velPersent[10] = {0.7f, 0.73f, 0.73f, 0.f ,0.f, 0.f, 0.f, 0.f, 0.f,0.f};
int GetTimeCounter(void)
{
	return timeCounter;
}

void CountTime(void)
{
	if(countTimeFlag)
	{
		timeCounter++;
	}
	timeCounter%=500000000;
}

void SetCountTimeFlag(void)
{
	countTimeFlag = 1;
}

/*  ********************************************Walk***************************************************** */
extern uint8_t mcuErrCnt;
static int count = 0;
void Walk(void)
{
	//连续通信失败20次 停下
	if( mcuErrCnt > 20)
	{
		printf(" MCU ERR! ");
		gRobot.walkStatus = stop;
	}
	if(gRobot.emergencyStopFlag == 1)
	{
		printf("emergencyStop!\r\nemergencyStop!\r\nemergencyStop!\r\n");
		gRobot.walkStatus = stop;
 	}	
	// if(ballCnt == 10)
	// {
	// 	gRobot.walkStatus = stop;
	// }
	if(remoteFlag == 1 && (gRobot.walkStatus == goForSecondPath ||gRobot.walkStatus == backForSecondPath) )
	{
		if(gRobot.nextFlag == 1 && JudgeSpeedLessEqual(50.0f) && pathFinishFlag==1)
		{
			pathFinishFlag = 0;
			continuousNextFlag = 1;
			gRobot.walkStatus ++;
			if(gRobot.walkStatus == waitForThirdPathGo)
				gRobot.walkStatus = waitForSecondPathGo;
			//gRobot.walkStatus = stop;
		}
		gRobot.nextFlag = 0;
	}
	switch(gRobot.walkStatus)
	{
		//等待触发0
		case waitForStart:
		{	
			//开始计时
			SetCountTimeFlag();
			OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);

			//触发后清除记录的轨迹长度
			ClearPathLen();	
			if(remoteFlag == 1)
			{
				if(gRobot.nextFlag == 1)
				{
					ClearPathLen();	
					gRobot.walkStatus = goForFirstPath;//开电后直接进入等待路径规划状态
					gRobot.nextFlag = 0;
				}
			}
			else
			{
				gRobot.walkStatus = goForFirstPath;
			}
			//gRobot.walkStatus = goForFirstPath;
			// if(gRobot.nextFlag == 1)
			// {
			// 	gRobot.nextFlag = 0;
			// 	gRobot.walkStatus = waitForSecondPathGo;//
			// 	//gRobot.walkStatus = stop;	
			// }					
			// if(GetX()> 150 || GetX()<-150 || GetY()> 150 || GetY()<-150)
			// {
			// 	printf("InitLocationWRONG!! \n");
			// 	// gRobot.walkStatus = stop;
			// }
			break;
		}
		//1
		case goForFirstPath:
		{
			float dis2FinalX = GetX() - firstPath[FIRST_PATH_NUM -1].point.x;
			float dis2FinalY = GetY() - firstPath[FIRST_PATH_NUM -1].point.y;
			float dis2FinalAngle = GetAngle() - firstPath[FIRST_PATH_NUM -1].direction;			
			if(sqrtf(dis2FinalX*dis2FinalX + dis2FinalY*dis2FinalY) < 50.0f && JudgeSpeedLessEqual(100.0f) \
				&& fabs(dis2FinalAngle) <= 2.f && fabs(GetWZ()) < 15.0f)
			{
				OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);				
				gRobot.walkStatus = waitForSecondPathGo;
				pathFinishFlag=0;
				//gRobot.walkStatus = stop;				
				return;
			}	
			if(GetX()>985.f)//走过中间
			{
				pathFinishFlag = 1;
			}			
			//跟随轨迹
			PathFollowing(velPersent[0]);
			//PathFollowing(0.3f);
			break;
		}
		//2
		case waitForSecondPathGo:
		{		
			gRobot.debugInfomation.robotlenVT = 0.f;	
			OutputVel2Wheel(0.0f,0.0f,0.0f);
			count++;
			if(count >= 15)
			{			
				if(remoteFlag == 1)
				{
					if(continuousNextFlag == 1)
					{
						continuousNextFlag = 0;
						gRobot.nextFlag =1;
					}
					if(gRobot.nextFlag == 1)
					{
						gRobot.pathPlanFlag = IS_PLANNING;
						Communicate2Mcu();
						ballCnt ++;
						count = 0;			
						ClearRingBuffer();
						secondPath[0].point.x = GetX();
						secondPath[0].point.y = GetY();
						secondPath[0].direction = GetAngle();
						InputPoints2RingBuffer(secondPath, SECOND_PATH_NUM);				
						OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);			
						ClearPathLen();
						gRobot.pathPlanFlag = NO_PLANNING;
						gRobot.walkStatus = goForSecondPath;
						// if(ballCnt == 10)
						// {
						// 	gRobot.walkStatus = stop;
						// }
						gRobot.nextFlag = 0;
						//gRobot.walkStatus = stop;
						break;
					}
				}
				else
				{
					gRobot.pathPlanFlag = IS_PLANNING;
					Communicate2Mcu();
					count = 0;			
					ClearRingBuffer();
					secondPath[0].point.x = GetX();
					secondPath[0].point.y = GetY();
					secondPath[0].direction = GetAngle();
					InputPoints2RingBuffer(secondPath, SECOND_PATH_NUM);				
					OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);			
					ClearPathLen();
					gRobot.pathPlanFlag = NO_PLANNING;
					gRobot.walkStatus = goForSecondPath;
					//gRobot.walkStatus = stop;
				}					
			}
			else
			{
				gRobot.nextFlag = 0;
			}
			break;
		}
		//3
		case goForSecondPath:
		{
			float dis2FinalX = GetX() - secondPath[SECOND_PATH_NUM-1].point.x;
			float dis2FinalY = GetY() - secondPath[SECOND_PATH_NUM-1].point.y;
			float dis2FinalAngle = GetAngle() - secondPath[SECOND_PATH_NUM-1].direction;
			
			if(GetY()<8100.f) //保证
			{
				pathFinishFlag=1;
			}
			if(fabsf(dis2FinalY) < 60.0f && fabsf(dis2FinalX) < 30.0f&& JudgeSpeedLessEqual(100.0f) \
				&& fabs(dis2FinalAngle) <= 3.f && fabs(GetWZ()) < 30.0f&&pathFinishFlag==1)
			{
				OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);				
				gRobot.walkStatus = waitForSecondPathBack;
				//gRobot.walkStatus = stop;
				pathFinishFlag=0;
				return;
			}	
			
			// if(GetX() > 5030.0f)
			// {
			// 	OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);				
			// 	gRobot.walkStatus = waitForSecondPathBack;
			// 	//gRobot.walkStatus = stop;
			// 	pathFinishFlag=0;
			// 	return;
			// }		
			//跟随轨迹
			PathFollowing(velPersent[1]);
			//PathFollowing(0.4f);
			break;
		}
		//4
		case waitForSecondPathBack:
		{
			gRobot.debugInfomation.robotlenVT = 0.f;	
			OutputVel2Wheel(0.0f,0.0f,0.0f);
			count++;
			if(count >= 10)
			{				
				if(remoteFlag == 1)
				{
					if(continuousNextFlag == 1)
					{
						continuousNextFlag = 0;
						gRobot.nextFlag =1;
					}
					if(gRobot.nextFlag == 1)
					{
						gRobot.pathPlanFlag = IS_PLANNING;
						ballCnt ++;
						Communicate2Mcu();
						count = 0;			
						ClearRingBuffer();
						secondPathBack[0].point.x = GetX();
						secondPathBack[0].point.y = GetY();
						secondPathBack[0].direction = GetAngle();
						InputPoints2RingBuffer(secondPathBack, SECOND_PATH_NUM_BACK);				
						OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);			
						ClearPathLen();
						gRobot.pathPlanFlag = NO_PLANNING;
						gRobot.walkStatus = backForSecondPath;
						gRobot.nextFlag = 0;
						// if(ballCnt == 10)
						// {
						// 	gRobot.walkStatus = stop;
						// }
						//gRobot.walkStatus = stop;
						break;
					}
				}
				else
				{
					gRobot.pathPlanFlag = IS_PLANNING;
					Communicate2Mcu();
					count = 0;			
					ClearRingBuffer();
					secondPath[0].point.x = GetX();
					secondPath[0].point.y = GetY();
					secondPath[0].direction = GetAngle();
					InputPoints2RingBuffer(secondPath, SECOND_PATH_NUM);				
					OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);			
					ClearPathLen();
					gRobot.pathPlanFlag = NO_PLANNING;
					gRobot.walkStatus = backForSecondPath;
					//gRobot.walkStatus = stop;
				}							
			}
			else
			{
				gRobot.nextFlag = 0;
			}
			break;
		}
		//5
		case backForSecondPath:
		{			
			float dis2FinalX = GetX() - secondPathBack[SECOND_PATH_NUM_BACK-1].point.x;
			float dis2FinalY = GetY() - secondPathBack[SECOND_PATH_NUM_BACK-1].point.y;
			float dis2FinalAngle = GetAngle() - secondPathBack[SECOND_PATH_NUM_BACK-1].direction;
			//起点和终点过近，判断是否走过路径，防止直接进入下一状态
			if(GetY()>8800.f)
			{
				pathFinishFlag=1;
			}
			if(sqrtf(dis2FinalX*dis2FinalX) < 30.0f && sqrtf(dis2FinalY*dis2FinalY) < 60.0f&& JudgeSpeedLessEqual(100.0f) \
				&& fabs(dis2FinalAngle) <= 3.f && fabs(GetWZ()) < 30.0f && pathFinishFlag==1)
			{
				OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);				
				gRobot.walkStatus = waitForSecondPathGo;
				//gRobot.walkStatus = stop;
				pathFinishFlag=0;
				return;
			}
			
			//跟随轨迹
			PathFollowing(velPersent[2]);
			//PathFollowing(0.4f);
			break;
		}
		//6
		case waitForThirdPathGo:
		{
			gRobot.debugInfomation.robotlenVT = 0.f;	
			gRobot.walkStatus = waitForSecondPathGo;
			break;
		}
		//7
		case goForThirdPath:
		{
			float dis2FinalX = GetX() - thirdPath[THIRD_PATH_NUM -1].point.x;
			float dis2FinalY = GetY() - thirdPath[THIRD_PATH_NUM -1].point.y;
			float dis2FinalAngle = GetAngle() - thirdPath[THIRD_PATH_NUM -1].direction;
			if(GetX()>4000.f)
			{
				pathFinishFlag=1;
			}
			// if(sqrtf(dis2FinalX*dis2FinalX + dis2FinalY*dis2FinalY) < 200.0f && JudgeSpeedLessEqual(200.0f) \
			// 	&& fabs(dis2FinalAngle) <= 2.0f && fabs(GetWZ()) < 10.0f && pathFinishFlag==1)
			// {
			// 	OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
			// 	pathFinishFlag=0;
			// 	gRobot.walkStatus = waitForForthPath;
			// 	//gRobot.walkStatus = stop;
			// 	return;
			// }
			if(GetX() > 5030.0f)
			{
				OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);				
				gRobot.walkStatus = waitForThirdPathBack;
				//gRobot.walkStatus = stop;
				pathFinishFlag=0;
				return;
			}				
			//跟随轨迹
			PathFollowing(velPersent[3]);
			//PathFollowing(0.4f);
			break;
		}
		//8
		case waitForThirdPathBack:
		{
			gRobot.pathPlanFlag = IS_PLANNING;
			Communicate2Mcu();
			OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
			count++;
			if(count >= 1)
			{		
				count = 0;				
				ClearRingBuffer();
				thirdPathBack[0].point.x = GetX();
				thirdPathBack[0].point.y = GetY();
				thirdPathBack[0].direction = GetAngle();//前6个在轨迹中都是180度，故改为当前角度
				thirdPathBack[1].direction = (thirdPathBack[1].direction == 180.f) ? GetAngle():thirdPathBack[1].direction;
				thirdPathBack[2].direction = (thirdPathBack[2].direction == 180.f) ? GetAngle():thirdPathBack[2].direction;
				thirdPathBack[3].direction = (thirdPathBack[3].direction == 180.f) ? GetAngle():thirdPathBack[3].direction;
				thirdPathBack[4].direction = (thirdPathBack[4].direction == 180.f) ? GetAngle():thirdPathBack[4].direction;
				thirdPathBack[5].direction = (thirdPathBack[5].direction == 180.f) ? GetAngle():thirdPathBack[5].direction;
				InputPoints2RingBuffer(thirdPathBack, THIRD_PATH_NUM_BACK);
				OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
				ClearPathLen();
				gRobot.pathPlanFlag = NO_PLANNING;		
				if(remoteFlag == 1)
				{
					if(gRobot.nextFlag == 1)
					{
						count = 0;				
						ClearRingBuffer();
						thirdPathBack[0].point.x = GetX();
						thirdPathBack[0].point.y = GetY();
						thirdPathBack[0].direction = GetAngle();//前6个在轨迹中都是180度，故改为当前角度
						thirdPathBack[1].direction = (thirdPathBack[1].direction == 180.f) ? GetAngle():thirdPathBack[1].direction;
						thirdPathBack[2].direction = (thirdPathBack[2].direction == 180.f) ? GetAngle():thirdPathBack[2].direction;
						thirdPathBack[3].direction = (thirdPathBack[3].direction == 180.f) ? GetAngle():thirdPathBack[3].direction;
						thirdPathBack[4].direction = (thirdPathBack[4].direction == 180.f) ? GetAngle():thirdPathBack[4].direction;
						thirdPathBack[5].direction = (thirdPathBack[5].direction == 180.f) ? GetAngle():thirdPathBack[5].direction;
						InputPoints2RingBuffer(thirdPathBack, THIRD_PATH_NUM_BACK);
						OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
						ClearPathLen();
						gRobot.pathPlanFlag = NO_PLANNING;
						gRobot.walkStatus = backForThirdPath;
						gRobot.nextFlag = 0;
						//gRobot.walkStatus = stop;
						break;
					}					
				}
				else
				{
					count = 0;				
					ClearRingBuffer();
					thirdPathBack[0].point.x = GetX();
					thirdPathBack[0].point.y = GetY();
					thirdPathBack[0].direction = GetAngle();//前6个在轨迹中都是180度，故改为当前角度
					thirdPathBack[1].direction = (thirdPathBack[1].direction == 180.f) ? GetAngle():thirdPathBack[1].direction;
					thirdPathBack[2].direction = (thirdPathBack[2].direction == 180.f) ? GetAngle():thirdPathBack[2].direction;
					thirdPathBack[3].direction = (thirdPathBack[3].direction == 180.f) ? GetAngle():thirdPathBack[3].direction;
					thirdPathBack[4].direction = (thirdPathBack[4].direction == 180.f) ? GetAngle():thirdPathBack[4].direction;
					thirdPathBack[5].direction = (thirdPathBack[5].direction == 180.f) ? GetAngle():thirdPathBack[5].direction;
					InputPoints2RingBuffer(thirdPathBack, THIRD_PATH_NUM_BACK);
					OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
					ClearPathLen();
					gRobot.pathPlanFlag = NO_PLANNING;
					gRobot.walkStatus = backForThirdPath;
					//gRobot.walkStatus = stop;
				}
				gRobot.walkStatus = backForThirdPath;
			}
			break;
		}
		//9
		case backForThirdPath:
		{
			float dis2FinalX = GetX() - thirdPathBack[THIRD_PATH_NUM_BACK -1].point.x;
			float dis2FinalY = GetY() - thirdPathBack[THIRD_PATH_NUM_BACK -1].point.y;
			float dis2FinalAngle = GetAngle() - thirdPathBack[THIRD_PATH_NUM_BACK -1].direction;
			if(GetX()<2000.f)
			{
				pathFinishFlag=1;
			}
			if(sqrtf(dis2FinalX*dis2FinalX) < 50.0f && sqrtf(dis2FinalY*dis2FinalY) < 300.0f && JudgeSpeedLessEqual(200.0f) \
				&& fabs(dis2FinalAngle) <= 2.0f && fabs(GetWZ()) < 10.0f && pathFinishFlag==1)
			{
				OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
				pathFinishFlag=0;
				gRobot.walkStatus = waitForForthPathGo;
				//gRobot.walkStatus = stop;
				return;
			}			
			//跟随轨迹
			PathFollowing(velPersent[4]);
			//PathFollowing(0.4f);
			break;
		}
		//10
		case waitForForthPathGo:
		{
			gRobot.pathPlanFlag = IS_PLANNING;
			Communicate2Mcu();
			OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
			count++;
			adjustXPosition();	
			if(count >= 1)
			{				
				if(remoteFlag == 1)
				{
					if(gRobot.nextFlag == 1)
					{
						count = 0;
						pathPlan(forthPath, &FORTH_PATH_NUM);
						gRobot.pathPlanFlag = NO_PLANNING;
						gRobot.nextFlag = 0;
						gRobot.walkStatus = goForForthPath;
						//gRobot.walkStatus = stop;
						break;
					}					
				}
				else
				{
					count = 0;				
					ClearRingBuffer();
					forthPath[0].point.x = GetX();
					forthPath[0].point.y = GetY();
					forthPath[0].direction = GetAngle();
					InputPoints2RingBuffer(forthPath, FORTH_PATH_NUM);
					OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
					ClearPathLen();
					gRobot.pathPlanFlag = NO_PLANNING;
					//gRobot.walkStatus = goForForthPath;
					gRobot.walkStatus = stop;
				}
			}
			break;
		}
		//11
		case goForForthPath:
		{
			float dis2FinalX = GetX() - forthPath[FORTH_PATH_NUM -1].point.x;
			float dis2FinalY = GetY() - forthPath[FORTH_PATH_NUM -1].point.y;
			float dis2FinalAngle = GetAngle() - forthPath[FORTH_PATH_NUM -1].direction;
			if(GetX()>4000.f)
			{
				pathFinishFlag=1;
			}
// 			if(sqrtf(dis2FinalX*dis2FinalX + dis2FinalY*dis2FinalY) < 200.0f && JudgeSpeedLessEqual(200.0f) \
// 				&& fabs(dis2FinalAngle) <= 2.0f && fabs(GetWZ()) < 10.0f&&
// pathFinishFlag==1)
// 			{
// 				OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
// 				pathFinishFlag=0;
// 				//gRobot.walkStatus = stop;
// 				gRobot.walkStatus = waitForFifthPath;
// 				return;
// 			}		
			if(GetX() > 5030.0f)
			{
				OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);				
				gRobot.walkStatus = waitForForthPathBack;
				//gRobot.walkStatus = stop;
				pathFinishFlag=0;
				return;
			}		
			//跟随轨迹
			PathFollowing(velPersent[5]);
			//PathFollowing(0.4f);
			break;
		}
		//12
		case waitForForthPathBack:
		{
			gRobot.pathPlanFlag = IS_PLANNING;
			Communicate2Mcu();
			OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
			count++;
			if(count >= 2)
			{
				count = 0;				
				ClearRingBuffer();
				forthPathBack[0].point.x = GetX();
				forthPathBack[0].point.y = GetY();
				forthPathBack[0].direction = GetAngle();//前6个在轨迹中都是180度，故改为当前角度
				forthPathBack[1].direction = (forthPathBack[1].direction == 180.f) ? GetAngle():forthPathBack[1].direction;
				forthPathBack[2].direction = (forthPathBack[2].direction == 180.f) ? GetAngle():forthPathBack[2].direction;
				forthPathBack[3].direction = (forthPathBack[3].direction == 180.f) ? GetAngle():forthPathBack[3].direction;
				forthPathBack[4].direction = (forthPathBack[4].direction == 180.f) ? GetAngle():forthPathBack[4].direction;
				forthPathBack[5].direction = (forthPathBack[5].direction == 180.f) ? GetAngle():forthPathBack[5].direction;
				InputPoints2RingBuffer(forthPathBack, FORTH_PATH_NUM_BACK);
				OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
				ClearPathLen();
				gRobot.pathPlanFlag = NO_PLANNING;
				if(remoteFlag == 1)
				{
					if(gRobot.nextFlag == 1)
					{
						count = 0;				
						ClearRingBuffer();
						forthPathBack[0].point.x = GetX();
						forthPathBack[0].point.y = GetY();
						forthPathBack[0].direction = GetAngle();//前6个在轨迹中都是180度，故改为当前角度
						forthPathBack[1].direction = (forthPathBack[1].direction == 180.f) ? GetAngle():forthPathBack[1].direction;
						forthPathBack[2].direction = (forthPathBack[2].direction == 180.f) ? GetAngle():forthPathBack[2].direction;
						forthPathBack[3].direction = (forthPathBack[3].direction == 180.f) ? GetAngle():forthPathBack[3].direction;
						forthPathBack[4].direction = (forthPathBack[4].direction == 180.f) ? GetAngle():forthPathBack[4].direction;
						forthPathBack[5].direction = (forthPathBack[5].direction == 180.f) ? GetAngle():forthPathBack[5].direction;
						InputPoints2RingBuffer(forthPathBack, FORTH_PATH_NUM_BACK);
						OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
						ClearPathLen();
						gRobot.pathPlanFlag = NO_PLANNING;
						gRobot.nextFlag = 0;
						gRobot.walkStatus = backForForthPath;
						//gRobot.walkStatus = stop;
						break;
					}					
				}
				else
				{
					count = 0;				
					ClearRingBuffer();
					forthPathBack[0].point.x = GetX();
					forthPathBack[0].point.y = GetY();
					forthPathBack[0].direction = GetAngle();//前6个在轨迹中都是180度，故改为当前角度
					forthPathBack[1].direction = (forthPathBack[1].direction == 180.f) ? GetAngle():forthPathBack[1].direction;
					forthPathBack[2].direction = (forthPathBack[2].direction == 180.f) ? GetAngle():forthPathBack[2].direction;
					forthPathBack[3].direction = (forthPathBack[3].direction == 180.f) ? GetAngle():forthPathBack[3].direction;
					forthPathBack[4].direction = (forthPathBack[4].direction == 180.f) ? GetAngle():forthPathBack[4].direction;
					forthPathBack[5].direction = (forthPathBack[5].direction == 180.f) ? GetAngle():forthPathBack[5].direction;
					InputPoints2RingBuffer(forthPathBack, FORTH_PATH_NUM_BACK);
					OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
					ClearPathLen();
					gRobot.pathPlanFlag = NO_PLANNING;
					gRobot.walkStatus = backForForthPath;
					//gRobot.walkStatus = stop;
				}
				gRobot.walkStatus = backForForthPath;
			}
			break;
		}
		//13
		case backForForthPath:
		{
			float dis2FinalX = GetX() - forthPathBack[FORTH_PATH_NUM_BACK -1].point.x;
			float dis2FinalY = GetY() - forthPathBack[FORTH_PATH_NUM_BACK -1].point.y;
			float dis2FinalAngle = GetAngle() - forthPathBack[FORTH_PATH_NUM_BACK -1].direction;
			if(GetX()<2000.f)
			{
				pathFinishFlag=1;
			}
			if(sqrtf(dis2FinalX*dis2FinalX) < 50.0f && sqrtf(dis2FinalY*dis2FinalY) < 300.0f && JudgeSpeedLessEqual(200.0f) \
				&& fabs(dis2FinalAngle) <= 2.0f && fabs(GetWZ()) < 10.0f&&
pathFinishFlag==1)
			{
				OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
				pathFinishFlag=0;
				//gRobot.walkStatus = stop;
				gRobot.walkStatus = waitForFifthPath;
				return;
			}			
			//跟随轨迹
			PathFollowing(velPersent[6]);
			//PathFollowing(0.4f);
			break;
		}
		//14
		case waitForFifthPath:
		{
			gRobot.pathPlanFlag = IS_PLANNING;
			Communicate2Mcu();
			OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
			count++;
			adjustXPosition();	
			if(count >= 1)
			{				
				if(remoteFlag == 1)
				{
					if(gRobot.nextFlag == 1)
					{	
						count = 0;
						pathPlan(fifthPath, &FIFTH_PATH_NUM);
						gRobot.pathPlanFlag = NO_PLANNING;						
						gRobot.nextFlag = 0;
						gRobot.walkStatus = goForFifthPath;
						//gRobot.walkStatus = stop;
						break;
					}					
				}
				else
				{
					count = 0;				
					ClearRingBuffer();
					fifthPath[0].point.x = GetX();
					fifthPath[0].point.y = GetY();
					fifthPath[0].direction = GetAngle();
					InputPoints2RingBuffer(fifthPath, FIFTH_PATH_NUM);
					OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
					ClearPathLen();
					gRobot.pathPlanFlag = NO_PLANNING;
					//gRobot.walkStatus = goForFifthPath;
					gRobot.walkStatus = stop;
				}
			}
			break;
		}
		//15
		case goForFifthPath:
		{
			float dis2FinalX = GetX() - fifthPath[FIFTH_PATH_NUM -1].point.x;
			float dis2FinalY = GetY() - fifthPath[FIFTH_PATH_NUM -1].point.y;
			float dis2FinalAngle = GetAngle() - fifthPath[FIFTH_PATH_NUM -1].direction;	
			if(GetX()>4000.f)
			{
				pathFinishFlag=1;
			}		
			// if(sqrtf(dis2FinalX*dis2FinalX + dis2FinalY*dis2FinalY) < 100.0f && JudgeSpeedLessEqual(200.0f) \
			// 	&& fabs(dis2FinalAngle) <= 2.0f && fabs(GetWZ()) < 10.0f)
			// {
			// 	OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);				
			// 	//gRobot.walkStatus = stop;	
			// 	gRobot.walkStatus = waitForSixthPath;
			// 	pathFinishFlag=0;			
			// 	return;
			// }		
			if(GetX() > 5030.0f)
			{
				OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);				
				gRobot.walkStatus = waitForSixthPath;
				//gRobot.walkStatus = stop;
				pathFinishFlag=0;
				return;
			}		
			//跟随轨迹
			PathFollowing(velPersent[7]);
			//PathFollowing(0.4f);
			break;
		}
		//16
		case waitForSixthPath:
		{
			gRobot.pathPlanFlag = IS_PLANNING;
			Communicate2Mcu();
			OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
			count++;
			if(count >=30)
			{			
				count = 0;				
				ClearRingBuffer();
				sixthPath[0].point.x = GetX();
				sixthPath[0].point.y = GetY();
				sixthPath[0].direction = GetAngle();
				InputPoints2RingBuffer(sixthPath, SIX_PATH_NUM);
				OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
				ClearPathLen();
				gRobot.pathPlanFlag = NO_PLANNING;	
				gRobot.nextFlag = 0;
				gRobot.walkStatus = goForSixthPath;	
			}
			break;
		}
		//17
		case goForSixthPath:
		{
			float dis2FinalX = GetX() - sixthPath[SIX_PATH_NUM -1].point.x;
			float dis2FinalY = GetY() - sixthPath[SIX_PATH_NUM -1].point.y;
			float dis2FinalAngle = GetAngle() - sixthPath[SIX_PATH_NUM -1].direction;	
			if(remoteFlag == 1)
			{
				if(gRobot.nextFlag == 1)
				{	
					gRobot.walkStatus = waitForSeventhPath;
					break;
				}					
			}		
			if(sqrtf(dis2FinalX*dis2FinalX + dis2FinalY*dis2FinalY) < 50.0f && JudgeSpeedLessEqual(200.0f) \
			 	&& fabs(dis2FinalAngle) <= 1.0f && fabs(GetWZ()) < 10.0f)
			{
			 	OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);		
				break;
			}
			//跟随轨迹
			PathFollowing(velPersent[8]);			
			break;
		}
		//18
		case waitForSeventhPath:
		{
			gRobot.pathPlanFlag = IS_PLANNING;
			Communicate2Mcu();
			OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
			count++;
			if(count > 1)
			{			
				// count = 0;	
				// ClearRingBuffer();
				// seventhPath[0].point.x = GetX();
				// seventhPath[0].point.y = GetY();
				// seventhPath[0].direction = GetAngle();
				// InputPoints2RingBuffer(seventhPath, SEVEN_PATH_NUM);
				// OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
				// ClearPathLen();
				// gRobot.pathPlanFlag = NO_PLANNING;
				// gRobot.nextFlag = 0;
				gRobot.walkStatus = goForSeventhPath;					

			}
			break;
		}
		//19
		case goForSeventhPath:
		{
			float dis2FinalX = GetX() - seventhPath[SEVEN_PATH_NUM -1].point.x;
			float dis2FinalY = GetY() - seventhPath[SEVEN_PATH_NUM -1].point.y;
			float dis2FinalAngle = GetAngle() - seventhPath[SEVEN_PATH_NUM -1].direction;			
			if(sqrtf(dis2FinalX*dis2FinalX + dis2FinalY*dis2FinalY) < 40.0f && JudgeSpeedLessEqual(200.0f) \
				&& fabs(dis2FinalAngle) <= 1.0f && fabs(GetWZ()) < 10.0f)
			{
				OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);	
				break;	
			}	
			GoStraight(seventhPath[SEVEN_PATH_NUM -1].point, seventhPath[SEVEN_PATH_NUM -1].direction);	
			//跟随轨迹
			//PathFollowing(velPersent[9]);
			break;
		}
		//20
		case testPara:
		{
			#define VEL (2000.0f)
			#define DIRECTION (0.0f)
			static uint8_t testStatus = 0;
			count++;

			count %= 10000;

			switch(testStatus)
			{
				case 0:
				{
					SendCmd2Driver(VEL,DIRECTION,VEL,DIRECTION,VEL,DIRECTION);

					if(fabs(GetX()>2500.0f || fabs(GetY()>2500.0f || count > 150)))
					{
						count = 0;
						testStatus = 1;
					}

					break;
				}
				case 1:
				{
					SendCmd2Driver(0.0f,DIRECTION,0.0f,DIRECTION,0.0f,DIRECTION);
					break;
				}
				default:
					break;
			}
			break;
		}
		//停止状态21
		case stop:
		{
			OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
			break;
		}
		default:
			break;
	}
}

/*  ********************************************Other Functions***************************************************** */
uint8_t JudgeSpeedLessEqual(float speedCompared)
{
	position_t actSpeed = GetSpeedWithoutOmega();
	float speedX = actSpeed.x;
	float speedY = actSpeed.y;
	
	float speed = sqrtf(speedX * speedX + speedY * speedY);
	
	if(speed>speedCompared)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}
//误差太大时调用此函数规划轨迹
void pathPlan(Pose_t *pathPoints, uint8_t *pathNum)
{			
	float subSlope[100] = {0};
	float minSlopp;
	int minSlopeindex = 0,i = 0;
	ClearRingBuffer();
	
	float xErr = pathPoints[0].point.x- GetX();		
	float yErr = pathPoints[0].point.y-GetY();
	pathPoints[0].point.x = GetX();
	pathPoints[0].point.y = GetY();
	pathPoints[0].direction = GetAngle();
	// pathPoints[0].point.x = 490;
	// pathPoints[0].point.y = 5695;
	// pathPoints[0].direction = 29.5;
	for(i = 0; i < *pathNum - 2; i ++) //斜率做差的绝对值
	{
		subSlope[i]= fabsf((pathPoints[i+2].point.y-pathPoints[i+1].point.y)/(pathPoints[i+2].point.x-pathPoints[i+1].point.x)-(pathPoints[i+1].point.y-pathPoints[0].point.y)/(pathPoints[i+1].point.x-pathPoints[0].point.x));
		//printf("%f %f %f %f %f %f %f \n ",subSlope[i],pathPoints[i+2].point.y,pathPoints[i+1].point.y,pathPoints[i+2].point.x,pathPoints[i+1].point.x,pathPoints[0].point.y,pathPoints[0].point.x);
	}
	minSlopp = subSlope[0];
	for(i = 0; i < *pathNum - 2; i ++)
	{
		//找最小值	
		if(subSlope[i] < minSlopp)
		{
			minSlopp = subSlope[i];
			minSlopeindex = i;
		}
		
	}
	printf("%d \n",minSlopeindex);
	//gRobot.walkStatus = waitForFifthPath;
	if(gRobot.walkStatus == waitForFifthPath)
	{
		minSlopeindex = 30;
		float deltaxErr = xErr/(minSlopeindex+1);		
		float deltayErr = yErr/(minSlopeindex+1);
		//printf("%f %f  FIFTHPATH\n",deltaxErr,deltayErr);
		for(i = 1; i < minSlopeindex+1; i++)
		{
			pathPoints[i].point.x = pathPoints[i].point.x - (minSlopeindex+1-i) * deltaxErr;
			pathPoints[i].point.y = pathPoints[i].point.y - (minSlopeindex+1-i) * deltayErr;
			printf("%f %f \n",pathPoints[i].point.x ,pathPoints[i].point.y);
		}
	}	
	else
	{
		//printf("FIFTHPATH\n");
		for(i = 1; minSlopeindex+i < *pathNum; i ++)
		{
			pathPoints[i].point.x = pathPoints[minSlopeindex+i].point.x;
			pathPoints[i].point.y = pathPoints[minSlopeindex+i].point.y;
			pathPoints[i].direction = pathPoints[minSlopeindex+i].direction;
			pathPoints[i].vel = pathPoints[minSlopeindex+i].vel;
		}
		*pathNum = i;
	}	
	InputPoints2RingBuffer(pathPoints, *pathNum);
	OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);
	ClearPathLen();
}
void adjustXPosition(void)
{
	if(GetX() > 580)
		OutputVel2Wheel(0.3f,GetRingBufferPointAngle(1),0.0f);
	else
		OutputVel2Wheel(0.0f,GetRingBufferPointAngle(1),0.0f);	
}

