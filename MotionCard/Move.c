#include "Move.h"
#include "math.h"
#include "calculate.h"
#include "moveBase.h"
#include "robot.h"
#include "timer.h"

#define ALPHA 45.0f 

static robotVel_t targetVel = {0.0f};

void SetTargetVel(float vel,float velDir,float omega)
{
	targetVel.vel = vel;
	targetVel.direction = velDir;
	targetVel.omega = omega;
	//发送两个信号量
	gSem.velctrlCmdSem = VELCTRL_CMD_SEM_SEND;
}

void VelControl(robotVel_t actVel)
{
	robotVel_t velErr = {0.0f};
	robotVel_t outputVel = {0.0f};
	float velXErr , velYErr = 0.0f;
	float velXOutput , velYOutput = 0.0f;
	float KpX = 1.0f , KpY = 1.0f;
	float expVelX , expVelY = 0.0f;
	float actVelX , actVelY = 0.0f;
	
	expVelX = targetVel.vel*cosf(targetVel.direction*CHANGE_TO_RADIAN);
	expVelY = targetVel.vel*sinf(targetVel.direction*CHANGE_TO_RADIAN);
	
	actVelX = actVel.vel*cosf(actVel.direction*CHANGE_TO_RADIAN);
	actVelY = actVel.vel*sinf(actVel.direction*CHANGE_TO_RADIAN);
	
	velXErr = expVelX - actVelX;
	velYErr = expVelY - actVelY;

#ifdef SEND_MOVEBASE_DEBUGINFO
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d %d ", (int)(expVelX) , (int)(expVelY));
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d %d ", (int)(actVelX) , (int)(actVelY));
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d %d T", (int)(velXErr) , (int)(velYErr));
#endif

	
				
	if(gRobot.walkStatus == goForFirstPath)
	{
		if(expVelX*actVelX>0.0f&&fabs(expVelX)>fabs(actVelX))
		{
			KpX = 1.0f;
		}
		else
		{
			KpX = 1.2f;
		}
		if(expVelY*actVelY>0.0f&&fabs(expVelY)>fabs(actVelY))
		{
			KpY = 1.1f;
		}
		else
		{
			KpY = 1.5f;
		}
	}
	else if(gRobot.walkStatus == goForSecondPath)
	{
		if(expVelX*actVelX>0.0f&&fabs(expVelX)>fabs(actVelX))
		{
			KpX = 1.0f;
		}
		else
		{
			KpX = 1.0f;
		}
		if(expVelY*actVelY>0.0f&&fabs(expVelY)>fabs(actVelY))
		{
			KpY = 1.1f;
		}
		else
		{
			KpY = 1.2f;
		}
	}
	else if(gRobot.walkStatus == backForSecondPath)
	{
		if(expVelX*actVelX>0.0f&&fabs(expVelX)>fabs(actVelX))
		{
			KpX = 1.0f;
		}
		else
		{
			KpX = 1.0f;
		}
		if(expVelY*actVelY>0.0f&&fabs(expVelY)>fabs(actVelY))
		{
			KpY = 1.1f;
		}
		else
		{
			KpY = 1.2f;
		}
	}
	else if(gRobot.walkStatus == goForThirdPath)
	{
		if(expVelX*actVelX>0.0f&&fabs(expVelX)>fabs(actVelX))
		{
			KpX = 1.0f;
		}
		else
		{
			KpX = 1.0f;
		}
		if(expVelY*actVelY>0.0f&&fabs(expVelY)>fabs(actVelY))
		{
			KpY = 1.0f;
		}
		else
		{
			KpY = 1.0f;
		}
	}
	else if(gRobot.walkStatus == backForThirdPath)
	{
		if(expVelX*actVelX>0.0f&&fabs(expVelX)>fabs(actVelX))
		{
			KpX = 1.0f;
		}
		else
		{
			KpX = 1.0f;
		}
		if(expVelY*actVelY>0.0f&&fabs(expVelY)>fabs(actVelY))
		{
			KpY = 1.0f;
		}
		else
		{
			KpY = 1.0f;
		}
	}
	else if(gRobot.walkStatus == goForForthPath)
	{
		if(expVelX*actVelX>0.0f&&fabs(expVelX)>fabs(actVelX))
		{
			KpX = 1.0f;
		}
		else
		{
			KpX = 1.0f;
		}
		if(expVelY*actVelY>0.0f&&fabs(expVelY)>fabs(actVelY))
		{
			KpY = 1.0f;
		}
		else
		{
			KpY = 1.0f;
		}
	}
	else if(gRobot.walkStatus == backForForthPath)
	{
		if(expVelX*actVelX>0.0f&&fabs(expVelX)>fabs(actVelX))
		{
			KpX = 1.0f;
		}
		else
		{
			KpX = 1.0f;
		}
		if(expVelY*actVelY>0.0f&&fabs(expVelY)>fabs(actVelY))
		{
			KpY = 1.0f;
		}
		else
		{
			KpY = 1.0f;
		}
	}
	else if(gRobot.walkStatus == goForFifthPath)
	{
		if(expVelX*actVelX>0.0f&&fabs(expVelX)>fabs(actVelX))
		{
			KpX = 1.0f;
		}
		else
		{
			KpX = 1.0f;
		}
		if(expVelY*actVelY>0.0f&&fabs(expVelY)>fabs(actVelY))
		{
			KpY = 1.0f;
		}
		else
		{
			KpY = 1.0f;
		}
	}
	else if(gRobot.walkStatus == goForSixthPath)
	{
		if(expVelX*actVelX>0.0f&&fabs(expVelX)>fabs(actVelX))
		{
			KpX = 1.0f;
		}
		else
		{
			KpX = 1.0f;
		}
		if(expVelY*actVelY>0.0f&&fabs(expVelY)>fabs(actVelY))
		{
			KpY = 1.0f;
		}
		else
		{
			KpY = 1.0f;
		}
	}
	else
	{
		if(expVelX*actVelX>0.0f&&fabs(expVelX)>fabs(actVelX))
		{
			KpX = 1.0f;
		}
		else
		{
			KpX = 1.0f;
		}
		if(expVelY*actVelY>0.0f&&fabs(expVelY)>fabs(actVelY))
		{
			KpY = 1.0f;
		}
		else
		{
			KpY = 1.0f;
		}
	}	
	//限幅&死区
	if(fabs(velXErr)>=1500.0f)
	{
		velXErr= velXErr / fabs(velXErr) * 1400.0f;
	}
	else
	{
		if(fabs(expVelX)<100.0f)
		{
			if(fabs(velXErr)<=50.0f)
			{
				velXErr = 0.0f;
			}
		}
		else if(fabs(velXErr)<=50.0f)
		{
			velXErr = 0.0f;
		}		
	}

	
	if(fabs(velYErr)>=1500.0f)
	{
		velYErr= velYErr / fabs(velYErr) * 1400.0f;
	}
	else
	{
		if(fabs(expVelY)<100.0f)
		{
			if(fabs(velYErr)<=50.0f)
			{
				velYErr = 0.0f;
			}
		}
		else if(fabs(velYErr)<=50.0f)
		{
			velYErr = 0.0f;
		}		
	}

	velXErr*=KpX;
	velYErr*=KpY;
	if(gRobot.walkStatus == goForFirstPath)//针对第一段翘头进行速度增量限幅
	{
		velYErr = velYErr>900?900:velYErr;
	}
	gRobot.debugInfomation.velXErr = velXErr;
	gRobot.debugInfomation.velYErr = velYErr;

	velErr.vel = sqrtf(velXErr*velXErr + velYErr*velYErr);
	
	velErr.direction = atan2f(velYErr,velXErr)*CHANGE_TO_ANGLE;
	
	if(velErr.vel>=2000.0f)
	{
		velErr.vel = 2000.0f;
	}
	
	if(velErr.vel<50.0f)
	{
		velErr.vel = 0.0f;
		velErr.direction = targetVel.direction;
	}
	
	//fixme 如果发生抖动，小目标速度下可直接不闭环
	if(targetVel.vel<300.0f)
	{
		if(velErr.vel > (targetVel.vel*0.8f))
		{
			velErr.vel = velErr.vel * 0.5f;
		}	
	}
	
	velXOutput = velErr.vel*cosf(velErr.direction*CHANGE_TO_RADIAN) + targetVel.vel*cosf(targetVel.direction*CHANGE_TO_RADIAN);
	velYOutput = velErr.vel*sinf(velErr.direction*CHANGE_TO_RADIAN) + targetVel.vel*sinf(targetVel.direction*CHANGE_TO_RADIAN);
	
	outputVel.vel = sqrtf(velXOutput*velXOutput + velYOutput*velYOutput);
	outputVel.direction = atan2f(velYOutput , velXOutput)*CHANGE_TO_ANGLE;
	
	
	if(outputVel.vel < 1.0f)
	{
		outputVel.direction=targetVel.direction;
	}
	
	if(outputVel.vel>=GetVelMax())
	{
		outputVel.vel = GetVelMax();
	}
	
#ifdef SEND_MOVEBASE_DEBUGINFO
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d %d ", (int)(velXErr) , (int)(velYErr));
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d %d ", (int)(targetVel.vel) , (int)(targetVel.direction));
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d %d ", (int)(targetVel.vel*cosf(targetVel.direction*CHANGE_TO_RADIAN)) , (int)(targetVel.vel*sinf(targetVel.direction*CHANGE_TO_RADIAN)));
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d %d pp ", (int)(outputVel.vel) , (int)(outputVel.direction));
#endif	
	
	gRobot.debugInfomation.outputVel = outputVel.vel;
	gRobot.debugInfomation.outputDirection = outputVel.direction;

	OutputVel2Wheel(outputVel.vel, outputVel.direction , targetVel.omega);
	
}

//三轮控制函数
//speed     单位mm/s
//direction 速度的方向  单位 度
//rotationVell 旋转速度 单位 度/s 
//void ThreeWheelVelControl(float speed, float direction, float rotationVell)
//{

//	TriWheelVel_t vell;
//	float Vx, Vy;
//	float theta;
//	float robotR = 0.0f;
//	static uint8_t outputErrCounter[3] = {0};
//	
//	speed*=GAIN_COMPENSATION;
//	
//	rotationVell*=GAIN_COMPENSATION;
//	
//	if(speed>(GetVelMax()+1000.0f))
//	{
//		speed = GetVelMax();
//		outputErrCounter[0]++;
//		if(outputErrCounter[0]>10)
//		{
//			outputErrCounter[0] = 11;

//			debugInfo.opsStatus|=0x10;
//			USART_OUT(DEBUG_USART,"\r\n2\r\n");
//		}
//	}
//	
//	if(rotationVell>200.0f)
//	{
//		rotationVell = 200.0f;
//		outputErrCounter[1]++;
//		if(outputErrCounter[1]>10)
//		{
//			outputErrCounter[1]=11;
//			USART_OUT(DEBUG_USART,"\r\n3\r\n");

//			debugInfo.opsStatus|=0x20;	
//		}
//	}
//	else if(rotationVell < -200.0f)
//	{
//		rotationVell = -200.0f;
//		outputErrCounter[2]++;
//		if(outputErrCounter[2]>10)
//		{
//			USART_OUT(DEBUG_USART,"\r\n4\r\n");
//			outputErrCounter[2]=11;

//			debugInfo.opsStatus|=0x40;	
//		}
//	}
//	
//	robotR = 100.0f;
//	rotationVell = rotationVell / CHANGE_TO_ANGLE;
//	Vx = speed * arm_cos_f32(direction * CHANGE_TO_RADIAN);
//	Vy = speed * arm_sin_f32(direction * CHANGE_TO_RADIAN);

//	gRobot.speed.aimX=Vx;
//	gRobot.speed.aimY=Vy;
//	
//	if(isnan(speed))
//		USART_OUT(USART2,"\tspnan\t");
//	if(isnan(direction))
//		USART_OUT(DEBUG_USART,"\tdinan\t");
//	
//	theta = GetAngleZ();
//	
//	vell.v1 = (float)(-arm_cos_f32((ALPHA + theta) * CHANGE_TO_RADIAN) * Vx - arm_sin_f32((ALPHA + theta) * CHANGE_TO_RADIAN) * Vy + rotationVell * robotR + 1 * 0);
//	vell.v2 = (float)( arm_cos_f32((ALPHA - theta) * CHANGE_TO_RADIAN) * Vx - arm_sin_f32((ALPHA - theta) * CHANGE_TO_RADIAN) * Vy + rotationVell * robotR + 2 * 0);
//	vell.v3 = (float)( arm_cos_f32((ALPHA + theta) * CHANGE_TO_RADIAN) * Vx + arm_sin_f32((ALPHA + theta) * CHANGE_TO_RADIAN) * Vy + rotationVell * robotR + 3 * 0);
//	vell.v4 = (float)(-arm_cos_f32((ALPHA - theta) * CHANGE_TO_RADIAN) * Vx + arm_sin_f32((ALPHA - theta) * CHANGE_TO_RADIAN) * Vy + rotationVell * robotR + 4 * 0);
//	
//	debugInfo.expCarVel.carVel = speed;
//	debugInfo.expCarVel.velAngle = direction;
//	debugInfo.expAngularVel = rotationVell * CHANGE_TO_ANGLE;
//	
//	VelControlTriWheel(vell.v1,vell.v2,vell.v3,vell.v4);
//	
//}

//void VelControlTriWheel(float v1,float v2,float v3,float v4)
//{
//		gRobot.wheelVelWant.wheelVel1Want = v1;
//		gRobot.wheelVelWant.wheelVel2Want = v2;
//		gRobot.wheelVelWant.wheelVel3Want = v3;
//		gRobot.wheelVelWant.wheelVel4Want = v4;
//	
//		debugInfo.wheelExpVel.wheel1 = Vel2Pulse(v1);
//		debugInfo.wheelExpVel.wheel2 = Vel2Pulse(v2);
//		debugInfo.wheelExpVel.wheel3 = Vel2Pulse(v3);
//		debugInfo.wheelExpVel.wheel4 = Vel2Pulse(v4);
//	
//	
//		VelCrl(CAN1, 4, Vel2Pulse(v1));
//		VelCrl(CAN1, 1, Vel2Pulse(v2));
//		VelCrl(CAN1, 2, Vel2Pulse(v3));
//		VelCrl(CAN1, 3, Vel2Pulse(v4));
//}

//返回三个轮子轮速
//speed       单位mm/s
//direction 速度的方向  单位 度
//rotationVell 旋转速度 单位 度/s 
//posAngle 机器人的姿态  单位 度
//TriWheelVel_t CaculateThreeWheelVel(float speed, float direction, float rotationVell,float angleZ)
//{
//	TriWheelVel_t vell;
//	float Vx, Vy;
//	float theta;
//	float robotR = 0.0f;
//	
//	robotR = 100.0f;
//	rotationVell = rotationVell / CHANGE_TO_ANGLE;
//	Vx = speed * arm_cos_f32(direction * CHANGE_TO_RADIAN);
//	Vy = speed * arm_sin_f32(direction * CHANGE_TO_RADIAN);

//	theta = angleZ;

//	vell.v1 = (float)(-arm_cos_f32((ALPHA + theta) * CHANGE_TO_RADIAN) * Vx - arm_sin_f32((ALPHA + theta) * CHANGE_TO_RADIAN) * Vy + rotationVell * robotR + 1 * 0);
//	vell.v2 = (float)( arm_cos_f32((ALPHA - theta) * CHANGE_TO_RADIAN) * Vx - arm_sin_f32((ALPHA - theta) * CHANGE_TO_RADIAN) * Vy + rotationVell * robotR + 2 * 0);
//	vell.v3 = (float)( arm_cos_f32((ALPHA + theta) * CHANGE_TO_RADIAN) * Vx + arm_sin_f32((ALPHA + theta) * CHANGE_TO_RADIAN) * Vy + rotationVell * robotR + 3 * 0);
//	vell.v4 = (float)(-arm_cos_f32((ALPHA - theta) * CHANGE_TO_RADIAN) * Vx + arm_sin_f32((ALPHA - theta) * CHANGE_TO_RADIAN) * Vy + rotationVell * robotR + 4 * 0);

//	return vell;
//}



///*************************************************************************************
//* @name        GetTrueVEll
//* @param      brief       由三轮的转速计算实际的速度
//* @param      wheelVell   三轮的速度
//*
//*
//*************************************************************************************/
//TriWheelVel2_t GetTrueVell(TriWheelVel_t wheelVell, float zAngle)
//{
//	float **B; 
//	float **M;
//	TriWheelVel2_t trueVell;

//	B = CreateMemory(4,4);
//	M = CreateMemory(4,4);

//	float robotR = 100.0f;
//	M[0][0] = -arm_cos_f32((45 + zAngle) * CHANGE_TO_RADIAN);
//	M[0][1] = -arm_sin_f32((45 + zAngle) * CHANGE_TO_RADIAN);
//	M[0][2] = robotR;
//	M[0][3] = 1;
//	M[1][0] =  arm_cos_f32((45 - zAngle) * CHANGE_TO_RADIAN);
//	M[1][1] = -arm_sin_f32((45 - zAngle) * CHANGE_TO_RADIAN);
//	M[1][2] = robotR;
//	M[1][3] = 2;
//	M[2][0] =  arm_cos_f32((45 + zAngle) * CHANGE_TO_RADIAN); 
//	M[2][1] =  arm_sin_f32((45 + zAngle) * CHANGE_TO_RADIAN);
//	M[2][2] = robotR;
//	M[2][3] = 3;
//	M[3][0] = -arm_cos_f32((45 - zAngle) * CHANGE_TO_RADIAN);
//	M[3][1] =  arm_sin_f32((45 - zAngle) * CHANGE_TO_RADIAN);
//	M[3][2] = robotR;
//	M[3][3] = 4;

//	Gauss(M, B, 4);

//	float xVell = B[0][0] * wheelVell.v1 + B[0][1] * wheelVell.v2 + B[0][2] * wheelVell.v3 + B[0][3] * wheelVell.v4;
//	float yVell = B[1][0] * wheelVell.v1 + B[1][1] * wheelVell.v2 + B[1][2] * wheelVell.v3 + B[1][3] * wheelVell.v4;
//	trueVell.rotationVell = B[2][0] * wheelVell.v1 + B[2][1] * wheelVell.v2 + B[2][2] * wheelVell.v3 + B[2][3] * wheelVell.v4;

//	trueVell.rotationVell *= (CHANGE_TO_ANGLE);
//	trueVell.speed = sqrt(xVell * xVell + yVell * yVell);
//	trueVell.direction = atan2f(yVell, xVell)*CHANGE_TO_ANGLE;
//	FreeMemory(B, 4);
//	FreeMemory(M, 4);
//	return trueVell;
//}
