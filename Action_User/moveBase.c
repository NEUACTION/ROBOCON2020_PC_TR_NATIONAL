/**
  ******************************************************************************
  * @file	  moveBase.c
  * @author	  Action
  * @version   V1.0.0
  * @date	  2018/08/09
  * @brief	 2018省赛底盘运动控制部分
  ******************************************************************************
  * @attention
  *			None
  ******************************************************************************
  */
/* Includes -------------------------------------------------------------------------------------------*/

  #include "moveBase.h"
  #include "calculate.h"
  #include "send.h"
  #include "robot.h"
  #include <stdint.h>
  #include <math.h>

/* Private typedef ------------------------------------------------------------------------------------*/
/* Private define -------------------------------------------------------------------------------------*/
/* Private macro --------------------------------------------------------------------------------------*/
/* Private variables ----------------------------------------------------------------------------------*/
/* Private function prototypes ------------------------------------------------------------------------*/
/* Private functions ----------------------------------------------------------------------------------*/

/**
* @brief  Vel2Pulse将速度转换为脉冲
  * @note
  * @param  vel:速度（mm/s）
  * @retval 脉冲速度  
  */
int Vel2Pulse(float vel)
{
	return (int)(vel/(PI*WHEEL_DIAMETER)*COUNTS_PER_ROUND*WHEEL_REDUCTION_RATIO);
}

/**
* @brief  Pulse2Vel将速度转换为脉冲
  * @note
* @param  pulse:脉冲速度
  * @retval 速度（mm/s）
  */
float Pulse2Vel(int pulse)
{
	return ((float)pulse/COUNTS_PER_ROUND)/WHEEL_REDUCTION_RATIO*PI*WHEEL_DIAMETER;
}


/**
* @brief  OutputVel2Wheel计算每个轮子的速度和朝向并输出到电机上
  * @note
* @param  vel:平移速度大小（mm/s）
		  direction:平移速度方向（-180°到180°）
		  omega:绕底盘中心旋转角速度(度/s),顺时针为负逆时针为正
  * @retval 
  */
void OutputVel2Wheel(float vel, float direction, float omega)
{
	
	wheel_t outputVel = {0.0f};
	gRobot.debugInfomation.outputVel = vel;
	gRobot.debugInfomation.outputDirection = direction;
	//限幅
	if(vel>GetVelMax())
	{
		vel = GetVelMax();
	}
	else if(vel<0.0f)
	{
		vel = 0.0f;
	}
	
	if(omega > 240.0f)
	{
		omega = 240.0f;
	}
	else if(omega < -240.0f)
	{
		omega = -240.0f;
	}
#ifdef SEND_MOVEBASE_DEBUGINFO
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d %d %d ", (int)(vel) , (int)(direction) , (int)(omega));
#endif
	//计算每个轮子的和速度大小和方向
	outputVel.one = CalcWheelSpeed(vel , direction , omega , ONE_VERTICAL_ANG , GetAngle());
	
	outputVel.two = CalcWheelSpeed(vel , direction , omega , TWO_VERTICAL_ANG , GetAngle());
	
	outputVel.three = CalcWheelSpeed(vel , direction , omega , THREE_VERTICAL_ANG , GetAngle());
	
#ifdef SEND_MOVEBASE_DEBUGINFO
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"WW %d %d %d %d ", (int)(outputVel.leftFront.direction) , (int)(outputVel.rightFront.direction) ,\
				(int)(outputVel.leftRear.direction) , (int)(outputVel.rightRear.direction));
#endif
	// printf("WWw %d %d %d %d %d %d", (int)(outputVel.one.direction *10) , (int)(outputVel.two.direction*10) ,\
	// 		(int)(outputVel.three.direction*10) , (int)(outputVel.one.vel) , (int)(outputVel.two.vel) ,\
	// 		(int)(outputVel.three.vel));
	//将和速度输出
	WheelVelControl(outputVel);
}

/**
* @brief  WheelVelControl控制电机速度和位置
  * @note
* @param  wheelVel:四个轮子速度大小和方向结构体
  * @retval 
  */
//记录各个轮子朝向变量
static float oneAng = 180.0f, twoAng = 180.0f, threeAng = 180.0f;//same with mcu direction init

void WheelVelControl(wheel_t wheelVel)
{
#ifdef SEND_MOVEBASE_DEBUGINFO
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d %d %d %d ", (int)(wheelVel.leftFront.direction) , (int)(wheelVel.rightFront.direction) ,\
				(int)(wheelVel.leftRear.direction) , (int)(wheelVel.rightRear.direction));
#endif
	
	//将定位系统坐标系下角度转换为机器人坐标系下角度 direction-=GetAngle()
	Transform2RobotCoodinate(&wheelVel);

#ifdef SEND_MOVEBASE_DEBUGINFO	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d %d %d %d ", (int)(wheelVel.leftFront.direction) , (int)(wheelVel.rightFront.direction) ,\
				(int)(wheelVel.leftRear.direction) , (int)(wheelVel.rightRear.direction));
#endif	

// 	//将机器人坐标系下角度转换为和电机一致 direction = 90.0f - direction
// 	Transform2WheelCoodinate(&wheelVel);

// #ifdef SEND_MOVEBASE_DEBUGINFO	
// 	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
// 			(uint8_t *)"%d %d %d %d ", (int)(wheelVel.leftFront.direction) , (int)(wheelVel.rightFront.direction) ,\
// 				(int)(wheelVel.leftRear.direction) , (int)(wheelVel.rightRear.direction));
// #endif	
// printf("BFX %d %d %d %d %d %d", (int)(wheelVel.one.direction *10) , (int)(wheelVel.two.direction*10) ,\
// 			(int)(wheelVel.three.direction*10) , (int)(wheelVel.one.vel) , (int)(wheelVel.two.vel) ,\
// 			(int)(wheelVel.three.vel));
	//判断是否需要将轮速反向
	JudgeVelDirection(&wheelVel.one, oneAng);
	JudgeVelDirection(&wheelVel.two, twoAng);
	JudgeVelDirection(&wheelVel.three, threeAng);
// printf("AFX %d %d %d %d %d %d", (int)(wheelVel.one.direction *10) , (int)(wheelVel.two.direction*10) ,\
// 			(int)(wheelVel.three.direction*10) , (int)(wheelVel.one.vel) , (int)(wheelVel.two.vel) ,\
// 			(int)(wheelVel.three.vel));
#ifdef SEND_MOVEBASE_DEBUGINFO
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d %d %d %d ", (int)(wheelVel.leftFront.direction) , (int)(wheelVel.rightFront.direction) ,\
				(int)(wheelVel.leftRear.direction) , (int)(wheelVel.rightRear.direction));
#endif	
	//保证旋转为劣弧
	oneAng = TurnInferiorArc(wheelVel.one.direction , oneAng);
	twoAng = TurnInferiorArc(wheelVel.two.direction , twoAng);
	threeAng = TurnInferiorArc(wheelVel.three.direction , threeAng);
// printf("ALH %d %d %d %d %d %d", (int)(oneAng *10) , (int)(twoAng*10) ,\
// 			(int)(threeAng*10) , (int)(wheelVel.one.vel) , (int)(wheelVel.two.vel) ,\
// 			(int)(wheelVel.three.vel));
#ifdef SEND_MOVEBASE_DEBUGINFO
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt ,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d %d %d %d \r\n", (int)(leftFrontAng) , (int)(rightFrontAng) ,\
				(int)(leftRearAng) , (int)(rightRearAng));
#endif
	
	//向电机发送速度和位置命令
	SendCmd2Driver(wheelVel.one.vel , oneAng , wheelVel.two.vel , twoAng,
				   wheelVel.three.vel , threeAng);
}

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
#define LIMIT_VEL (10000.0f)
void SendCmd2Driver(float oneVel , float oneDir , float twoVel , float twoDir,
					float threeVel , float threeDir)
{
	//记录各个轮子实际给出的控制量
	gRobot.wheelState.oneTarget.vel = oneVel;
	gRobot.wheelState.oneTarget.direction = oneDir;
	
	gRobot.wheelState.twoTarget.vel = twoVel;
	gRobot.wheelState.twoTarget.direction = twoDir; 
	
	gRobot.wheelState.threeTarget.vel = threeVel;
	gRobot.wheelState.threeTarget.direction = threeDir;

	if(fabs(gRobot.wheelState.oneTarget.vel) > LIMIT_VEL)
	{
		gRobot.wheelState.oneTarget.vel = gRobot.wheelState.oneTarget.vel/fabs(gRobot.wheelState.oneTarget.vel)*LIMIT_VEL;
	}
	if(fabs(gRobot.wheelState.twoTarget.vel) > LIMIT_VEL)
	{
		gRobot.wheelState.twoTarget.vel = gRobot.wheelState.twoTarget.vel/fabs(gRobot.wheelState.twoTarget.vel)*LIMIT_VEL;
	}
	if(fabs(gRobot.wheelState.threeTarget.vel) > LIMIT_VEL)
	{
		gRobot.wheelState.threeTarget.vel = gRobot.wheelState.threeTarget.vel/fabs(gRobot.wheelState.threeTarget.vel)*LIMIT_VEL;
	}

	
	// //将每个轮子实际的速度和方向发送给电机驱动器
	// VelCrl(CAN1 , ONE_ID , Vel2Pulse(oneVel));
	// PosCrl(CAN1 , ONE_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(oneDir));
	
	// VelCrl(CAN1 , TWO_ID , Vel2Pulse(twoVel));
	// PosCrl(CAN1 , TWO_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(twoDir));
	
	// VelCrl(CAN1 , THREE_ID , Vel2Pulse(threeVel));
	// PosCrl(CAN1 , THREE_TURNING_ID , ABSOLUTE_MODE , WheelAngle2PositionTransform(threeDir));	

}

/**
* @brief  WheelAngle2PositionTransform将轮子朝向角度转化为脉冲
  * @note
* @param  angle:轮子朝向角度
* @retval 对应的电机脉冲位置
  */
int WheelAngle2PositionTransform(float angle)
{
	return (int)(((angle / 360.0f)* WHEEL_TURNING_REDUCTION_RATIO)* COUNTS_PER_ROUND );
}
/**
* @brief  WheelAngle2PositionInverseTransform将轮子脉冲位置转化为角度
  * @note
* @param  position:轮子脉冲位置
* @retval 轮子朝向角度
  */
float WheelAngle2PositionInverseTransform(int position)
{
	return (float)(((float)position / COUNTS_PER_ROUND)/ WHEEL_TURNING_REDUCTION_RATIO * 360.0f);
}
/**
* @brief  Transform2RobotCoodinate将世界坐标系下轮子朝向转换到机器人坐标系下
  * @note
* @param  wheelVel:要转换的轮子速度结构体指针
* @retval
  */
void Transform2RobotCoodinate(wheel_t * wheelVel)
{
	//将定位系统坐标系下角度转换为机器人坐标系下角度
	wheelVel->one.direction-=GetAngle();
	wheelVel->two.direction-=GetAngle();
	wheelVel->three.direction-=GetAngle();

	//将角度限制在180度到-180度范围内
	AngleLimit(&wheelVel->one.direction);
	AngleLimit(&wheelVel->two.direction);
	AngleLimit(&wheelVel->three.direction);

}
/**
* @brief  Transform2WheelCoodinate将机器人坐标系下轮子朝向转换到轮子坐标系下
  * @note
* @param  wheelVel:要转换的轮子速度结构体指针
* @retval 
  */
void Transform2WheelCoodinate(wheel_t * wheelVel)
{
	//将机器人坐标系下轮子朝向转换为轮子坐标系下角度
	wheelVel->one.direction = 90.0f - wheelVel->one.direction;
	wheelVel->two.direction = 90.0f - wheelVel->two.direction;
	wheelVel->three.direction = 90.0f - wheelVel->three.direction;
	
	//将角度限制在-180°到180°
	AngleLimit(&wheelVel->one.direction);
	AngleLimit(&wheelVel->two.direction);
	AngleLimit(&wheelVel->three.direction);

}
/**
* @brief  AngleLimit角度限幅，将角度限制在-180°到180°
  * @note
* @param  angle:要限制的值
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
* @brief  ReturnLimitAngle返回限制后的角度值
  * @note
* @param  angle:要限制的值
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


/**
* @brief  JudgeVelDirection判断轮子是否需要反转
  * @note
* @param  targetVel:目标速度大小和方向
		  actualAngle：当前轮子正方向角度
* @retval 
  */
void JudgeVelDirection(wheelVel_t *targetVel , float actualAngle)
{
	int n = 0;
	float angleErr = 0.0f;
	
	//将目标角度和当前实际角度转换到一个360度周期中
	n = (int)(actualAngle/180.0f) - (int)(actualAngle/360.0f);
	
	targetVel->direction = n * 360.0f + targetVel->direction;
	
	//计算目标角度和实际角度的误差
	angleErr = targetVel->direction - actualAngle;
	
	//将误差限制在-180度到180度
	AngleLimit(&angleErr);
	
	//如果角度误差大于90度则将速度反向并将目标角度加180度
	if(fabs(angleErr)>90.0f)
	{
		targetVel->vel = -(targetVel->vel);
		targetVel->direction = targetVel->direction + 180.0f;
		
		//保证处理后的目标角度和当前实际角度在一个周期中
		if(targetVel->direction>(n * 360.0f + 180.0f))
		{
			targetVel->direction -=360.0f;
		}
		else if(targetVel->direction<(n * 360.0f - 180.0f))
		{
			targetVel->direction+=360.0f;
		}
	}
}

/**
* @brief  TurnInferiorArc确保旋转角度为劣弧
  * @note
* @param  targetAngle:目标角度
		  actualAngle：当前角度
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
* @brief  CalcWheelSpeed计算轮子的和速度大小和方向
  * @note
* @param  vel:平移速度大小（mm/s）
		  direction:平移速度方向（-180°到180°）
		  omega:绕底盘中心旋转角速度(度/s),顺时针为负逆时针为正
		  angleN:在机器人坐标系下旋转线速度的正方向(单位：度)
* @retval 
}
  */
wheelVel_t CalcWheelSpeed(float vel , float direction , float omega , float angleN , float postureAngle)
{
	wheelVel_t sumVel = {0.0f};
	float velX , velY = 0.0f;
	float velN ,velNDirection= 0.0f;
	float sumVelX , sumVelY = 0.0f;
	
	//计算平移速度的X，Y分量
	velX = vel*cosf(ANGLE2RAD(direction));
	velY = vel*sinf(ANGLE2RAD(direction));
	
	//计算旋转的线速度
	velN = ANGLE2RAD(omega)*MOVEBASE_RADIUS;
	
	velNDirection = angleN + postureAngle;
	AngleLimit(&velNDirection);
	
	//计算和速度大小和方向
	sumVelX = velX + velN * cosf(ANGLE2RAD(velNDirection));
	sumVelY = velY + velN * sinf(ANGLE2RAD(velNDirection));

	sumVel.vel = sqrtf(sumVelX * sumVelX + sumVelY * sumVelY);
	
	//计算合成速度方向时将0向量单独处理
	if(sumVel.vel>0.01f)
	{
		sumVel.direction = RAD2ANGLE(atan2f(sumVelY , sumVelX));
	}
	else
	{
		sumVel.direction = direction;
	}
	
	return sumVel;
}

/********************* (C) COPYRIGHT NEU_ACTION_2018 ****************END OF FILE************************/
