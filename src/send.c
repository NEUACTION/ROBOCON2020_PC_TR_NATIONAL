#include "send.h"
#include <stdint.h>
#include "spi_init.h"
#include "moveBase.h"
#include "robot.h"
#include "timer.h"
#include <unistd.h>
#include <math.h>
#include <stdio.h>

//向 mcu master发送消息  
uint8_t tx[LENGTH] = {0};
//接受 mcu master 的消息
uint8_t rx[LENGTH] = {0};
uint8_t rxDebugOutput[LENGTH] = {0};
//mcu初始化结束标志位

static uint8_t mcuTalkOk = 0;

uint8_t mcuErrCnt = 0;

transData_t sendMessage;

transData_t receiveMessage;

static pps_t ppsPos;
static pps_t realPos;

//等待mcu初始化
void WaitMcuPrepare(void)
{

    tx[0] = 'A';
    tx[1] = 'T';
    tx[28] = '\r';
    tx[29] = '\n';
    tx[30] = 0;

    while(!mcuTalkOk)  
    {
        //要延时5ms
        usleep(5000);
        
        //向mcu发送AT并读回mcu的消息
        SPIDataRW(0,tx,rx,LENGTH);

        //如果mcu回复OK 通信成功
        if(rx[0] == 'O' && rx[1] == 'K' && rx[28] == '\r' && rx[29] == '\n')
        {
            mcuTalkOk = 1;

            //解决第一次不经过计算直接给MCU发0
            gSem.periodSem = PERIOD_SEM_SEND;

            ReadIndomation(0);
        }
        else if(rx[1] == 'O' && rx[2] == 'K' && rx[29] == '\r' && rx[30] == '\n')
        {
            mcuTalkOk = 1;

            //解决第一次不经过计算直接给MCU发0
            gSem.periodSem = PERIOD_SEM_SEND;

            ReadIndomation(1);
        }

         //printf("rx ");
         //将处理完的接收数组发出并清空
         for(uint8_t i = 0; i < LENGTH; i++)
         {
             //printf("%d ",(int)rx[i]);
             rx[i] = 0;
         }
         //printf("\n");
    }
}



void Communicate2Mcu(void)
{
    WriteCmd();

    //与mcu通信
    SPIDataRW(0,tx,rx,LENGTH);

    if(rx[0] == 'H' && rx[1] == 'M' && rx[28] == '\r' && rx[29] == '\n')//没有移位
    {
        //gRobot.mcuHeart--;
        mcuErrCnt = 0;

        ReadIndomation(0);
    }
    else if(rx[1] == 'H' && rx[2] == 'M' && rx[29] == '\r' && rx[30] == '\n')//移位
    {
        //gRobot.mcuHeart--;
        mcuErrCnt = 0;

        ReadIndomation(1);
    }
    else
    {
        mcuErrCnt++;
        //限幅
        if(mcuErrCnt >= 100)
            mcuErrCnt = 100;
    }

    //将处理完的接收数组清空
    for(uint8_t i = 0; i < LENGTH; i++)
    {
        rxDebugOutput[i] = rx[i];
        rx[i] = 0;
    }
}

void WriteCmd(void)
{	
    tx[0] = 'H';
    tx[1] = 'D';

    sendMessage.dataf = gRobot.wheelState.oneTarget.vel;
    for(uint8_t i = 0; i < 4; i++)
    {
        tx[i+2] = sendMessage.data8[i];
    }

    sendMessage.dataf = gRobot.wheelState.twoTarget.vel;
    for(uint8_t i = 0; i < 4; i++)
    {
        tx[i+6] = sendMessage.data8[i];
    }

    sendMessage.dataf = gRobot.wheelState.threeTarget.vel;
    for(uint8_t i = 0; i < 4; i++)
    {
        tx[i+10] = sendMessage.data8[i];
    }   

    sendMessage.dataf = gRobot.wheelState.oneTarget.direction;
    for(uint8_t i = 0; i < 4; i++)
    {
        tx[i+14] = sendMessage.data8[i];
    }

    sendMessage.dataf = gRobot.wheelState.twoTarget.direction;
    for(uint8_t i = 0; i < 4; i++)
    {
        tx[i+18] = sendMessage.data8[i];
    }

    sendMessage.dataf = gRobot.wheelState.threeTarget.direction;
    for(uint8_t i = 0; i < 4; i++)
    {
        tx[i+22] = sendMessage.data8[i];
    }

    tx[26] = gRobot.walkStatus;
    if(gRobot.pathPlanFlag == IS_PLANNING)
        tx[27] = 1;
    else if(gRobot.pathPlanFlag == NO_PLANNING)
        tx[27] = 0;

    tx[28] = '\r';
    tx[29] = '\n';
}

void ReadIndomation(uint8_t transFlag)
{
    if(transFlag == 0)
    {
	    for(uint8_t i = 0; i < 4; i++)
        {
            receiveMessage.data8[i] = rx[i+10];
        }
        SetAngle(receiveMessage.dataf);

        for(uint8_t i = 0; i < 4; i++)
        {
            receiveMessage.data8[i] = rx[i+2];
        }
        SetX(receiveMessage.dataf);

        for(uint8_t i = 0; i < 4; i++)
        {
            receiveMessage.data8[i] = rx[i+6];
        }
        SetY(receiveMessage.dataf);

        for(uint8_t i = 0; i < 4; i++)
        {
            receiveMessage.data8[i] = rx[i+10];
        }
        SetAngle(receiveMessage.dataf);

        for(uint8_t i = 0; i < 4; i++)
        {
            receiveMessage.data8[i] = rx[i+14];
        }
        SetSpeedX(receiveMessage.dataf);

        for(uint8_t i = 0; i < 4; i++)
        {
            receiveMessage.data8[i] = rx[i+18];
        }
        SetSpeedY(receiveMessage.dataf);

        for(uint8_t i = 0; i < 4; i++)
        {
            receiveMessage.data8[i] = rx[i+22];
        }
        SetWZ(receiveMessage.dataf);
        if(rx[26] == 1)
         gRobot.nextFlag = 1;
        gRobot.emergencyStopFlag = rx[27];
    }
    else
    {
        for(uint8_t i = 0; i < 4; i++)
        {
            receiveMessage.data8[i] = rx[i+11];
        }
        SetAngle(receiveMessage.dataf);

        for(uint8_t i = 0; i < 4; i++)
        {
            receiveMessage.data8[i] = rx[i+3];
        }
        SetX(receiveMessage.dataf);

        for(uint8_t i = 0; i < 4; i++)
        {
            receiveMessage.data8[i] = rx[i+7];
        }
        SetY(receiveMessage.dataf);

        for(uint8_t i = 0; i < 4; i++)
        {
            receiveMessage.data8[i] = rx[i+11];
        }
        SetAngle(receiveMessage.dataf);

        for(uint8_t i = 0; i < 4; i++)
        {
            receiveMessage.data8[i] = rx[i+15];
        }
        SetSpeedX(receiveMessage.dataf);

        for(uint8_t i = 0; i < 4; i++)
        {
            receiveMessage.data8[i] = rx[i+19];
        }
        SetSpeedY(receiveMessage.dataf);

        for(uint8_t i = 0; i < 4; i++)
        {
            receiveMessage.data8[i] = rx[i+23];
        }
        SetWZ(receiveMessage.dataf);
        if(rx[27] == 1)
         gRobot.nextFlag = 1;
        gRobot.emergencyStopFlag = rx[28];
    }
	//计算路程
	CaculatePath();          
}

void SetX(float setValue)
{
    static float oldPosX = 0.0f;
	
	ppsPos.x = setValue;
    //ppsPos.x = setValue + DIS_OPS2CENTER *cosf(ANGLE2RAD(GetAngle())) + DISX_OPS2CENTER ;
    //realPos.x = -ppsPos.y + 135.0f*cosf(ANGLE2RAD(ppsPos.angle)) - 135.0f;

    //TR检测异常横坐标
    if(gRobot.courdID == BLUE_COURT)
    {
        if(ppsPos.x  > 6500.0f || ppsPos.x < -700.0f)
        {
            printf("POSX ERROR er %d ", (int)ppsPos.x);

            ppsPos.x = oldPosX + GetSpeedX()*INTERVAL/1000000.0f;

            printf("cr %d \r\n", (int)ppsPos.x);
        }
    }
    else if(gRobot.courdID == RED_COURT)
    {
        if(ppsPos.x  < -6500.0f || ppsPos.x > 700.0f)
        {
            printf("POSX ERROR er %d ", (int)ppsPos.x);

            ppsPos.x = oldPosX + GetSpeedX()*INTERVAL/1000000.0f;

            printf("cr %d \r\n", (int)ppsPos.x);
        }
    }
    oldPosX = ppsPos.x;
}

void SetY(float setValue)
{
    static float oldPosY = 0.0f;
	ppsPos.y = setValue;

    //ppsPos.y = setValue + DIS_OPS2CENTER*sinf(ANGLE2RAD(GetAngle())) + DISY_OPS2CENTER ;
    //realPos.y = ppsPos.x + 135.0f*sin(ANGLE2RAD(ppsPos.angle));

    //TR检测异常纵坐标
    if(ppsPos.y  > 10300.0f || ppsPos.y < -700.0f)
    {
        printf("POSY ERROR er %d ", (int)ppsPos.y);

        ppsPos.y = oldPosY + GetSpeedY()*INTERVAL/1000000.0f;

        printf("cr %d \r\n", (int)ppsPos.y);
    }
    oldPosY = ppsPos.y;
}

void SetAngle(float setValue)
{
    static float oldAngle = 0.0f;

	ppsPos.angle = setValue;

    //检测异常角度
    if(fabs(ppsPos.angle) > 270.0f)
    {
        printf("ANGLE ERR er %d ",(int)ppsPos.angle);
        
        ppsPos.angle = oldAngle;

        printf("cr %d \r\n",(int)ppsPos.angle);
    }
    oldAngle = ppsPos.angle;
}

void SetSpeedX(float setValue)
{
    static float oldSpeedX = 0.0f;

    ppsPos.speedX = setValue;

    //检测异常速度
    if(fabs(ppsPos.speedX) > 10000.0f)
    {
        printf("SPEEDX ERR er %d ",(int)ppsPos.speedX);

        ppsPos.speedX = oldSpeedX;

        printf("cr %d \r\n",(int)ppsPos.speedX);
    }
    oldSpeedX = ppsPos.speedX;
}

void SetSpeedY(float setValue)
{
    static float oldSpeedY = 0.0f;

    ppsPos.speedY = setValue;

    //检测异常速度
    if(fabs(ppsPos.speedY) > 10000.0f)
    {
        printf("SPEEDY ERR er %d ",(int)ppsPos.speedY);

        ppsPos.speedY = oldSpeedY;

        printf("cr %d \r\n",(int)ppsPos.speedY);
    }
    oldSpeedY = ppsPos.speedY;
}

void SetWZ(float setValue)
{
	ppsPos.WZ = setValue;
}

float GetX(void)
{
	return ppsPos.x;
}

float GetY(void)
{
	return ppsPos.y;
}

float GetAngle(void)
{
	return ppsPos.angle;
}

float GetSpeedX(void)
{
	return ppsPos.speedX;	
}

float GetSpeedY(void)
{
   	return ppsPos.speedY;	
}

float GetWZ(void)
{
	return ppsPos.WZ;
}

//返回减去绕机器人中心旋转角速度在定位系统位置产生的线速度后的速度
position_t GetSpeedWithoutOmega(void)
{
	position_t vel = {0.0f};
	
	float rotateVel , rotateVelDirection = 0.0f;
	
	rotateVel = ANGLE2RAD(GetWZ())*sqrtf(DISX_OPS2CENTER*DISX_OPS2CENTER + DISY_OPS2CENTER*DISY_OPS2CENTER);
	
	// rotateVelDirection = 90.0f + RAD2ANGLE(atan2f(DISY_OPS2CENTER,DISX_OPS2CENTER)) + GetAngle();
	rotateVelDirection = RAD2ANGLE(atan2f(-135.0f,0.0f)) + GetAngle();
	
	AngleLimit(&rotateVelDirection);
	
	vel.x = GetSpeedX() - rotateVel * cosf(ANGLE2RAD(rotateVelDirection));
	vel.y = GetSpeedY() - rotateVel * sinf(ANGLE2RAD(rotateVelDirection));
	
	return vel;
}













