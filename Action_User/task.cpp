
#include <unistd.h>
#include <stdio.h>
#include <math.h>
#include <iostream>
#include "own_serial.h"
extern "C"
{
	#include "spi_init.h"
	#include "timer.h"
	#include "moveBase.h"
	#include "task.h"
	#include "includes.h"
	#include "robot.h"
	#include "path.h"
	#include "send.h"
	#include "ringbuffer.h"
	#include "Move.h"
}

gRobot_t gRobot = {0};
own_serial ownSerial("/dev/ttyUSB0",B115200);//tx2 /dev/ttyTHS2 
                                            //up squared /dev/ttyS5 
                                            //nuc /dev/ttyUSB0

/*
   ===============================================================
   初始化任务
   ===============================================================
   */
void ConfigTask(void)
{
	//fpWrite=fopen("data.txt","w");
	ThreadBindCPU();
	/*初始化*/
	printf("init ..\n");
	//暂时设置为蓝场
	gRobot.courdID = BLUE_COURT;
	//初始化SPI
	initSPI(); 
	//初始化定时器
	TimeInit();
	//TestPath();
	// SerialInit();
	// while(1)
	// {
	// 	std::cout << "this = " <<  ownSerial.writeData("OK%d\r\n", 8, 1555) << std::endl;
	// 	sleep(1);
	// }
	//// gRobot.walkStatus = waitForStart;
	//规划第一段轨迹 需要定位系统的坐标值
	BufferZizeInit(200);
	//ClearRingBuffer();
	//InputPoints2RingBuffer(firstPath, FIRST_PATH_NUM);	
	//等待MCU初始化
	WaitMcuPrepare();
	ClearRingBuffer();
	firstPath[0].point.x=GetX();
	firstPath[0].point.y=GetY();
	firstPath[0].direction=GetAngle();
	InputPoints2RingBuffer(firstPath, FIRST_PATH_NUM);
}

void VelCtrlTask(void)
{
	float vel = 0.0f;
	float velDireciton = 0.0f;
	position_t actVel = {0.0f};

	//获取实际车速
	actVel = GetSpeedWithoutOmega();

	vel = sqrtf(actVel.x * actVel.x + actVel.y * actVel.y);

	if(vel>0.01f)
	{
		velDireciton = atan2f(actVel.y,actVel.x)*CHANGE_TO_ANGLE;
	}
	else
	{
		velDireciton = velDireciton;
	}
		
	//速度环
	// VelControl((robotVel_t){vel , velDireciton , GetWZ()});
	VelControl((robotVel_t){vel , velDireciton , GetWZ()});

}

void WalkTask(void)
{
	Walk();
}
extern uint8_t mcuErrCnt;
extern struct timeval start, end;
extern struct timeval start2, end2;
extern long long total_time;
extern long long total_time2;
long long total_t;
//发送调试数据
void SendDebugInfo(void)
{
	printf("S %d E %d ",(int)gRobot.walkStatus,(int)mcuErrCnt);

	printf("T %d %lld %lld %lld ",(int)GetTimeCounter(),total_time,total_time2,total_t);

	//printf("\n");

	printf("h %d ",(int)gRobot.mcuHeart);	

	printf("p %d %d %d %d %d %d %d %d ",(int)GetX(),(int)GetY(),(int)(GetAngle()*10),\
		(int)GetSpeedX(),(int)GetSpeedY(),(int)(GetWZ()*10),\
		(int)GetSpeedWithoutOmega().x,(int)GetSpeedWithoutOmega().y);

	printf("vp %d %d %d ",(int)gRobot.debugInfomation.virtualPos.point.x,\
		(int)gRobot.debugInfomation.virtualPos.point.y,\
		(int)gRobot.debugInfomation.virtualPos.direction);

	printf("vt %d %d %d ",(int)gRobot.debugInfomation.virtualTarget.point.x,\
		(int)gRobot.debugInfomation.virtualTarget.point.y,\
		(int)gRobot.debugInfomation.virtualTarget.direction);

	printf("2p %d %d %d %d z %d %d %d %d L %d %d ",(int)gRobot.debugInfomation.disRealPos2VirPos,\
		(int)gRobot.debugInfomation.disRealPos2VirPos2,(int)gRobot.debugInfomation.disRealPos2VirTarget,\
		(int)gRobot.debugInfomation.VIEW_L,(int)(gRobot.debugInfomation.posAngleVP*10),\
		(int)(gRobot.debugInfomation.posAngleVT*10),(int)(gRobot.debugInfomation.omega*10),(int)(gRobot.debugInfomation.outputOmega*10),\
		(int)gRobot.debugInfomation.robotlenVP,(int)gRobot.debugInfomation.robotlenVT);

	printf("v %d %d %d %d %d %d %d %d %d %d %d %d ",\
		(int)(gRobot.debugInfomation.originVel*cosf(ANGLE2RAD(gRobot.debugInfomation.originVelDir))),\
		(int)(gRobot.debugInfomation.originVel*sinf(ANGLE2RAD(gRobot.debugInfomation.originVelDir))),\
		(int)gRobot.debugInfomation.fixedVel,\
		(int)(gRobot.debugInfomation.adjustVel.vel*cosf(ANGLE2RAD(gRobot.debugInfomation.adjustVel.direction))),\
		(int)(gRobot.debugInfomation.adjustVel.vel*sinf(ANGLE2RAD(gRobot.debugInfomation.adjustVel.direction))),\
		(int)(gRobot.debugInfomation.sumVel*cosf(ANGLE2RAD(gRobot.debugInfomation.sumVelDir))),\
		(int)(gRobot.debugInfomation.sumVel*sin(ANGLE2RAD(gRobot.debugInfomation.sumVelDir))),\
		(int)gRobot.debugInfomation.velXErr,\
		(int)gRobot.debugInfomation.velYErr,\
		(int)(gRobot.debugInfomation.outputVel*cosf(ANGLE2RAD(gRobot.debugInfomation.outputDirection))),\
		(int)(gRobot.debugInfomation.outputVel*sinf(ANGLE2RAD(gRobot.debugInfomation.outputDirection))),\
		(int)(10*gRobot.debugInfomation.outputDirection));

	printf("w %d %d %d %d %d %d ",(int)gRobot.wheelState.oneTarget.vel,\
	(int)gRobot.wheelState.twoTarget.vel,(int)gRobot.wheelState.threeTarget.vel,\
	(int)gRobot.wheelState.oneTarget.direction,(int)gRobot.wheelState.twoTarget.direction,\
	(int)gRobot.wheelState.threeTarget.direction);
	
	printf("r ");
	printf("%d ",ballCnt);
	for(uint8_t i = 0; i < LENGTH; i++)
	{
		printf("%d ",rxDebugOutput[i]);
	}
	
	//printf("t ");
	// for(uint8_t i = 0; i < LENGTH; i++)
	// {
	// 	printf("%d ",tx[i]);
	// }
	printf("\n");
}

void TestTurnWeel(void)
{
	static uint32_t timeCnt = 0;
	static uint8_t flag = 0;
	static int8_t cnt = 0;

	timeCnt++;
	if(timeCnt % 200 == 0)
	{
		if(flag)
		{
			cnt--;
		}
		else
		{
			cnt++;
		}

		if(cnt >=4 )
		{
			flag = 1;
		}
		else if(cnt <= -4)
		{
			flag = 0;
		}
	}

	gRobot.wheelState.oneTarget.vel = 0.0f;
	gRobot.wheelState.oneTarget.direction = 90.0f * cnt;
	
	gRobot.wheelState.twoTarget.vel = 0.0f;
	gRobot.wheelState.twoTarget.direction = 90.0f * cnt;
	
	gRobot.wheelState.threeTarget.vel = 0.0f;
	gRobot.wheelState.threeTarget.direction = 90.0f * cnt;

	Communicate2Mcu();
}

int8_t ThreadBindCPU(void)
{
	 int cpus = 0;
     cpu_set_t mask;
 
     cpus = sysconf(_SC_NPROCESSORS_CONF);
     printf("cpus: %d\n", cpus);
 
     CPU_ZERO(&mask);    /* 初始化set集，将set置为空*/
     CPU_SET(0, &mask);  /* 依次将0、1、2、3号cpu加入到集合，前提是你的机器是多核处理器*/
     
     /*设置cpu 亲和性（affinity）*/
     if (sched_setaffinity(0, sizeof(mask), &mask) == -1) {
         printf("Set CPU affinity failue, ERROR:%s\n", strerror(errno));
         return -1; 
     }   
     usleep(1000); /* 让当前的设置有足够时间生效*/

}

void SerialInit(void)
{
	int fd2 = 0;

    ownSerial.init(fd2);
    //fd = ownSerial.isopen();
    if(fd2 ==-1)
    {
        std :: cout<<"can't open serial\n";
        exit(0);
    }

}
void TestPath(void)
{
	printf("test path running... \r\n");
	BufferZizeInit(200);
	ClearRingBuffer();
	//规划测试轨迹
	//InputPoints2RingBuffer(testPath, TEST_PATH_NUM);
	pathPlan(testPath, &TEST_PATH_NUM);
	while(1);
}



