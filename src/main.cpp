
#include <stdio.h>
// #define __USE_GNU
// #define _GNU_SOURCE
#include <sched.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <pthread.h> 

#include <sys/time.h>
#include <unistd.h>

extern "C"
{
	#include "includes.h"
	#include "spi_init.h"
	#include "timer.h"
	#include "send.h"
	#include "moveBase.h" 
	#include "gpio.h"
	#include "task.h"
	#include "robot.h"
}

struct timeval start, endTime;
long long total_time;
int main(void)
{	
	//初始化任务
	ConfigTask();

	while(1)
	{
		// printf("ok\n");
		//程序周期5ms
		if(GetTimeFlag())
		{
			// gRobot.mcuHeart++;
			// TestTurnWeel();
			// //发送调试数据
			// SendDebugInfo();

			gettimeofday(&start, NULL);

				if( gSem.periodSem )
				{
					//位置环10ms周期信号量清零
					gSem.periodSem = PERIOD_SEM_NONE;
				
					//执行走形任务
					WalkTask();

					//发送调试数据
					SendDebugInfo();
				}

				if( gSem.velctrlPeriodSem && gSem.velctrlCmdSem )
				{
					//速度环5ms周期信号量清零
					gSem.velctrlPeriodSem = VELCTRL_PERIOD_SEM_NONE;
				
					//速度环命令信号量减一
					gSem.velctrlCmdSem--;

					if(gSem.velctrlCmdSem <= VELCTRL_CMD_SEM_NONE)
					{
						gSem.velctrlCmdSem = VELCTRL_CMD_SEM_NONE;
					}

					//执行速度环任务
					VelCtrlTask();
				}	
			//与mcu通信
			Communicate2Mcu();

			gettimeofday(&endTime, NULL);  
			total_time = (endTime.tv_sec - start.tv_sec) * 1000000 + (endTime.tv_usec - start.tv_usec);
		}
	}	

	return (0);
}





