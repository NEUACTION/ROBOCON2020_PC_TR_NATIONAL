#ifndef _TASK_H
#define _TASK_H

void ConfigTask(void);

void VelCtrlTask(void);

void WalkTask(void);

void SendDebugInfo(void);

void TestTurnWeel(void);

void TestPath(void);

int8_t ThreadBindCPU(void);

void SerialInit(void);
#endif
