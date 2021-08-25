#ifndef _SEND_H
#define _SEND_H

#include <stdint.h>

#define LENGTH (31)

extern uint8_t tx[LENGTH];

extern uint8_t rx[LENGTH];
extern uint8_t rxDebugOutput[LENGTH];
typedef union 
{
	uint8_t data8[4];
	int32_t data32;
	float dataf;
}transData_t;

typedef struct
{
	float x;
	float y;
	float angle;
	float speedX;
	float speedY;
	float WZ;
}pps_t;


typedef struct
{
	float x;
	float y;
}position_t;


void WaitMcuPrepare(void);

void Communicate2Mcu(void);

void WriteCmd(void);

void ReadIndomation(uint8_t transFlag);

void SetX(float setValue);

void SetY(float setValue);

void SetAngle(float setValue);

void SetSpeedX(float setValue);

void SetSpeedY(float setValue);

void SetWZ(float setValue);

float GetX(void);

float GetY(void);

float GetAngle(void);

float GetSpeedX(void);

float GetSpeedY(void);

float GetWZ(void);

position_t GetSpeedWithoutOmega(void);


#endif
