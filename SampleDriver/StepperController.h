#pragma once
#include <stdint.h>
typedef enum
{
	Reset = 0,
	Clockwise,
	Counterclockwise
}Direction;

typedef enum
{
	Speed_Full,
	Speed_75,
	Speed_50,
	Speed_25,
	Speed_Size,
}Speed;

void initPins();
void InitPWM();
void StepBlocking(uint32_t numberOfMicroSteps_60, Direction dir);
void StepNonblocking(uint16_t degreesTenths, Speed speed);
void InitToZero();