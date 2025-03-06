#pragma once
#include <stdint.h>
typedef enum
{
	Reset = 0,
	Clockwise,
	Counterclockwise
}Direction;

void initPins();
void Step(uint32_t numberOfMicroSteps_60, Direction dir);