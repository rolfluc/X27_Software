#include "StepperController.h"
#include "PinDefs.h"

// Coil0:
//  012345678910
//	--    --
//    -  -  -
//     --    --

// Coil1
//  012345678910
//	-    --
//   -  -  -  -
//    --    --

typedef enum 
{
	State_0,
	State_1,
	State_2,
	State_3,
	State_4,
	State_5,
	State_Size
}StepperMotorState;

static StepperMotorState currentState = State_0;

static inline void SetPinState(StepperMotorState state)
{
	// TODO assumes all on same bank.
	switch (state)
	{
	case State_0:
		{
			//1001
			// Set
			Pin0.PinPort->BSRR = Pin0.PinNum | Pin3.PinNum;
			// Reset
			Pin0.PinPort->BRR = Pin1.PinNum | Pin2.PinNum;
			break;
		}
	case State_1:
		{
			//1000
			// Set
			Pin0.PinPort->BSRR = Pin0.PinNum;
			// Reset
			Pin0.PinPort->BRR = Pin1.PinNum | Pin2.PinNum | Pin3.PinNum;
			break;
		}
	case State_2:
		{
			//1110
			// Set
			Pin0.PinPort->BSRR = Pin0.PinNum | Pin1.PinNum | Pin2.PinNum;
			// Reset
			Pin0.PinPort->BRR = Pin3.PinNum;
			break;
		}
	case State_3:
		{
			//0110
			// Set
			Pin0.PinPort->BSRR = Pin1.PinNum | Pin2.PinNum;
			// Reset
			Pin0.PinPort->BRR = Pin0.PinNum | Pin3.PinNum;
			break;
		}
	case State_4:
		{
			//0111
			// Set
			Pin0.PinPort->BSRR = Pin1.PinNum | Pin2.PinNum | Pin3.PinNum;
			// Reset
			Pin0.PinPort->BRR = Pin0.PinNum;
			break;
		}
	case State_5:
		{
			//0001
			// Set
			Pin0.PinPort->BSRR = Pin3.PinNum;
			// Reset
			Pin0.PinPort->BRR = Pin0.PinNum | Pin1.PinNum | Pin2.PinNum;
			break;
		}
	case State_Size:
	default: // Intentional fallthrough
		{
			//0000
			Pin0.PinPort->BRR = Pin0.PinNum | Pin1.PinNum | Pin2.PinNum | Pin3.PinNum;
			break;
		}
	}
}


void initPins()
{
	__GPIOA_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.Pin = GPIO_PIN_7 | GPIO_PIN_6 | GPIO_PIN_5 | GPIO_PIN_4;

	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
}

//Dirty dirty implementation. But is technically correct and works.
void Step(uint32_t numberOfMicroSteps_60, Direction dir)
{
	if (dir == Reset)
	{
		SetPinState(State_Size);
		return;
	}
	int8_t accumulator = dir == Clockwise ? 1 : -1;
	for (uint32_t i = 0; i < numberOfMicroSteps_60; i++)
	{
		currentState += accumulator;
		// Rolled over going counter clockwise
		if (currentState > State_Size)
		{
			currentState = State_5;
		}
		// Rolled over going clockwise
		if (currentState == State_Size)
		{
			currentState = State_0;
		}
		SetPinState(currentState);
		HAL_Delay(10);
	}
}