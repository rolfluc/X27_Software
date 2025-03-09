#include "StepperController.h"
#include "PinDefs.h"
#include <stdbool.h>

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

typedef void(*stepcb)(void);

static inline void Step1()
{
	//1001
	// Set
	Pin0.PinPort->BSRR = Pin0.PinNum | Pin3.PinNum;
	// Reset
	Pin0.PinPort->BRR = Pin1.PinNum | Pin2.PinNum;
}

static inline void Step2()
{
	//1000
	// Set
	Pin0.PinPort->BSRR = Pin0.PinNum;
	// Reset
	Pin0.PinPort->BRR = Pin1.PinNum | Pin2.PinNum | Pin3.PinNum;
}

static inline void Step3()
{
	//1110
	// Set
	Pin0.PinPort->BSRR = Pin0.PinNum | Pin1.PinNum | Pin2.PinNum;
	// Reset
	Pin0.PinPort->BRR = Pin3.PinNum;
}

static inline void Step4()
{
	//0110
	// Set
	Pin0.PinPort->BSRR = Pin1.PinNum | Pin2.PinNum;
	// Reset
	Pin0.PinPort->BRR = Pin0.PinNum | Pin3.PinNum;
}

static inline void Step5()
{
	//0111
	// Set
	Pin0.PinPort->BSRR = Pin1.PinNum | Pin2.PinNum | Pin3.PinNum;
	// Reset
	Pin0.PinPort->BRR = Pin0.PinNum;
}

static inline void Step6()
{
	//0001
	// Set
	Pin0.PinPort->BSRR = Pin3.PinNum;
	// Reset
	Pin0.PinPort->BRR = Pin0.PinNum | Pin1.PinNum | Pin2.PinNum;
}

static inline void StepNone()
{
	Pin0.PinPort->BRR = Pin0.PinNum | Pin1.PinNum | Pin2.PinNum | Pin3.PinNum;
}

stepcb clockwiseCallbacks[6] = {
	 Step1,
	 Step2,
	 Step3,
	 Step4,
	 Step5,
	 Step6
};
stepcb counterCLockwiseCallbacks[6] = { 
	Step6,
	Step5,
	Step4,
	Step3,
	Step2,
	Step1
};


static const uint32_t maxDegreesTenths = 3150; 
static const uint32_t tenthsDegreesPerSubstep = 3; 
static stepcb* cbDir = clockwiseCallbacks; // Okay to set to CW to start, since will be at 0 degrees
static StepperMotorState currentState = State_Size;
static bool running = false;
static uint32_t currentDegreesTenths = 0; // Actually is zero because we first reset
static int8_t dir = 1;
static uint32_t desiredDegreesTenths = 0;

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

TIM_HandleTypeDef htim;

void TIM1_BRK_TIM15_IRQHandler(void)
{
	__HAL_TIM_CLEAR_FLAG(&htim, TIM_FLAG_UPDATE);
	// Below no longer translates 1:1 with Stepper state. It is used to index into the respective cb awway
	currentState = (currentState + 1) % State_Size;
	cbDir[(uint8_t)currentState]();
	currentDegreesTenths += dir*tenthsDegreesPerSubstep;
	if (currentDegreesTenths == desiredDegreesTenths)
	{
		HAL_TIM_Base_Stop_IT(&htim);
		StepNone();
	}
	
	// HAL_TIM_Base_Start_IT(&htim);
	// TODO restart timer?
}


void InitPWM(void)
{
	__HAL_RCC_TIM15_CLK_ENABLE();
	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	htim.Instance = TIM15;
	htim.Init.Prescaler = 32;
	htim.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim.Init.Period = 20000;
	htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
	htim.Init.RepetitionCounter = 0;
	htim.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim) != HAL_OK)
	{
		__ASM("BKPT 255");
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim, &sClockSourceConfig) != HAL_OK)
	{
		__ASM("BKPT 255");
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig) != HAL_OK)
	{
		__ASM("BKPT 255");
	}
	HAL_NVIC_SetPriority(TIM1_BRK_TIM15_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(TIM1_BRK_TIM15_IRQn);
}

// TODO void. Void bad.
void StepNonblocking(uint16_t degreesTenths, Speed speed)
{
	switch (speed)
	{
	case Speed_Full:
		{
			htim.Instance->PSC = 2;
			break;
		}
	case Speed_75:
		{
			htim.Instance->PSC = 4;
			break;
		}
	case Speed_50:
		{
			htim.Instance->PSC = 8;
			break;
		}
	case Speed_25:
		{
			htim.Instance->PSC = 16;
			break;
		}
	default:
		{
			htim.Instance->PSC = 16;
			break;
		}
	}
	if (degreesTenths == currentDegreesTenths)
	{
		return;
	}
	if (degreesTenths > currentDegreesTenths) 
	{ 
		dir = 1;
		cbDir = clockwiseCallbacks;
	}
	else
	{
		dir = -1;
		cbDir = counterCLockwiseCallbacks;
	}
	if (degreesTenths > maxDegreesTenths)
	{
		// TODO maybe error?
		degreesTenths = maxDegreesTenths;
	}
	degreesTenths = degreesTenths - (degreesTenths % tenthsDegreesPerSubstep);
	desiredDegreesTenths = degreesTenths;
	
	//Kick it off!
	HAL_TIM_Base_Start_IT(&htim);
}

void StepBlocking(uint32_t numberOfMicroSteps_60, Direction dir)
{
	if (dir == Reset)
	{
		StepNone();
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
		clockwiseCallbacks[(uint8_t)currentState]();
		HAL_Delay(1);
	}
}

void InitToZero()
{
	StepBlocking(maxDegreesTenths, Counterclockwise);
}
