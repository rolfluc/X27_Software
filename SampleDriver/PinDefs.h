#pragma once
#include <stm32g4xx_hal.h>
typedef struct
{
	uint32_t PinNum;
	GPIO_TypeDef* PinPort;
}PinConfig;

const PinConfig Pin0 = { .PinNum = GPIO_PIN_5, .PinPort = GPIOA };
const PinConfig Pin1 = { .PinNum = GPIO_PIN_4, .PinPort = GPIOA }; 
const PinConfig Pin2 = { .PinNum = GPIO_PIN_6, .PinPort = GPIOA }; 
const PinConfig Pin3 = { .PinNum = GPIO_PIN_7, .PinPort = GPIOA }; 