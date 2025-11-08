#pragma once
// BASE STM32 HEADER
#include "stm32f4xx.h"

extern TIM_HandleTypeDef adc_trigger;
extern ADC_HandleTypeDef adc_handle;
extern TIM_HandleTypeDef zero_handle;

// Function prototypes for ISR handlers
void ADC_IRQHandler(void);
void TIM3_IRQHandler(void);

