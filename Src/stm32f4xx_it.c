#include "stm32f4xx_it.h"

// ADC interrupt handler
void ADC_IRQHandler(void)
{
    HAL_ADC_IRQHandler(&adc_handle);
}

// TIM3 input capture interrupt handler
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&zero_handle);
}


