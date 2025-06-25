#include "stm32f10x.h"                  // Device header
#include "TIM.h"

void TIM1_Init(uint16_t prescaler, uint16_t period)
{
    // 1. 使能定时器1时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    // 2. 配置定时器1
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = period; // 自动重装载值
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler; // 预分频值
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // 时钟分频因子
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // 向上计数模式
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    // 3. 使能定时器1
    TIM_Cmd(TIM1, ENABLE);//开启定时器
}

void Measure_Start(void)
{
    TIM_SetCounter(TIM1, 0); // 清零定时器计数器
}

uint32_t Measure_End(uint32_t clock_freq, uint16_t prescaler)
{
    return TIM_GetCounter(TIM1) * (prescaler + 1) * 1000000 / clock_freq; // 读取当前计数值
}
