#include "stm32f10x.h"                  // Device header
#include "TIM.h"

void TIM1_Init(uint16_t prescaler, uint16_t period)
{
    // 1. ʹ�ܶ�ʱ��1ʱ��
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    // 2. ���ö�ʱ��1
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructure.TIM_Period = period; // �Զ���װ��ֵ
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler; // Ԥ��Ƶֵ
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; // ʱ�ӷ�Ƶ����
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up; // ���ϼ���ģʽ
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    // 3. ʹ�ܶ�ʱ��1
    TIM_Cmd(TIM1, ENABLE);//������ʱ��
}

void Measure_Start(void)
{
    TIM_SetCounter(TIM1, 0); // ���㶨ʱ��������
}

uint32_t Measure_End(uint32_t clock_freq, uint16_t prescaler)
{
    return TIM_GetCounter(TIM1) * (prescaler + 1) * 1000000 / clock_freq; // ��ȡ��ǰ����ֵ
}
