#ifndef __TIM1_H
#define __TIM1_H

#include "stdint.h"

void TIM1_Init(uint16_t prescaler, uint16_t period);
void Measure_Start(void);
uint32_t Measure_End(uint32_t clock_freq, uint16_t prescaler);

#endif /* __TIM1_H */
