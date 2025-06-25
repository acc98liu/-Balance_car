#include "stdint.h"
#ifndef __MOTOR_H
#define __MOTOR_H

void Motor_Init(void);
void MotorA_SetSpeed(int8_t Speed);
void MotorB_SetSpeed(int8_t Speed);
void PD_SetSpeed(int8_t Speed);

#endif
