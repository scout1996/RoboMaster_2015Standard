#ifndef __PWM_H
#define __PWM_H

#include "sys.h"

#define PWMA TIM2->CCR1
#define PWMB TIM2->CCR2

void PWM_Init(void);

#endif

